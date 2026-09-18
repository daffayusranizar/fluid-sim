Shader "FluidSim/FluidSurface"
{
    // Screen-space surface built from the particle impostors' depth.
    //
    // Particles rendered as individual discs read as a cloud of dots no matter how
    // small or well antialiased they are, because every disc keeps its own
    // silhouette. The standard fix is to stop drawing discs and start drawing the
    // SURFACE they collectively imply:
    //
    //   depth prepass (min)  ->  bilateral blur  ->  normals from the blurred depth
    //                        ->  shade  ->  composite
    //
    // The bilateral blur is what makes this work. A plain blur would melt the
    // silhouette into the background, so the range term weights a neighbour by how
    // similar its depth is, keeping the edge sharp while smoothing the interior
    // where discs meet. That smoothed interior is what turns per-disc silhouettes
    // into one continuous sheet, and it is also what makes the depth differentiable
    // enough to reconstruct normals from.

    Properties
    {
        _FarDepth ("Far Depth", Float) = 10000
    }

    SubShader
    {
        Tags
        {
            "RenderType" = "Transparent"
            "Queue" = "Transparent"
            "IgnoreProjector" = "True"
            "RenderPipeline" = "UniversalPipeline"
        }

        HLSLINCLUDE
        #include "Packages/com.unity.render-pipelines.universal/ShaderLibrary/Core.hlsl"

        Texture2D<float4> _FluidDepth;
        SamplerState sampler_FluidDepth;

        Texture2D<float4> _FluidColour;
        SamplerState sampler_FluidColour;

        float _FarDepth;
        float4 _FluidDepth_TexelSize;

        float2 _BlurDir;
        float _RangeSigma;

        float2 _ViewSize;

        float4 _SurfaceLightDir;
        float _SurfaceAmbient;
        float _SurfaceSpecular;
        float _SurfaceShininess;
        float _EdgeSoftness;

        float SampleDepth(float2 uv)
        {
            return _FluidDepth.SampleLevel(sampler_FluidDepth, uv, 0).r;
        }
        ENDHLSL

        // ------------------------------------------------------------------
        // Composite: reconstruct normals from the smoothed depth and shade
        // ------------------------------------------------------------------
        Pass
        {
            Name "Composite"

            Tags { "LightMode" = "UniversalForward" }

            Blend SrcAlpha OneMinusSrcAlpha
            ZTest Always
            ZWrite Off
            Cull Off

            HLSLPROGRAM
            #pragma vertex vert
            #pragma fragment frag
            #pragma target 4.5

            struct Attributes { float4 vertex : POSITION; float2 uv : TEXCOORD0; };
            struct Varyings { float4 positionCS : SV_POSITION; float2 uv : TEXCOORD0; };

            Varyings vert(Attributes IN)
            {
                Varyings OUT;
                OUT.positionCS = TransformObjectToHClip(IN.vertex.xyz);
                OUT.uv = IN.uv;
                return OUT;
            }

            half4 frag(Varyings IN, out float outDepth : SV_Depth) : SV_Target
            {
                float2 uv = IN.uv;
                float d = SampleDepth(uv);

                outDepth = 1.0;

                if (d >= _FarDepth * 0.5) discard;

                // View-space position of the surface point. The camera is
                // orthographic and axis-aligned, so uv maps linearly onto the view
                // plane and no projection matrix is needed.
                float3 P = float3((uv - 0.5) * _ViewSize, -d);

                // Swapped so the normal points back at the camera (negative z).
                float3 n = normalize(cross(ddy(P), ddx(P)));

                float4 c = _FluidColour.SampleLevel(sampler_FluidColour, uv, 0);

                // The colour target was accumulated with alpha blending, so dividing
                // by the accumulated alpha recovers the average surface colour.
                float3 base = c.rgb / max(c.a, 1e-3);

                float3 L = normalize(_SurfaceLightDir.xyz);
                float ndl = saturate(dot(n, L));
                float3 shaded = base * (_SurfaceAmbient + (1.0 - _SurfaceAmbient) * ndl);

                // A specular highlight sweeping across the reconstructed normals is
                // what reads as "wet surface" rather than as shaded paint.
                float ndh = saturate(dot(n, normalize(L + float3(0, 0, -1))));
                shaded += pow(ndh, max(1.0, _SurfaceShininess)) * _SurfaceSpecular;

                // Antialias the silhouette against the depth jump that defines it.
                float gradient = length(float2(ddx(d), ddy(d)));
                float alpha = saturate(1.0 - gradient * _EdgeSoftness);

                return half4(shaded, alpha);
            }
            ENDHLSL
        }
        // ------------------------------------------------------------------
        // Bilateral blur, one axis per dispatch
        // ------------------------------------------------------------------
        // PASS ORDER IS LOAD-BEARING. Composite is pass 0 because
        // Graphics.DrawMesh draws pass 0 and has no shaderPass argument; Blur is
        // pass 1 and is always blitted with an explicit pass index, because
        // Graphics.Blit's default pass (-1) runs more than one pass and would
        // otherwise composite the surface into the depth target.
        Pass
        {
            Name "Blur"

            ZTest Always
            ZWrite Off
            Cull Off

            HLSLPROGRAM
            #pragma vertex vert
            #pragma fragment frag
            #pragma target 4.5

            struct Attributes { float4 vertex : POSITION; float2 uv : TEXCOORD0; };
            struct Varyings { float4 positionCS : SV_POSITION; float2 uv : TEXCOORD0; };

            Varyings vert(Attributes IN)
            {
                Varyings OUT;
                OUT.positionCS = float4(IN.vertex.xy, 0.0, 1.0);
                OUT.uv = IN.uv;
                return OUT;
            }

            float frag(Varyings IN) : SV_Target
            {
                const int Radius = 6;

                float centre = SampleDepth(IN.uv);

                // Background must not bleed into the surface, or the silhouette
                // dissolves. Nothing else in the kernel removes it.
                if (centre >= _FarDepth * 0.5) return _FarDepth;

                float2 step = _BlurDir * _FluidDepth_TexelSize.xy;

                float sum = 0.0;
                float weightSum = 0.0;

                for (int i = -Radius; i <= Radius; i++)
                {
                    float2 uv = IN.uv + step * i;
                    float d = SampleDepth(uv);

                    if (d >= _FarDepth * 0.5) continue;

                    float spatial = exp(-(float)(i * i) / (2.0 * 12.0));
                    float diff = d - centre;
                    float range = exp(-(diff * diff) / (2.0 * _RangeSigma * _RangeSigma));

                    float w = spatial * range;
                    sum += d * w;
                    weightSum += w;
                }

                return weightSum > 1e-5 ? sum / weightSum : centre;
            }
            ENDHLSL
        }

    }

    Fallback Off
}
