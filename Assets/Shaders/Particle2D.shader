Shader "FluidSim/Particle2D"
{
    // Renders one instanced quad per particle. Particle state arrives as a single
    // structured buffer whose layout must match the C# Particle2D struct field for
    // field: 40 bytes, position/velocity/force as float2, then four floats.
    //
    // Each quad is a SPHERICAL IMPOSTOR, not a flat disc. The fragment reconstructs
    // the front face of a hemisphere from the quad's local coordinate, which gives
    // two things a flat quad cannot:
    //
    //   1. a normal, so a particle reads as a lit sphere instead of a coloured dot;
    //   2. a depth that varies as sqrt(1 - r^2) across the face.
    //
    // Two passes. "ParticleColour" is the visible look and can be drawn straight to
    // the camera. "FluidDepth" writes view depth only, min-blended, and is the input
    // the screen-space surface is built from. The depth pass exists because the
    // colour pass is alpha-blended and cannot produce a usable depth: the surface
    // needs the FRONT-MOST depth per pixel, which is a min-reduction, not a blend.

    Properties
    {
        _ParticleRadius ("Particle Radius", Float) = 0.1
        _VelocityMax ("Velocity Display Max", Float) = 5

        _LightDir ("Impostor Light Direction", Vector) = (-0.4, 0.5, 0.8, 0)
        _Ambient ("Impostor Ambient", Range(0, 1)) = 0.35
        _Specular ("Impostor Specular", Range(0, 1)) = 0.35
        _Shininess ("Impostor Shininess", Float) = 32

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

        struct Particle
        {
            float2 position;
            float2 velocity;
            float2 force;
            float density;
            float pressure;
            float nearDensity;
            float nearPressure;
        };

        StructuredBuffer<Particle> Particles;

        Texture2D<float4> ColourMap;
        SamplerState sampler_ColourMap;

        float _ParticleRadius;
        float _VelocityMax;

        float4 _LightDir;
        float _Ambient;
        float _Specular;
        float _Shininess;
        float _FarDepth;

        struct Attributes
        {
            float4 vertex : POSITION;
            float2 uv : TEXCOORD0;
        };

        struct Varyings
        {
            float4 positionCS : SV_POSITION;
            float3 colour : TEXCOORD0;
            float2 local : TEXCOORD1;    // quad-local, spanning [-1, 1]
            float2 centre : TEXCOORD2;   // particle centre in world XY
            float speed : TEXCOORD3;
        };

        Varyings vert(Attributes IN, uint instanceID : SV_InstanceID)
        {
            Varyings OUT;

            Particle p = Particles[instanceID];

            float speed = length(p.velocity);
            float t = saturate(speed / max(_VelocityMax, 1e-5));
            OUT.colour = ColourMap.SampleLevel(sampler_ColourMap, float2(t, 0.5f), 0).rgb;
            OUT.speed = speed;

            // Particle positions are already world space, so build the quad in world
            // space directly and skip the object transform entirely. The quad is flat
            // here; the fragment is what makes it a hemisphere.
            float3 worldPos = float3(p.position + IN.vertex.xy * _ParticleRadius, 0);

            OUT.positionCS = TransformWorldToHClip(worldPos);
            OUT.local = IN.vertex.xy * 2.0;
            OUT.centre = p.position;

            return OUT;
        }

        // World-space front face of the impostor, and its view depth.
        void ImpostorFront(float2 local, float2 centre, out float3 worldFront, out float h)
        {
            float sqrDst = dot(local, local);
            h = sqrt(max(0.0, 1.0 - sqrDst));

            // The quad spans -0.5..0.5 before scaling, so the disc's world radius is
            // half of _ParticleRadius.
            float discRadius = 0.5 * _ParticleRadius;
            worldFront = float3(centre + local * discRadius, -h * discRadius);
        }

        // One-pixel-wide antialiased disc mask.
        half DiscAlpha(float sqrDst)
        {
            float delta = fwidth(sqrt(sqrDst));
            return 1.0 - smoothstep(1.0 - delta, 1.0 + delta, sqrDst);
        }
        ENDHLSL

        // ------------------------------------------------------------------
        // Visible pass
        // ------------------------------------------------------------------
        Pass
        {
            Name "ParticleColour"

            Blend SrcAlpha OneMinusSrcAlpha
            ZWrite Off
            Cull Off

            HLSLPROGRAM
            #pragma vertex vert
            #pragma fragment frag
            #pragma target 4.5

            void frag(Varyings IN, out half4 outColour : SV_Target, out float outDepth : SV_Depth)
            {
                float sqrDst = dot(IN.local, IN.local);

                if (sqrDst > 1.0)
                {
                    outColour = half4(0, 0, 0, 0);
                    outDepth = 1.0;
                    return;
                }

                float3 worldFront;
                float h;
                ImpostorFront(IN.local, IN.centre, worldFront, h);

                float4 clip = TransformWorldToHClip(worldFront);
                outDepth = clip.z / clip.w;

                float3 normal = float3(IN.local, h);

                float3 L = normalize(_LightDir.xyz);
                float ndl = saturate(dot(normal, L));
                float3 shaded = IN.colour * (_Ambient + (1.0 - _Ambient) * ndl);

                float3 V = float3(0, 0, 1);
                float3 H = normalize(L + V);
                shaded += pow(saturate(dot(normal, H)), max(1.0, _Shininess)) * _Specular;

                outColour = half4(shaded, DiscAlpha(sqrDst));
            }
            ENDHLSL
        }

        // ------------------------------------------------------------------
        // Depth prepass: front-most view depth per pixel
        // ------------------------------------------------------------------
        Pass
        {
            Name "FluidDepth"

            // Min-reduction of a single channel. The render target is cleared to
            // _FarDepth, so every fragment that is not fluid writes the far value and
            // loses the min, which is what makes "no particle here" the absence of a
            // signal rather than a special case.
            Blend One One
            BlendOp Min
            ZWrite Off
            ZTest Always
            Cull Off
            ColorMask R

            HLSLPROGRAM
            #pragma vertex vert
            #pragma fragment frag
            #pragma target 4.5

            float frag(Varyings IN) : SV_Target
            {
                float sqrDst = dot(IN.local, IN.local);

                // Outside the disc: must not win the min.
                if (sqrDst > 1.0) return _FarDepth;

                float3 worldFront;
                float h;
                ImpostorFront(IN.local, IN.centre, worldFront, h);

                // View depth, so the value is camera-relative and needs no assumption
                // about where the particle plane sits.
                return -TransformWorldToView(worldFront).z;
            }
            ENDHLSL
        }
    }

    Fallback Off
}
