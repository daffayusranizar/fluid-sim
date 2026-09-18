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
    //   2. a depth that varies as sqrt(1 - r^2) across the face, which is what makes
    //      a depth buffer meaningful. A flat quad writes one depth for the whole
    //      particle, so a bilateral pass over it and normals reconstructed from it
    //      would both see a plane. T-033's screen-space smoothing depends on this.
    //
    // Colour comes from a 1D gradient texture sampled by speed, and the disc edge is
    // antialiased with fwidth so particles read as smooth fluid at any zoom rather
    // than as hard-edged dots.

    Properties
    {
        _ParticleRadius ("Particle Radius", Float) = 0.1
        _VelocityMax ("Velocity Display Max", Float) = 5

        _LightDir ("Impostor Light Direction", Vector) = (-0.4, 0.5, 0.8, 0)
        _Ambient ("Impostor Ambient", Range(0, 1)) = 0.35
        _Specular ("Impostor Specular", Range(0, 1)) = 0.35
        _Shininess ("Impostor Shininess", Float) = 32
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

        Blend SrcAlpha OneMinusSrcAlpha
        ZWrite Off
        Cull Off

        Pass
        {
            HLSLPROGRAM
            #pragma vertex vert
            #pragma fragment frag
            #pragma target 4.5

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
            };

            Varyings vert(Attributes IN, uint instanceID : SV_InstanceID)
            {
                Varyings OUT;

                Particle p = Particles[instanceID];

                // Colour by speed, normalised against the display maximum
                float speed = length(p.velocity);
                float t = saturate(speed / max(_VelocityMax, 1e-5));
                OUT.colour = ColourMap.SampleLevel(sampler_ColourMap, float2(t, 0.5f), 0).rgb;

                // Particle positions are already world space, so build the quad in
                // world space directly and skip the object transform entirely.
                float3 worldPos = float3(p.position + IN.vertex.xy * _ParticleRadius, 0);

                OUT.positionCS = TransformWorldToHClip(worldPos);
                OUT.local = IN.vertex.xy * 2.0;
                OUT.centre = p.position;

                return OUT;
            }

            void frag(Varyings IN, out half4 outColour : SV_Target, out float outDepth : SV_Depth)
            {
                float2 local = IN.local;
                float sqrDst = dot(local, local);

                // Outside the unit disc: still has to emit a depth. Park it on the
                // far plane so the discarded fragments cannot occlude anything.
                if (sqrDst > 1.0)
                {
                    outColour = half4(0, 0, 0, 0);
                    outDepth = 1.0;
                    return;
                }

                // Hemisphere front face. The world-space disc radius is half the
                // quad extent, because the quad spans -0.5..0.5 before scaling.
                float discRadius = 0.5 * _ParticleRadius;
                float h = sqrt(max(0.0, 1.0 - sqrDst));

                float3 normal = float3(local, h);

                float3 worldFront = float3(
                    IN.centre + local * discRadius,
                    -h * discRadius);

                float4 clip = TransformWorldToHClip(worldFront);
                outDepth = clip.z / clip.w;

                // Cheap impostor shading. Not URP lighting: a fixed direction so a
                // particle reads as a sphere without needing a light in the scene.
                float3 L = normalize(_LightDir.xyz);
                float ndl = saturate(dot(normal, L));
                float3 shaded = IN.colour * (_Ambient + (1.0 - _Ambient) * ndl);

                float3 V = float3(0, 0, 1);
                float3 H = normalize(L + V);
                shaded += pow(saturate(dot(normal, H)), max(1.0, _Shininess)) * _Specular;

                // fwidth gives the change in radius per pixel, so the softened band
                // is always one pixel wide regardless of zoom or particle size
                float delta = fwidth(sqrt(sqrDst));
                float alpha = 1.0f - smoothstep(1.0f - delta, 1.0f + delta, sqrDst);

                outColour = half4(shaded, alpha);
            }
            ENDHLSL
        }
    }

    Fallback Off
}
