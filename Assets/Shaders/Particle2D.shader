Shader "FluidSim/Particle2D"
{
    // Renders one instanced quad per particle. Particle state arrives as a single
    // structured buffer whose layout must match the C# Particle2D struct field for
    // field: 40 bytes, position/velocity/force as float2, then four floats.
    //
    // Colour comes from a 1D gradient texture sampled by speed, and the disc edge
    // is antialiased with fwidth so particles read as smooth fluid at any zoom
    // rather than as hard-edged dots.

    Properties
    {
        _ParticleRadius ("Particle Radius", Float) = 0.1
        _VelocityMax ("Velocity Display Max", Float) = 5
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

            struct Attributes
            {
                float4 vertex : POSITION;
                float2 uv : TEXCOORD0;
            };

            struct Varyings
            {
                float4 positionCS : SV_POSITION;
                float2 uv : TEXCOORD0;
                float3 colour : TEXCOORD1;
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
                OUT.uv = IN.uv;

                return OUT;
            }

            half4 frag(Varyings IN) : SV_Target
            {
                // Map UV to [-1, 1] and keep only the unit disc
                float2 offset = (IN.uv - 0.5f) * 2.0f;
                float sqrDst = dot(offset, offset);

                // fwidth gives the change in radius per pixel, so the softened band
                // is always one pixel wide regardless of zoom or particle size
                float delta = fwidth(sqrt(sqrDst));
                float alpha = 1.0f - smoothstep(1.0f - delta, 1.0f + delta, sqrDst);

                return half4(IN.colour, alpha);
            }
            ENDHLSL
        }
    }

    Fallback Off
}
