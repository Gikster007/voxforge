#pragma once

enum class LightType
{
    POINT,
    DIRECTIONAL,
    SPOT,
    AREA,
    LINE
};

struct alignas(32) Light
{
    Light(LightType _type, float3 _pos = float3(1.0f, 1.0f, 1.0f), float3 _color = float3(1.0f, 1.0f, 1.0f), float3 _dir = float3(1.0f, 1.0f, 1.0f), float _cutoff_angle = 0.0f,
          float _spot_exponent = 0.0f, float _radius = 1.0f)
        : type(_type), pos(_pos), color(_color), dir(_dir), cutoff_angle(_cutoff_angle), spot_exponent(_spot_exponent), radius(_radius)
    {
    }

    float3 pos;
    float3 color;

    float cutoff_angle;
    float spot_exponent;

    float3 dir;

    float radius;

    LightType type;
    
    float3 dummy;
    
    /*float inner_angle;
    float outer_angle;*/
};