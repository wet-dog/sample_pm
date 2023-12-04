#ifndef POINT_LIGHT_H
#define POINT_LIGHT_H

#include "sampler.h"

struct SurfaceInfo {
    Vector3 position;
    Vector3 geometricNormal;
    Vector3 shadingNormal;
    Vector3 dpdu;  // tangent vector
    Vector3 dpdv;  // bitangent vector
    Vector2 texcoords;
    Vector2 barycentric;
};

// light interface
class Light {
public:
    virtual Vector3 Le(const SurfaceInfo& info, const Vector3& dir) const = 0;
    virtual SurfaceInfo samplePoint(Sampler& sampler, float& pdf) const = 0;
    virtual Vector3 sampleDirection(const SurfaceInfo& surfInfo, Sampler& sampler,
        float& pdf) const = 0;
};

class PointLight final : public Light
{
public:
    explicit PointLight(const Vector3& le, const Vector3& pos) : le(le), pos(pos) {}

    Vector3 Le(const SurfaceInfo& info, const Vector3& dir) const override
    {
        return le;
    }

    SurfaceInfo samplePoint(Sampler& sampler, float& pdf_pos) const override
    {
        SurfaceInfo surfaceInfo;
        surfaceInfo.position = pos;
        surfaceInfo.shadingNormal = UniformSampleSphere(sampler.getNext2D());
        pdf_pos = UniformSpherePdf();
        return surfaceInfo;
    }

    Vector3 sampleDirection(const SurfaceInfo& surfInfo, Sampler& sampler,
                            float& pdf) const override
    {
        Vector3 dir = UniformSampleSphere(sampler.getNext2D());
        return dir;
    }

private:
    Vector3 le = { 1.0f, 1.0f, 1.0f };
    Vector3 pos;
};

//class AreaLight : public Light {
//private:
//    const Vector3 le;  // emission
//
//public:
//    // return emission
//    Vector3 Le(const SurfaceInfo& info, const Vector3& dir) const override {
//        return le;
//    }
//
//    // sample point on the light
//    SurfaceInfo samplePoint(Sampler& sampler, float& pdf) const override {
//        return triangle->samplePoint(sampler, pdf);
//    }
//
//    // sample direction from the light
//    Vector3 sampleDirection(const SurfaceInfo& surfInfo, Sampler& sampler,
//        float& pdf) const override {
//        const Vector3 dir = sampleCosineHemisphere(sampler.getNext2D(), pdf);
//
//        // transform direction from local to world
//        return localToWorld(dir, surfInfo.dpdu, surfInfo.shadingNormal,
//            surfInfo.dpdv);
//    }
//};


#endif // POINT_LIGHT_H
