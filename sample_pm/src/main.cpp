//-------------------------------------------------------------------------------------------------
// File : main.cpp
// Desc : Main Entry Point.
// Copyright(c) Project Asura. All right reserved.
//-------------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------------
// Inlcudes
//-------------------------------------------------------------------------------------------------
#include <r3d_math.h>
#include <r3d_camera.h>
#include <r3d_shape.h>
#include <vector>
#include <stb/stb_image_write.h>
#include "photon_map.h"
#include "point_light.h"
#include "sampler.h"

#define PHOTON_DIFFUSE_ONLY 1
#define INDEX_LOOP 0

namespace
{
    void break_if_nan(const Vector3& v)
    {
        if (v != v)
        {
            __debugbreak();
        }
    }

    void divide(std::vector<Vector3>& image, int samples)
    {
        for (auto& pixel : image)
        {
            pixel /= samples;
        }
    }

    // Frisvad orthonormal basis function
    inline void orthonormal_basis(const Vector3& n, Vector3& u, Vector3& v) {
        if (n.z < -0.9999999f) // Handle the singularity
        {
            u = Vector3(0.0f, -1.0f, 0.0f);
            v = Vector3(-1.0f, 0.0f, 0.0f);
            return;
        }
        const float a = 1.0f / (1.0f + n.z);
        const float b = -n.x * n.y * a;
        u = Vector3(1.0f - n.x * n.x * a, b, -n.x);
        v = Vector3(b, 1.0f - n.y * n.y * a, -n.y);
    }

    // transform direction from world to local
    inline Vector3 worldToLocal(const Vector3& v, const Vector3& lx, const Vector3& ly,
        const Vector3& lz) {
        return Vector3(dot(v, lx), dot(v, ly), dot(v, lz));
    }

    // transform direction from local to world
    inline Vector3 localToWorld(const Vector3& v, const Vector3& lx, const Vector3& ly,
        const Vector3& lz) {
        return Vector3(
            lx.x * v.x + ly.x * v.y + lz.x * v.z,
            lx.y * v.x + ly.y * v.y + lz.y * v.z,
            lx.z * v.x + ly.z * v.y + lz.z * v.z);
    }
}

namespace {

//-------------------------------------------------------------------------------------------------
// Global Varaibles.
//-------------------------------------------------------------------------------------------------
const int     g_max_depth = 100;
const Vector3 g_back_ground (0.0,   0.0,    0.0);
#if PHOTON_DIFFUSE_ONLY
static const std::vector<Sphere> g_spheres = {
    // Left wall
    Sphere(1e5,     Vector3(-1e5 + 99.0,   40.8,          81.6), Vector3(0.14,  0.45,  0.091), ReflectionType::Diffuse,          Vector3(0, 0, 0)),
    // Right wall
    Sphere(1e5,     Vector3( 1e5 + 1.0,    40.8,          81.6), Vector3(0.63,  0.065,  0.05), ReflectionType::Diffuse,          Vector3(0, 0, 0)),
    Sphere(1e5,     Vector3(50.0,          40.8,           1e5), Vector3(0.75,  0.75,  0.75), ReflectionType::Diffuse,          Vector3(0, 0, 0)),
    Sphere(1e5,     Vector3(50.0,          40.8,  -1e5 + 170.0), Vector3(0.01,  0.01,  0.01), ReflectionType::Diffuse,          Vector3(0, 0, 0)),
    Sphere(1e5,     Vector3(50.0,           1e5,          81.6), Vector3(0.75,  0.75,  0.75), ReflectionType::Diffuse,          Vector3(0, 0, 0)),
    Sphere(1e5,     Vector3(50.0,   -1e5 + 81.6,          81.6), Vector3(0.75,  0.75,  0.75), ReflectionType::Diffuse,          Vector3(0, 0, 0)),
    // Two spheres
    Sphere(16.5,    Vector3(27.0,          16.5,          47.0), Vector3(0.99,  0.99,  0.99), ReflectionType::Diffuse,          Vector3(0, 0, 0)),
    Sphere(16.5,    Vector3(73.0,          16.5,          78.0), Vector3(0.99,  0.99,  0.99), ReflectionType::Diffuse,          Vector3(0, 0, 0)),
    // Point light
    Sphere(5.0,     Vector3(50.0,          81.6,          81.6), Vector3(),                   ReflectionType::Diffuse,          Vector3(12, 12, 12))
};
#else // PHOTON_DIFFUSE_ONLY
const Sphere  g_spheres[] = {
    Sphere(1e5,     Vector3(1e5 + 1.0,    40.8,          81.6), Vector3(0.25,  0.75,  0.25), ReflectionType::Diffuse,          Vector3(0, 0, 0)),
    Sphere(1e5,     Vector3(-1e5 + 99.0,   40.8,          81.6), Vector3(0.25,  0.25,  0.75), ReflectionType::Diffuse,          Vector3(0, 0, 0)),
    Sphere(1e5,     Vector3(50.0,          40.8,           1e5), Vector3(0.75,  0.75,  0.75), ReflectionType::Diffuse,          Vector3(0, 0, 0)),
    Sphere(1e5,     Vector3(50.0,          40.8,  -1e5 + 170.0), Vector3(0.01,  0.01,  0.01), ReflectionType::Diffuse,          Vector3(0, 0, 0)),
    Sphere(1e5,     Vector3(50.0,           1e5,          81.6), Vector3(0.75,  0.75,  0.75), ReflectionType::Diffuse,          Vector3(0, 0, 0)),
    Sphere(1e5,     Vector3(50.0,   -1e5 + 81.6,          81.6), Vector3(0.75,  0.75,  0.75), ReflectionType::Diffuse,          Vector3(0, 0, 0)),
    Sphere(16.5,    Vector3(27.0,          16.5,          47.0), Vector3(0.75,  0.25,  0.25), ReflectionType::PerfectSpecular,  Vector3(0, 0, 0)),
    Sphere(16.5,    Vector3(73.0,          16.5,          78.0), Vector3(0.99,  0.99,  0.99), ReflectionType::Refraction,       Vector3(0, 0, 0)),
    Sphere(5.0,     Vector3(50.0,          81.6,          81.6), Vector3(),                   ReflectionType::Diffuse,          Vector3(12, 12, 12))
};
#endif // PHOTON_DIFFUSE_ONLY
#if PHOTON_DIFFUSE_ONLY
const int      g_lightId = static_cast<int>(g_spheres.size()) - 1;

PointLight g_point_light(Vector3(1000.0, 1000.0, 1000.0), Vector3(50.0, 81.6, 81.6));

UniformSampler g_sampler;

#else // PHOTON_DIFFUSE_ONLY
const int      g_lightId        = 8;
#endif // PHOTON_DIFFUSE_ONLY
const double   g_gather_radius  = 10.0;
const int      g_gather_count   = 100;
const int      g_final_gathering_depth = 4;
static constexpr double RAY_EPS = 1.0e-5f;
const int g_samples = 256;

enum class TransportDirection { FROM_LIGHT, FROM_CAMERA };
static float cosTerm(const Vector3& wo, const Vector3& wi,
    const Vector3& shadingNormal, const Vector3& geometricNormal,
    const TransportDirection& transport_dir) {
    const float wi_ns = dot(wi, shadingNormal);
    const float wi_ng = dot(wi, geometricNormal);
    const float wo_ns = dot(wo, shadingNormal);
    const float wo_ng = dot(wo, geometricNormal);

    // prevent light leaks
    if (wi_ng * wi_ns <= 0 || wo_ng * wo_ns <= 0) {
        throw std::exception();
    }

    if (transport_dir == TransportDirection::FROM_CAMERA) {
        return std::abs(wi_ns);
    } else if (transport_dir == TransportDirection::FROM_LIGHT) {
        return std::abs(wo_ns) * std::abs(wi_ng) / std::abs(wo_ng);
    } else {
        throw std::exception();
    }
}

static float cosTheta(const Vector3& v) { return v.z; }

// Lambertian diffuse
Vector3 Evaluate(const Vector3& wo, const Vector3& wi, const Vector3& colour)
{
    // when wo, wi is under the surface, return 0
    const float cosThetaO = cosTheta(wo);
    const float cosThetaI = cosTheta(wi);
    if (cosThetaO < 0 || cosThetaI < 0)
        return Vector3(0.0, 0.0, 0.0);

    return colour / PI;
};

// Lambertian diffuse
Vector3 EvaluateBxDF(const Vector3& wo, const Vector3& wi,
    const Vector3& normal, const Vector3& dpdu,
    const Vector3& dpdv, const Sphere* hitPrimitive)
{
    // world to local transform
    const Vector3 wo_l = worldToLocal(wo, dpdu, dpdv, normal);
    const Vector3 wi_l = worldToLocal(wi, dpdu, dpdv, normal);

    return Evaluate(wo_l, wi_l, hitPrimitive->color);
};

// Lambertian diffuse
Vector3 SampleDirection(const Vector3& wo, Vector3& wi, const Vector3& normal,
    Sampler& sampler, float& pdf,
    const Sphere* hitPrimitive)
{
    wi = CosineSampleHemisphere(sampler.getNext2D(), pdf);
    return Evaluate(wo, wi, hitPrimitive->color);
}

Vector3 Sample(const Vector3& wo, const Vector3& normal, Sampler& sampler,
    Vector3& wi, float& pdf, const Vector3& dpdu,
    const Vector3& dpdv, const Sphere* hitPrimitive)
{
    // world to local transform
    const Vector3 wo_l = worldToLocal(wo, dpdu, dpdv, normal);

    // sample direction in tangent space
    Vector3 wi_l;

    Vector3 f = SampleDirection(wo_l, wi_l, normal, sampler, pdf, hitPrimitive);

    // local to world transform
    wi = localToWorld(wi_l, dpdu, dpdv, normal);

    return f;
}

//-------------------------------------------------------------------------------------------------
//      シーンとの交差判定を行います.
//-------------------------------------------------------------------------------------------------
inline bool intersect_scene(const Ray& ray, double* t, int* id)
{
#if PHOTON_DIFFUSE_ONLY
    auto n = g_spheres.size() - 1;
#else // PHOTON_DIFFUSE_ONLY
    auto n = static_cast<int>(sizeof(g_spheres) / sizeof(g_spheres[0]));
#endif // PHOTON_DIFFUSE_ONLY

    *t  = D_MAX;
    *id = -1;

    for (auto i = 0; i < n; ++i)
    {
        auto d = g_spheres[i].intersect(ray);
        if (d > D_HIT_MIN && d < *t)
        {
            *t  = d;
            *id = i;
        }
    }

    return (*t < D_HIT_MAX);
}

//-------------------------------------------------------------------------------------------------
bool intersect_scene_no_light(const Ray& ray, double t_max, int* id)
{
    auto n = g_spheres.size() - 1;

    *id = -1;

    double t = D_MAX;

    for (auto i = 0; i < n; ++i)
    {
        auto d = g_spheres[i].intersect(ray);
        if (d > D_HIT_MIN && d < t)
        {
            t = d;
            *id = i;
        }
    }

    return (t < t_max);
}

//-------------------------------------------------------------------------------------------------
//      フォトンを生成します.
//-------------------------------------------------------------------------------------------------
void generate_photon(Ray* ray, Vector3* flux, int count, Random* random)
{
    const auto& light = g_spheres[g_lightId];
    const auto pos = light.pos + (light.radius + D_HIT_MIN) *
                     UniformSampleSphere(g_sampler.getNext2D());
    const auto light_pos_pdf = UniformSpherePdf() * (1.0 / (light.radius*light.radius));

    const auto nrm = normalize(pos - light.pos);
    Vector3 u, v;
    orthonormal_basis(nrm, u, v);

    float light_dir_pdf = 0.0f;
    const auto sample = CosineSampleHemisphere(g_sampler.getNext2D(), light_dir_pdf);
    const auto dir = normalize(u * sample.x + v * sample.y + nrm * sample.z);

    ray->pos = pos;
    ray->dir = dir;

    *flux = light.emission / light_dir_pdf / light_pos_pdf / count *
            std::abs(dot(dir, nrm));
}

//-------------------------------------------------------------------------------------------------
//      フォトンを追跡します.
//-------------------------------------------------------------------------------------------------
void photon_trace(const Ray& emit_ray, const Vector3& emit_flux, photon_map* photon_map, Random* random)
{
    Ray ray(emit_ray.pos, emit_ray.dir);
    Vector3 flux = emit_flux;

    while (true)
    {
        double t;
        int   id;

        // ゼロなら追ってもしょうがないので打ち切り.
        if (fabs(flux.x) < DBL_EPSILON
         && fabs(flux.y) < DBL_EPSILON
         && fabs(flux.z) < DBL_EPSILON)
        { break; }

        // シーンとの交差判定.
        if (!intersect_scene(ray, &t, &id))
        { break; }

        // 交差物体.
        const auto& obj = g_spheres[id];

        // 交差位置.
        const auto hit_pos = ray.pos + ray.dir * t;

        // 法線ベクトル.
        const auto normal  = normalize(hit_pos - obj.pos);

        // 物体からのレイの入出を考慮した法線ベクトル.
        const auto orienting_normal = (dot(normal, ray.dir) < 0.0) ? normal : -normal;

        switch (obj.type)
        {
        case ReflectionType::Diffuse:
            {
                photon photon;
                photon.pos  = hit_pos;
                photon.dir  = ray.dir;
                photon.flux = flux;

                // photon map に格納.
                photon_map->store(photon);

                // 青本に従って色の平均値を確率として用いる.
                auto p = (obj.color.x + obj.color.y + obj.color.z) / 3.0;

                // ロシアンルーレット.
                if (p < random->get_as_double())
                {
                    // 反射ならレイを飛ばす.

                    // 基底ベクトル.
                    Vector3 dpdu, dpdv;
                    orthonormal_basis(orienting_normal, dpdu, dpdv);

                    // Sample direction by BxDF
                    // There would be TransportDirection::FROM_LIGHT here too if a different
                    // material was being used. E.g., refraction behaves differently.
                    Vector3 dir;
                    float pdf_dir;
                    Vector3 f = Sample(-ray.dir, orienting_normal, g_sampler, dir, pdf_dir, dpdu, dpdv, &obj);

                    // update throughput and ray
                    flux *= f * cosTerm(-ray.dir, dir, orienting_normal,
                                        orienting_normal,
                                        TransportDirection::FROM_LIGHT);
                    flux /= pdf_dir / p;

                    ray = Ray(hit_pos, dir);
                }
                else
                {
                    // 吸収したら追跡終了.
                    return;
                }
            }
            break;

        case ReflectionType::PerfectSpecular:
            {
                ray = Ray(hit_pos, reflect(ray.dir, normal));
                flux *= obj.color;
            }
            break;

        case ReflectionType::Refraction:
            {
                Ray reflect_ray = Ray(hit_pos, reflect(ray.dir, normal));
                auto into = dot(normal, orienting_normal) > 0.0;

                const auto nc = 1.0;
                const auto nt = 1.5;
                const auto nnt = (into) ? (nc / nt) : (nt / nc);
                const auto ddn = dot(ray.dir, orienting_normal);
                const auto cos2t = 1.0 - nnt * nnt * (1.0 - ddn * ddn);

                if (cos2t < DBL_EPSILON)
                {
                    ray = reflect_ray;
                    flux *= obj.color;
                    break;
                }

                auto dir = normalize(ray.dir * nnt - normal * ((into) ? 1.0 : -1.0) * (ddn * nnt + sqrt(cos2t)));

                const auto a = nt - nc;
                const auto b = nt + nc;
                const auto R0 = (a * a) / (b * b);
                const auto c = 1.0 - ((into) ? -ddn : dot(dir, normal));
                const auto Re = R0 + (1.0 - R0) * pow(c, 5.0);
                //const auto Tr = 1.0 - Re;
                const auto prob = 0.25 + 0.5 * Re;

                if (random->get_as_double() < prob)
                {
                    ray = reflect_ray;
                }
                else
                {
                    ray = Ray(hit_pos, dir);
                }

                flux *= obj.color;
            }
            break;
        }
    }
}

Vector3 compute_direct_illumination(const Vector3& wo,
    const Vector3& hit_pos, const Vector3& normal, Sampler& sampler,
    const Light& light, const Vector3& dpdu, const Vector3& dpdv, const Sphere& obj)
{
    Vector3 Ld;

    // 1.0f because 1 light in the scene
    float pdf_choose_light = 1.0f;

    float pdf_pos_light;
    const SurfaceInfo light_surf = light.samplePoint(sampler, pdf_pos_light);

    // convert positional pdf to directional pdf
    const Vector3 wi = normalize(light_surf.position - hit_pos);
    const float r = length(light_surf.position - hit_pos);
    // TODO: Understand this
    const float pdf_dir =
        pdf_pos_light * r * r / std::abs(dot(-wi, light_surf.shadingNormal));

    // create shadow ray
    Ray ray_shadow(hit_pos, wi);
    // ray_shadow.tmax = r - RAY_EPS;

    // trace ray to the light
    double t = r - RAY_EPS;
    int   id;
    // if another object isn't hit
    if (!intersect_scene_no_light(ray_shadow, t, &id))
    {
        // light_surf and -wi unused
        const Vector3 Le = light.Le(light_surf, -wi);
        const Vector3 f = EvaluateBxDF(wo, wi, normal, dpdu, dpdv, &obj);
        const double cos = std::abs(dot(wi, normal));
        Ld = f * cos * Le / (pdf_choose_light * pdf_dir);
    }

    return Ld;
};

Vector3 compute_radiance_with_photon_map(const Vector3& hit_pos, const Vector3& normal,
                                         const Sphere& obj, photon_map* photon_map, double p)
{
    photon_query query;
    query.pos = hit_pos;
    query.normal = normal;
    query.count = g_gather_count;
    query.max_dist2 = g_gather_radius * g_gather_radius;

    nearest_photon result(g_gather_count);
    photon_map->search(query, result);

    //Vector3 accumulated_flux(0, 0, 0);
    double max_dist2 = -1;

    for (size_t i = 0; i < result.size(); ++i)
    {
        max_dist2 = max(max_dist2, result[i].dist2);
    }

    // 円錐フィルタ.
    {
        Vector3 accumulated_flux(0, 0, 0);

        const auto max_dist = sqrt(max_dist2);
        const auto k = 1.1;

        for (size_t i = 0; i < result.size(); ++i)
        {
            const auto w = 1.0 - (sqrt(result[i].dist2) / (k * max_dist));

            auto flux = result[i].point->flux;
            flux = flux != flux ? Vector3(0, 0, 0) : flux;

            const auto v = (obj.color * flux) / D_PI;
            accumulated_flux += w * v;
            break_if_nan(accumulated_flux);
            //printf_s("nan\n");
        }
        accumulated_flux /= (1.0 - 2.0 / (3.0 * k));
        break_if_nan(accumulated_flux);

        if (max_dist2 > 0)
        {
            return obj.emission + accumulated_flux / (D_PI * max_dist2) / p;
        }
    }

    return Vector3(0.0, 0.0, 0.0);
}

Vector3 compute_indirect_illumination_recursive(const Vector3& hit_pos, const Vector3& normal,
    const Sphere& obj, photon_map* photon_map, double p, const Vector3& wo,
    Sampler& sampler, const Vector3& dpdu, const Vector3& dpdv, int depth)
{
    if (depth >= g_max_depth)
    {
        return Vector3(0.0, 0.0, 0.0);
    }

    Vector3 Li;

    // sample direction by BxDF
    Vector3 dir;
    float pdf_dir;
    const Vector3 f = Sample(wo, normal, sampler, dir, pdf_dir, dpdu, dpdv, &obj);
    const float cos = std::abs(dot(normal, dir));

    // trace final gathering ray
    Ray ray_fg(hit_pos, dir);
    double t;
    int id;
    if (intersect_scene(ray_fg, &t, &id))
    {
        auto& obj2 = g_spheres[id];

        // 交差位置.
        const auto hit_pos2 = ray_fg.pos + ray_fg.dir * t;

        // 法線ベクトル.
        const auto normal2 = normalize(hit_pos - obj.pos);

        // 物体からのレイの入出を考慮した法線ベクトル.
        const auto orienting_normal = (dot(normal2, ray_fg.dir) < 0.0) ? normal2 : -normal2;

        // if bxdf type is diffuse
        Li += f * cos *
            compute_radiance_with_photon_map(hit_pos2, orienting_normal, obj2, photon_map, p) /
            pdf_dir;
        // ... if bxdf type is specular ...
        // recursively call this function
    }

    return Li;
}

Vector3 compute_indirect_illumination(const Vector3& hit_pos, const Vector3& normal,
    const Sphere& obj, photon_map* photon_map, double p, const Vector3& wo,
    Sampler& sampler, const Vector3& dpdu, const Vector3& dpdv)
{
    return compute_indirect_illumination_recursive(hit_pos, normal, obj, photon_map, p, wo, sampler, dpdu, dpdv, 0);
}

//-------------------------------------------------------------------------------------------------
//      放射輝度を求めます.
//-------------------------------------------------------------------------------------------------
Vector3 radiance(const Ray& ray, int depth, Random* random, photon_map* photon_map)
{
    double t;
    int   id;

    // シーンとの交差判定.
    if (!intersect_scene(ray, &t, &id))
    { return g_back_ground; }

    const auto& obj = g_spheres[id];
    const auto hit_pos = ray.pos + ray.dir * t;
    const auto normal  = normalize(hit_pos - obj.pos);
    const auto orienting_normal = (dot(normal, ray.dir) < 0.0) ? normal : -normal;

    Vector3 dpdu;
    Vector3 dpdv;
    orthonormal_basis(orienting_normal, dpdu, dpdv);

    auto p = max(obj.color.x, max(obj.color.y, obj.color.z));

    // 打ち切り深度に達したら終わり.
    if(depth > g_max_depth)
    {
        if (random->get_as_double() >= p)
        { return obj.emission; }
    }
    else
    {
        p = 1.0;
    }

    switch (obj.type)
    {
    case ReflectionType::Diffuse:
        if (false)
        // Compute radiance with photon map
        {
            return compute_radiance_with_photon_map(hit_pos, orienting_normal, obj, photon_map, p);
        }
        // Compute different illuminations
        else
        {
            // Compute direct illumination by explicit light sampling
            const Vector3 Ld = compute_direct_illumination(-ray.dir, hit_pos, orienting_normal,
                                                           g_sampler, g_point_light, dpdu, dpdv, obj);

            // compute indirect illumination with final gathering
            const Vector3 Li =
                compute_indirect_illumination(hit_pos, orienting_normal, obj, photon_map,
                    p, -ray.dir, g_sampler, dpdu, dpdv);

            return obj.emission + obj.color * (Ld + Li);
        }
        break;
#if !PHOTON_DIFFUSE_ONLY
    case ReflectionType::PerfectSpecular:
        {
            return obj.emission + obj.color * radiance(Ray(hit_pos, reflect(ray.dir, normal)), depth + 1, random, photon_map) / p;
        }
        break;

    case ReflectionType::Refraction:
        {
            Ray reflect_ray = Ray(hit_pos, reflect(ray.dir, normal));
            auto into = dot(normal, orienting_normal) > 0.0;

            const auto nc = 1.0;
            const auto nt = 1.5;
            const auto nnt = (into) ? (nc / nt) : (nt / nc);
            const auto ddn = dot(ray.dir, orienting_normal);
            const auto cos2t = 1.0 - nnt * nnt * (1.0 - ddn * ddn);

            if (cos2t < DBL_EPSILON)
            {
                return obj.emission + obj.color * radiance(reflect_ray, depth + 1, random, photon_map) / p;
            }

            auto dir = normalize(ray.dir * nnt - normal * ((into) ? 1.0 : -1.0) * (ddn * nnt + sqrt(cos2t)));

            const auto a = nt - nc;
            const auto b = nt + nc;
            const auto R0 = (a * a) / (b * b);
            const auto c = 1.0 - ((into) ? -ddn : dot(dir, normal));
            const auto Re = R0 + (1.0 - R0) * pow(c, 5.0);
            const auto Tr = 1.0 - Re;
            const auto prob = 0.25 + 0.5 * Re;

            Ray refract_ray(hit_pos, dir);

            if (depth <= 3)
            {
                return obj.emission + obj.color * (
                      radiance(reflect_ray, depth + 1, random, photon_map) * Re
                    + radiance(refract_ray, depth + 1, random, photon_map) * Tr) / p;
            }

            if (random->get_as_double() < prob)
            {
                return obj.emission + obj.color * radiance(reflect_ray, depth + 1, random, photon_map) * Re / prob / p;
            }
            else
            {
                return obj.emission + obj.color * radiance(refract_ray, depth + 1, random, photon_map) * Tr / (1.0 - prob) / p;
            }
        }
        break;
#endif // !PHOTON_DIFFUSE_ONLY
    }

    return Vector3(0, 0, 0);
}

//-------------------------------------------------------------------------------------------------
//      BMPファイルに保存します.
//-------------------------------------------------------------------------------------------------
void save_to_bmp(const char* filename, int width, int height, const double* pixels)
{
    std::vector<uint8_t> images;
    images.resize(width * height * 3);

    const double inv_gamma = 1.0 / 2.2;

    for(auto i=0; i<width * height * 3; i+=3)
    {
        auto r = pow(pixels[i + 0], inv_gamma);
        auto g = pow(pixels[i + 1], inv_gamma);
        auto b = pow(pixels[i + 2], inv_gamma);

        r = saturate(r);
        g = saturate(g);
        b = saturate(b);

        images[i + 0] = static_cast<uint8_t>( r * 255.0 + 0.5 );
        images[i + 1] = static_cast<uint8_t>( g * 255.0 + 0.5 );
        images[i + 2] = static_cast<uint8_t>( b * 255.0 + 0.5 );
    }

    stbi_write_bmp(filename, width, height, 3, images.data());
}

} // namespace


//-------------------------------------------------------------------------------------------------
//      メインエントリーポイントです.
//-------------------------------------------------------------------------------------------------
int main()
{
    // レンダーターゲットのサイズ.
    int width   = 512;
    int height  = 512;
    int photons = 100000;

    // カメラ用意.
    Camera camera(
        Vector3(50.0, 52.0, 295.6),
        normalize(Vector3(0.0, -0.042612, -1.0)),
        Vector3(0.0, 1.0, 0.0),
        0.5135,
        double(width) / double(height),
        130.0
    );

    // レンダーターゲット生成.
    std::vector<Vector3> image;
    image.resize(width * height);

    Random random(123456);
    photon_map photon_map;

    // レンダーターゲットをクリア.
    for (size_t i = 0; i < image.size(); ++i)
    { image[i] = g_back_ground; }

    printf_s("generate photon phase ...");

    for(auto i=0; i<photons; ++i)
    {
        Ray     ray;
        Vector3 flux;

        // フォトンを生成.
        generate_photon(&ray, &flux, photons, &random);

        // フォトン追跡.
        photon_trace(ray, flux, &photon_map, &random);
    }

    printf_s("done.\n");
    printf_s("build photon map phase ...");

    // kd-tree構築.
    photon_map.build();

    printf_s("done.\n");

#if INDEX_LOOP
    int start_x = 106;
    int end_x = start_x + 5;
    int start_y = 25;
    int end_y = start_y + 5;
#else // INDEX_LOOP
    int start_x = 0;
    int end_x = width;
    int start_y = 0;
    int end_y = height;
#endif // INDEX_LOOP

    // 放射輝度推定.
    {
#pragma omp parallel for schedule(dynamic)
        for (auto y = start_y; y < end_y; ++y)
        {
            printf_s("radiance estimate %.2lf%%\r", double(y) / double(height) * 100.0);
            for (auto x = start_x; x < end_x; ++x)
            {
                UniformSampler sampler(x + width * y);

                auto idx = y * width + x;

                auto offset_divisor = 10.0;
                auto x_offset = sampler.getNext1D() / offset_divisor;
                auto y_offset = sampler.getNext1D() / offset_divisor;
                auto fx = (double(x) + x_offset) / double(width) - 0.5;
                auto fy = (double(y) + y_offset) / double(height) - 0.5;

                for (int k = 0; k < g_samples; ++k)
                {
                    // Let's レイトレ！
                    image[idx] += radiance(camera.emit(fx, fy), 0, &random, &photon_map);
                }
            }
        }
    }

    divide(image, g_samples);

    // レンダーターゲットの内容をファイルに保存.
    save_to_bmp("image.bmp", width, height, &image.data()->x);

    // レンダーターゲットクリア.
    image.clear();

    return 0;
}