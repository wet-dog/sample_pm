#ifndef SAMPLER_H
#define SAMPLER_H

#include <algorithm>
#include <cstdint>
#include <limits>
#include <memory>

#include "r3d_math.h"

constexpr float PI = 3.14159265359;

constexpr float PI_MUL_2 = 2.0f * PI;
constexpr float PI_MUL_4 = 4.0f * PI;

constexpr float PI_DIV_2 = 0.5f * PI;
constexpr float PI_DIV_4 = 0.25f * PI;

constexpr float PI_INV = 1.0f / PI;
constexpr float PI_MUL_2_INV = 1.0f / PI_MUL_2;
constexpr float PI_MUL_4_INV = 1.0f / PI_MUL_4;

constexpr float RAY_EPS = 1e-5f;

// compute cartesian coordinates from spherical coordinates
inline Vector3 sphericalToCartesian(float theta, float phi) {
    return Vector3(std::cos(phi) * std::sin(theta), std::cos(theta),
        std::sin(phi) * std::sin(theta));
}

// *Really* minimal PCG32 code / (c) 2014 M.E. O'Neill / pcg-random.org
// Licensed under Apache License 2.0 (NO WARRANTY, etc. see website)
typedef struct {
    uint64_t state;
    uint64_t inc;
} pcg32_random_t;

inline uint32_t pcg32_random_r(pcg32_random_t* rng) {
    uint64_t oldstate = rng->state;
    // Advance internal state
    rng->state = oldstate * 6364136223846793005ULL + (rng->inc | 1);
    // Calculate output function (XSH RR), uses old state for max ILP
    uint32_t xorshifted = ((oldstate >> 18u) ^ oldstate) >> 27u;
    uint32_t rot = oldstate >> 59u;
    #pragma warning(disable : 4146)
    return (xorshifted >> rot) | (xorshifted << ((-rot) & 31));
    #pragma warning(default : 4146)
}

// random number generator
class RNG {
private:
    pcg32_random_t state;

public:
    RNG() {
        state.state = 1;
        state.inc = 1;
    }
    RNG(uint64_t seed) {
        state.state = seed;
        state.inc = 1;
    }

    uint64_t getSeed() const { return state.state; }
    void setSeed(uint64_t seed) { state.state = seed; }

    float getNext() {
        constexpr float divider = 1.0f / std::numeric_limits<uint32_t>::max();
        return pcg32_random_r(&state) * divider;
    }
};

// sampler interface
class Sampler {
protected:
    RNG rng;

public:
    Sampler() {}

    Sampler(uint64_t seed) : rng(seed) {}

    uint64_t getSeed() const { return rng.getSeed(); }
    void setSeed(uint64_t seed) { rng.setSeed(seed); }

    virtual float getNext1D() = 0;
    virtual Vector2 getNext2D() = 0;
};

// uniform distribution sampler
class UniformSampler : public Sampler {
public:
    UniformSampler() : Sampler() {}
    UniformSampler(uint64_t seed) : Sampler(seed) {}

    float getNext1D() override { return rng.getNext(); }
    Vector2 getNext2D() override { return Vector2(rng.getNext(), rng.getNext()); }
};

// sample direction in the hemisphere
// its pdf is propotional to cosine
inline Vector3 sampleCosineHemisphere(const Vector2& uv, float& pdf) {
    const float theta =
        0.5 * std::acos(std::clamp(1.0 - 2.0 * uv[0], -1.0, 1.0));
    const float phi = PI_MUL_2 * uv[1];
    const float cosTheta = std::cos(theta);
    pdf = PI_INV * cosTheta;
    return sphericalToCartesian(theta, phi);
}

Vector3 UniformSampleSphere(const Vector2& uv) {
    float z = 1 - 2 * uv[0];
    float r = std::sqrt(std::max(0.0f, 1.0f - z * z));
    float phi = 2 * PI * uv[1];
    return Vector3(r * std::cos(phi), r * std::sin(phi), z);
}

float UniformSpherePdf() {
    return PI_MUL_4_INV;
}
#endif // SAMPLER_H
