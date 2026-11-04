// License: Apache 2.0. See LICENSE file in root directory.
//
// Narrow wrapper around Kazhdan's screened Poisson reconstruction (third_party/PoissonRecon, header library). Only
// poisson_recon.cpp includes the PoissonRecon headers, which keeps their compile time and warnings in one place.

#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

namespace fr
{
namespace poisson
{
struct Input
{
    const float * points = nullptr;    // 3 per point
    const float * normals = nullptr;   // 3 per point, oriented outward
    const uint8_t * colors = nullptr;  // 3 per point or nullptr
    size_t count = 0;
};

struct Options
{
    int depth = 10;
    float point_weight = 4.f;
    float samples_per_node = 1.5f;
    bool parallel = true;
};

struct Output
{
    std::vector< float > positions;    // 3 per vertex
    std::vector< float > density;      // 1 per vertex (sampling density estimate)
    std::vector< float > colors;       // 3 per vertex in [0, 1] when the input had colors
    std::vector< uint32_t > indices;   // 3 per triangle, PoissonRecon's orientation
};

// Throws std::exception on failure. Serialized internally: PoissonRecon keeps global thread-pool state.
void reconstruct( const Input & input, const Options & options, Output & output );
}  // namespace poisson
}  // namespace fr
