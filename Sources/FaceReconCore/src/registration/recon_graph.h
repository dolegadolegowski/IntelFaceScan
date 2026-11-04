// License: Apache 2.0. See LICENSE file in root directory.
//
// Definition of the opaque pose graph type (shared by the pose graph and the whole-scan registration).

#pragma once

#include "fr_registration.h"
#include "recon_math.h"

#include <vector>

struct fr_pose_graph
{
    struct Edge
    {
        uint32_t source = 0, target = 0;
        Eigen::Matrix4d measurement = Eigen::Matrix4d::Identity();   // target_from_source
        fr::Matrix6d information = fr::Matrix6d::Identity();
        bool uncertain = false;
        bool pruned = false;   // set by fr_pose_graph_optimize; pruned edges are ignored afterwards
    };
    std::vector< Eigen::Matrix4d > nodes;
    std::vector< Edge > edges;
};
