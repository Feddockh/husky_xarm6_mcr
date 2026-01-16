#pragma once

#include <octomap/octomap.h>
#include <vector>

namespace husky_xarm6_mcr_nbv_planner
{
    struct Cluster
    {
        int label;
        octomap::point3d center;
        int size;
        std::vector<octomap::point3d> points;
    };
}
