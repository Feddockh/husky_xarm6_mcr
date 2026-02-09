#pragma once

#include <octomap/octomap.h>
#include <vector>

namespace husky_xarm6_mcr_nbv_planner
{
    struct SemanticPoint
    {
        int id;                 // Unique identifier for the semantic point (e.g., marker ID)
        int32_t class_id = -1;
        octomap::point3d position;  // 3D position in map frame
    };

    /**
     * @brief Result of matching semantic clusters to ground truth points
     */
    struct MatchResult
    {
        struct Match
        {
            SemanticPoint gt_point;
            std::vector<Cluster> clusters;
            std::vector<double> distances;
        };

        std::vector<Match> correct_matches;      // TP: GT and clusters with matching class
        std::vector<Match> class_mismatches;     // Class mismatch: GT and clusters matched but wrong class
        std::vector<SemanticPoint> unmatched_gt; // FN: GT points with no matching cluster
        std::vector<Cluster> unmatched_clusters; // FP: Clusters with no matching GT
    };
}