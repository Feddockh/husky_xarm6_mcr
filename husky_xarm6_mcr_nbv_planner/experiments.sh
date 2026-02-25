#!/bin/bash
set -e

# Simulation Common
# COMMON="use_sim_time:=true use_gazebo:=true visualize:=false keep_alive:=false gt_points_file:=sim_aruco_gt_points.yaml detection_model_trt:=best_sim_seg_v2.plan"
# Lab Common
COMMON="use_sim_time:=false use_gazebo:=false visualize:=true keep_alive:=false gt_points_file:=aruco_gt_points_lab.yaml detection_model_trt:=best_lab_seg_v2.plan"
VOL_DEFAULTS="planner_type:=volumetric alpha_cost_weight:=0.1 beta_semantic_weight:=0.7 camera_max_range:=0.9 conf_thresh:=0.3 octomap_resolution:=0.04 semantic_confidence_boost:=0.3 semantic_mismatch_penalty:=0.2"
SEM_DEFAULTS="planner_type:=semantic alpha_cost_weight:=0.1 beta_semantic_weight:=0.7 camera_max_range:=0.9 conf_thresh:=0.3 octomap_resolution:=0.04 semantic_confidence_boost:=0.3 semantic_mismatch_penalty:=0.2"

LAUNCH="ros2 launch husky_xarm6_mcr_nbv_planner nbv_demo.launch.py"

# ==============================================================================
# Baseline — semantic_confidence_boost x semantic_mismatch_penalty grid
# ==============================================================================
# # Run 10 times to average out randomness
# $LAUNCH $COMMON planner_type:=baseline camera_max_range:=0.9 conf_thresh:=0.3 octomap_resolution:=0.04 semantic_confidence_boost:=0.3 semantic_mismatch_penalty:=0.2 metrics_dir:=metrics/final_runs run:=baseline n_runs:=10
$LAUNCH $COMMON planner_type:=baseline camera_max_range:=0.9 conf_thresh:=0.3 octomap_resolution:=0.04 semantic_confidence_boost:=0.3 semantic_mismatch_penalty:=0.2 metrics_dir:=metrics/video_runs run:=baseline n_runs:=1

# for boost in 0.1 0.2 0.3; do
#     for penalty in 0.1 0.2 0.3; do
#         $LAUNCH $COMMON planner_type:=baseline camera_max_range:=0.8 conf_thresh:=0.5 octomap_resolution:=0.04 semantic_confidence_boost:=${boost} semantic_mismatch_penalty:=${penalty} metrics_dir:=metrics/ablation_study/baseline/semantic_boost_penalty run:=sem_boost_${boost}_penalty_${penalty}
#     done
# done

# # Baseline — camera_max_range
# for range in 0.4 0.5 0.6 0.7 0.8 0.9 1.0 1.2 1.5; do
#     $LAUNCH $COMMON planner_type:=baseline camera_max_range:=${range} conf_thresh:=0.5 octomap_resolution:=0.04 semantic_confidence_boost:=0.1 semantic_mismatch_penalty:=0.15 metrics_dir:=metrics/ablation_study/baseline/camera_max_range run:=camera_max_range_${range}
# done

# # Baseline — octomap_resolution
# for res in 0.02 0.04 0.06 0.08 0.10; do
#     $LAUNCH $COMMON planner_type:=baseline camera_max_range:=0.8 conf_thresh:=0.5 octomap_resolution:=${res} semantic_confidence_boost:=0.1 semantic_mismatch_penalty:=0.15 metrics_dir:=metrics/ablation_study/baseline/octomap_resolution run:=octomap_resolution_${res}
# done

# ==============================================================================
# Volumetric planner
# ==============================================================================
# # Run 10 times to average out randomness
# $LAUNCH $COMMON $VOL_DEFAULTS metrics_dir:=metrics/final_runs run:=volumetric n_runs:=10
# $LAUNCH $COMMON $VOL_DEFAULTS metrics_dir:=metrics/video_runs run:=volumetric n_runs:=1

# # Volumetric — semantic_confidence_boost x semantic_mismatch_penalty grid
# for boost in 0.1 0.2 0.3; do
#     for penalty in 0.1 0.2 0.3; do
#         $LAUNCH $COMMON $VOL_DEFAULTS semantic_confidence_boost:=${boost} semantic_mismatch_penalty:=${penalty} metrics_dir:=metrics/ablation_study/volumetric/semantic_boost_penalty run:=sem_boost_${boost}_penalty_${penalty}
#     done
# done

# # Volumetric — alpha_cost_weight
# # for alpha in 0.1 0.3 0.5 0.7 0.9; do
# for alpha in 0.5 0.7 0.9; do
#     $LAUNCH $COMMON $VOL_DEFAULTS alpha_cost_weight:=${alpha} metrics_dir:=metrics/ablation_study/volumetric/alpha_cost_weight run:=alpha_${alpha}
# done

# # Volumetric — camera_max_range
# # for range in 0.4 0.5 0.6 0.7 0.8 0.9 1.0 1.2 1.5; do
# for range in 1.0 1.2 1.5; do
#     $LAUNCH $COMMON $VOL_DEFAULTS camera_max_range:=${range} metrics_dir:=metrics/ablation_study/volumetric/camera_max_range run:=camera_max_range_${range}
# done

# # Volumetric — confidence_threshold
# for thresh in 0.3 0.4 0.5 0.6 0.7 0.8 0.9; do
#     $LAUNCH $COMMON $VOL_DEFAULTS conf_thresh:=${thresh} metrics_dir:=metrics/ablation_study/volumetric/confidence_threshold run:=conf_thresh_${thresh}
# done

# # Volumetric — octomap_resolution
# for res in 0.02 0.04 0.06 0.08 0.10; do
#     $LAUNCH $COMMON $VOL_DEFAULTS octomap_resolution:=${res} metrics_dir:=metrics/ablation_study/volumetric/octomap_resolution run:=octomap_resolution_${res}
# done

# ==============================================================================
# Semantic planner
# ==============================================================================
# # Run 10 times to average out randomness
# $LAUNCH $COMMON $SEM_DEFAULTS metrics_dir:=metrics/final_runs run:=semantic n_runs:=10
# $LAUNCH $COMMON $SEM_DEFAULTS metrics_dir:=metrics/video_runs run:=semantic n_runs:=1

# # Semantic — semantic_confidence_boost x semantic_mismatch_penalty grid
# for boost in 0.1 0.2 0.3; do
#     for penalty in 0.1 0.2 0.3; do
#         $LAUNCH $COMMON $SEM_DEFAULTS semantic_confidence_boost:=${boost} semantic_mismatch_penalty:=${penalty} metrics_dir:=metrics/ablation_study/semantic/semantic_boost_penalty run:=sem_boost_${boost}_penalty_${penalty}
#     done
# done

# # Semantic — alpha_cost_weight
# for alpha in 0.1 0.3 0.5 0.7 0.9; do
#     $LAUNCH $COMMON $SEM_DEFAULTS alpha_cost_weight:=${alpha} metrics_dir:=metrics/ablation_study/semantic/alpha_cost_weight run:=alpha_${alpha}
# done

# # Semantic — beta_semantic_weight
# for beta in 0.1 0.3 0.5 0.7 0.9; do
#     $LAUNCH $COMMON $SEM_DEFAULTS beta_semantic_weight:=${beta} metrics_dir:=metrics/ablation_study/semantic/beta_semantic_weight run:=beta_${beta}
# done

# # Semantic — camera_max_range
# for range in 0.4 0.5 0.6 0.7 0.8 0.9 1.0 1.2 1.5; do
#     $LAUNCH $COMMON $SEM_DEFAULTS camera_max_range:=${range} metrics_dir:=metrics/ablation_study/semantic/camera_max_range run:=camera_max_range_${range}
# done

# # Semantic — confidence_threshold
# for thresh in 0.3 0.4 0.5 0.6 0.7 0.8 0.9; do
#     $LAUNCH $COMMON $SEM_DEFAULTS conf_thresh:=${thresh} metrics_dir:=metrics/ablation_study/semantic/confidence_threshold run:=conf_thresh_${thresh}
# done

# # Semantic — octomap_resolution
# for res in 0.02 0.04 0.06 0.08 0.10; do
#     $LAUNCH $COMMON $SEM_DEFAULTS octomap_resolution:=${res} metrics_dir:=metrics/ablation_study/semantic/octomap_resolution run:=octomap_resolution_${res}
# done

# # Semantic — max semantic certainty
# for certainty in 0.5 0.6 0.7 0.8 0.9 1.0; do
#     $LAUNCH $COMMON $SEM_DEFAULTS max_semantic_certainty:=${certainty} metrics_dir:=metrics/ablation_study/semantic/max_semantic_certainty run:=max_semantic_certainty_${certainty}
# done