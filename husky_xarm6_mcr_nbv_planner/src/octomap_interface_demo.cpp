#include <rclcpp/rclcpp.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <algorithm>
#include "husky_xarm6_mcr_nbv_planner/octomap_interface.hpp"
#include "husky_xarm6_mcr_nbv_planner/nbv_visualizer.hpp"

using namespace husky_xarm6_mcr_nbv_planner;

void waitForSimClock(const std::shared_ptr<rclcpp::Node> &node)
{
    RCLCPP_INFO(node->get_logger(), "Waiting for /clock...");
    auto clock_sub = node->create_subscription<rosgraph_msgs::msg::Clock>(
        "/clock", 10, [](const rosgraph_msgs::msg::Clock::SharedPtr) {});

    rclcpp::Rate rate(10);
    auto start = std::chrono::steady_clock::now();
    while (rclcpp::ok())
    {
        rclcpp::spin_some(node);
        if (node->now().seconds() > 0.0)
        {
            RCLCPP_INFO(node->get_logger(), "Clock synchronized at %.2f seconds",
                        node->now().seconds());
            break;
        }
        auto elapsed = std::chrono::steady_clock::now() - start;
        if (elapsed > std::chrono::seconds(10))
        {
            RCLCPP_WARN(node->get_logger(), "Timeout waiting for /clock. Proceeding anyway...");
            break;
        }
        rate.sleep();
    }
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<rclcpp::Node>(
        "octomap_interface_demo",
        rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

    // Wait for sim clock if using sim time
    if (node->get_parameter("use_sim_time").as_bool())
    {
        waitForSimClock(node);
    }

    // Get parameters
    std::string octomap_topic = node->get_parameter("octomap_topic").as_string();
    std::string visualization_topic = node->get_parameter("visualization_topic").as_string();
    std::string map_frame = node->get_parameter("map_frame").as_string();
    int min_unknown_neighbors = node->get_parameter("min_unknown_neighbors").as_int();
    int n_clusters = node->get_parameter("n_clusters").as_int();
    bool use_bbox = node->get_parameter("use_bbox").as_bool();
    double update_rate_hz = node->get_parameter("update_rate_hz").as_double();
    std::string gt_points_file = node->get_parameter("gt_points_file").as_string();
    bool enable_evaluation = node->get_parameter("enable_evaluation").as_bool();
    double eval_threshold_radius = node->get_parameter("eval_threshold_radius").as_double();

    // Log parameters
    RCLCPP_INFO(node->get_logger(), "\n=== Octomap Interface Demo Parameters ===");
    RCLCPP_INFO(node->get_logger(), "  octomap_topic: %s", octomap_topic.c_str());
    RCLCPP_INFO(node->get_logger(), "  visualization_topic: %s", visualization_topic.c_str());
    RCLCPP_INFO(node->get_logger(), "  map_frame: %s", map_frame.c_str());
    RCLCPP_INFO(node->get_logger(), "  min_unknown_neighbors: %d", min_unknown_neighbors);
    RCLCPP_INFO(node->get_logger(), "  n_clusters: %d (negative = auto)", n_clusters);
    RCLCPP_INFO(node->get_logger(), "  use_bbox: %s", use_bbox ? "true" : "false");
    RCLCPP_INFO(node->get_logger(), "  update_rate: %.1f Hz", update_rate_hz);
    RCLCPP_INFO(node->get_logger(), "  enable_evaluation: %s", enable_evaluation ? "true" : "false");
    if (enable_evaluation)
    {
        RCLCPP_INFO(node->get_logger(), "  gt_points_file: %s", gt_points_file.c_str());
        RCLCPP_INFO(node->get_logger(), "  eval_threshold_radius: %.3f m", eval_threshold_radius);
    }

    // Initialize octomap interface
    RCLCPP_INFO(node->get_logger(), "\n=== Initializing ===");
    auto octomap_interface = std::make_shared<OctoMapInterface>(node, octomap_topic, true);

    // Load ground truth if evaluation is enabled
    if (enable_evaluation)
    {
        if (!gt_points_file.empty())
        {
            if (octomap_interface->loadGroundTruthSemantics(gt_points_file))
            {
                RCLCPP_INFO(node->get_logger(), "Ground truth loaded successfully");
            }
            else
            {
                RCLCPP_ERROR(node->get_logger(), "Failed to load ground truth file: %s", gt_points_file.c_str());
                enable_evaluation = false;
            }
        }
        else
        {
            RCLCPP_WARN(node->get_logger(), "Evaluation enabled but no ground truth file specified");
            enable_evaluation = false;
        }
    }

    // Initialize visualizer
    auto visualizer = std::make_shared<NBVVisualizer>(node, map_frame, visualization_topic);   

    // State variables for callbacks
    bool octomap_received = false;
    bool has_data = false;
    std::vector<Cluster> latest_clusters;
    std::vector<std::vector<ClassMetrics>> all_metrics;

    // Create update timer
    auto update_callback = [&]()
    {
        // Get latest octomap snapshot
        auto tree = octomap_interface->getTreeSnapshot();
        if (!tree)
        {
            if (!octomap_received)
            {
                RCLCPP_WARN_THROTTLE(node->get_logger(), *node->get_clock(), 5000,
                                     "Waiting for octomap on topic: %s", octomap_topic.c_str());
            }
            return;
        }

        // First octomap received
        if (!octomap_received)
        {
            octomap_received = true;
            RCLCPP_INFO(node->get_logger(), "\n=== First Octomap Received ===");
            RCLCPP_INFO(node->get_logger(), "  Tree type: %s", octomap_interface->getTreeTypeString().c_str());
        }

        // Run evaluation if enabled and this is a semantic tree
        if (enable_evaluation && octomap_interface->isSemanticTree())
        {
            // RCLCPP_INFO(node->get_logger(), "Running semantic octomap evaluation...");
            // octomap_interface->evaluateSemanticOctomap(eval_threshold_radius);

            // Step 1: Cluster semantic voxels by class
            latest_clusters = octomap_interface->clusterSemanticVoxels(false);

            // Step 2: Match clusters to ground truth
            auto match_result = octomap_interface->matchClustersToGroundTruth(latest_clusters, eval_threshold_radius, false);
            visualizer->publishMatchResults(match_result, eval_threshold_radius*2, 0.8f); // Visualizer uses diameter

            // Step 3: Evaluate and print metrics
            std::vector<ClassMetrics> eval_results = octomap_interface->evaluateMatchResults(match_result, false);
            all_metrics.push_back(eval_results);
            std::vector<double> x_data;
            for (size_t i = 0; i < all_metrics.size(); ++i)
                x_data.push_back(static_cast<double>(i+1));
            visualizer->plotClassMetrics(all_metrics, x_data, "viewpoint", "Class Metrics", "./nbv_metrics_plot.png");

            has_data = true;
        }

        // // Log octomap stats (updated every time)
        // static int stats_counter = 0;
        // if (++stats_counter % 5 == 0) // Log every 5 updates to avoid spam
        // {
        //     RCLCPP_INFO(node->get_logger(), "\n=== Octomap Stats ===");
        //     octomap_interface->printOctomapStats();
        // }

        // Visualize clusters if we have data and this is a semantic tree
        if (has_data && octomap_interface->isSemanticTree())
        {
            std::vector<std_msgs::msg::ColorRGBA> cluster_colors;
            for (const auto &cluster : latest_clusters)
                cluster_colors.push_back(visualizer->colorForLabel(cluster.class_id, 0.8f));
            // for (int i = 0; i < latest_clusters.size(); ++i)
            //     cluster_colors.push_back(visualizer->colorForLabel(i, 0.8f));
            visualizer->publishClusteredVoxels(latest_clusters, octomap_interface->getResolution(), cluster_colors, false, 0.8f);
        }

        // // Visualize ground truth points once after loading
        // const auto &gt_points = octomap_interface->getGroundTruthPoints();
        // if (enable_evaluation && !gt_points.empty())
        //     visualizer->publishSemanticPoints(gt_points, eval_threshold_radius*2, 0.8f, true);
    };

    auto update_period = std::chrono::duration<double>(1.0 / update_rate_hz);
    auto update_timer = node->create_wall_timer(
        std::chrono::duration_cast<std::chrono::milliseconds>(update_period),
        update_callback);

    RCLCPP_INFO(node->get_logger(), "Demo node initialized. Waiting for octomap...");
    RCLCPP_INFO(node->get_logger(), "\n=== Demo Running ===");
    RCLCPP_INFO(node->get_logger(), "Press Ctrl+C to exit.");

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
