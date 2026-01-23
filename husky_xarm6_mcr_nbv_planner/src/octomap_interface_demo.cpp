#include <rclcpp/rclcpp.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <algorithm>
#include "husky_xarm6_mcr_nbv_planner/octomap_interface.hpp"
#include "husky_xarm6_mcr_nbv_planner/nbv_visualizer.hpp"

using namespace husky_xarm6_mcr_nbv_planner;

/**
 * @brief Demo node that continuously processes octomap updates and visualizes frontiers/clusters
 */
class OctomapInterfaceDemoNode : public rclcpp::Node
{
public:
    OctomapInterfaceDemoNode() : Node("octomap_interface_demo")
    {
        // Declare parameters
        this->declare_parameter("octomap_topic", "/planning_scene_world");
        this->declare_parameter("visualization_topic", "nbv_markers");
        this->declare_parameter("map_frame", "map");
        this->declare_parameter("min_unknown_neighbors", 1);
        this->declare_parameter("n_clusters", -1);  // -1 = auto-select
        this->declare_parameter("use_bbox", false);
        this->declare_parameter("update_rate_hz", 2.0);
        this->declare_parameter("visualization_rate_hz", 2.0);

        // Get parameters
        octomap_topic_ = this->get_parameter("octomap_topic").as_string();
        visualization_topic_ = this->get_parameter("visualization_topic").as_string();
        map_frame_ = this->get_parameter("map_frame").as_string();
        min_unknown_neighbors_ = this->get_parameter("min_unknown_neighbors").as_int();
        n_clusters_ = this->get_parameter("n_clusters").as_int();
        use_bbox_ = this->get_parameter("use_bbox").as_bool();
        update_rate_hz_ = this->get_parameter("update_rate_hz").as_double();
        visualization_rate_hz_ = this->get_parameter("visualization_rate_hz").as_double();

        // Log parameters
        RCLCPP_INFO(this->get_logger(), "\n=== Octomap Interface Demo Parameters ===");
        RCLCPP_INFO(this->get_logger(), "  octomap_topic: %s", octomap_topic_.c_str());
        RCLCPP_INFO(this->get_logger(), "  visualization_topic: %s", visualization_topic_.c_str());
        RCLCPP_INFO(this->get_logger(), "  map_frame: %s", map_frame_.c_str());
        RCLCPP_INFO(this->get_logger(), "  min_unknown_neighbors: %d", min_unknown_neighbors_);
        RCLCPP_INFO(this->get_logger(), "  n_clusters: %d (negative = auto)", n_clusters_);
        RCLCPP_INFO(this->get_logger(), "  use_bbox: %s", use_bbox_ ? "true" : "false");
        RCLCPP_INFO(this->get_logger(), "  update_rate: %.1f Hz", update_rate_hz_);
        RCLCPP_INFO(this->get_logger(), "  visualization_rate: %.1f Hz", visualization_rate_hz_);

        // Setup frontier color
        frontier_color_.r = 0.0f;
        frontier_color_.g = 1.0f;
        frontier_color_.b = 0.0f;
        frontier_color_.a = 0.7f;

        RCLCPP_INFO(this->get_logger(), "Demo node constructor complete. Call initialize() to start.");
    }

    void initialize()
    {
        // Initialize octomap interface (requires shared_from_this)
        RCLCPP_INFO(this->get_logger(), "\n=== Initializing ===");
        octomap_interface_ = std::make_shared<OctoMapInterface>(
            shared_from_this(), octomap_topic_, true);

        // Initialize visualizer
        visualizer_ = std::make_shared<NBVVisualizer>(
            shared_from_this(), map_frame_, visualization_topic_);

        // Create update timer
        auto update_period = std::chrono::duration<double>(1.0 / update_rate_hz_);
        update_timer_ = this->create_wall_timer(
            std::chrono::duration_cast<std::chrono::milliseconds>(update_period),
            std::bind(&OctomapInterfaceDemoNode::updateCallback, this));

        // Create visualization timer
        auto viz_period = std::chrono::duration<double>(1.0 / visualization_rate_hz_);
        viz_timer_ = this->create_wall_timer(
            std::chrono::duration_cast<std::chrono::milliseconds>(viz_period),
            std::bind(&OctomapInterfaceDemoNode::visualizationCallback, this));

        RCLCPP_INFO(this->get_logger(), "Demo node initialized. Waiting for octomap...");
    }

private:
    void updateCallback()
    {
        // Get latest octomap snapshot
        auto tree = octomap_interface_->getTreeSnapshot();
        if (!tree)
        {
            if (!octomap_received_)
            {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                     "Waiting for octomap on topic: %s", octomap_topic_.c_str());
            }
            return;
        }

        // First octomap received
        if (!octomap_received_)
        {
            octomap_received_ = true;
            RCLCPP_INFO(this->get_logger(), "\n=== First Octomap Received ===");
            logOctomapStats(tree);
        }

        // Find frontiers
        auto start = std::chrono::high_resolution_clock::now();
        std::vector<octomap::point3d> frontiers;
        if (use_bbox_)
            frontiers = octomap_interface_->findFrontiers(min_unknown_neighbors_, true);
        else
            frontiers = octomap_interface_->findFrontiers(min_unknown_neighbors_, false);
        
        auto end = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);

        if (frontiers.empty())
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 10000,
                                 "No frontiers found. Try lowering min_unknown_neighbors (current: %d)",
                                 min_unknown_neighbors_);
            // Clear visualization if no frontiers
            latest_frontiers_.clear();
            latest_clusters_.clear();
            return;
        }

        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                             "Found %zu frontiers in %ld ms", frontiers.size(), duration.count());

        // Determine number of clusters
        int n_clusters = n_clusters_;
        if (n_clusters <= 0)
        {
            n_clusters = std::max(1, static_cast<int>(frontiers.size()) / 40);
        }

        // Cluster frontiers
        start = std::chrono::high_resolution_clock::now();
        auto clusters = octomap_interface_->kmeansCluster(frontiers, n_clusters);
        end = std::chrono::high_resolution_clock::now();
        duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);

        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                             "Clustered into %zu groups in %ld ms", clusters.size(), duration.count());

        // Store latest results
        latest_frontiers_ = frontiers;
        latest_clusters_ = clusters;
        voxel_size_ = tree->getResolution();
        has_data_ = true;

        // Log cluster details occasionally
        static int update_count = 0;
        if (++update_count % 10 == 0)
        {
            RCLCPP_INFO(this->get_logger(), "\nCluster details:");
            for (size_t i = 0; i < clusters.size() && i < 5; ++i)
            {
                const auto &cluster = clusters[i];
                RCLCPP_INFO(this->get_logger(), "  Cluster %d: %d points, center [%.3f, %.3f, %.3f]",
                            cluster.label, cluster.size,
                            cluster.center.x(), cluster.center.y(), cluster.center.z());
            }
            if (clusters.size() > 5)
            {
                RCLCPP_INFO(this->get_logger(), "  ... and %zu more clusters", clusters.size() - 5);
            }
        }
    }

    void visualizationCallback()
    {
        if (!has_data_)
        {
            return;
        }

        // Republish frontiers and clusters
        visualizer_->publishFrontiers(latest_frontiers_, voxel_size_, frontier_color_);
        visualizer_->publishClusteredFrontiers(latest_clusters_, voxel_size_, true);
    }

    void logOctomapStats(const std::shared_ptr<octomap::OcTree> &tree)
    {
        RCLCPP_INFO(this->get_logger(), "  Resolution: %.4f m", tree->getResolution());
        RCLCPP_INFO(this->get_logger(), "  Total nodes: %zu", tree->size());
        RCLCPP_INFO(this->get_logger(), "  Leaf nodes: %zu", tree->getNumLeafNodes());

        size_t occupied = 0, free_space = 0;
        for (auto it = tree->begin_leafs(); it != tree->end_leafs(); ++it)
        {
            if (tree->isNodeOccupied(*it))
                occupied++;
            else
                free_space++;
        }
        RCLCPP_INFO(this->get_logger(), "  Occupied voxels: %zu", occupied);
        RCLCPP_INFO(this->get_logger(), "  Free voxels: %zu", free_space);
    }

    // Parameters
    std::string octomap_topic_;
    std::string visualization_topic_;
    std::string map_frame_;
    int min_unknown_neighbors_;
    int n_clusters_;
    bool use_bbox_;
    double update_rate_hz_;
    double visualization_rate_hz_;

    // Components
    std::shared_ptr<OctoMapInterface> octomap_interface_;
    std::shared_ptr<NBVVisualizer> visualizer_;
    rclcpp::TimerBase::SharedPtr update_timer_;
    rclcpp::TimerBase::SharedPtr viz_timer_;

    // State
    bool octomap_received_ = false;
    bool has_data_ = false;
    std::vector<octomap::point3d> latest_frontiers_;
    std::vector<Cluster> latest_clusters_;
    double voxel_size_ = 0.1;
    std_msgs::msg::ColorRGBA frontier_color_;
};

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

    auto node = std::make_shared<OctomapInterfaceDemoNode>();

    // Wait for sim clock if using sim time
    if (node->get_parameter("use_sim_time").as_bool())
    {
        waitForSimClock(node);
    }

    // Initialize after shared_ptr is created
    node->initialize();

    RCLCPP_INFO(node->get_logger(), "\n=== Demo Running ===");
    RCLCPP_INFO(node->get_logger(), "Press Ctrl+C to exit.");

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
