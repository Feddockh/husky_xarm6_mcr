#include <rclcpp/rclcpp.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <chrono>
#include "husky_xarm6_mcr_nbv_planner/moveit_interface.hpp"
#include "husky_xarm6_mcr_nbv_planner/manipulation_workspace.hpp"

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
        "moveit_interface_node",
        rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

    // Wait for sim time if needed
    if (node->get_parameter("use_sim_time").as_bool())
    {
        waitForSimClock(node);
    }

    // Initialize MoveIt interface
    std::string manipulator_group_name = node->get_parameter("manipulator_group_name").as_string();
    auto interface = std::make_shared<MoveItInterface>(node, manipulator_group_name);

    // Get workspace parameters
    bool learn_workspace = node->get_parameter("learn_workspace").as_bool();
    std::string workspace_file = node->get_parameter("manipulation_workspace_file").as_string();
    int num_samples = node->get_parameter("num_samples").as_int();
    bool visualize_learning = node->get_parameter("visualize_learning").as_bool();
    std::string visualization_topic = node->get_parameter("visualization_topic").as_string();

    // Initialize visualizer if visualization is requested
    std::shared_ptr<NBVVisualizer> visualizer;
    if (visualize_learning)
    {
        std::string map_frame = "map"; // Could be a parameter
        visualizer = std::make_shared<NBVVisualizer>(node, map_frame, visualization_topic);
        RCLCPP_INFO(node->get_logger(), "Visualizer initialized on topic: %s", visualization_topic.c_str());
    }

    // Initialize manipulation workspace
    RCLCPP_INFO(node->get_logger(), "Initializing Manipulation Workspace...");
    auto workspace = std::make_shared<ManipulationWorkspace>(interface, visualizer);

    // Learn or load workspace based on parameter
    if (learn_workspace)
    {
        RCLCPP_INFO(node->get_logger(), "Learning manipulation workspace with %d samples...", num_samples);
        if (visualize_learning)
        {
            RCLCPP_INFO(node->get_logger(), "Visualization enabled - this may slow down learning");
        }
        
        if (workspace->learnWorkspace(num_samples, visualize_learning))
        {
            RCLCPP_INFO(node->get_logger(), "Workspace learning complete: %zu reachable voxels",
                       workspace->getNumReachableVoxels());
            
            // Save the learned workspace
            if (workspace->saveWorkspaceToFile(workspace_file))
            {
                RCLCPP_INFO(node->get_logger(), "Workspace saved to: %s", workspace_file.c_str());
            }
            else
            {
                RCLCPP_ERROR(node->get_logger(), "Failed to save workspace to: %s", workspace_file.c_str());
            }
        }
        else
        {
            RCLCPP_ERROR(node->get_logger(), "Failed to learn workspace");
            rclcpp::shutdown();
            return 1;
        }
    }
    else
    {
        RCLCPP_INFO(node->get_logger(), "Loading manipulation workspace from: %s", workspace_file.c_str());
        
        if (workspace->loadWorkspaceFromFile(workspace_file))
        {
            RCLCPP_INFO(node->get_logger(), "Workspace loaded successfully: %zu reachable voxels",
                       workspace->getNumReachableVoxels());
        }
        else
        {
            RCLCPP_ERROR(node->get_logger(), "Failed to load workspace from: %s", workspace_file.c_str());
            RCLCPP_INFO(node->get_logger(), "Hint: Set learn_workspace:=true to learn a new workspace");
            rclcpp::shutdown();
            return 1;
        }
    }

    // TODO: Implement NBV volumetric planner logic here
    // This would include:
    // - Initialize octomap interface
    // - Create NBV planner
    // - Plan next best views
    // - Execute plans using the manipulation workspace

    RCLCPP_INFO(node->get_logger(), "NBV Volumetric Planner Demo Ready");
    RCLCPP_INFO(node->get_logger(), "Workspace voxel size: %.4f m", workspace->getVoxelSize());
    RCLCPP_INFO(node->get_logger(), "Spinning node (Ctrl+C to exit)...");

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
} 
