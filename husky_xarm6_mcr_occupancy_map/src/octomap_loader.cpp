/**
 * @file octomap_loader.cpp
 * @brief Simple node to load octomap from file and publish continuously
 */

#include <rclcpp/rclcpp.hpp>
#include <octomap_msgs/msg/octomap.hpp>
#include <octomap/octomap.h>
#include <octomap_msgs/conversions.h>
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <std_msgs/msg/color_rgba.hpp>

class OctomapLoader : public rclcpp::Node
{
public:
    OctomapLoader() : Node("octomap_loader")
    {
        // Declare parameters
        this->declare_parameter("input_file", "saved_map.bt");
        this->declare_parameter("output_topic", "/octomap_binary");
        this->declare_parameter("frame_id", "map");
        this->declare_parameter("publish_rate_hz", 1.0);
        this->declare_parameter("publish_free_voxels", false);
        this->declare_parameter("enable_visualization", true);
        this->declare_parameter("visualization_topic", "occupancy_map_markers");
        this->declare_parameter("visualization_rate_hz", 1.0);
        this->declare_parameter("show_bounding_box", false);
        this->declare_parameter("bbx_min_x", -1.0);
        this->declare_parameter("bbx_min_y", -1.0);
        this->declare_parameter("bbx_min_z", -1.0);
        this->declare_parameter("bbx_max_x", 1.0);
        this->declare_parameter("bbx_max_y", 1.0);
        this->declare_parameter("bbx_max_z", 1.0);
        
        std::string input_file = this->get_parameter("input_file").as_string();
        std::string topic = this->get_parameter("output_topic").as_string();
        frame_id_ = this->get_parameter("frame_id").as_string();
        double rate = this->get_parameter("publish_rate_hz").as_double();
        publish_free_ = this->get_parameter("publish_free_voxels").as_bool();
        enable_viz_ = this->get_parameter("enable_visualization").as_bool();
        std::string viz_topic = this->get_parameter("visualization_topic").as_string();
        double viz_rate = this->get_parameter("visualization_rate_hz").as_double();
        show_bbox_ = this->get_parameter("show_bounding_box").as_bool();
        
        if (show_bbox_)
        {
            bbx_min_ = octomap::point3d(
                this->get_parameter("bbx_min_x").as_double(),
                this->get_parameter("bbx_min_y").as_double(),
                this->get_parameter("bbx_min_z").as_double());
            bbx_max_ = octomap::point3d(
                this->get_parameter("bbx_max_x").as_double(),
                this->get_parameter("bbx_max_y").as_double(),
                this->get_parameter("bbx_max_z").as_double());
        }
        
        RCLCPP_INFO(this->get_logger(), "Octomap Loader Node");
        RCLCPP_INFO(this->get_logger(), "  Loading from: %s", input_file.c_str());
        RCLCPP_INFO(this->get_logger(), "  Publishing to: %s", topic.c_str());
        RCLCPP_INFO(this->get_logger(), "  Frame ID: %s", frame_id_.c_str());
        RCLCPP_INFO(this->get_logger(), "  Publish rate: %.1f Hz", rate);
        
        // Load octomap from file
        RCLCPP_INFO(this->get_logger(), "Loading octomap from file...");
        
        std::string extension = input_file.substr(input_file.find_last_of(".") + 1);
        
        if (extension == "bt")
        {
            // Binary format - create OcTree and use readBinary
            RCLCPP_INFO(this->get_logger(), "Reading binary format (.bt)");
            tree_ = std::make_shared<octomap::OcTree>(0.1);  // Temporary resolution, will be overwritten
            if (!tree_->readBinary(input_file))
            {
                RCLCPP_ERROR(this->get_logger(), "Failed to load binary octomap from: %s", input_file.c_str());
                rclcpp::shutdown();
                return;
            }
        }
        else
        {
            // Text format (.ot) - use AbstractOcTree::read
            RCLCPP_INFO(this->get_logger(), "Reading text format (.ot)");
            octomap::AbstractOcTree* abstract_tree = octomap::AbstractOcTree::read(input_file);
            
            if (!abstract_tree)
            {
                RCLCPP_ERROR(this->get_logger(), "Failed to load text octomap from: %s", input_file.c_str());
                rclcpp::shutdown();
                return;
            }
            
            tree_ = std::shared_ptr<octomap::OcTree>(dynamic_cast<octomap::OcTree*>(abstract_tree));
            if (!tree_)
            {
                RCLCPP_ERROR(this->get_logger(), "Loaded octomap is not an OcTree");
                delete abstract_tree;
                rclcpp::shutdown();
                return;
            }
        }
        
        if (!tree_)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to load octomap");
            rclcpp::shutdown();
            return;
        }
        
        RCLCPP_INFO(this->get_logger(), "Successfully loaded octomap:");
        RCLCPP_INFO(this->get_logger(), "  Resolution: %.4f m", tree_->getResolution());
        RCLCPP_INFO(this->get_logger(), "  Size: %zu nodes", tree_->size());
        RCLCPP_INFO(this->get_logger(), "  Leaf nodes: %zu", tree_->getNumLeafNodes());
        
        // Count occupied/free voxels
        size_t occupied = 0, free_voxels = 0;
        for (auto it = tree_->begin_leafs(); it != tree_->end_leafs(); ++it)
        {
            if (tree_->isNodeOccupied(*it))
                occupied++;
            else
                free_voxels++;
        }
        RCLCPP_INFO(this->get_logger(), "  Occupied voxels: %zu", occupied);
        RCLCPP_INFO(this->get_logger(), "  Free voxels: %zu", free_voxels);
        
        // Create publisher with transient local (latched)
        pub_ = this->create_publisher<octomap_msgs::msg::Octomap>(
            topic,
            rclcpp::QoS(10).transient_local());
        
        // Create marker publisher for visualization (if enabled)
        if (enable_viz_)
        {
            marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
                viz_topic,
                rclcpp::QoS(10));
            RCLCPP_INFO(this->get_logger(), "  Visualization topic: %s", viz_topic.c_str());
        }
        
        // Publish immediately
        publishOctomap();
        if (enable_viz_)
        {
            publishVisualization();
        }
        
        // Create timer for periodic octomap republishing
        auto period = std::chrono::duration<double>(1.0 / rate);
        timer_ = this->create_wall_timer(
            std::chrono::duration_cast<std::chrono::milliseconds>(period),
            std::bind(&OctomapLoader::publishOctomap, this));
        
        // Create separate timer for visualization (if enabled)
        if (enable_viz_)
        {
            auto viz_period = std::chrono::duration<double>(1.0 / viz_rate);
            viz_timer_ = this->create_wall_timer(
                std::chrono::duration_cast<std::chrono::milliseconds>(viz_period),
                std::bind(&OctomapLoader::publishVisualization, this));
        }
        
        RCLCPP_INFO(this->get_logger(), "Publishing octomap (press Ctrl+C to stop)");
    }

private:
    void publishOctomap()
    {
        octomap_msgs::msg::Octomap msg;
        msg.header.stamp = this->now();
        msg.header.frame_id = frame_id_;
        
        if (octomap_msgs::binaryMapToMsg(*tree_, msg))
        {
            pub_->publish(msg);
            
            static int count = 0;
            if (++count % 10 == 0)
            {
                RCLCPP_DEBUG(this->get_logger(), "Published octomap (%d times)", count);
            }
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to convert octomap to message");
        }
    }
    
    void publishVisualization()
    {
        if (!enable_viz_ || !marker_pub_)
        {
            return;
        }
        
        visualization_msgs::msg::MarkerArray marker_array;
        auto stamp = this->now();
        
        // Collect occupied and free voxels
        std::vector<octomap::point3d> occupied_points;
        std::vector<octomap::point3d> free_points;
        
        for (auto it = tree_->begin_leafs(); it != tree_->end_leafs(); ++it)
        {
            if (tree_->isNodeOccupied(*it))
            {
                occupied_points.push_back(it.getCoordinate());
            }
            else if (publish_free_)
            {
                free_points.push_back(it.getCoordinate());
            }
        }
        
        // Create occupied voxels marker
        if (!occupied_points.empty())
        {
            visualization_msgs::msg::Marker occupied_marker;
            occupied_marker.header.frame_id = frame_id_;
            occupied_marker.header.stamp = stamp;
            occupied_marker.ns = "occupied_voxels";
            occupied_marker.id = 0;
            occupied_marker.type = visualization_msgs::msg::Marker::CUBE_LIST;
            occupied_marker.action = visualization_msgs::msg::Marker::ADD;
            occupied_marker.scale.x = tree_->getResolution();
            occupied_marker.scale.y = tree_->getResolution();
            occupied_marker.scale.z = tree_->getResolution();
            occupied_marker.color.r = 1.0;
            occupied_marker.color.g = 0.0;
            occupied_marker.color.b = 0.0;
            occupied_marker.color.a = 0.8;
            occupied_marker.pose.orientation.w = 1.0;
            
            for (const auto& point : occupied_points)
            {
                geometry_msgs::msg::Point p;
                p.x = point.x();
                p.y = point.y();
                p.z = point.z();
                occupied_marker.points.push_back(p);
            }
            
            marker_array.markers.push_back(occupied_marker);
        }
        
        // Create free voxels marker
        if (publish_free_ && !free_points.empty())
        {
            visualization_msgs::msg::Marker free_marker;
            free_marker.header.frame_id = frame_id_;
            free_marker.header.stamp = stamp;
            free_marker.ns = "free_voxels";
            free_marker.id = 1;
            free_marker.type = visualization_msgs::msg::Marker::CUBE_LIST;
            free_marker.action = visualization_msgs::msg::Marker::ADD;
            free_marker.scale.x = tree_->getResolution();
            free_marker.scale.y = tree_->getResolution();
            free_marker.scale.z = tree_->getResolution();
            free_marker.color.r = 0.0;
            free_marker.color.g = 1.0;
            free_marker.color.b = 0.0;
            free_marker.color.a = 0.2;
            free_marker.pose.orientation.w = 1.0;
            
            for (const auto& point : free_points)
            {
                geometry_msgs::msg::Point p;
                p.x = point.x();
                p.y = point.y();
                p.z = point.z();
                free_marker.points.push_back(p);
            }
            
            marker_array.markers.push_back(free_marker);
        }
        
        // Create bounding box marker if enabled
        if (show_bbox_)
        {
            visualization_msgs::msg::Marker bbox_marker;
            bbox_marker.header.frame_id = frame_id_;
            bbox_marker.header.stamp = stamp;
            bbox_marker.ns = "bounding_box";
            bbox_marker.id = 100;
            bbox_marker.type = visualization_msgs::msg::Marker::LINE_LIST;
            bbox_marker.action = visualization_msgs::msg::Marker::ADD;
            bbox_marker.scale.x = 0.02;  // Line width
            bbox_marker.color.r = 0.0;
            bbox_marker.color.g = 0.5;
            bbox_marker.color.b = 1.0;
            bbox_marker.color.a = 1.0;
            bbox_marker.pose.orientation.w = 1.0;
            
            // Define 8 corners of the bounding box
            std::array<geometry_msgs::msg::Point, 8> corners;
            corners[0].x = bbx_min_.x(); corners[0].y = bbx_min_.y(); corners[0].z = bbx_min_.z();
            corners[1].x = bbx_max_.x(); corners[1].y = bbx_min_.y(); corners[1].z = bbx_min_.z();
            corners[2].x = bbx_max_.x(); corners[2].y = bbx_max_.y(); corners[2].z = bbx_min_.z();
            corners[3].x = bbx_min_.x(); corners[3].y = bbx_max_.y(); corners[3].z = bbx_min_.z();
            corners[4].x = bbx_min_.x(); corners[4].y = bbx_min_.y(); corners[4].z = bbx_max_.z();
            corners[5].x = bbx_max_.x(); corners[5].y = bbx_min_.y(); corners[5].z = bbx_max_.z();
            corners[6].x = bbx_max_.x(); corners[6].y = bbx_max_.y(); corners[6].z = bbx_max_.z();
            corners[7].x = bbx_min_.x(); corners[7].y = bbx_max_.y(); corners[7].z = bbx_max_.z();
            
            // Bottom face edges
            bbox_marker.points.push_back(corners[0]); bbox_marker.points.push_back(corners[1]);
            bbox_marker.points.push_back(corners[1]); bbox_marker.points.push_back(corners[2]);
            bbox_marker.points.push_back(corners[2]); bbox_marker.points.push_back(corners[3]);
            bbox_marker.points.push_back(corners[3]); bbox_marker.points.push_back(corners[0]);
            
            // Top face edges
            bbox_marker.points.push_back(corners[4]); bbox_marker.points.push_back(corners[5]);
            bbox_marker.points.push_back(corners[5]); bbox_marker.points.push_back(corners[6]);
            bbox_marker.points.push_back(corners[6]); bbox_marker.points.push_back(corners[7]);
            bbox_marker.points.push_back(corners[7]); bbox_marker.points.push_back(corners[4]);
            
            // Vertical edges
            bbox_marker.points.push_back(corners[0]); bbox_marker.points.push_back(corners[4]);
            bbox_marker.points.push_back(corners[1]); bbox_marker.points.push_back(corners[5]);
            bbox_marker.points.push_back(corners[2]); bbox_marker.points.push_back(corners[6]);
            bbox_marker.points.push_back(corners[3]); bbox_marker.points.push_back(corners[7]);
            
            marker_array.markers.push_back(bbox_marker);
        }
        
        if (!marker_array.markers.empty())
        {
            marker_pub_->publish(marker_array);
            RCLCPP_DEBUG(this->get_logger(), "Published visualization with %zu occupied, %zu free voxels",
                        occupied_points.size(), free_points.size());
        }
    }

    rclcpp::Publisher<octomap_msgs::msg::Octomap>::SharedPtr pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr viz_timer_;
    std::shared_ptr<octomap::OcTree> tree_;
    std::string frame_id_;
    bool publish_free_;
    bool enable_viz_;
    bool show_bbox_;
    octomap::point3d bbx_min_;
    octomap::point3d bbx_max_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<OctomapLoader>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
