/**
 * @file octomap_saver.cpp
 * @brief Simple node to save octomap from topic to file
 */

#include <rclcpp/rclcpp.hpp>
#include <octomap_msgs/msg/octomap.hpp>
#include <octomap/octomap.h>
#include <octomap_msgs/conversions.h>
#include <fstream>
#include <sys/stat.h>

class OctomapSaver : public rclcpp::Node
{
public:
    OctomapSaver() : Node("octomap_saver")
    {
        // Declare parameters
        this->declare_parameter("output_file", "saved_map.bt");
        this->declare_parameter("octomap_topic", "/octomap_binary");
        
        output_file_ = this->get_parameter("output_file").as_string();
        std::string topic = this->get_parameter("octomap_topic").as_string();
        
        RCLCPP_INFO(this->get_logger(), "Octomap Saver Node");
        RCLCPP_INFO(this->get_logger(), "  Subscribing to: %s", topic.c_str());
        RCLCPP_INFO(this->get_logger(), "  Will save to: %s", output_file_.c_str());
        RCLCPP_INFO(this->get_logger(), "Waiting for octomap message...");
        
        // Subscribe to octomap topic
        sub_ = this->create_subscription<octomap_msgs::msg::Octomap>(
            topic, 
            rclcpp::QoS(10).transient_local(),
            std::bind(&OctomapSaver::octomapCallback, this, std::placeholders::_1));
    }

private:
    void octomapCallback(const octomap_msgs::msg::Octomap::SharedPtr msg)
    {
        if (saved_)
        {
            return; // Already saved
        }
        
        RCLCPP_INFO(this->get_logger(), "Received octomap message");
        
        // Convert message to OcTree
        octomap::AbstractOcTree* abstract_tree = octomap_msgs::msgToMap(*msg);
        if (!abstract_tree)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to convert message to octomap");
            rclcpp::shutdown();
            return;
        }
        
        octomap::OcTree* tree = dynamic_cast<octomap::OcTree*>(abstract_tree);
        if (!tree)
        {
            RCLCPP_ERROR(this->get_logger(), "Octomap is not an OcTree");
            delete abstract_tree;
            rclcpp::shutdown();
            return;
        }
        
        RCLCPP_INFO(this->get_logger(), "Octomap info:");
        RCLCPP_INFO(this->get_logger(), "  Resolution: %.4f m", tree->getResolution());
        RCLCPP_INFO(this->get_logger(), "  Size: %zu nodes", tree->size());
        
        // Save to file
        RCLCPP_INFO(this->get_logger(), "Writing to file: %s", output_file_.c_str());
        
        bool success = false;
        if (output_file_.substr(output_file_.find_last_of(".") + 1) == "bt")
        {
            success = tree->writeBinary(output_file_);
        }
        else
        {
            success = tree->write(output_file_);
        }
        
        if (success)
        {
            RCLCPP_INFO(this->get_logger(), "Successfully saved octomap to: %s", output_file_.c_str());
            
            // Get file size
            struct stat st;
            if (stat(output_file_.c_str(), &st) == 0)
            {
                RCLCPP_INFO(this->get_logger(), "File size: %.2f MB", 
                           static_cast<double>(st.st_size) / (1024.0 * 1024.0));
            }
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to save octomap to: %s", output_file_.c_str());
        }
        
        delete tree;
        saved_ = true;
        
        // Shutdown after saving
        RCLCPP_INFO(this->get_logger(), "Shutting down...");
        rclcpp::shutdown();
    }

    rclcpp::Subscription<octomap_msgs::msg::Octomap>::SharedPtr sub_;
    std::string output_file_;
    bool saved_ = false;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<OctomapSaver>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
