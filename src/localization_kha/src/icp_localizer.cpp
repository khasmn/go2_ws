#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl_conversions/pcl_conversions.h>
#include <Eigen/Dense>

class IcpLocalizer : public rclcpp::Node {
public:
    IcpLocalizer() : Node("icp_localizer") {
        this->declare_parameter<std::string>("pcd_filename", "");
        std::string pcd_filename;
        this->get_parameter("pcd_filename", pcd_filename);

        // Map Publisher setup
        rclcpp::QoS qos(rclcpp::KeepLast(1));
        qos.transient_local();
        map_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/global_map", qos);

        target_map_.reset(new pcl::PointCloud<pcl::PointXYZI>());
        if (pcl::io::loadPCDFile<pcl::PointXYZI>(pcd_filename, *target_map_) == -1) {
            RCLCPP_ERROR(this->get_logger(), "Couldn't read map file. Check the path!");
            return;
        }
        RCLCPP_INFO(this->get_logger(), "Loaded map successfully. Publishing to /global_map...");

        // Convert PCL to ROS Message and publish the global map
        sensor_msgs::msg::PointCloud2 map_msg;
        pcl::toROSMsg(*target_map_, map_msg);
        map_msg.header.frame_id = "map";
        map_msg.header.stamp = this->now();
        map_pub_->publish(map_msg);

        icp_.setInputTarget(target_map_);
        icp_.setMaxCorrespondenceDistance(1.0); 
        icp_.setMaximumIterations(50);
        icp_.setTransformationEpsilon(1e-8);

        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        
        cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/cloud_registered_body", 10, std::bind(&IcpLocalizer::cloud_callback, this, std::placeholders::_1));
            
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/Odometry", 10, std::bind(&IcpLocalizer::odom_callback, this, std::placeholders::_1));
    }

private:
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        latest_odom_ = msg; 
    }

    void cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        if (!latest_odom_) return; 

        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::fromROSMsg(*msg, *cloud_in);

        // Convert ROS Odometry to Eigen::Matrix4f for the Initial Guess
        Eigen::Matrix4f initial_guess = Eigen::Matrix4f::Identity();
        Eigen::Quaternionf q(
            latest_odom_->pose.pose.orientation.w,
            latest_odom_->pose.pose.orientation.x,
            latest_odom_->pose.pose.orientation.y,
            latest_odom_->pose.pose.orientation.z
        );
        initial_guess.block<3, 3>(0, 0) = q.toRotationMatrix();
        initial_guess(0, 3) = latest_odom_->pose.pose.position.x;
        initial_guess(1, 3) = latest_odom_->pose.pose.position.y;
        initial_guess(2, 3) = latest_odom_->pose.pose.position.z;

        // Run ICP using the initial guess
        pcl::PointCloud<pcl::PointXYZI> Final;
        icp_.setInputSource(cloud_in);
        icp_.align(Final, initial_guess); 

        if (icp_.hasConverged()) {
            Eigen::Matrix4f transformation = icp_.getFinalTransformation();
            publish_tf(transformation, msg->header.stamp);
        } else {
            RCLCPP_WARN(this->get_logger(), "ICP did not converge!");
        }
    }

    void publish_tf(const Eigen::Matrix4f& transform_matrix, rclcpp::Time stamp) {
        geometry_msgs::msg::TransformStamped t;
        t.header.stamp = stamp;
        t.header.frame_id = "map";
        t.child_frame_id = "odom";

        t.transform.translation.x = transform_matrix(0, 3);
        t.transform.translation.y = transform_matrix(1, 3);
        t.transform.translation.z = transform_matrix(2, 3);

        Eigen::Matrix3f rot_matrix = transform_matrix.block<3, 3>(0, 0);
        Eigen::Quaternionf q(rot_matrix);
        t.transform.rotation.x = q.x();
        t.transform.rotation.y = q.y();
        t.transform.rotation.z = q.z();
        t.transform.rotation.w = q.w();

        tf_broadcaster_->sendTransform(t);
    }

    pcl::PointCloud<pcl::PointXYZI>::Ptr target_map_;
    pcl::IterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI> icp_;
    nav_msgs::msg::Odometry::SharedPtr latest_odom_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_pub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<IcpLocalizer>());
    rclcpp::shutdown();
    return 0;
}