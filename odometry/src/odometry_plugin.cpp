
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/Joint.hh>
#include <gazebo/physics/Link.hh>
#include <odometry/odometry_plugin.hpp>
#include <gazebo_ros/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <ignition/math/Pose3.hh>
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <gazebo/common/Console.hh>
#include <tf2_ros/transform_broadcaster.h>


#include <memory>

namespace gazebo{


class OdometryPluginPrivate
{
  /**
   * @brief Class to hold data members and methods for plugin
   * 
   */
public:
  /// Connection to world update event. Callback is called while this is alive.
  gazebo::event::ConnectionPtr update_connection_;

  /// Node for ROS communication.
  gazebo_ros::Node::SharedPtr ros_node_;

  // gazebo::physics::LinkPtr base_link_;
  gazebo::physics::LinkPtr baseLink;

  std::string base_link_name;
  std::string world_frame_name;

  /// Publisher to publsih the pose to /odom topic
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr odometryPublisher;

  /// Publisher to publsih the velocity to /velocity topic
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocityPublisher;

  /// TF Broadcaster to pubshil the transform between 2 frames
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  // Callback function to perform task with each iteration in Gazebo
  void OnUpdate();

};


/**
 * @brief Construct a new Odometry Plugin:: Odometry Plugin object
 * 
 */
OdometryPlugin::OdometryPlugin()
: impl_(std::make_unique<OdometryPluginPrivate>())
{
  printf("Initiated Odometry Plugin !\n");
}

/**
 * @brief Destroy the Odometry Plugin:: Odometry Plugin object
 * 
 */
OdometryPlugin::~OdometryPlugin()
{
}

/**
 * @brief Load the SDF/URDF model of the robot and access the links/joints.
 * 
 * @param model 
 * @param sdf 
 */
void OdometryPlugin::Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf)
{   


    std::cout << "Odometry Plugin Initialized" << std::endl;

    impl_->ros_node_ = gazebo_ros::Node::Get(sdf);
    impl_->odometryPublisher = impl_->ros_node_->create_publisher<geometry_msgs::msg::PoseStamped>("/odom", 10);
    impl_->velocityPublisher = impl_->ros_node_->create_publisher<geometry_msgs::msg::Twist>("/velocity", 10);
    impl_->tf_broadcaster_ =std::make_unique<tf2_ros::TransformBroadcaster>(impl_->ros_node_);

    std::cout << "Links in the model:" << std::endl;
    for (const auto &link : model->GetLinks()) {
        std::cout << " - " << link->GetName() << std::endl;
    }

    /// Check for the frame element in the URDF
    if (sdf->HasElement("frame")) {
        std::cout  << "Reference link  found!" << std::endl;
        impl_->base_link_name  = sdf->GetElement("frame")->Get<std::string>();

        std::cout  << "Base link  name: " <<impl_->base_link_name<< std::endl;
    } else {
        // Handle the case where the element doesn't exist
        std::cerr << "Reference link not found!" << std::endl;
        return;
    }
    // Access the base link of the model .
    impl_->baseLink = model->GetLink(impl_->base_link_name);
    

    // Check if the base link exists.
    if (!impl_->baseLink) {
        std::cout  << "Base link not found!" << std::endl;
        return;
    }


    /// Check for the world_frame element in the URDF
    if (sdf->HasElement("world_frame")) {
        std::cout  << "World frame  found!" << std::endl;
        impl_->world_frame_name  = sdf->GetElement("world_frame")->Get<std::string>();

        std::cout  << "World frame  name: " <<impl_->world_frame_name<< std::endl;
    } else {
        // Handle the case where the element doesn't exist
        std::cerr  << "World frame not found!" << std::endl;
        return;
    }



    

    // Set up a Gazebo update callback function.
    impl_->update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
        std::bind(&OdometryPluginPrivate::OnUpdate, impl_.get()));

    if (impl_->update_connection_) {
    std::cout << "Successfully connected to the world update event!" << std::endl;
    } else {
        std::cerr << "Failed to connect to the world update event!" << std::endl;
    }
}


/**
 * @brief This method is called at every time interval in Gazebo
 * 
 */
void OdometryPluginPrivate::OnUpdate()
{
    geometry_msgs::msg::PoseStamped odometryMessage;

    // Get the current pose of the base link.
    ignition::math::Pose3 baseLinkPose = baseLink->WorldPose();

    // Get the linear and angular velocities of the base link.
    ignition::math::Vector3d linearVelocity = baseLink->WorldLinearVel();
    ignition::math::Vector3d angularVelocity = baseLink->WorldAngularVel();

    // Extract position and orientation information
    odometryMessage.header.stamp = ros_node_->now();
    odometryMessage.header.frame_id = base_link_name;  
    odometryMessage.pose.position.x = baseLinkPose.Pos().X();
    odometryMessage.pose.position.y = baseLinkPose.Pos().Y();
    odometryMessage.pose.position.z = baseLinkPose.Pos().Z();
    odometryMessage.pose.orientation.x = baseLinkPose.Rot().X();
    odometryMessage.pose.orientation.y = baseLinkPose.Rot().Y();
    odometryMessage.pose.orientation.z = baseLinkPose.Rot().Z();
    odometryMessage.pose.orientation.w = baseLinkPose.Rot().W();

    // Create and set linear and angular velocities in the Twist message
    geometry_msgs::msg::Twist twist;

    twist.linear.x = linearVelocity.X();
    twist.linear.y = linearVelocity.Y();
    twist.linear.z = linearVelocity.Z();
    twist.angular.x = angularVelocity.X();
    twist.angular.y = angularVelocity.Y();
    twist.angular.z = angularVelocity.Z();

    geometry_msgs::msg::TransformStamped t;

    // Read message content and assign it to
    // corresponding tf variables
    t.header.stamp = ros_node_->now();
    t.header.frame_id = world_frame_name;
    t.child_frame_id = base_link_name;

    t.transform.translation.x = baseLinkPose.Pos().X();
    t.transform.translation.y = baseLinkPose.Pos().Y();
    t.transform.translation.z = baseLinkPose.Pos().Z();
    t.transform.rotation.x = baseLinkPose.Rot().X();
    t.transform.rotation.y = baseLinkPose.Rot().Y();
    t.transform.rotation.z = baseLinkPose.Rot().Z();
    t.transform.rotation.w = baseLinkPose.Rot().W();

    // Send the transformation
    tf_broadcaster_->sendTransform(t);

    velocityPublisher->publish(twist);

    odometryPublisher->publish(odometryMessage);
    


}

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(OdometryPlugin)

}