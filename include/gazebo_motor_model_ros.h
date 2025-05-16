#ifndef GAZEBO_MOTOR_MODEL_ROS_H
#define GAZEBO_MOTOR_MODEL_ROS_H

#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <ignition/math/Vector3.hh>
#include <vector>

namespace gazebo {

class GazeboMotorModelROS : public ModelPlugin {
public:
  GazeboMotorModelROS() = default;
  virtual ~GazeboMotorModelROS();

  void Load(physics::ModelPtr model, sdf::ElementPtr sdf) override;

private:
  void OnUpdate(const common::UpdateInfo &info);
  void CommandCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg);

  // Gazebo
  physics::ModelPtr model_;
  std::vector<physics::JointPtr> joints_;
  event::ConnectionPtr update_conn_;

  // ROS 2
  std::shared_ptr<rclcpp::Node> ros_node_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_;

  std::vector<double> ref_speeds_;
};

GZ_REGISTER_MODEL_PLUGIN(GazeboMotorModelROS)

}  // namespace gazebo

#endif  // GAZEBO_MOTOR_MODEL_ROS_H
