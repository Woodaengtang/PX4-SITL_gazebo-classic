#include "gazebo_motor_model_ros.h"
#include <gazebo/transport/transport.hh>
#include <vector>

using namespace gazebo;

GazeboMotorModelROS::~GazeboMotorModelROS() {
  rclcpp::shutdown();
}

void GazeboMotorModelROS::Load(physics::ModelPtr model, sdf::ElementPtr sdf) {
  model_ = model;

  // 1) 조인트 이름들 SDF로부터 읽기 (여기선 4개 하드코딩 예시)
  std::vector<std::string> joint_names = {
    "prop1_joint", "prop2_joint", "prop3_joint", "prop4_joint"
  };
  for (auto &jn : joint_names) {
    joints_.push_back(model_->GetJoint(jn));
  }
  ref_speeds_.assign(joints_.size(), 0.0);

  // 2) Gazebo 업데이트 콜백
  update_conn_ = event::Events::ConnectWorldUpdateBegin(
    std::bind(&GazeboMotorModelROS::OnUpdate, this, std::placeholders::_1));

  // 3) ROS 2 초기화 & 노드 생성
  if (!rclcpp::ok()) rclcpp::init(0, nullptr);
  ros_node_ = std::make_shared<rclcpp::Node>("gazebo_motor_model_ros");

  // 4) ROS 구독자 생성
  sub_ = ros_node_->create_subscription<std_msgs::msg::Float32MultiArray>(
    "/my_drone/motor_speeds", 10,
    std::bind(&GazeboMotorModelROS::CommandCallback, this, std::placeholders::_1));

  // 5) 스핀 스레드 띄우기
  std::thread([this]() { rclcpp::spin(ros_node_); }).detach();
}

void GazeboMotorModelROS::CommandCallback(
    const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
  // msg->data 크기 확인
  for (size_t i = 0; i < joints_.size() && i < msg->data.size(); ++i) {
    ref_speeds_[i] = msg->data[i];
  }
}

void GazeboMotorModelROS::OnUpdate(const common::UpdateInfo & /*info*/) {
  // 각 모터 조인트에 속도 반영
  for (size_t i = 0; i < joints_.size(); ++i) {
    // turning_direction = +1 으로 가정
    joints_[i]->SetVelocity(0, ref_speeds_[i]);
  }
}
