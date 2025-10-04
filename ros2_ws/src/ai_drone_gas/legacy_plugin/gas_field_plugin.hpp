#pragma once
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <random>


namespace ai_drone_gas {
class GasFieldPlugin : public gazebo::WorldPlugin {
public:
void Load(gazebo::physics::WorldPtr _world, sdf::ElementPtr _sdf) override;
void Reset() override {}
~GasFieldPlugin();
private:
double ConcentrationAt(const ignition::math::Vector3d& p) const;
ignition::math::Vector3d DronePosition();
void OnTimer();


gazebo::physics::WorldPtr world_;
rclcpp::Node::SharedPtr node_;
rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_;
rclcpp::TimerBase::SharedPtr timer_;
std::thread spinner_;


ignition::math::Vector3d origin_{8,0,1.0};
double sigma_{2.0}, cmax_{100.0}, noise_std_{1.0}, pub_rate_{20.0};
std::string topic_{"/gas_concentration"};


std::mt19937 rng_;
std::normal_distribution<double> dist_;
};
}