#include "ai_drone_gas/gas_field_plugin.hpp"


using ignition::math::Vector3d;


namespace ai_drone_gas {


void GasFieldPlugin::Load(gazebo::physics::WorldPtr _world, sdf::ElementPtr _sdf)
{
world_ = _world;
node_ = std::make_shared<rclcpp::Node>("gas_field_plugin");


if (_sdf->HasElement("origin")) {
auto v = _sdf->Get<Vector3d>("origin");
origin_ = Vector3d(v.X(), v.Y(), v.Z());
}
sigma_ = _sdf->Get<double>("sigma", 2.0).first;
cmax_ = _sdf->Get<double>("Cmax", 100.0).first;
topic_ = _sdf->Get<std::string>("topic", "/gas_concentration").first;
noise_std_ = _sdf->Get<double>("noise_std", 1.0).first;
pub_rate_ = _sdf->Get<double>("publish_rate", 20.0).first;


pub_ = node_->create_publisher<std_msgs::msg::Float32>(topic_, 10);


using namespace std::chrono_literals;
timer_ = node_->create_wall_timer(
std::chrono::milliseconds((int)(1000.0/pub_rate_)),
std::bind(&GasFieldPlugin::OnTimer, this));


rng_.seed(std::random_device{}());
dist_ = std::normal_distribution<double>(0.0, noise_std_);


// ROS2 spin ayrı thread
spinner_ = std::thread([this](){ rclcpp::spin(this->node_); });
}


GasFieldPlugin::~GasFieldPlugin()
{
rclcpp::shutdown();
if (spinner_.joinable()) spinner_.join();
}


// Gauss alanı
double GasFieldPlugin::ConcentrationAt(const Vector3d& p) const
{
auto d2 = (p - origin_).SquaredLength();
return cmax_ * std::exp(- d2 / (2.0 * sigma_ * sigma_));
}


Vector3d GasFieldPlugin::DronePosition()
{
auto models = world_->Models();
for (auto& m : models) {
auto name = m->GetName();
if (name.find("iris") != std::string::npos || name.find("drone") != std::string::npos) {
return m->WorldPose().Pos();
}
}
return Vector3d(0,0,0);
}


void GasFieldPlugin::OnTimer()
{
auto p = DronePosition();
double c = ConcentrationAt(p) + dist_(rng_);
std_msgs::msg::Float32 msg; msg.data = static_cast<float>(c);
pub_->publish(msg);
}


GZ_REGISTER_WORLD_PLUGIN(GasFieldPlugin)
} // namespace ai_drone_gas