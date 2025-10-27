#include "ai_drone_gas/gas_field_plugin.hpp"
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/common/Events.hh>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <thread>
#include <random>

namespace gazebo
{
  class GasFieldPlugin : public WorldPlugin
  {
  public:
    GasFieldPlugin() : WorldPlugin()
    {
    }

    void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf) override
    {
      world_ = _world;

      if (!rclcpp::ok())
        rclcpp::init(0, nullptr);

      node_ = std::make_shared<rclcpp::Node>("gas_field_plugin");
      publisher_ = node_->create_publisher<std_msgs::msg::Float32>("/gas_concentration", 10);

      // Thread içinde ROS2 node'u spin ettir
      ros_thread_ = std::thread([this]() {
        rclcpp::spin(node_);
      });

      // Timer thread: gaz verisini düzenli olarak yayınla
      publish_thread_ = std::thread([this]() {
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_real_distribution<float> dist(0.0, 1.0);

        rclcpp::Rate rate(2.0); // 2 Hz = her 0.5 saniyede bir
        while (rclcpp::ok())
        {
          std_msgs::msg::Float32 msg;
          msg.data = dist(gen);  // Şimdilik rastgele gaz yoğunluğu
          publisher_->publish(msg);
          rate.sleep();
        }
      });

      RCLCPP_INFO(node_->get_logger(), "✅ GasFieldPlugin başarıyla yüklendi ve veri yayını başladı!");
    }

    ~GasFieldPlugin()
    {
      if (publish_thread_.joinable())
        publish_thread_.join();

      if (ros_thread_.joinable())
      {
        rclcpp::shutdown();
        ros_thread_.join();
      }
    }

  private:
    physics::WorldPtr world_;
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr publisher_;
    std::thread ros_thread_;
    std::thread publish_thread_;
  };

  GZ_REGISTER_WORLD_PLUGIN(GasFieldPlugin)
}

