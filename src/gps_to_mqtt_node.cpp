#include <memory>
#include <string>
#include <sstream>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include <mosquitto.h>

class GpsToMqttNode : public rclcpp::Node
{
public:
  GpsToMqttNode()
  : Node("gps_to_mqtt_node"), mosq_(nullptr)
  {
    broker_address_ = this->declare_parameter<std::string>("broker_address", "183.111.206.213");
    broker_port_ = this->declare_parameter<int>("broker_port", 2219);
    mqtt_topic_ = this->declare_parameter<std::string>("mqtt_topic", "status");
    qos_level_ = this->declare_parameter<int>("qos_level", 0);
    client_id_ = this->declare_parameter<std::string>("client_id", "gps_bridge");

    RCLCPP_INFO(this->get_logger(), "broker: %s:%d", broker_address_.c_str(), broker_port_);
    RCLCPP_INFO(this->get_logger(), "mqtt_topic: %s", mqtt_topic_.c_str());

    mosquitto_lib_init();

    mosq_ = mosquitto_new(client_id_.c_str(), true, this);
    if (!mosq_) {
      RCLCPP_FATAL(this->get_logger(), "Failed to create mosquitto client");
      throw std::runtime_error("mosquitto_new failed");
    }

    mosquitto_connect_callback_set(mosq_, on_connect_wrapper);

    int rc = mosquitto_connect(mosq_, broker_address_.c_str(), broker_port_, 60);
    if (rc != MOSQ_ERR_SUCCESS) {
      RCLCPP_FATAL(this->get_logger(), "Connection failed: %s", mosquitto_strerror(rc));
      throw std::runtime_error("mosquitto_connect failed");
    }

    mosquitto_loop_start(mosq_);

    // 🔥 ROS2 subscriber
    sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
      "/ublox_gps_node/fix",
      10,
      std::bind(&GpsToMqttNode::callback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "GPS → MQTT bridge started");
  }

  ~GpsToMqttNode()
  {
    if (mosq_) {
      mosquitto_loop_stop(mosq_, true);
      mosquitto_disconnect(mosq_);
      mosquitto_destroy(mosq_);
    }
    mosquitto_lib_cleanup();
  }

private:
  static void on_connect_wrapper(struct mosquitto *mosq, void *obj, int rc)
  {
    if (rc == 0) {
      RCLCPP_INFO(rclcpp::get_logger("gps_to_mqtt"), "Connected to MQTT broker");
    } else {
      RCLCPP_ERROR(rclcpp::get_logger("gps_to_mqtt"), "Connect failed: %s", mosquitto_strerror(rc));
    }
  }

  void callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
  {
    double lat = msg->latitude;
    double lon = msg->longitude;

    // 🔥 JSON 생성
    std::stringstream ss;
    ss << "{"
       << "\"lat\":" << lat << ","
       << "\"lon\":" << lon
       << "}";

    std::string payload = ss.str();

    RCLCPP_INFO(this->get_logger(), "GPS → MQTT: %s", payload.c_str());

    int rc = mosquitto_publish(
      mosq_,
      nullptr,
      mqtt_topic_.c_str(),
      payload.size(),
      payload.c_str(),
      qos_level_,
      false);

    if (rc != MOSQ_ERR_SUCCESS) {
      RCLCPP_ERROR(this->get_logger(), "Publish error: %s", mosquitto_strerror(rc));
    }
  }

  struct mosquitto *mosq_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr sub_;

  std::string broker_address_;
  int broker_port_;
  std::string mqtt_topic_;
  int qos_level_;
  std::string client_id_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<GpsToMqttNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
