#include <memory>
#include <string>
#include <sstream>
#include <iomanip>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "geometry_msgs/msg/twist_with_covariance_stamped.hpp"
#include "ublox_msgs/msg/nav_pvt.hpp"

#include <mosquitto.h>

class GpsToMqttNode : public rclcpp::Node
{
public:
  GpsToMqttNode()
  : Node("gps_to_mqtt_node")
  {
    this->declare_parameter<std::string>("broker_address", "183.111.206.213");
    this->declare_parameter<int>("broker_port", 2219);
    this->declare_parameter<std::string>("topic_name", "status");
    this->declare_parameter<std::string>("client_id", "ros2_gps_mqtt_node");

    broker_address_ = this->get_parameter("broker_address").as_string();
    broker_port_ = this->get_parameter("broker_port").as_int();
    topic_name_ = this->get_parameter("topic_name").as_string();
    client_id_ = this->get_parameter("client_id").as_string();

    mosquitto_lib_init();
    mosq_ = mosquitto_new(client_id_.c_str(), true, nullptr);
    if (!mosq_) {
      throw std::runtime_error("mosquitto_new failed");
    }

    int rc = mosquitto_connect(mosq_, broker_address_.c_str(), broker_port_, 60);
    if (rc != MOSQ_ERR_SUCCESS) {
      std::ostringstream oss;
      oss << "mosquitto_connect failed: " << mosquitto_strerror(rc);
      throw std::runtime_error(oss.str());
    }

    // 1) 위치
    fix_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
      "/ublox_gps_node/fix",
      10,
      std::bind(&GpsToMqttNode::fixCallback, this, std::placeholders::_1));

    // 2) 속도 (ROS driver 가공본)
    vel_sub_ = this->create_subscription<geometry_msgs::msg::TwistWithCovarianceStamped>(
      "/ublox_gps_node/fix_velocity",
      10,
      std::bind(&GpsToMqttNode::velCallback, this, std::placeholders::_1));

    // 3) heading + ground speed (u-blox NAV-PVT)
    navpvt_sub_ = this->create_subscription<ublox_msgs::msg::NavPVT>(
      "/ublox_gps_node/navpvt",
      10,
      std::bind(&GpsToMqttNode::navpvtCallback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "gps_to_mqtt_node started");
  }

  ~GpsToMqttNode() override
  {
    if (mosq_) {
      mosquitto_disconnect(mosq_);
      mosquitto_destroy(mosq_);
      mosq_ = nullptr;
    }
    mosquitto_lib_cleanup();
  }

private:
  // -----------------------------
  // Subscribers
  // -----------------------------
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr fix_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr vel_sub_;
  rclcpp::Subscription<ublox_msgs::msg::NavPVT>::SharedPtr navpvt_sub_;

  // -----------------------------
  // MQTT
  // -----------------------------
  struct mosquitto *mosq_{nullptr};
  std::string broker_address_;
  int broker_port_;
  std::string topic_name_;
  std::string client_id_;

  // -----------------------------
  // Latest data
  // -----------------------------
  double latitude_{0.0};
  double longitude_{0.0};
  double altitude_{0.0};

  double vel_x_{0.0};
  double vel_y_{0.0};
  double vel_z_{0.0};
  double speed_mps_{0.0};

  double ground_speed_mps_{0.0};
  double heading_deg_{0.0};       // heading of motion
  double vehicle_heading_deg_{0.0}; // headVeh
  int fix_type_{0};

  bool has_fix_{false};
  bool has_vel_{false};
  bool has_navpvt_{false};

  void fixCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
  {
    latitude_ = msg->latitude;
    longitude_ = msg->longitude;
    altitude_ = msg->altitude;
    has_fix_ = true;

    publishCombined();
  }

  void velCallback(const geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr msg)
  {
    vel_x_ = msg->twist.twist.linear.x;
    vel_y_ = msg->twist.twist.linear.y;
    vel_z_ = msg->twist.twist.linear.z;

    speed_mps_ = std::sqrt(
      vel_x_ * vel_x_ +
      vel_y_ * vel_y_ +
      vel_z_ * vel_z_);

    has_vel_ = true;

    publishCombined();
  }

  void navpvtCallback(const ublox_msgs::msg::NavPVT::SharedPtr msg)
  {
    // NavPVT.gSpeed 는 mm/s
    ground_speed_mps_ = static_cast<double>(msg->g_speed) / 1000.0;

    // heading, headVeh 는 deg / 1e-5
    heading_deg_ = static_cast<double>(msg->heading) / 100000.0;
    vehicle_heading_deg_ = static_cast<double>(msg->head_veh) / 100000.0;

    fix_type_ = msg->fix_type;
    has_navpvt_ = true;

    publishCombined();
  }

  std::string makeJson() const
  {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(8);

    oss << "{";
    oss << "\"latitude\":" << latitude_ << ",";
    oss << "\"longitude\":" << longitude_ << ",";
    //oss << "\"altitude\":" << altitude_ << ",";

    //oss << "\"vel_x_mps\":" << vel_x_ << ",";
    //oss << "\"vel_y_mps\":" << vel_y_ << ",";
    //oss << "\"vel_z_mps\":" << vel_z_ << ",";
    //oss << "\"speed_mps\":" << speed_mps_ << ",";

    oss << "\"ground_speed_mps\":" << ground_speed_mps_ << ",";
    oss << "\"heading_deg\":" << heading_deg_ << ",";
    //oss << "\"vehicle_heading_deg\":" << vehicle_heading_deg_ << ",";
    oss << "\"fix_type\":" << fix_type_;

    oss << "}";
    return oss.str();
  }

  void publishCombined()
  {
    if (!has_fix_) {
      return;
    }

    std::string payload = makeJson();

    int rc = mosquitto_publish(
      mosq_,
      nullptr,
      topic_name_.c_str(),
      static_cast<int>(payload.size()),
      payload.c_str(),
      0,
      false);

    if (rc != MOSQ_ERR_SUCCESS) {
      RCLCPP_ERROR(this->get_logger(), "mosquitto_publish failed: %s", mosquitto_strerror(rc));
      return;
    }

    RCLCPP_INFO(this->get_logger(), "Published: %s", payload.c_str());
    mosquitto_loop(mosq_, 0, 1);
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<GpsToMqttNode>();
    rclcpp::spin(node);
  } catch (const std::exception & e) {
    fprintf(stderr, "Exception: %s\n", e.what());
  }
  rclcpp::shutdown();
  return 0;
}
