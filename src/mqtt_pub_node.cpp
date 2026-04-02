#include <chrono>
#include <cstring>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include <mosquitto.h>

using namespace std::chrono_literals;

class MqttPublisherNode : public rclcpp::Node
{
public:
  MqttPublisherNode()
  : Node("mqtt_pub_node"),
    mosq_(nullptr),
    published_(false)
  {
    broker_address_ = this->declare_parameter<std::string>("broker_address", "183.111.206.213");
    broker_port_ = this->declare_parameter<int>("broker_port", 2219);
    topic_name_ = this->declare_parameter<std::string>("topic_name", "SAMPLE_TOPIC");
    qos_level_ = this->declare_parameter<int>("qos_level", 0);
    client_id_ = this->declare_parameter<std::string>("client_id", "publisher_client");
    payload_ = this->declare_parameter<std::string>(
      "payload", "This is sample message from ROS2 publisher...");

    mosquitto_lib_init();

    mosq_ = mosquitto_new(client_id_.c_str(), true, this);
    if (!mosq_) {
      RCLCPP_FATAL(this->get_logger(), "Failed to create mosquitto client.");
      throw std::runtime_error("mosquitto_new failed");
    }

    mosquitto_connect_callback_set(mosq_, &MqttPublisherNode::on_connect_wrapper);
    mosquitto_disconnect_callback_set(mosq_, &MqttPublisherNode::on_disconnect_wrapper);
    mosquitto_publish_callback_set(mosq_, &MqttPublisherNode::on_publish_wrapper);
    mosquitto_log_callback_set(mosq_, &MqttPublisherNode::on_log_wrapper);

    int rc = mosquitto_connect(mosq_, broker_address_.c_str(), broker_port_, 60);
    if (rc != MOSQ_ERR_SUCCESS) {
      RCLCPP_FATAL(
        this->get_logger(),
        "Initial connection failed: %s", mosquitto_strerror(rc));
      cleanup();
      throw std::runtime_error("mosquitto_connect failed");
    }

    rc = mosquitto_loop_start(mosq_);
    if (rc != MOSQ_ERR_SUCCESS) {
      RCLCPP_FATAL(
        this->get_logger(),
        "Failed to start mosquitto network loop: %s", mosquitto_strerror(rc));
      cleanup();
      throw std::runtime_error("mosquitto_loop_start failed");
    }

    timer_ = this->create_wall_timer(
      1s, std::bind(&MqttPublisherNode::publish_once, this));

    RCLCPP_INFO(this->get_logger(), "MQTT publisher node started.");
  }

  ~MqttPublisherNode() override
  {
    cleanup();
  }

private:
  static void on_connect_wrapper(struct mosquitto * mosq, void * obj, int rc)
  {
    auto * self = static_cast<MqttPublisherNode *>(obj);
    if (self) {
      self->on_connect(mosq, rc);
    }
  }

  static void on_disconnect_wrapper(struct mosquitto * mosq, void * obj, int rc)
  {
    auto * self = static_cast<MqttPublisherNode *>(obj);
    if (self) {
      self->on_disconnect(mosq, rc);
    }
  }

  static void on_publish_wrapper(struct mosquitto * mosq, void * obj, int mid)
  {
    auto * self = static_cast<MqttPublisherNode *>(obj);
    if (self) {
      self->on_publish(mosq, mid);
    }
  }

  static void on_log_wrapper(struct mosquitto * mosq, void * obj, int level, const char * str)
  {
    auto * self = static_cast<MqttPublisherNode *>(obj);
    if (self) {
      self->on_log(mosq, level, str);
    }
  }

  void on_connect(struct mosquitto *, int rc)
  {
    if (rc == 0) {
      RCLCPP_INFO(
        this->get_logger(),
        "[%s] Connected to MQTT broker at %s:%d",
        client_id_.c_str(), broker_address_.c_str(), broker_port_);
    } else {
      RCLCPP_ERROR(
        this->get_logger(),
        "[%s] Connection failed: %s",
        client_id_.c_str(), mosquitto_strerror(rc));
    }
  }

  void on_disconnect(struct mosquitto *, int rc)
  {
    RCLCPP_INFO(
      this->get_logger(),
      "[%s] Disconnected from MQTT broker (RC: %d)",
      client_id_.c_str(), rc);
  }

  void on_publish(struct mosquitto *, int mid)
  {
    RCLCPP_INFO(
      this->get_logger(),
      "[%s] Message with mid %d has been published.",
      client_id_.c_str(), mid);
  }

  void on_log(struct mosquitto *, int, const char * str)
  {
    RCLCPP_DEBUG(
      this->get_logger(),
      "[%s] Mosquitto log: %s",
      client_id_.c_str(), str);
  }

  void publish_once()
  {
    if (published_) {
      return;
    }

    RCLCPP_INFO(
      this->get_logger(),
      "[%s] Attempting to publish '%s' to topic '%s' (QoS: %d)",
      client_id_.c_str(), payload_.c_str(), topic_name_.c_str(), qos_level_);

    int rc = mosquitto_publish(
      mosq_,
      nullptr,
      topic_name_.c_str(),
      static_cast<int>(payload_.size()),
      payload_.c_str(),
      qos_level_,
      false);

    if (rc != MOSQ_ERR_SUCCESS) {
      RCLCPP_ERROR(
        this->get_logger(),
        "[%s] Error publishing message: %s",
        client_id_.c_str(), mosquitto_strerror(rc));
    } else {
      published_ = true;
      RCLCPP_INFO(this->get_logger(), "[%s] Publish finished.", client_id_.c_str());

      timer_->cancel();

      std::this_thread::sleep_for(std::chrono::milliseconds(300));
      rclcpp::shutdown();
    }
  }

  void cleanup()
  {
    if (mosq_) {
      mosquitto_loop_stop(mosq_, true);
      mosquitto_disconnect(mosq_);
      mosquitto_destroy(mosq_);
      mosq_ = nullptr;
    }
    mosquitto_lib_cleanup();
  }

  struct mosquitto * mosq_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::string broker_address_;
  int broker_port_;
  std::string topic_name_;
  int qos_level_;
  std::string client_id_;
  std::string payload_;

  bool published_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MqttPublisherNode>();
  rclcpp::spin(node);
  return 0;
}
