#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/header.hpp>

namespace composable_node_tutorials {

class HeaderPublisherNode : public rclcpp::Node {
public:
  explicit HeaderPublisherNode(const rclcpp::NodeOptions &options)
      : Node("header_publisher_node", options), count_(0) {
    publisher_ = create_publisher<std_msgs::msg::Header>("header_topic", 10);
    timer_ = create_wall_timer(
        std::chrono::seconds(1),
        std::bind(&HeaderPublisherNode::timer_callback, this));

    // Print the process ID and thread ID
    std::thread::id thread_id = std::this_thread::get_id();
    RCLCPP_INFO_STREAM(get_logger(),
                       "Process ID: " << getpid()
                                      << " Thread ID: " << thread_id);
  }

  private:
  void timer_callback() {
    auto message = std_msgs::msg::Header();
    message.stamp = now();
    message.frame_id = std::to_string(count_++);

    RCLCPP_INFO(get_logger(), "Publishing: '%s'", message.frame_id.c_str());
    publisher_->publish(message);
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::Header>::SharedPtr publisher_;
  uint16_t count_;
};

}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(composable_node_tutorials::HeaderPublisherNode);