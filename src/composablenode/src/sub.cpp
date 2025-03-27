#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/header.hpp>
#include <std_msgs/msg/u_int32.hpp>

namespace composable_node_tutorials {

    class HeaderSubscriberNode : public rclcpp::Node {
        public:
          explicit HeaderSubscriberNode(const rclcpp::NodeOptions &options)
              : Node("header_subscriber_node", options) {
            subscription_ = create_subscription<std_msgs::msg::Header>(
                "header_topic", 10,
                std::bind(&HeaderSubscriberNode::topic_callback, this,
                          std::placeholders::_1));
        
            publisher_ =
                create_publisher<std_msgs::msg::UInt32>("~/time_difference_topic", 10);
        
            // Print the process ID and thread ID
            std::thread::id thread_id = std::this_thread::get_id();
            RCLCPP_INFO_STREAM(get_logger(),
                               "Process ID: " << getpid()
                                              << " Thread ID: " << thread_id);
          }
private:
  void topic_callback(const std_msgs::msg::Header::SharedPtr msg) const {
    // Get the current time
    auto current_time = now();

    // Calculate the difference in nanoseconds
    auto message_time = rclcpp::Time(msg->stamp);
    auto time_difference = current_time - message_time;

    RCLCPP_INFO(get_logger(), "[%s] Time difference: %ld nanoseconds",
                msg->frame_id.c_str(), time_difference.nanoseconds());

    // Publish the time difference as an Int32 message
    auto time_diff_msg = std::make_unique<std_msgs::msg::UInt32>();
    time_diff_msg->data = static_cast<int32_t>(time_difference.nanoseconds());
    publisher_->publish(std::move(time_diff_msg));
  }

  rclcpp::Subscription<std_msgs::msg::Header>::SharedPtr subscription_;
  rclcpp::Publisher<std_msgs::msg::UInt32>::SharedPtr publisher_;
};

} // namespace composable_node_tutorials

// This function is needed to create a shared pointer to the node
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(
    composable_node_tutorials::HeaderSubscriberNode);