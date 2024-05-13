#include "topic_benchmarker/topic_benchmarker.hpp"

namespace rclcpp_benchmark {

template<typename MessageT , typename CallbackT, typename SubscriptionT = rclcpp::Subscription<MessageT>>
std::shared_ptr<SubscriptionT>
BenchMarkableNode::create_subscription (
    const std::string & topic_name,
    const rclcpp::QoS & qos,
    CallbackT && callback) {
    return Node::create_subscription<MessageT>(topic_name, qos, benchmark_callback_creator<MessageT, CallbackT>(callback));
}

template <typename MessageT, typename CallbackT>
auto BenchMarkableNode::benchmark_callback_creator(CallbackT callback) {
  return  [this, callback](MessageT msg) -> void {
    RCLCPP_INFO(this->get_logger(), "%lu", get_clock()->now().nanoseconds());
    callback(msg);
  };

}

}