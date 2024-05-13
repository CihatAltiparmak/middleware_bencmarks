#ifndef _MIDDLEWARE_BENCHMARKS__TOPIC_BENCHMARKER_
#define _MIDDLEWARE_BENCHMARKS__TOPIC_BENCHMARKER_

#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <functional>

namespace rclcpp_benchmark {

class BenchMarkableNode : public rclcpp::Node {
public:
  BenchMarkableNode(const std::string & node_name) : Node(node_name){

  }

  template<typename MessageT , typename CallbackT, typename SubscriptionT = rclcpp::Subscription<MessageT>>
  std::shared_ptr<SubscriptionT>
  create_subscription (
    const std::string &,
    const rclcpp::QoS &,
    CallbackT &&);

private:
  template <typename MessageT, typename CallbackT>
  auto benchmark_callback_creator(CallbackT);
};

}

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

#endif