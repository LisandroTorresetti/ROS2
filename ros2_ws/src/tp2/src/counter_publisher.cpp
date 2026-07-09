#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "common.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_srvs/srv/empty.hpp"

using namespace std::chrono_literals;

static const std::string nodeName = "counter_publisher";

class CounterPublisher : public rclcpp::Node {
public:
    CounterPublisher(): Node(nodeName) {
        publisher_ = this->create_publisher<std_msgs::msg::Int32>(COUNTER_TOPIC_NAME, 10);
        timer_ = this->create_wall_timer(200ms, std::bind(&CounterPublisher::timer_callback, this)); // TODO: make frequency updatable
        count_ = 0;

        reset_service_ = this->create_service<std_srvs::srv::Empty>(
            "reset_counter",
            std::bind(&CounterPublisher::reset_counter_callback, this, std::placeholders::_1, std::placeholders::_2));
    }

private:
    void timer_callback() {
        auto message = std_msgs::msg::Int32();
        message.data = count_++;

        RCLCPP_INFO(this->get_logger(), "Publishing counter value: %d", count_ - 1);
        publisher_->publish(message);
    }

    void reset_counter_callback(
    const std::shared_ptr<std_srvs::srv::Empty::Request> request,
    std::shared_ptr<std_srvs::srv::Empty::Response> response) {
        (void)request;
        (void)response;

        count_ = 0;

        RCLCPP_INFO(this->get_logger(), "Counter reset.");
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr publisher_;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reset_service_;
    int32_t count_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CounterPublisher>());
    rclcpp::shutdown();
    return 0;
}
