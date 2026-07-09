#include <functional>
#include <memory>

#include "common.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_srvs/srv/empty.hpp"

using std::placeholders::_1;

static const std::string nodeName = "counter_subscriber";
static const std::string max_counter_value_param_name = "max_counter_value";

class CounterSubscriber : public rclcpp::Node {

    public:
    CounterSubscriber(): Node(nodeName) {
        subscription_ = this->create_subscription<std_msgs::msg::Int32>(
          COUNTER_TOPIC_NAME, 10, std::bind(&CounterSubscriber::topic_callback, this, _1));

        reset_client_ = this->create_client<std_srvs::srv::Empty>(RESET_ENDPOINT);

        this->declare_parameter<int32_t>(max_counter_value_param_name, 25);
        max_counter_value_ = this->get_parameter(max_counter_value_param_name).as_int();

        RCLCPP_INFO(this->get_logger(), "Max counter value is: %d", max_counter_value_);

        counter_ = 0;
    }

private:
    void topic_callback(const std_msgs::msg::Int32 & msg) {
        RCLCPP_INFO(this->get_logger(), "Message received: %d", msg.data);

        counter_ = msg.data;
        if (counter_ == max_counter_value_) {
            // fire & forget
            auto request = std::make_shared<std_srvs::srv::Empty::Request>();
            reset_client_->async_send_request(request);
        }
    }

    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr subscription_;
    rclcpp::Client<std_srvs::srv::Empty>::SharedPtr reset_client_;

    int32_t counter_;
    int32_t max_counter_value_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CounterSubscriber>());
    rclcpp::shutdown();
    return 0;
}
