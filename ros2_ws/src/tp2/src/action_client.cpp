#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "tp2/action/split_text.hpp"
//#include "../../../build/tp2/rosidl_generator_cpp/tp2/action/detail/split_text__struct.hpp"

using namespace std::placeholders;

static const std::string node_name = "action_client";
static const std::string action_server_endpoint = "split_text";
static const std::string text_param_name = "text";

class TextSplitterClient: public rclcpp::Node {
public:
    using SplitText = tp2::action::SplitText;
    using GoalHandleSplitText = rclcpp_action::ClientGoalHandle<SplitText>;

    TextSplitterClient(): Node(node_name) {
        this->declare_parameter<std::string>(text_param_name, "hola que tal tu como estas? dime si eres feliz");
        text_ = this->get_parameter(text_param_name).as_string();


        this->client_ptr_ = rclcpp_action::create_client<SplitText>(this, action_server_endpoint);

        this->timer_ = this->create_wall_timer(
            std::chrono::milliseconds(1000),
            std::bind(&TextSplitterClient::send_goal, this));
    }

private:
    rclcpp_action::Client<SplitText>::SharedPtr client_ptr_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::string text_;

    void send_goal() {
        using namespace std::placeholders;

        this->timer_->cancel();

        if (!this->client_ptr_->wait_for_action_server()) {
            RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
            rclcpp::shutdown();
            return;
        }

        SplitText::Goal goal;
        goal.text = text_;
        RCLCPP_INFO(this->get_logger(), "Republicando: %s", goal.text.c_str());

        auto send_goal_options = rclcpp_action::Client<SplitText>::SendGoalOptions();

        send_goal_options.goal_response_callback = std::bind(&TextSplitterClient::goal_response_callback, this, _1);
        send_goal_options.feedback_callback = std::bind(&TextSplitterClient::feedback_callback, this, _1, _2);
        send_goal_options.result_callback = std::bind(&TextSplitterClient::result_callback, this, _1);

        this->client_ptr_->async_send_goal(goal, send_goal_options);
    }

    void goal_response_callback(
        GoalHandleSplitText::SharedPtr goal_handle)
    {
        if (!goal_handle) {
            RCLCPP_ERROR(this->get_logger(), "Goal rejected.");
        } else {
            RCLCPP_INFO(this->get_logger(), "Goal accepted.");
        }
    }

    void feedback_callback(
        GoalHandleSplitText::SharedPtr,
        const std::shared_ptr<const SplitText::Feedback>)
    {
    }

    void result_callback(
        const GoalHandleSplitText::WrappedResult & result)
    {
        switch (result.code)
        {
            case rclcpp_action::ResultCode::SUCCEEDED:
                RCLCPP_INFO(
                    this->get_logger(),
                    "%s",
                    result.result->eof_msg.c_str());
                break;

            case rclcpp_action::ResultCode::ABORTED:
                RCLCPP_ERROR(this->get_logger(), "Goal aborted.");
                break;

            case rclcpp_action::ResultCode::CANCELED:
                RCLCPP_WARN(this->get_logger(), "Goal canceled.");
                break;

            default:
                RCLCPP_ERROR(this->get_logger(), "Unknown result.");
                break;
        }

        rclcpp::shutdown();
    }
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TextSplitterClient>();
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}