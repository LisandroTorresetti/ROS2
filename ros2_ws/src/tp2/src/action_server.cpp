#include "../../../build/tp2/rosidl_generator_cpp/tp2/action/detail/split_text__struct.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

using namespace std::placeholders;

static const std::string node_name = "action_server";
static const std::string action_server_endpoint = "split_text";

class TextSplitterService : public rclcpp::Node {
    public:
    using SplitText = tp2::action::SplitText;
    using GoalHandleSplitText = rclcpp_action::ServerGoalHandle<SplitText>;

    TextSplitterService() : Node(node_name) {
        this->action_server_ = rclcpp_action::create_server<SplitText>(
            this,
            action_server_endpoint,
            std::bind(&TextSplitterService::handle_goal, this, _1, _2),
            std::bind(&TextSplitterService::handle_cancel, this, _1),
            std::bind(&TextSplitterService::handle_accepted, this, _1));
    }

private:
    rclcpp_action::Server<SplitText>::SharedPtr action_server_;

    rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const SplitText::Goal> goal) {
        RCLCPP_INFO(this->get_logger(), "Text to be split: %s", goal->text.c_str());
        (void)uuid;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleSplitText> goal_handle) {
        RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(const std::shared_ptr<GoalHandleSplitText> goal_handle) {
        using namespace std::placeholders;
        // this needs to return quickly to avoid blocking the executor, so spin up a new thread
        std::thread{std::bind(&TextSplitterService::execute, this, _1), goal_handle}.detach();
    }

    void execute(const std::shared_ptr<GoalHandleSplitText> goal_handle) {
        RCLCPP_INFO(this->get_logger(), "Executing goal");

        rclcpp::Rate loop_rate(1); // 1Hz

        auto goal = goal_handle->get_goal();
        auto feedback = std::make_shared<SplitText::Feedback>();
        auto result = std::make_shared<SplitText::Result>();

        std::stringstream ss(goal->text);
        std::string word;

        while (ss >> word) {
            if (goal_handle->is_canceling()) {
                result->eof_msg = "Action canceled";
                goal_handle->canceled(result);
                return;
            }

            feedback->word = word;
            goal_handle->publish_feedback(feedback);

            RCLCPP_INFO(this->get_logger(),"Feedback: %s", word.c_str());
            loop_rate.sleep();
        }

        result->eof_msg = "Texto republicado!";
        goal_handle->succeed(result);
    }
};


int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TextSplitterService>();
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}