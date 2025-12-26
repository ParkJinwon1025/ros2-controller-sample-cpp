#include <chrono>
#include <memory>
#include <string>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "test_interfaces/msg/position.hpp"
#include "test_interfaces/srv/calculate_distance.hpp"
#include "test_interfaces/action/move_to.hpp"

using namespace std::chrono_literals;
using CalculateDistance = test_interfaces::srv::CalculateDistance;
using MoveTo = test_interfaces::action::MoveTo;
using GoalHandleMoveTo = rclcpp_action::ClientGoalHandle<MoveTo>;

class NodeA : public rclcpp::Node
{
public:
    NodeA() : Node("node_a"), x_(0.0), y_(0.0), z_(0.0), angle_(0.0)
    {
        topic_publisher_ = this->create_publisher<test_interfaces::msg::Position>(
            "position_topic", 10);
        
        topic_subscriber_ = this->create_subscription<test_interfaces::msg::Position>(
            "position_topic", 10,
            std::bind(&NodeA::topic_callback, this, std::placeholders::_1));
        
        service_client_ = this->create_client<CalculateDistance>("calculate_distance");
        
        action_client_ = rclcpp_action::create_client<MoveTo>(this, "move_to_action");
        
        // 타이머 주석 처리 - 자동 발행 끄기
        // topic_timer_ = this->create_wall_timer(
        //     2s, std::bind(&NodeA::publish_position, this));
        
        // service_timer_ = this->create_wall_timer(
        //     5s, std::bind(&NodeA::call_service, this));
        
        // action_timer_ = this->create_wall_timer(
        //     10s, std::bind(&NodeA::send_goal, this));
        
        RCLCPP_INFO(this->get_logger(), "Node A started (manual mode)");
    }

private:
    void publish_position()
    {
        static double t = 0.0;
        x_ = 5.0 * cos(t);
        y_ = 5.0 * sin(t);
        z_ = t * 0.1;
        angle_ = t * 180.0 / M_PI;
        t += 0.2;
        
        auto message = test_interfaces::msg::Position();
        message.sender = "NodeA";
        message.x = x_;
        message.y = y_;
        message.z = z_;
        message.angle = angle_;
        message.timestamp = this->now().nanoseconds();
        
        RCLCPP_INFO(this->get_logger(), 
            "[TOPIC] Position: x=%.2f, y=%.2f, z=%.2f, angle=%.2f", 
            x_, y_, z_, angle_);
        
        topic_publisher_->publish(message);
    }
    
    void topic_callback(const test_interfaces::msg::Position::SharedPtr msg)
    {
        if (msg->sender != "NodeA") {
            RCLCPP_INFO(this->get_logger(), 
                "[TOPIC] Received from %s: x=%.2f, y=%.2f, z=%.2f, angle=%.2f", 
                msg->sender.c_str(), msg->x, msg->y, msg->z, msg->angle);
        }
    }
    
    void call_service()
    {
        if (!service_client_->wait_for_service(1s)) {
            RCLCPP_WARN(this->get_logger(), "Service server not available");
            return;
        }
        
        auto request = std::make_shared<CalculateDistance::Request>();
        request->x1 = x_;
        request->y1 = y_;
        request->z1 = z_;
        request->x2 = 10.0;
        request->y2 = 10.0;
        request->z2 = 5.0;
        
        RCLCPP_INFO(this->get_logger(), 
            "[SERVICE] Request: (%.2f,%.2f,%.2f) to (%.2f,%.2f,%.2f)", 
            request->x1, request->y1, request->z1,
            request->x2, request->y2, request->z2);
        
        service_client_->async_send_request(request,
            [this](rclcpp::Client<CalculateDistance>::SharedFuture future) {
                auto response = future.get();
                RCLCPP_INFO(this->get_logger(), 
                    "[SERVICE] Response: %s (distance: %.2fm)", 
                    response->message.c_str(), response->distance);
            });
    }
    
    void send_goal()
    {
        if (!action_client_->wait_for_action_server(1s)) {
            RCLCPP_WARN(this->get_logger(), "Action server not available");
            return;
        }
        
        auto goal_msg = MoveTo::Goal();
        goal_msg.target_x = 10.0;
        goal_msg.target_y = 5.0;
        goal_msg.target_z = 2.0;
        goal_msg.target_angle = 90.0;
        
        RCLCPP_INFO(this->get_logger(), 
            "[ACTION] Sending goal: (%.2f, %.2f, %.2f, %.2f)", 
            goal_msg.target_x, goal_msg.target_y, 
            goal_msg.target_z, goal_msg.target_angle);
        
        auto send_goal_options = rclcpp_action::Client<MoveTo>::SendGoalOptions();
        
        send_goal_options.feedback_callback =
            [this](GoalHandleMoveTo::SharedPtr,
                   const std::shared_ptr<const MoveTo::Feedback> feedback) {
                RCLCPP_INFO(this->get_logger(), 
                    "[ACTION] Feedback: (%.2f, %.2f, %.2f, %.2f) - %.1f%%", 
                    feedback->current_x, feedback->current_y, 
                    feedback->current_z, feedback->current_angle,
                    feedback->progress_percent);
            };
        
        send_goal_options.result_callback =
            [this](const GoalHandleMoveTo::WrappedResult & result) {
                RCLCPP_INFO(this->get_logger(), 
                    "[ACTION] Result: %s - final(%.2f, %.2f, %.2f, %.2f)", 
                    result.result->status.c_str(),
                    result.result->final_x, result.result->final_y,
                    result.result->final_z, result.result->final_angle);
            };
        
        action_client_->async_send_goal(goal_msg, send_goal_options);
    }
    
    rclcpp::Publisher<test_interfaces::msg::Position>::SharedPtr topic_publisher_;
    rclcpp::Subscription<test_interfaces::msg::Position>::SharedPtr topic_subscriber_;
    rclcpp::Client<CalculateDistance>::SharedPtr service_client_;
    rclcpp_action::Client<MoveTo>::SharedPtr action_client_;
    // rclcpp::TimerBase::SharedPtr topic_timer_;      // 주석 처리
    // rclcpp::TimerBase::SharedPtr service_timer_;    // 주석 처리
    // rclcpp::TimerBase::SharedPtr action_timer_;     // 주석 처리
    double x_, y_, z_, angle_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<NodeA>());
    rclcpp::shutdown();
    return 0;
}
