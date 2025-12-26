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
using GoalHandleMoveTo = rclcpp_action::ServerGoalHandle<MoveTo>;

class NodeB : public rclcpp::Node
{
public:
    NodeB() : Node("node_b"), x_(0.0), y_(0.0), z_(0.0), angle_(0.0)
    {
        topic_publisher_ = this->create_publisher<test_interfaces::msg::Position>(
            "position_topic", 10);
        
        topic_subscriber_ = this->create_subscription<test_interfaces::msg::Position>(
            "position_topic", 10,
            std::bind(&NodeB::topic_callback, this, std::placeholders::_1));
        
        service_server_ = this->create_service<CalculateDistance>(
            "calculate_distance",
            std::bind(&NodeB::handle_service, this, 
                std::placeholders::_1, std::placeholders::_2));
        
        action_server_ = rclcpp_action::create_server<MoveTo>(
            this,
            "move_to_action",
            std::bind(&NodeB::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&NodeB::handle_cancel, this, std::placeholders::_1),
            std::bind(&NodeB::handle_accepted, this, std::placeholders::_1));
        
        // 타이머 주석 처리 - 자동 발행 끄기
        // topic_timer_ = this->create_wall_timer(
        //     3s, std::bind(&NodeB::publish_position, this));
        
        RCLCPP_INFO(this->get_logger(), "Node B started (manual mode)");
    }

private:
    void publish_position()
    {
        static double t = 0.0;
        x_ = t;
        y_ = t * 0.5;
        z_ = 1.0;
        angle_ = 45.0;
        t += 1.0;
        
        auto message = test_interfaces::msg::Position();
        message.sender = "NodeB";
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
        if (msg->sender != "NodeB") {
            RCLCPP_INFO(this->get_logger(), 
                "[TOPIC] Received from %s: x=%.2f, y=%.2f, z=%.2f, angle=%.2f", 
                msg->sender.c_str(), msg->x, msg->y, msg->z, msg->angle);
        }
    }
    
    void handle_service(
        const std::shared_ptr<CalculateDistance::Request> request,
        std::shared_ptr<CalculateDistance::Response> response)
    {
        RCLCPP_INFO(this->get_logger(), 
            "[SERVICE] Request received: (%.2f,%.2f,%.2f) to (%.2f,%.2f,%.2f)", 
            request->x1, request->y1, request->z1,
            request->x2, request->y2, request->z2);
        
        double dx = request->x2 - request->x1;
        double dy = request->y2 - request->y1;
        double dz = request->z2 - request->z1;
        
        response->distance = sqrt(dx*dx + dy*dy + dz*dz);
        response->message = "Distance calculated";
        
        RCLCPP_INFO(this->get_logger(), 
            "[SERVICE] Response: %.2fm", response->distance);
    }
    
    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const MoveTo::Goal> goal)
    {
        RCLCPP_INFO(this->get_logger(), 
            "[ACTION] Goal received: (%.2f, %.2f, %.2f, %.2f)", 
            goal->target_x, goal->target_y, goal->target_z, goal->target_angle);
        (void)uuid;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }
    
    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandleMoveTo> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "[ACTION] Cancel requested");
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }
    
    void handle_accepted(const std::shared_ptr<GoalHandleMoveTo> goal_handle)
    {
        std::thread{std::bind(&NodeB::execute, this, std::placeholders::_1), goal_handle}.detach();
    }
    
    void execute(const std::shared_ptr<GoalHandleMoveTo> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "[ACTION] Executing");
        
        const auto goal = goal_handle->get_goal();
        auto feedback = std::make_shared<MoveTo::Feedback>();
        auto result = std::make_shared<MoveTo::Result>();
        
        double start_x = x_;
        double start_y = y_;
        double start_z = z_;
        double start_angle = angle_;
        
        int steps = 10;
        for (int i = 0; i <= steps; ++i) {
            if (goal_handle->is_canceling()) {
                result->status = "Canceled";
                result->final_x = x_;
                result->final_y = y_;
                result->final_z = z_;
                result->final_angle = angle_;
                goal_handle->canceled(result);
                return;
            }
            
            double progress = static_cast<double>(i) / steps;
            
            x_ = start_x + (goal->target_x - start_x) * progress;
            y_ = start_y + (goal->target_y - start_y) * progress;
            z_ = start_z + (goal->target_z - start_z) * progress;
            angle_ = start_angle + (goal->target_angle - start_angle) * progress;
            
            feedback->current_x = x_;
            feedback->current_y = y_;
            feedback->current_z = z_;
            feedback->current_angle = angle_;
            feedback->progress_percent = progress * 100.0;
            
            goal_handle->publish_feedback(feedback);
            
            RCLCPP_INFO(this->get_logger(), 
                "[ACTION] Progress: %.1f%% (%.2f, %.2f, %.2f, %.2f)", 
                progress * 100.0, x_, y_, z_, angle_);
            
            std::this_thread::sleep_for(500ms);
        }
        
        result->status = "Completed";
        result->final_x = x_;
        result->final_y = y_;
        result->final_z = z_;
        result->final_angle = angle_;
        goal_handle->succeed(result);
        
        RCLCPP_INFO(this->get_logger(), "[ACTION] Completed");
    }
    
    rclcpp::Publisher<test_interfaces::msg::Position>::SharedPtr topic_publisher_;
    rclcpp::Subscription<test_interfaces::msg::Position>::SharedPtr topic_subscriber_;
    rclcpp::Service<CalculateDistance>::SharedPtr service_server_;
    rclcpp_action::Server<MoveTo>::SharedPtr action_server_;
    // rclcpp::TimerBase::SharedPtr topic_timer_;  // 주석 처리
    double x_, y_, z_, angle_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<NodeB>());
    rclcpp::shutdown();
    return 0;
}