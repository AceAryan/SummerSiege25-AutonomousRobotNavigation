#include "rclcpp/rclcpp.hpp"
#include "turtlesim/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <cmath>

class TurtleChase : public rclcpp::Node
{
public:
    TurtleChase()
    : Node("turtle_chase")
    {
        // Subscriptions
        sub_turtle1_pose_ = this->create_subscription<turtlesim::msg::Pose>(
            "/turtle1/pose", 10,
            std::bind(&TurtleChase::turtle1_callback, this, std::placeholders::_1));

        sub_turtle2_pose_ = this->create_subscription<turtlesim::msg::Pose>(
            "/turtle2/pose", 10,
            std::bind(&TurtleChase::turtle2_callback, this, std::placeholders::_1));

        // Publisher
        pub_cmd_vel_ = this->create_publisher<geometry_msgs::msg::Twist>("/turtle2/cmd_vel", 10);

        // Timer for control loop
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&TurtleChase::control_loop, this));
    }

private:
    // Pose callbacks
    void turtle1_callback(const turtlesim::msg::Pose::SharedPtr msg)
    {
        turtle1_pose_ = *msg;
        got_turtle1_ = true;
    }

    void turtle2_callback(const turtlesim::msg::Pose::SharedPtr msg)
    {
        turtle2_pose_ = *msg;
        got_turtle2_ = true;
    }

    // Main control loop
    void control_loop()
    {
        if (!got_turtle1_ || !got_turtle2_)
            return;

        float dx = turtle1_pose_.x - turtle2_pose_.x;
        float dy = turtle1_pose_.y - turtle2_pose_.y;

        float distance = std::sqrt(dx * dx + dy * dy);
        float angle_to_target = std::atan2(dy, dx);
        float angle_diff = angle_to_target - turtle2_pose_.theta;

        // Normalize angle to [-pi, pi]
        angle_diff = std::atan2(std::sin(angle_diff), std::cos(angle_diff));

        geometry_msgs::msg::Twist cmd_vel;
        cmd_vel.linear.x = std::min(2.0f, distance);
        cmd_vel.angular.z = 4.0f * angle_diff;

        pub_cmd_vel_->publish(cmd_vel);
    }

    // Subscribers and Publisher
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr sub_turtle1_pose_;
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr sub_turtle2_pose_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd_vel_;
    rclcpp::TimerBase::SharedPtr timer_;

    // State
    turtlesim::msg::Pose turtle1_pose_;
    turtlesim::msg::Pose turtle2_pose_;
    bool got_turtle1_ = false;
    bool got_turtle2_ = false;
};
 
int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TurtleChase>());
    rclcpp::shutdown();
    return 0;
}
