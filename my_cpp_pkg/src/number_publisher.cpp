#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/msg/int64.hpp"

using namespace std::chrono_literals;
 
class NumberPublisherNode : public rclcpp::Node // MODIFY NAME
{
public:
    NumberPublisherNode() : Node("number_publisher"), counter_(2) // MODIFY NAME
    {
        publisher_ = this -> create_publisher<example_interfaces::msg::Int64>("number",10);
        timer_ = this -> create_wall_timer(0.5s, std::bind(&NumberPublisherNode::PublishNumber, this));
        RCLCPP_INFO(this -> get_logger(), "Number Publisher has started publishing");
    }
 
private:
    void PublishNumber(){
        auto msg = example_interfaces::msg::Int64();
        msg.data = std::int64_t(counter_);
        publisher_ -> publish(msg) ;
    }

    std::int64_t counter_;
    rclcpp::Publisher<example_interfaces::msg::Int64>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

};
 
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<NumberPublisherNode>(); // MODIFY NAME
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}