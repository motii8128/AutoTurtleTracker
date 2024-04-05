#include <rclcpp/rclcpp.hpp>
#include <turtlesim/msg/pose.hpp>
#include <nav_msgs/msg/odometry.hpp>

class SubClass : public rclcpp::Node
{
public:
    SubClass() : Node("sub_node")
    {
        using namespace std::placeholders;
        subscription_ = this->create_subscription<turtlesim::msg::Pose>("/pose", 10, std::bind(&SubClass::topic_callback, this, _1));
        publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 0);
    }

private:
    void topic_callback(const turtlesim::msg::Pose::SharedPtr msg) const
    {
        auto send_msg = nav_msgs::msg::Odometry();

        send_msg.pose.pose.position.x = msg->x;
        send_msg.pose.pose.position.y = msg->y;
        send_msg.pose.pose.position.z = msg->theta;
        send_msg.twist.twist.linear.x = msg->linear_velocity;
        send_msg.twist.twist.angular.x = msg->angular_velocity;

        publisher_->publish(send_msg);
    }

    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr subscription_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publisher_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SubClass>());
    rclcpp::shutdown();
    return 0;
}