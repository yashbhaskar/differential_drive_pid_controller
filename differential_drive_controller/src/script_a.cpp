#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/float64.hpp"

class WheelRPMCalculator : public rclcpp::Node {
public:
    WheelRPMCalculator() : Node("wheel_rpm_calculator") {
        // Declare ROS 2 parameters
        this->declare_parameter("wheelbase", 0.19);    // Distance between wheels (m)
        this->declare_parameter("wheel_radius", 0.03); // Radius of wheels (m)
        this->declare_parameter("max_rpm", 100.0);     // Max RPM limit

        // Get parameter values
        wheelbase_ = this->get_parameter("wheelbase").as_double();
        wheel_radius_ = this->get_parameter("wheel_radius").as_double();
        max_rpm_ = this->get_parameter("max_rpm").as_double();

        // Subscribers & Publishers
        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10, std::bind(&WheelRPMCalculator::cmd_vel_callback, this, std::placeholders::_1));
        left_wheel_pub_ = this->create_publisher<std_msgs::msg::Float64>("/left_wheel_rpm", 10);
        right_wheel_pub_ = this->create_publisher<std_msgs::msg::Float64>("/right_wheel_rpm", 10);
    }

private:
    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        double v = msg->linear.x;  // Linear velocity (m/s)
        double w = msg->angular.z; // Angular velocity (rad/s)

        // Compute wheel velocities (m/s)
        double v_left = v - (wheelbase_ / 2.0) * w;
        double v_right = v + (wheelbase_ / 2.0) * w;

        // Convert to RPM: RPM = (velocity / circumference) * 60
        double rpm_left = (v_left / (2 * M_PI * wheel_radius_)) * 60.0;
        double rpm_right = (v_right / (2 * M_PI * wheel_radius_)) * 60.0;

        // Clamp RPM values for safety
        rpm_left = std::clamp(rpm_left, -max_rpm_, max_rpm_);
        rpm_right = std::clamp(rpm_right, -max_rpm_, max_rpm_);

        // Publish RPM values
        std_msgs::msg::Float64 left_msg, right_msg;
        left_msg.data = rpm_left;
        right_msg.data = rpm_right;
        left_wheel_pub_->publish(left_msg);
        right_wheel_pub_->publish(right_msg);

        // Debug logs
        RCLCPP_INFO(this->get_logger(), "Left RPM: %.2f, Right RPM: %.2f", rpm_left, rpm_right);
    }

    // Parameters
    double wheelbase_;
    double wheel_radius_;
    double max_rpm_;

    // ROS 2 Subscriptions & Publishers
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr left_wheel_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr right_wheel_pub_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WheelRPMCalculator>());
    rclcpp::shutdown();
    return 0;
}
