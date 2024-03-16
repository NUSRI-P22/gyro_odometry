#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist_with_covariance_stamped.hpp"
#include "serial/serial.h"
#include <cstring>
#include <iostream>

float data_value[13];
class TwistWithCovarianceNode : public rclcpp::Node
{
public:

    // Replace "/dev/ttyUSB0" with the actual serial port name
    TwistWithCovarianceNode() : Node("TwistWithCovariance"), serial_port_("/dev/ttyACM0", 115200, serial::Timeout::simpleTimeout(1000))
    {
        publisher_ = this->create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>("TwistWithCovariance", 1000/period_ms);
        
        timer_ = this->create_wall_timer(std::chrono::milliseconds(period_ms), std::bind(&TwistWithCovarianceNode::serialCallback, this));
    }

private:
    void serialCallback()
    {
        if (serial_port_.available() > 0)
        {
            geometry_msgs::msg::TwistWithCovarianceStamped msg;
            // data: "x, y, z, Q.x, Q.y, Q.z, Q.w, linear.x, linear.y, linear.z, angular.x, angular.y, angular.z\n"
            std::string data = serial_port_.readline();
            int len = data.length();
            int tot = 0;
            std::string tmp = "";

            for (int i = 0; i < len; i++) {
                if (data[i] == ' ' || data[i] == '\n') {
                    // 跳过空格和换行符
                    continue;
                }
                if (data[i] == ',') {
                    // 将 tmp 转换为浮点数并存储到 data_value 数组中
                    try {
                        data_value[tot++] = std::stof(tmp);
                    } catch (const std::invalid_argument& e) {
                        std::cerr << "Error converting string to float: " << e.what() << std::endl;
                        // 处理无效的浮点数表示
                    }
                    tmp = "";
                } else {
                    tmp += data[i];
                }
            }

            // // 检查是否成功解析了13个值
            // if (tot != 12) {
            //     std::cerr << "Error: Expected 13 values, but found " << tot << std::endl;
            // } 
            // std::cout << data << std::endl;

            msg.header.stamp = rclcpp::Clock{}.now();
            msg.header.frame_id = "TwistWithCovariance";
            msg.twist.twist.linear.x = data_value[7];
            msg.twist.twist.linear.y = data_value[8];
            msg.twist.twist.linear.z = data_value[9];
            msg.twist.twist.angular.x = data_value[10];
            msg.twist.twist.angular.y = data_value[11];
            msg.twist.twist.angular.z = data_value[12];
            publisher_->publish(msg);
        }
    }

    serial::Serial serial_port_;
    rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    int period_ms = 20; // pubilish period
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TwistWithCovarianceNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
