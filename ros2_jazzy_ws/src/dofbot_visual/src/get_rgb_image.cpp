//
// Created by yahboom on 2021/4/29.
//

#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/highgui/highgui.hpp>

using namespace std;
using namespace cv;

class AstraRgbImageNode : public rclcpp::Node {
public:
    AstraRgbImageNode() : Node("astra_rgb_image_cpp") {
        subscriber_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/usb_cam/image_raw",
            10,
            std::bind(&AstraRgbImageNode::rgb_callback, this, std::placeholders::_1));
    }

private:
    void rgb_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            imshow("color_image", cv_ptr->image);
            waitKey(1);
        } catch (const cv_bridge::Exception &e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscriber_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<AstraRgbImageNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

