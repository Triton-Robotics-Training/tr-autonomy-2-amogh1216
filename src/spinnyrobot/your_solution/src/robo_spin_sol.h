#ifndef YOUR_SOLUTION_SRC_SPIN_SOL_H_
#define YOUR_SOLUTION_SRC_SPIN_SOL_H_

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include <sensor_msgs/msg/image.hpp>
#include "image_transport/image_transport.h"
#include <cv_bridge/cv_bridge.h>
#include "sensor_msgs/image_encodings.hpp"

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cmath>

using Angle = std_msgs::msg::Float32;
using Image = sensor_msgs::msg::Image::ConstSharedPtr;

using std::placeholders::_1;
using namespace std::chrono_literals;

class RoboSpinSolution : public rclcpp::Node {
 public:
  RoboSpinSolution();
 private:
    // subscriber method to receive latest angle value
    void topic_callback_angle(const Angle &msg);
    // subscriber method to receive latest image from cam
    void topic_callback_img(const Image &msg);
    // publisher method to publish to desired_angle topic
    void timer_callback();

    rclcpp::Subscription<Angle>::SharedPtr subscription_angle;
    rclcpp::Subscription<Image>::SharedPtr subscription_cam;
    rclcpp::Publisher<Angle>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    Angle curr_angle;
    Image ros2img;
    size_t count_;
};

#endif //YOUR_SOLUTION_SRC_SPIN_SOL_H_
