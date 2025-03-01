#include "robo_spin_sol.h"

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RoboSpinSolution>());
    rclcpp::shutdown();
    return 0;
  }
  
  //your code here
  RoboSpinSolution::RoboSpinSolution() : Node("robo_spin_sol"), count_(0) {

    // subsription_cam = this->create_subscription<ArrayMsg>
    subscription_angle = this->create_subscription<Angle>(
        "current_angle", 10, std::bind(&RoboSpinSolution::topic_callback_angle, this, _1));

    subscription_cam = this->create_subscription<Image>(
        "robotcam", 10, std::bind(&RoboSpinSolution::topic_callback_img, this, _1));

    publisher_ = this->create_publisher<Angle>("desired_angle", 10);
    timer_ = this->create_wall_timer(100ms, std::bind(&RoboSpinSolution::timer_callback, this));
  }

  // receive angle and update curr_angle variable to be processed
  void RoboSpinSolution::topic_callback_angle(const Angle &msg)
  {
    RCLCPP_INFO(this->get_logger(), "I heard position %f", msg.data);
    curr_angle = msg;
  }

  // receive ros2 image and image process via OpenCV
  void RoboSpinSolution::topic_callback_img(const Image &msg)
  {
    cv::namedWindow(OPENCV_WINDOW);
    cv_bridge::CvImagePtr cv_ptr;

    // attempts to set cv_ptr
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, msg->encoding);
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception:");
    }
    catch (cv_bridge::Exception &e)
    {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }

    cv::imshow(OPENCV_WINDOW, cv_ptr->image);
    cv::waitKey(3);

    ros2img = cv_ptr->toImageMsg();
  }

  RoboSpinSolution::~RoboSpinSolution()
  {
    cv::destroyWindow(OPENCV_WINDOW);
  }



  void RoboSpinSolution::timer_callback()
  {
    return;
  }

