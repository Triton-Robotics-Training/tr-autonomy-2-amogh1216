#include "robo_spin_sol.h"

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RoboSpinSolution>());
    rclcpp::shutdown();
    return 0;
}
  
  RoboSpinSolution::RoboSpinSolution() : Node("robo_spin_sol"), count_(0) {
    
    cv::namedWindow(OPENCV_WINDOW);

    subscription_cam = this->create_subscription<Image>("robotcam", 10,
        std::bind(&RoboSpinSolution::topic_callback_img, this, _1));

    subscription_angle = this->create_subscription<Angle>(
        "current_angle", 10, std::bind(&RoboSpinSolution::topic_callback_angle, this, _1));

    publisher_ = this->create_publisher<Angle>("desired_angle", 10);
    timer_ = this->create_wall_timer(100ms, std::bind(&RoboSpinSolution::timer_callback, this));
  }

  // receive angle and update curr_angle variable to be processed
  void RoboSpinSolution::topic_callback_angle(const Angle &msg)
  {
    RCLCPP_INFO(this->get_logger(), "I heard position %f", msg.data);
    curr_angle.data = msg.data;
  }

  // receive ros2 image, convert to open cv
  void RoboSpinSolution::topic_callback_img(const Image::SharedPtr msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, msg->encoding);
    }
    catch (cv_bridge::Exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }

    process_image(cv_ptr->image);
    cv::waitKey(3);
  }

  // process image to find center of 
  void RoboSpinSolution::process_image(cv::Mat &cv_img) {
    cv::Mat hsv_img, mask1, mask2, red_mask, result;
    
    // Convert BGR image to HSV
    cv::cvtColor(cv_img, hsv_img, cv::COLOR_BGR2HSV);

    // Define lower and upper bounds for red color in HSV space
    cv::Scalar lower_red1(0, 120, 70), upper_red1(10, 255, 255);
    cv::Scalar lower_red2(170, 120, 70), upper_red2(180, 255, 255);

    // Create masks for both red ranges
    cv::inRange(hsv_img, lower_red1, upper_red1, mask1);
    cv::inRange(hsv_img, lower_red2, upper_red2, mask2);

    // Combine both masks
    red_mask = mask1 | mask2;

    // Apply morphological operations to remove noise
    cv::erode(red_mask, red_mask, cv::Mat(), cv::Point(-1, -1), 2);
    cv::dilate(red_mask, red_mask, cv::Mat(), cv::Point(-1, -1), 2);

    // Find contours of the red cube
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(red_mask, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    if (!contours.empty()) {
        // Find the largest contour based on area
        double max_area = 0;
        int max_index = -1;
        for (size_t i = 0; i < contours.size(); i++) {
            double area = cv::contourArea(contours[i]);
            if (area > max_area) {
                max_area = area;
                max_index = i;
            }
        }

        // Compute the centroid of the largest contour
        if (max_index != -1) {
            cv::Moments M = cv::moments(contours[max_index]);
            if (M.m00 != 0) {  // To avoid division by zero
                int cx = static_cast<int>(M.m10 / M.m00);
                int cy = static_cast<int>(M.m01 / M.m00);

                // Draw the contour and centroid on the image
                cv::drawContours(cv_img, contours, max_index, cv::Scalar(0, 255, 0), 2); // Green contour
                cv::circle(cv_img, cv::Point(cx, cy), 5, cv::Scalar(255, 0, 0), -1); // Blue dot for center

                // negative means the cube is left, positive means the cube is right
                dist = cx-360;
                // Log the detected position
                RCLCPP_INFO(this->get_logger(), "Red blob detected at: (%d, %d)", cx, cy);
            }
        }
    }
    // if for any reason the camera isn't viewing the red cube, just keep looking
    else {
        dist = 360;
    }
    
    // Show the processed image
    cv::imshow(OPENCV_WINDOW, cv_img);
    cv::waitKey(3);
}


  void RoboSpinSolution::timer_callback()
  {
    
    Angle msg;
    try {

        // P controller to move camera
        float32_t P = -0.001f;
        // left => positive, right => negative
        float32_t increment = dist * P;
        // once we're close enough no point in moving
        if (abs(dist) < 5) increment = 0;
        msg.data = curr_angle.data + increment;
        RCLCPP_INFO(this->get_logger(), "Publishing desired angle: %f", msg.data);
        publisher_->publish(msg);
        return;
    } catch(...) {
      RCLCPP_WARN(this->get_logger(), "Error while publishing angle");
    }
  }


  RoboSpinSolution::~RoboSpinSolution()
  {
    try {
        cv::destroyWindow(OPENCV_WINDOW);
    } catch (...) {
        RCLCPP_WARN(this->get_logger(), "Attempted to destroy a non-existent OpenCV window.");
    }
  }



