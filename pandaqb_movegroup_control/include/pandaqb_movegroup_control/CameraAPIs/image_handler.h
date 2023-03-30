#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <pandaqb_movegroup_control/RequestGrasp.h>

class ImageHandler{
public:
    ImageHandler();
    ~ImageHandler();

    void set_ros_image(std::string image_path);
    sensor_msgs::Image get_ros_image();


private:
    bool set_cv_image(std::string image_path);
    sensor_msgs::ImagePtr cvMatToROSImage(cv::Mat& cv_image);

    sensor_msgs::ImagePtr ros_img;
    cv::Mat cv_image;
};