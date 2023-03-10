#include <pandaqb_movegroup_control/CameraAPIs/image_handler.h>

ImageHandler::ImageHandler(){
};

ImageHandler::~ImageHandler(){
};

// Take the path of an image in a directory and turn it to Image ros msg
void ImageHandler::set_ros_image(std::string image_path){
    if (set_cv_image(image_path)){
        ros_img = cvMatToROSImage(cv_image);
    }
}

sensor_msgs::Image ImageHandler::get_ros_image(){
    return *ros_img;
}


// Take an opencv loaded image and turn it to IMage ros msg
sensor_msgs::ImagePtr ImageHandler::cvMatToROSImage(cv::Mat& cv_image){
    sensor_msgs::ImagePtr ros_image = boost::make_shared<sensor_msgs::Image>();
    ros_image->header.stamp = ros::Time::now();
    ros_image->header.frame_id = "camera_frame"; // Replace with frame ID

    ros_image->height = cv_image.rows;
    ros_image->width = cv_image.cols;
    ros_image->encoding = "bgr8";
    ros_image->step = cv_image.cols * cv_image.elemSize();
    size_t size = cv_image.rows * ros_image->step;
    ros_image->data.resize(size);

    memcpy(&ros_image->data[0], cv_image.data, size);

    return ros_image;
}



bool ImageHandler::set_cv_image(std::string image_path){
    cv_image = cv::imread(image_path);

    if (cv_image.empty()) {
        ROS_INFO_STREAM("Could not open or find the image, wrong path: " << image_path);
        return false;
    }

    else{
        return true;     
    }
}