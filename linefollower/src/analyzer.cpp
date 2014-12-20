#include "analyzer.h"

#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

static const std::string OPENCV_WINDOW = "Image window";

Analyzer::Analyzer()
: it_(nh_)
{
  // Subscrive to input video feed and publish output video feed
  image_transport::TransportHints hints("compressed", ros::TransportHints());
  image_sub_ = it_.subscribe("/camera/image", 1, &Analyzer::imageCb, this, hints);

  cv::namedWindow(OPENCV_WINDOW);
}

Analyzer::~Analyzer()
{
  cv::destroyWindow(OPENCV_WINDOW);
}

void
Analyzer::imageCb(const sensor_msgs::ImageConstPtr& msg)
{
  cv_bridge::CvImageConstPtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8);
  } catch (cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  cv::Mat out1;
  cv::Mat gray_out;
  cv::Mat canny_out;
  cv::Mat gray_out1;
  cv::Mat img1;
  cv::cvtColor(cv_ptr->image, gray_out, CV_BGR2GRAY);
  cv::GaussianBlur(gray_out, gray_out, cv::Size(9, 9), 0, 0);
  cv::Canny(gray_out, canny_out, 25, 125, 3);
  cv::imshow(OPENCV_WINDOW, canny_out);
  cv::waitKey(3);
}

int
main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  Analyzer analyze;
  ros::spin();
  return 0;
}

