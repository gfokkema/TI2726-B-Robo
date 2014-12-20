#include "analyzer.h"

#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

static const std::string OPENCV_WINDOW = "Image window";

Analyzer::Analyzer()
: it_(nh_),
  canny_kernel(3), canny_ratio(4),
  canny_min(30), canny_max(100),
  gaussian_min(3), gaussian_max(15),
  hough_min(20), hough_max(100),
  hough_line_min(20), hough_line_max(100),
  hough_gap_min(10), hough_gap_max(50)
{
  // Subscrive to input video feed and publish output video feed
  image_transport::TransportHints hints("compressed", ros::TransportHints());
  image_sub_ = it_.subscribe("/camera/image", 1, &Analyzer::imageCb, this, hints);

  cv::namedWindow(OPENCV_WINDOW);
  cv::createTrackbar("Gaussian: ", OPENCV_WINDOW, &gaussian_min, gaussian_max);
  cv::createTrackbar("Canny edge min: ", OPENCV_WINDOW, &canny_min, canny_max);
  cv::createTrackbar("Hough min: ", OPENCV_WINDOW, &hough_min, hough_max);
  cv::createTrackbar("Hough line min: ", OPENCV_WINDOW, &hough_line_min, hough_line_max);
  cv::createTrackbar("Hough gap min: ", OPENCV_WINDOW, &hough_gap_min, hough_gap_max);
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

  cv::Mat gray_out;
  cv::Mat canny_out;
  cv::Mat cdst;
  std::vector<cv::Vec4i> lines;

  cv::cvtColor(cv_ptr->image, gray_out, CV_BGR2GRAY);
  cv::GaussianBlur(gray_out, gray_out, cv::Size(gaussian_min, gaussian_min), 0, 0);
  cv::Canny(gray_out, canny_out, canny_min, canny_min * canny_ratio, canny_kernel);
  cv::HoughLinesP(canny_out, lines, 1, CV_PI / 180, 50, 50, 10);

  cv::cvtColor(canny_out, cdst, CV_GRAY2BGR);
  for (size_t i = 0; i < lines.size(); i++) {
    cv::Vec4i l = lines[i];
    cv::line(cdst, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]), cv::Scalar(0,0,255), 3);
  }

  cv::imshow(OPENCV_WINDOW, cdst);
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

