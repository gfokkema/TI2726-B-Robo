#include "analyzer.h"

#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

static const std::string OPENCV_HOUGH = "Hough transform";
static const std::string OPENCV_HOUGH_UTIL = "Hough utils";
static const std::string OPENCV_CONTOUR = "Contour detection";

Analyzer::Analyzer()
: it_(nh_),
  canny_kernel(3), canny_ratio(4),
  canny_min(50), canny_max(100),
  gaussian_min(3), gaussian_max(9),
  hough_min(50), hough_max(100),
  hough_line_min(50), hough_line_max(100),
  hough_gap_min(10), hough_gap_max(20)
{
  // Subscrive to input video feed and publish output video feed
  image_transport::TransportHints hints("compressed", ros::TransportHints());
  image_sub_ = it_.subscribe("/camera/image", 1, &Analyzer::imageCb, this, hints);

  cv::namedWindow(OPENCV_HOUGH);
  cv::namedWindow(OPENCV_HOUGH_UTIL);
  cv::namedWindow(OPENCV_CONTOUR);
  cv::createTrackbar("Gaussian: ",       OPENCV_HOUGH_UTIL, &gaussian_min, gaussian_max);
  cv::createTrackbar("Canny edge min: ", OPENCV_HOUGH_UTIL, &canny_min, canny_max);
  cv::createTrackbar("Hough min: ",      OPENCV_HOUGH_UTIL, &hough_min, hough_max);
  cv::createTrackbar("Hough line: ",     OPENCV_HOUGH_UTIL, &hough_line_min, hough_line_max);
  cv::createTrackbar("Hough gap: ",      OPENCV_HOUGH_UTIL, &hough_gap_min, hough_gap_max);
}

Analyzer::~Analyzer()
{
  cv::destroyWindow(OPENCV_HOUGH);
  cv::destroyWindow(OPENCV_HOUGH_UTIL);
  cv::destroyWindow(OPENCV_CONTOUR);
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
  cv::Mat hdst;
  std::vector<cv::Vec4i> lines;
  std::vector<std::vector<cv::Point> > contours;
  std::vector<cv::Vec4i> hierarchy;

  cv::cvtColor(cv_ptr->image, gray_out, CV_BGR2GRAY);
  cv::GaussianBlur(gray_out, gray_out, cv::Size(gaussian_min, gaussian_min), 0, 0);
  cv::Canny(gray_out, canny_out, canny_min, canny_min * canny_ratio, canny_kernel);
  cv::HoughLinesP(canny_out, lines, 1, CV_PI / 180, hough_min, hough_line_min, hough_gap_min);
  cv::findContours(canny_out, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0));

  cv::cvtColor(canny_out, hdst, CV_GRAY2BGR);
  for (size_t i = 0; i < lines.size(); i++) {
    cv::Vec4i l = lines[i];
    cv::line(hdst, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]), cv::Scalar(0,0,255), 3);
  }

  cv::Mat cdst = cv::Mat::zeros(canny_out.size(), CV_8UC3);
  for (size_t i = 0; i < contours.size(); i++) {
    cv::drawContours(cdst, contours, i, cv::Scalar(0,0,255), 2, 8, hierarchy, 0, cv::Point());
  }

  cv::imshow(OPENCV_HOUGH, hdst);
  cv::imshow(OPENCV_CONTOUR, cdst);
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

