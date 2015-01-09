#include "analyzer.h"

#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

static const std::string OPENCV_HOUGH = "Hough transform";
static const std::string OPENCV_HOUGH_UTIL = "Hough utils";

Analyzer::Analyzer()
: it_(nh_),
  canny_kernel(3), canny_ratio(4),
  canny_min(50), canny_max(100),
  hough_min(50), hough_max(100),
  hough_line_min(50), hough_line_max(100),
  hough_gap_min(10), hough_gap_max(20)
{
  // Subscribe to input video feed on /camera/image
  image_transport::TransportHints hints("compressed", ros::TransportHints());
  image_sub_ = it_.subscribe("/camera/image", 1, &Analyzer::imageCb, this, hints);

  // Open a window for Hough detection
  cv::namedWindow(OPENCV_HOUGH);
  // Open a window for the Hough toolbar
  cv::namedWindow(OPENCV_HOUGH_UTIL);
  cv::createTrackbar("Canny edge min: ", OPENCV_HOUGH_UTIL, &canny_min, canny_max);
  cv::createTrackbar("Hough min: ",      OPENCV_HOUGH_UTIL, &hough_min, hough_max);
  cv::createTrackbar("Hough line: ",     OPENCV_HOUGH_UTIL, &hough_line_min, hough_line_max);
  cv::createTrackbar("Hough gap: ",      OPENCV_HOUGH_UTIL, &hough_gap_min, hough_gap_max);
}

Analyzer::~Analyzer()
{
  cv::destroyWindow(OPENCV_HOUGH);
  cv::destroyWindow(OPENCV_HOUGH_UTIL);
}

void
Analyzer::imageCb(const sensor_msgs::ImageConstPtr& msg)
{
  // Read the image from msg and store it in cv_ptr
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

  // Rotate and scale the image
  double r[3][3] = { {0, -1, cv_ptr->image.rows - 1},
                     {1,  0, 0},
                     {0,  0, 1}};
  cv::Mat transform = cv::Mat(2, 3, CV_64F, r);
  cv::warpAffine(cv_ptr->image, gray_out, transform, cv::Size(cv_ptr->image.rows, cv_ptr->image.cols));
  cv::resize(gray_out, gray_out, cv::Size(gray_out.cols / 2, gray_out.rows / 2));

  // Do some preprocessing (GRAY -> CANNY EDGE)
  cv::cvtColor(gray_out, gray_out, CV_BGR2GRAY);
  cv::Canny(gray_out, canny_out, canny_min, canny_min * canny_ratio, canny_kernel);

  // Perform Hough line detection and draw the result
  cv::HoughLinesP(canny_out, lines, 1, CV_PI / 180, hough_min, hough_line_min, hough_gap_min);
  cv::cvtColor(canny_out, hdst, CV_GRAY2BGR);

  // Values for the angle are between -M_PI / 2 and M_PI / 2 for atan2
  // Therefore we draw all the lines with absolute angle greater than M_PI / 6
  for (size_t i = 0; i < lines.size(); i++) {
    double angle = std::abs(atan2((double)lines[i][3] - lines[i][1],
                                  (double)lines[i][2] - lines[i][0]));
    if (angle > M_PI / 6) cv::line(hdst, cv::Point(lines[i][0], lines[i][1]),
                                         cv::Point(lines[i][2], lines[i][3]),
                                         cv::Scalar(0,0,255), 3);
  }
  cv::imshow(OPENCV_HOUGH, hdst);
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

