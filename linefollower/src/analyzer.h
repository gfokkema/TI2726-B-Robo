#ifndef ANALYZER_H_
#define ANALYZER_H_

#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>

class Analyzer
{
public:
  Analyzer();
  virtual ~Analyzer();

  void detect (const cv::Mat& src, cv::Mat& dst, cv::vector<cv::Vec4i>& lines);
  void project(const cv::Mat& src, cv::Mat& dst);
  void rotate (const cv::Mat& src, cv::Mat& dst);
  void imageCb(const sensor_msgs::ImageConstPtr& msg);
private:
  int canny_ratio;
  int canny_kernel;
  int canny_min, canny_max;
  int hough_min, hough_max;
  int hough_line_min, hough_line_max;
  int hough_gap_min, hough_gap_max;

  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
};

#endif /* ANALYZER_H_ */
