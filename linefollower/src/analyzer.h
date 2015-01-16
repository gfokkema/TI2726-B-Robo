#ifndef ANALYZER_H_
#define ANALYZER_H_

#include <ros/ros.h>
#include <geometry_msgs/Point32.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/image_encodings.h>

class Analyzer
{
public:
  Analyzer();
  virtual ~Analyzer();

  void imageCb(const sensor_msgs::ImageConstPtr& msg);

  void display(const cv::Mat& src, const std::string& window);
  void detect (const cv::Mat& src, cv::Mat& dst, cv::vector<cv::Vec4i>& lines);
  void filter (const cv::Mat& src, const cv::vector<cv::Vec4i>& lines,
               cv::Mat& dst, cv::Point& best1, cv::Point& best2, double& bestangle);
  void project(const cv::Mat& src, cv::Mat& dst);
  void rotate (const cv::Mat& src, cv::Mat& dst);
  void sendmessage(const cv::Point& best1, const cv::Point& best2, const double& bestangle);
  bool withinbounds(const cv::Point& point);
private:
  int canny_ratio;
  int canny_kernel;
  int canny_min, canny_max;
  int gaussian_min, gaussian_max;
  int gaussian_dev_min, gaussian_dev_max;
  int hough_min, hough_max;
  int hough_line_min, hough_line_max;
  int hough_gap_min, hough_gap_max;

  geometry_msgs::Point32 cmd_vel_msg_avg_;
  ros::NodeHandle nh_;
  ros::Publisher cmd_vel_pub_;
  ros::Time cmd_vel_last_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
};

#endif /* ANALYZER_H_ */
