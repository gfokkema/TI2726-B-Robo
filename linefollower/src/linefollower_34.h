#ifndef ANALYZER_H_
#define ANALYZER_H_

#include <ros/ros.h>
#include <geometry_msgs/Point32.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/image_encodings.h>

/**
 * This class sets up a ROS subscription on /camera/image/compressed
 * and listens for incoming images.
 * It then tries to scan and analyze these images for the best line
 * and determines a speed and angular velocity based on this.
 * This speed is published to /cmd_vel at a frequency of at most 4 Hz.
 */
class Analyzer
{
public:
  /**
   * Constructs an image analyzer.
   * This sets up awindow for debug viewing and a window for sliders.
   * The window for sliders can be used to customize thresholds
   * for Gaussian Blur, Canny Edge detection and Hough Line detection at runtime.
   */
  Analyzer();

  /**
   * Destructor for an image analyzer (clean up after ourselves).
   */
  virtual ~Analyzer();

  /**
   * ROS image callback, called when there's incoming data from the phone.
   * From this callback our OpenCV pipeline for detecting lines is called
   * in several steps. The relevant functions are, in order:
   * rotate -> project -> detect -> filter -> sendmessage
   */
  void imageCb(const sensor_msgs::ImageConstPtr& msg);

  /**
   * Rotate the camera by 90 degrees so we can view the camera in portrait mode.
   */
  void rotate (const cv::Mat& src, cv::Mat& dst);

  /**
   * Use perspective projection to correct for the phone's viewing angle of approx. 45 degrees.
   * This way, lines that are parallel on the floor are parallel on the screen as well.
   * Because of this we can use linear and angular velocity without having to correct for our viewing angle.
   */
  void project(const cv::Mat& src, cv::Mat& dst);

  /**
   * Check whether a point falls within the projected camera bounds (see project()).
   */
  bool withinbounds(const cv::Point& point);

  /**
   * Detect all lines in src, draw them on dst and store them in lines.
   * This function uses the following techniques to detect lines, in the specified order:
   * gaussian blur -> bgr2gray -> canny edge -> hough lines -> gray2bgr
   */
  void detect (const cv::Mat& src, cv::Mat& dst, cv::vector<cv::Vec4i>& lines);

  /**
   * Filters the best line out of all detected lines.
   * Only lines satisfying the following conditions are considered for the best line:
   * - both points of the line are within the projected camera bounds
   * - the line has an angle smaller than 1/12 * pi relative to the x-axis
   * - at least one of both points of the line has an y-coordinate above the origin
   * All detected lines are drawn to the screen in red.
   *
   * The 'best' line was then characterized as follows:
   * - it has the shortest distance to a certain origin
   *   (usually just above the midbottom coordinate, with a resolution of 1080x1920 this is 540x1820).
   * - it has the smallest angle relative to the y-axis
   *   (ie. the line is as straight as possible relative to the robot).
   * The line selected as best line is drawn to the screen in blue.
   */
  void filter (const cv::Point& origin, const cv::vector<cv::Vec4i>& lines,
               cv::Mat& dst, cv::Point& best1, cv::Point& best2, double& bestangle);

  /**
   * Generate and publish a ROS message with linear and angular velocity based on the best line.
   * The linear and angular speeds are generated as follows:
   * - of the best line the highest point is selected
   *   - the distance from this point to the origin is used to generate linear speed
   *   - the angle of the line from this point to the origin is used to generate angular speed
   *     this way the robot always steers towards the end point of the best line
   * - at most 4 messages are sent every second
   *   this is done by averaging linear and angular velocity until 0.25 second has passed
   */
  void sendmessage(const cv::Point& origin, const cv::Point& best1, const cv::Point& best2, const double& bestangle);

  /**
   * Displays src on an OpenCV window called window.
   * Primarily used for debugging.
   */
  void display(const cv::Mat& src, const std::string& window);

private:
  // These variables are used to construct the sliders.
  // The sliders can then be used to dynamically adjust thresholds for line detection.
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
