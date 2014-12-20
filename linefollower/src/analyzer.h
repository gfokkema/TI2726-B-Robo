#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>

class Analyzer
{
public:
  Analyzer();
  virtual ~Analyzer();

  void imageCb(const sensor_msgs::ImageConstPtr& msg);
private:
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
};

