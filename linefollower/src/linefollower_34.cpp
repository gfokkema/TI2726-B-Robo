#include "linefollower_34.h"

#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

static const std::string OPENCV_HOUGH = "Hough transform";
static const std::string OPENCV_HOUGH_UTIL = "Hough utils";

Analyzer::Analyzer() :
		it_(nh_), canny_kernel(3), canny_ratio(4),
		gaussian_min(5), gaussian_max(10),		// Kernel size:	x * 2 + 1	1 ->  21
		gaussian_dev_min(15), gaussian_dev_max(20),	// Deviation:	x / 5		0 ->   4
		canny_min(40), canny_max(100),			// Canny thres:	x		0 -> 100
		hough_min(40), hough_max(200),			// Hough thres:	x		0 -> 200
		hough_line_min(80), hough_line_max(800),	// Line min:	x		0 -> 800
		hough_gap_min(20), hough_gap_max(50) {		// Line gap:	x		0 ->  50
	// Subscribe to input video feed on /camera/image
	image_transport::TransportHints hints("compressed", ros::TransportHints());
	image_sub_ = it_.subscribe("/camera/image", 1, &Analyzer::imageCb, this, hints);
	cmd_vel_pub_ = nh_.advertise<geometry_msgs::Point32>("/cmd_vel", 1000);

	// Open a window for Hough detection
	cv::namedWindow(OPENCV_HOUGH);
	// Open a window for the sliders
	cv::namedWindow(OPENCV_HOUGH_UTIL);
	cv::createTrackbar("Gauss kernel (1 -   41): ", OPENCV_HOUGH_UTIL, &gaussian_min, gaussian_max);
	cv::createTrackbar("Gauss stddev (0 -    4): ", OPENCV_HOUGH_UTIL, &gaussian_dev_min, gaussian_dev_max);
	cv::createTrackbar("Canny edge   (0 -  100): ", OPENCV_HOUGH_UTIL, &canny_min, canny_max);
	cv::createTrackbar("Hough min    (0 -  200): ", OPENCV_HOUGH_UTIL, &hough_min, hough_max);
	cv::createTrackbar("Hough line   (0 - 1000): ", OPENCV_HOUGH_UTIL, &hough_line_min, hough_line_max);
	cv::createTrackbar("Hough gap    (0 -  200): ", OPENCV_HOUGH_UTIL, &hough_gap_min, hough_gap_max);
}

Analyzer::~Analyzer() {
	// Destroy all opened windows
	cv::destroyWindow(OPENCV_HOUGH);
	cv::destroyWindow(OPENCV_HOUGH_UTIL);
}

void Analyzer::imageCb(const sensor_msgs::ImageConstPtr& msg) {
	cv_bridge::CvImageConstPtr cv_ptr;
	cv::Mat hdst;
	cv::vector<cv::Vec4i> lines;
	cv::Point best1, best2;
	double bestangle;

	// Read the image from msg and store it in cv_ptr
	try {
		cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8);
	} catch (cv_bridge::Exception& e) {
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}

	// Process the image using our OpenCV 'pipeline'
	rotate(cv_ptr->image, hdst);
	project(hdst, hdst);
	detect(hdst, hdst, lines);

	// Set the origin to 540x1880
	cv::Point midbottom(hdst.cols / 2, hdst.rows - 100);

	filter(midbottom, lines, hdst, best1, best2, bestangle);
	sendmessage(midbottom, best1, best2, bestangle);

	// Show a debug window with the best line
	display(hdst, OPENCV_HOUGH);
}

void Analyzer::display(const cv::Mat& src, const std::string& window) {
	cv::Mat dst;
	cv::resize(src, dst, cv::Size(src.cols / 2, src.rows / 2));
	cv::imshow(window, dst);
	cv::waitKey(3);
}

void Analyzer::detect(const cv::Mat& src, cv::Mat& dst, cv::vector<cv::Vec4i>& lines) {
	// Do some preprocessing (gaussian, gray scale)
	cv::GaussianBlur(src, dst, cv::Size(gaussian_min * 2 + 1, gaussian_min * 2 + 1), gaussian_dev_min / 5.f);
	cv::cvtColor(dst, dst, CV_BGR2GRAY);

	// Perform Canny edge detection
	cv::Canny(dst, dst, canny_min, canny_min * canny_ratio, canny_kernel);

	// Perform Hough line detection
	cv::HoughLinesP(dst, lines, 1, CV_PI / 360, hough_min, hough_line_min, hough_gap_min);

	// Do some postprocessing
	cv::cvtColor(dst, dst, CV_GRAY2BGR);
}

void Analyzer::filter(const cv::Point& origin, const cv::vector<cv::Vec4i>& lines,
	cv::Mat& dst, cv::Point& best1, cv::Point& best2, double& bestangle) {
	double bestscore = DBL_MAX;
	for (size_t i = 0; i < lines.size(); i++) {
		cv::Point p1(lines[i][0], lines[i][1]);
		cv::Point p2(lines[i][2], lines[i][3]);

		// Continue if both points are below the origin
		if (p1.y > origin.y && p2.y > origin.y) continue;
		// Continue if either point is outside of the projection bounds
		if (!withinbounds(p1) || !withinbounds(p2)) continue;

		// Calculate the angle of the line
		double angle = atan2(p2.y - p1.y, p2.x - p1.x);
		// -M_PI / 2 <= atan2 <= M_PI / 2, therefore we add M_PI when atan2 < 0 for a domain from 0 to M_PI
		if (angle < 0) angle += M_PI;

		// Continue if the absolute angle of the line relative to the x-axis > 1/12 pi
		if (angle > 1 * M_PI / 12 && angle < 11 * M_PI / 12) {
			cv::line(dst, p1, p2, cv::Scalar(0, 0, 255), 3);

			// Calculate distance to origin
			double dist = std::min(cv::norm(p1 - origin), cv::norm(p2 - origin));
			// Calculate angle relative to the y-axis
			double dx = std::abs(p1.x - p2.x) / cv::norm(p1 - p2);

			// Calculate a score based on these 2 values and store the best result
			double score = dist * dist * dx;
			if (score < bestscore) { best1 = p1, best2 = p2; bestangle = angle; bestscore = score; }
		}
	}
	// Draw the best result to the screen
	cv::line(dst, best1, best2, cv::Scalar(255, 0, 0), 3);
}

void Analyzer::project(const cv::Mat& src, cv::Mat& dst) {
	cv::Point2f srccoords[4] = {
		cv::Point2f(1 * src.cols / 4, 0),	// left up
		cv::Point2f(3 * src.cols / 4, 0),	// right up
		cv::Point2f(src.cols, src.rows),	// right down
		cv::Point2f(0, src.rows)		// left down
	};
	cv::Point2f dstcoords[4] = {
		cv::Point2f(0, 0),				// left up
		cv::Point2f(src.cols, 0),			// right up
		cv::Point2f(4 * src.cols / 5, src.rows),	// right down
		cv::Point2f(1 * src.cols / 5, src.rows)		// left down
	};

	// Perform perspective projection using the defined values to correct
	// for the 45 degree viewing angle of the phone,
	// making lines that are parallel in real life parallel on the screen as well.
	cv::Mat transform = cv::getPerspectiveTransform(srccoords, dstcoords);
	cv::warpPerspective(src, dst, transform, src.size());
}

void Analyzer::rotate(const cv::Mat& src, cv::Mat& dst) {
	// Rotate the image by 90 degrees
	double r[3][3] = {
		{ 0, -1, src.rows - 1 },
		{ 1, 0, 0 },
		{ 0, 0, 1 }
	};
	cv::Mat transform = cv::Mat(2, 3, CV_64F, r);
	cv::warpAffine(src, dst, transform, cv::Size(src.rows, src.cols));
}

void Analyzer::sendmessage(const cv::Point& origin, const cv::Point& best1, const cv::Point& best2, const double& bestangle)
{
	if (best1.x == 0 && best1.y == 0 && best2.x == 0 && best2.y == 0) return;

	cv::Point lowest = best1;
	cv::Point highest = best2;
	if (best2.y > best1.y) { lowest = best2; highest = best1; }

	geometry_msgs::Point32 cmd_vel_msg_new;
	cmd_vel_msg_new.x = std::max(80, std::min(150, (origin - highest).y / 3));	// x ==> linear speed, capped at 150
	cmd_vel_msg_new.z = (-atan2(highest.y - origin.y, highest.x - origin.x) - M_PI / 2) * 180 / M_PI;

	// Send no more than 4 messages per second
	// Average messages if we analyze more than 1 image every 0.25 second
	if ((ros::Time::now() - cmd_vel_last_).toSec() > 0.25) {
		cmd_vel_pub_.publish(cmd_vel_msg_new);
		cmd_vel_msg_avg_ = cmd_vel_msg_new;
		cmd_vel_last_ = ros::Time::now();
	} else {
		cmd_vel_msg_avg_.x = (cmd_vel_msg_avg_.x + cmd_vel_msg_new.x) / 2;
		cmd_vel_msg_avg_.z = (cmd_vel_msg_avg_.z + cmd_vel_msg_new.z) / 2;
	}
}

bool Analyzer::withinbounds(const cv::Point& point)
{
	// Check whether a specific point is within the bounds of our perspective projection
	return  point.y - (point.x +  540) * 160 / 63 < -10 &&
		point.y + (point.x - 1620) * 160 / 63 < -10;
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "image_converter");
	Analyzer analyze;

	ros::spin();
	return 0;
}
