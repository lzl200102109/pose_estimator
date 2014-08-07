#include "Corners.h"
#include "Geometry.h"

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <iostream>
using namespace std;

#define ESCAPE_KEY 27

static const char* INPUT_WINDOW = "Input";
static const char* CONTOUR_WINDOW = "Contours";
//static const char* CANNY_WINDOW = "Canny";

void imageCallback(const sensor_msgs::ImageConstPtr& imageMsg)
{


    cv_bridge::CvImagePtr input_bridge;		// convert the ROS image message to opencv IplImage
	Mat frame;

	// subscribe ROS image message
	try{
		input_bridge = cv_bridge::toCvCopy(imageMsg, sensor_msgs::image_encodings::RGB8);
	}
	catch (cv_bridge::Exception& ex) {
		ROS_ERROR("cv_bridge exception: %s", ex.what());
		return;
	}

	// convert ROS image message to image
	frame = input_bridge->image;

	// display image and corner detection on separate windows
	Mat_<double> corners = detectCorners(
		frame,
		INPUT_WINDOW,
		NULL, // cannyWindowHandle,
		CONTOUR_WINDOW);

	// display simplePose data in the terminal
	if(corners.rows) {
		Mat_<double> calibratedCorners = calibrateImagePoints(corners);
		Mat_<double> simplePose = estimatePose(calibratedCorners);
		cout << simplePose << endl;
	} else {
		cout << "could not detect all corners" << endl;
	}
}


int main(int argc, char *argv[])
{
	ros::init(argc, argv, "TestPoseFromMessage");

	ros::NodeHandle node;
	image_transport::ImageTransport it(node);

    image_transport::Subscriber sub = it.subscribe("image", 1, &imageCallback);

	cv::namedWindow(INPUT_WINDOW, CV_WINDOW_AUTOSIZE);
	cv::namedWindow(CONTOUR_WINDOW, CV_WINDOW_AUTOSIZE);
//    cv::namedWindow(CANNY_WINDOW, CV_WINDOW_AUTOSIZE);

	cv::startWindowThread();

	ros::spin();
    cv::destroyAllWindows();

    return 0;
}
