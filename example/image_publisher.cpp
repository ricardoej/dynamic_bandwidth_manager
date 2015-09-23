#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv/cvwimage.h>
#include <opencv/highgui.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

int main(int argc, char** argv) {
	ros::init(argc, argv, "image_publisher");
	ros::NodeHandle nh;
	image_transport::ImageTransport it(nh);
	image_transport::Publisher pub = it.advertise("webcam/image", 1);

	int cam;
	ros::param::param<int>("~camera", cam, 0);
	CvCapture* capture = cvCaptureFromCAM(cam);

	ros::Rate loop_rate(50.0);

	while (nh.ok()) {
		IplImage* frame = cvQueryFrame(capture);
		cv_bridge::CvImage cvImage;
		cvImage.encoding = "bgr8";
		cvImage.image = frame;

		cv::Mat HSV, threshold;
		cv::cvtColor(cvImage.image, HSV, CV_BGR2HSV);
		cv::inRange(HSV, cv::Scalar(100, 150, 0), cv::Scalar(140, 255, 255),
				threshold);
		cv::erode(threshold, threshold,
				cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)));
		cv::dilate(threshold, threshold,
				cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)));

		cv::Moments oMoments = cv::moments(threshold);

		double dM01 = oMoments.m01;
		double dM10 = oMoments.m10;
		double dArea = oMoments.m00;

		if (dArea > 10000)
		{
			nh.setParam("/webcam/image/dbm/priority", 1);
		}
		else
		{
			nh.setParam("/webcam/image/dbm/priority", 0);	
		}

		pub.publish(cvImage.toImageMsg());
		ros::spinOnce();
		loop_rate.sleep();
	}
}