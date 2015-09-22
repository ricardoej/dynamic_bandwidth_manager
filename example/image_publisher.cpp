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

	CvCapture* capture = cvCaptureFromCAM(CV_CAP_ANY);

	ros::Rate loop_rate(50.0);

	while (nh.ok()) {
		IplImage* frame = cvQueryFrame(capture);
		cv_bridge::CvImage cvImage;
		cvImage.encoding = "bgr8";
		cvImage.image = frame;
		pub.publish(cvImage.toImageMsg());
		ros::spinOnce();
		loop_rate.sleep();
	}
}