#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "ardrone_autonomy/navdata_altitude.h"
#include "ardrone_autonomy/Navdata.h"
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/Point.h>
#include "std_msgs/Float32.h"
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include "math.h"

using namespace cv;
using namespace std;

#define R_DFNT 40
#define B_DFNT 40
#define LEVEL 0.4
#define VERTICAL 0.4

class FindTwo
{
public:
	FindTwo();
private:
	ros::NodeHandle node;
	ros::Subscriber img_sub;
	ros::Publisher angle_pub;
	ros::Publisher dist_pub;

	void imageCallback(const sensor_msgs::Image &msg);
	void image_process(Mat img);
	void get_distant(std_msgs::Float32 &);
	void find_two_red(Mat &img, Mat imgThresholded);
	void find_two_blue(Mat &img, Mat imgThresholded);

	double level;
	double vertical;
	std_msgs::Float32 distant;
	std_msgs::Float32 angle;
};

FindTwo::FindTwo()
{
	img_sub = node.subscribe("/ardrone/image_raw", 1, &FindTwo::imageCallback, this);//"/ardrone/image_raw" "CamImage"	
	angle_pub = node.advertise<std_msgs::Float32>("ardrone_angle", 1000);
	dist_pub = node.advertise<std_msgs::Float32>("ardrone_distant", 1000);
}

void FindTwo::imageCallback(const sensor_msgs::Image &msg)
{
	cv_bridge::CvImagePtr cv_ptr;
	try
	{
		cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
	}
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}

	image_process(cv_ptr->image);
}

void FindTwo::image_process(Mat img)
{
	ros::Rate loop_rate(10);

	Size image_size = img.size();

	/*find two red points*/
	Mat imgThresholded = Mat(image_size, CV_8UC1);
	Mat img_cp = img.clone();

	find_two_blue(img, imgThresholded);
	find_two_red(img_cp, imgThresholded);
	
	if (fabs(level) <= 20)
	{
		//turn level to angle
		angle_pub.publish(angle);
	}
	else {
		get_distant(distant);
		dist_pub.publish(distant);
	}

	//imshow("monitor0", img);//testing
	imshow("monitor1", img_cp);//testing
	/*if (char(waitKey(1)) == 'o')
		destroyWindow("monitor0");*/
	if (char(waitKey(1)) == 'o')
		destroyWindow("monitor1");
	
	loop_rate.sleep();
}

void FindTwo::find_two_red(Mat &img, Mat imgThresholded)
{
	for (int i = 0; i < img.rows; ++i)
	{
		for (int j = 0; j < img.cols; ++j)
		{
			int tmp0 = img.at<Vec3b>(i,j)[0];
			int tmp1 = img.at<Vec3b>(i,j)[1];
			int tmp2 = img.at<Vec3b>(i,j)[2];
			if ((tmp2-tmp1)>=R_DFNT && (tmp2-tmp0)>=R_DFNT)
			{
				imgThresholded.at<uchar>(i,j) = 255;
				continue;
			}
			imgThresholded.at<uchar>(i,j) = 0;
		}
	}

	/*image operation*/
	Mat element = getStructuringElement(MORPH_RECT, Size(4,4));
	morphologyEx(imgThresholded, imgThresholded, MORPH_OPEN, element);
	morphologyEx(imgThresholded, imgThresholded, MORPH_CLOSE, element);	
	erode(imgThresholded, imgThresholded, element);
	dilate(imgThresholded, imgThresholded, element);
	imgThresholded = 255 - imgThresholded;

	/*find contours*/
	vector<vector<Point> > contours;
	vector<Vec4i> hierarchy;
	vector<Rect> rb;
	findContours(imgThresholded, contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_NONE);
	
	/*erase unqualified contours*/
	vector<vector<Point> >::iterator it = contours.begin();
	while(it != contours.end())
	{
		if (contourArea(*it, true) < 1000)
			it = contours.erase(it);
		else
		{
			Rect r = boundingRect(Mat(*it));
			int d = abs(r.width-r.height);
			if ( d >= 10)
				it = contours.erase(it);
			else {
				rb.push_back(r);
				rectangle(img, r, Scalar(0, 255, 0), 3);
				++it;
			}
		}
	}

	level = 0.25*(rb[0].tl().x + rb[0].br().x + rb[1].tl().x + rb[1].br().x) - 0.5*img.cols;
}

void FindTwo::find_two_blue(Mat &img, Mat imgThresholded)
{
	for (int i = 0; i < img.rows; ++i)
	{
		for (int j = 0; j < img.cols; ++j)
		{
			int tmp0 = img.at<Vec3b>(i,j)[0];
			int tmp1 = img.at<Vec3b>(i,j)[1];
			int tmp2 = img.at<Vec3b>(i,j)[2];
			if ((tmp0-tmp1)>=B_DFNT && (tmp0-tmp2)>=B_DFNT)
			{
				imgThresholded.at<uchar>(i,j) = 255;
				continue;
			}
			imgThresholded.at<uchar>(i,j) = 0;
		}
	}

	/*image operation*/
	Mat element = getStructuringElement(MORPH_RECT, Size(4,4));
	morphologyEx(imgThresholded, imgThresholded, MORPH_OPEN, element);
	morphologyEx(imgThresholded, imgThresholded, MORPH_CLOSE, element);	
	erode(imgThresholded, imgThresholded, element);
	dilate(imgThresholded, imgThresholded, element);
	imgThresholded = 255 - imgThresholded;

	/*find contours*/
	vector<vector<Point> > contours;
	vector<Vec4i> hierarchy;
	vector<Rect> rb;
	findContours(imgThresholded, contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_NONE);
	
	/*erase unqualified contours*/
	vector<vector<Point> >::iterator it = contours.begin();
	while(it != contours.end())
	{
		if (contourArea(*it, true) < 1000)
			it = contours.erase(it);
		else
		{
			Rect r = boundingRect(Mat(*it));
			int d = abs(r.width-r.height);
			if ( d >= 10)
				it = contours.erase(it);
			else {
				rb.push_back(r);
				rectangle(img, r, Scalar(0, 255, 0), 3);
				++it;
			}
		}
	}

	vertical = fabs(0.5*(rb[0].tl().y + rb[0].br().y - rb[1].tl().y - rb[1].br().y));
}

void FindTwo::get_distant(std_msgs::Float32 &distant)
{
	//get distant from vertical
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "find_two");
	FindTwo t;
	ros::spin();
	return 0;
}