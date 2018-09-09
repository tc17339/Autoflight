#include <iostream>
#include <string>
#include <vector>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/video/video.hpp>
#include <opencv2/features2d/features2d.hpp>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Bool.h>

int LowH = 90;
int HighH = 139;

int LowS = 129; 
int HighS = 245;

int LowV = 67;
int HighV = 213;

using namespace cv;
using namespace std;

Point2f image_centre;// the centre of the image
cv_bridge::CvImagePtr convert_image;   //original image container

const double kp=0.0002;
const double ki=0.0001;
const double kd=0.0001;

ros::Publisher bebop_takeoff;	//declare a publisher to send the takeoff message 					 
ros::Publisher bebop_land;	//declare a publisher to send the land message						
ros::Publisher bebop_cmd_vel_pub,bebop_cmd_vel_pub_1;//declare a publisher to send velocity message

std_msgs::Empty take_off,land;	  //takeoff and land variables 
geometry_msgs::Twist cmd_vel_bebop_y,cmd_vel_bebop_z,cmd_vel_bebop;   //velocity variable

	
void bebop_image_centre(Mat& image)
{
	image_centre.x = image.cols/2; 
	image_centre.y = image.rows/2;
	//cout<<"image center of x "<<image_centre.x<<endl;
	//cout<<"image_center of y "<<image_centre.y<<endl;
}

void hover()
{
	cmd_vel_bebop.linear.x = 0;
	cmd_vel_bebop.linear.y = 0;
	cmd_vel_bebop.linear.z = 0;
	cmd_vel_bebop.angular.z = 0;
	bebop_cmd_vel_pub.publish (cmd_vel_bebop);
}

void tracking_IF(Point2f centrePoint)
{
	
	if(centrePoint.x >0 )	
	{
		if(centrePoint.x < image_centre.x)
		{
			cmd_vel_bebop_y.angular.z = 0.35;
			cout<<" go left"<<endl;
		}
		if(centrePoint.x > image_centre.x)
		{
			cmd_vel_bebop_y.angular.z = -0.35;
			cout<<"go right"<<endl;
		}
		bebop_cmd_vel_pub.publish(cmd_vel_bebop_z);
	}
	if(centrePoint.y>0)
	{

		if(centrePoint.y > image_centre.y)
		{
			cmd_vel_bebop_z.linear.z = -0.25;
			cout<<"go down"<<endl;
		}
		if(centrePoint.y < image_centre.y)
		{
			cmd_vel_bebop_z.linear.z = 0.25;
			cout<<"go up"<<endl;
		}
	
		//bebop_cmd_vel_pub.publish(cmd_vel_bebop_z);
		bebop_cmd_vel_pub_1.publish(cmd_vel_bebop_y);
	}
	else
	{
		hover();
	}
}


void tracking_PID(Point2f centrePoint)
{
	if(centrePoint.x> 0 && centrePoint.y >0)
	{
		static double errorI_x = 0;
		static double errorI_y = 0;
		static double errorPre_x = 0;
		static double errorPre_y = 0;

		double error_x = image_centre.x - centrePoint.x;  // calcualted propotional error
		double error_y = image_centre.y - centrePoint.y;
	
		errorI_x = errorI_x + error_x;  // calculated intergal error
		errorI_y = errorI_y + error_y;

		double errorD_x = error_x - errorPre_x; //calculated derivative error
		double errorD_y = error_y - errorPre_y;
	
		errorPre_x = error_x;
		errorPre_y = error_y;
		
		cmd_vel_bebop_y.angular.z = kp*error_x + ki*errorI_x + kd*errorD_x;// pid controller calculate velocity
		bebop_cmd_vel_pub.publish(cmd_vel_bebop_y);

		cmd_vel_bebop_z.linear.z = kp*error_y + ki*errorI_y + kd*errorD_y;
		bebop_cmd_vel_pub_1.publish(cmd_vel_bebop_z);
	
		//bebop_cmd_vel_pub.publish(cmd_vel_bebop_y);
		//bebop_cmd_vel_pub.publish(cmd_vel_bebop_z);
	}
	else
	{
		hover();
	}
}







