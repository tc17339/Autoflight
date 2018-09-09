#include "bebop_tracking.h"


void find_contour(Mat& image)
{
	//*********************first step,convert bgr image to binary image*********//
	Mat imgHSV;
  	cvtColor(image, imgHSV, COLOR_BGR2HSV); //Convert the image from BGR to HSV

 	Mat imgThresholded;
  	inRange(imgHSV, Scalar(LowH, LowS, LowV), Scalar(HighH, HighS, HighV), imgThresholded); //Threshold the image
	
  	erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );//remove small objects
  	dilate( imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) ); 

  	dilate( imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) ); //remove small object
  	erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );

	//*******************second step,find the contour************************//
	vector<vector<Point> > contours;
	vector<Vec4i> heirarchy;
	vector<Point2i> center;
	vector<int> radius;
	bool enableRadiusCulling,minTargetRadius;

	findContours(imgThresholded, contours, heirarchy, CV_RETR_TREE, CV_CHAIN_APPROX_NONE);// find the contour in image

	size_t count= contours.size();
	for(int i=0; i < count; i++)
	{
		Point2f c;
		float r;
		minEnclosingCircle(contours[i], c, r);

		if(!enableRadiusCulling || r >= minTargetRadius)
		{
			center.push_back(c); //store the centre of circle
			radius.push_back(r); //store the radius of circle
		}
	}

	size_t count_center = center.size();
	Scalar red(255,0,0);
	Point2f target_centre;

	for(int i=0; i<count_center; i++)
	{
		if(contourArea(contours[i])>1000)
		{
			circle(image, center[i], radius[i], red, 3); //draw a circle with target
			target_centre.x = center[i].x;
			target_centre.y = center[i].y;
			cout<<"x position is "<<center[i].x<<endl;
			cout<<"y position is "<<center[i].y<<endl;
			cout<<"area is :"<<contourArea(contours[i])<<endl;
			//*********tracking part ********//
			//tracking_IF(target_centre);  // tracking algorithm based on if condition 
			tracking_PID(target_centre);  // tracking algorithm based on PID controller
		}		
	}

}

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
	Mat bebop_image;
	try
	{
		convert_image = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);// convert ros image to rgb image
		bebop_image = convert_image->image;
		bebop_image_centre(bebop_image); // calculated the image centre
		Mat bebop_image_clone = convert_image->image.clone();//clone a image
		find_contour(bebop_image_clone); // label the target

		imshow("camera see", bebop_image_clone); //create a window and show thw image
		int key = waitKey(30); // retunrn the vaule of the pressed key
		if(key == 27)
		{
			cout<<"esc key pressed"<<endl;
			return ;
		}
		if (key == 't') // if the k key was pressed, the drone will take off
		{
			ROS_INFO("TAKING OFF THE DRONE");
			bebop_takeoff.publish(take_off); 
		}
		if(key == ' ') // if the space key was pressed, the drone will be landed
		{
			ROS_INFO("LANDING ");
			bebop_land.publish(land);
		}

	}
	catch(cv_bridge::Exception& e)  // catch conversion error
	{

		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	} 	

}




int main(int argc, char** argv)
{
	ros::init(argc,argv,"bebop_tracking");				//initialize a ros node
	ros::NodeHandle nh,nh1;						//create the NodeHandler
	image_transport::ImageTransport it(nh);				// message for contain the image
	image_transport::Subscriber sub_image;				//subscriber to recieve the image

	sub_image = it.subscribe("/bebop/image_raw",1,imageCallback);    //subcribe the image_raw topic
	bebop_takeoff = nh.advertise<std_msgs::Empty>("/bebop/takeoff",1000);		//Publish data to the take-off topic
	bebop_land = nh.advertise<std_msgs::Empty>("/bebop/land",1000);		//Publish data to the land topic
	bebop_cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/bebop/cmd_vel",100);	//Publish data to the move topic
	bebop_cmd_vel_pub_1 = nh1.advertise<geometry_msgs::Twist>("bebop/cmd_vel",100);

	ros::spin();// let callback function called from the subscribers

}
