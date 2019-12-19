#include <ros/ros.h>
#include <iostream>
#include <std_msgs/Float32MultiArray.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h>
#include <opencv2/opencv.hpp>
#include "camera/FindObjects.h"
#define SMILE 4
#define ARROW_LEFT 3
#define ARROW_UP 5
#define ARROW_DOWN 6
#define SELFDEFINED_RANGE_X_MAX 640
#define SELFDEFINED_RANGE_Y_MAX 430
int id = 0;
ros::Publisher object_location_pub;
int camera_center = 320; // left 0, right 640
float max_ang_vel = 0.6;
float min_ang_vel = 0.4;
float ang_vel = 0;
std_msgs::Float32MultiArrayPtr temp;

bool getObject(camera::FindObjects::Request &req, camera::FindObjects::Response &res) {
	if(temp->data.size() > 0){
	if((int) temp->data[0] == 25 || (int) temp->data[0] == 26){
	res.object = "red arrow";
	return true;	
	}
	}
	return false;
	
}

void objectCallback(const std_msgs::Float32MultiArrayPtr &object)
{
	temp = object;
  

 std_msgs::String obj_loc;
   
   if (object->data.size() > 0)
   {
      id = object->data[0];
      //std::cout << "found object id: "<< id << std::endl;
      float objectWidth = object->data[1];
      float objectHeight = object->data[2];
      float x_pos;
      float y_pos;
      float speed_coefficient = (float)camera_center / max_ang_vel / 4;
       
      // Find corners OpenCV
      cv::Mat cvHomography(3, 3, CV_32F);
      std::vector<cv::Point2f> inPts, outPts;
      cvHomography.at<float>(0, 0) = object->data[3];
      cvHomography.at<float>(1, 0) = object->data[4];
      cvHomography.at<float>(2, 0) = object->data[5];
      cvHomography.at<float>(0, 1) = object->data[6];
      cvHomography.at<float>(1, 1) = object->data[7];
      cvHomography.at<float>(2, 1) = object->data[8];
      cvHomography.at<float>(0, 2) = object->data[9];
      cvHomography.at<float>(1, 2) = object->data[10];
      cvHomography.at<float>(2, 2) = object->data[11];

      inPts.push_back(cv::Point2f(0, 0));
      inPts.push_back(cv::Point2f(objectWidth, 0));
      inPts.push_back(cv::Point2f(0, objectHeight));
      inPts.push_back(cv::Point2f(objectWidth, objectHeight));
      cv::perspectiveTransform(inPts, outPts, cvHomography);

      x_pos = (int)(outPts.at(0).x + outPts.at(1).x + outPts.at(2).x +
                    outPts.at(3).x) /
              4;
      y_pos = (int) ((outPts.at(0).y + outPts.at(1).y + outPts.at(2).y +
                    outPts.at(3).y) /
              4);

      if(x_pos < SELFDEFINED_RANGE_X_MAX/2 && y_pos < SELFDEFINED_RANGE_Y_MAX/2)
      obj_loc.data ="TOPLEFT";

      if(x_pos > SELFDEFINED_RANGE_X_MAX/2 && y_pos < SELFDEFINED_RANGE_Y_MAX/2)
      obj_loc.data ="TOPRIGHT";

      if(x_pos < SELFDEFINED_RANGE_X_MAX/2 && y_pos > SELFDEFINED_RANGE_Y_MAX/2)
      obj_loc.data ="BOTTOMLEFT";
      
      if(x_pos > SELFDEFINED_RANGE_X_MAX/2 && y_pos > SELFDEFINED_RANGE_Y_MAX/2)
      obj_loc.data ="BOTTOMRIGHT";
      
      //printf("P0SITI0N: \n X: %d \n Y: %d",(int)x_pos,(int)y_pos);
   }
   else
   {
      obj_loc.data = "Nothing here";

   }

      object_location_pub.publish(obj_loc);
}

int main(int argc, char **argv)
{
   std_msgs::String s;
   std::string str;
   str.clear();
   str.append("");
   s.data = str;
   ros::init(argc, argv, "action_controller");
   ros::NodeHandle n("~");
   ros::Subscriber sub = n.subscribe("/objects", 1, objectCallback);
   ros::Rate loop_rate(50);
   object_location_pub = n.advertise<std_msgs::String>("/botty/camera/object_location",1);
   ros::ServiceServer service = n.advertiseService("find_object", getObject);
   while (ros::ok())
   {
      ros::spinOnce();
      loop_rate.sleep();
   }
}
