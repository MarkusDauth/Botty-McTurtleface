#include <ros/ros.h>
#include <iostream>
#include <std_msgs/Float32MultiArray.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h>
#include <opencv2/opencv.hpp>
#include <vector>
#include "camera/FindObjects.h"
#include "std_msgs/String.h"
#include <algorithm>


#define SMILE 4
#define ARROW_LEFT 3
#define ARROW_UP 5
#define ARROW_DOWN 6
#define CAMERA_RANGE_X_MAX 640
#define CAMERA_RANGE_Y_MAX 480
#define PUBLISH_OBJECT_LOCATION 0
int id = 0;
ros::Publisher object_location_pub;
int camera_center = 320; // left 0, right 640
float max_ang_vel = 0.6;
float min_ang_vel = 0.4;
float ang_vel = 0;
std_msgs::Float32MultiArrayPtr temp;


bool equals_cocaCola_id(int object_id){
	if(object_id == 37 ||object_id == 38 ||object_id == 39 ||object_id == 40 || object_id == 41 ||object_id == 42 ||object_id == 43 || object_id == 56 || object_id == 57)
		return true;
	return false;	
}

bool equals_chalk_id(int object_id){
	if(object_id == 44 ||object_id == 47 ||object_id == 48 ||object_id == 49)
		return true;
	return false;
}

bool equals_shall_welten_magazine(int object_id){
	if(object_id == 54 ||object_id == 55)
		return true;
	return false;
}

bool equals_flower(int object_id){
	if(object_id == 50 ||object_id == 51 ||object_id == 52 ||object_id == 53)
		return true;
	return false;
}

bool equals_red_arrow(int object_id){
	if(object_id == 27 ||object_id == 26)
		return true;
	return false;
}

void set_object_ids(std::vector<int> &vector){
	for(size_t i = 0; i < temp->data.size(); i+=12){
		vector.push_back(temp->data[i]);
	}
}

std::string object_id_to_string(int object_id){
	if(equals_cocaCola_id(object_id))
		return "Coca Cola Bottle";	
	else if(equals_chalk_id(object_id))
		return "Chalk";
	else if(equals_shall_welten_magazine(object_id))
		return "Schall welten Magazin";
	else if(equals_flower(object_id))
		return "Flower";
	else if(equals_red_arrow(object_id))
		return "Red Arrow";
	else
		return "invalid";
}

void map_object_id_to_string(const std::vector<int> &object_ids,std::vector<std::string> &objects_as_string){
	
	for(size_t i = 0; i < object_ids.size(); i++){
	int object_id = object_ids[i];
	std::string object_id_as_string = object_id_to_string(object_id);

	if(object_id_as_string != "invalid"){
	objects_as_string.push_back(object_id_as_string);
	}

	}
} 

void fill_found_objects_array(std::string found_objects [],const std::vector<std::string> &objects_as_string){
	for(size_t i = 0; i < objects_as_string.size(); i++){
		found_objects[i] = objects_as_string[i];
	}
}
template <typename T>
void remove_duplicates(std::vector<T>& vec)
{
  std::sort(vec.begin(), vec.end());
  vec.erase(std::unique(vec.begin(), vec.end()), vec.end());
}

bool getObject(camera::FindObjects::Request &req, camera::FindObjects::Response &res) {
	if(temp->data.size() > 0){

	std::vector<int> object_ids;
	set_object_ids(object_ids);

	std::vector<std::string> objects_as_string;
	map_object_id_to_string(object_ids, objects_as_string); 
	remove_duplicates(objects_as_string);
	res.object = objects_as_string;
	}
	else 
	return false;
	
}

void objectCallback(const std_msgs::Float32MultiArrayPtr &object)
{

	temp = object;
  
#if(PUBLISH_OBJECT_LOCATION > 0)
 std_msgs::String obj_loc;
   
   if (object->data.size() > 0)
   {
      id = object->data[0];
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
#endif
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
