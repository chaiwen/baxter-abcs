#include "ros/ros.h"
#include <sensor_msgs/PointCloud2.h>
#include "vision/visionService.h"
#include <std_msgs/String.h>

using namespace std;

int cb = 0;

bool return_point(vision::visionService::Request & req, vision::visionService::Response &res)
{
  res.x = 10.0;
  res.y = 10.1;
  res.z = 10.2;
  ROS_INFO("requested letter: %s", req.letter.c_str());
  ROS_INFO("sending back response: %lf, %lf, %lf ", (double)res.x,(double)res.y,(double)res.z);
  return true;
}

void callback(const std_msgs::StringConstPtr& str) {
	cb++; 
	cout << cb << endl; 
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "vision_service_server");
	ros::NodeHandle n; 

	//test subscribing to the cloud 
  //test subscribing to the test topic 
  ros::Subscriber s = n.subscribe<std_msgs::String>("service_test_topic", 1, callback);

	ros::ServiceServer service = n.advertiseService("vision_service", return_point);
	ROS_INFO("ready to return point");
	ros::spin();

	return 0;

}
