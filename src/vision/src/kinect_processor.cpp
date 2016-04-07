#include <iostream>
#include "ros/ros.h"
#include <tf/transform_broadcaster.h>

#include "turtlesim/Pose.h"

#include "std_msgs/String.h"

using namespace std;


std::string turtle_name;

void kinectCallback(const turtlesim::PoseConstPtr& msg)
{
    //ROS_INFO("I heard: [%s]", msg->data.c_str());

    static tf::TransformBroadcaster br;

    tf::Transform transform;
    transform.setOrigin( tf::Vector3(msg->x, msg->y, 0.0) );
    tf::Quaternion q;
    q.setRPY(0, 0, msg->theta);

    transform.setRotation(q);

    // send transform, timestamp, parent frame, child frame
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", turtle_name));
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "my_tf_broadcaster");
    if (argc != 2){ROS_ERROR("need turtle name as argument"); return -1;};
    turtle_name = argv[1];

    ros::NodeHandle node;  // access to ROS system

    // TODO: find correct kinect topic
    ros::Subscriber sub = node.subscribe(turtle_name+"/pose", 10, &kinectCallback);

    cout << "kinect????" << endl;

    ros::spin();

    return 0;
}
