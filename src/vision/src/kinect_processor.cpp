#include <iostream>
#include <vector>
#include <math.h>
#include <dirent.h>
#include "ros/ros.h"
#include <tf/transform_broadcaster.h>

#include "turtlesim/Pose.h"

#include "std_msgs/String.h"

#include <sensor_msgs/PointCloud.h> // for the kinect point cloud

#include <image_transport/image_transport.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include "blockABC.h"

using namespace std;

int kcbCount(0);
vector<cv::Mat> templates(3); 
vector<char> corresponding(3); //indices indicate the letter that the template image is

void kinectCallback(const sensor_msgs::ImageConstPtr& msg)
{
    //ROS_INFO("I heard: [%s]", msg->data.c_str());

    static tf::TransformBroadcaster br;

    /*tf::Transform transform;
    transform.setOrigin( tf::Vector3(msg->x, msg->y, 0.0) );
    tf::Quaternion q;
    q.setRPY(0, 0, msg->theta);

    transform.setRotation(q);
*/
    // send transform, timestamp, parent frame, child frame
    //br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", turtle_name));

		//on the first callback, create the dictionary of template locations
		//if (kcbCount == 0)
		//{
			try
			{
				cv::Mat tempImg, grayImg;
				tempImg = cv_bridge::toCvCopy(msg, "bgr8")->image;
				//cv::cvtColor(tempImg, grayImg, CV_BGR2GRAY);
				cv::imshow("view", tempImg);
				cv::waitKey(30);
				//writing doesn't seem to work no matter what I try... ugh. It's not too important.
        //grayImg.convertTo(grayImg, CV_8UC3, 255.0);
				//cv::imwrite("test.bmp", grayImg);
			}
			catch(cv_bridge::Exception& e)
			{
				ROS_ERROR("could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
			}
		//}
		//else
		//{




		//}

    kcbCount++;
}

//cubesnippet will be the image within the bounds of the cube found by the point cloud data
char bestMatch(cv::Mat cubeSnippet)
{ 
/*
	double curMaxVal;
	int bestMatchIndex;
	Point matchLoc; 
	int matchMethod = cv::CV_TM_CCORR_NORMED;
	*/
	//use each image in the template library as the template to match against cubesnippet
	/*
	for (int i=0; i<templates.size(); i++){ 
		cv::Mat result;
		double minVal, maxVal; 
		Point minLoc, maxLoc;

		int result_cols = cubeSnippet.cols - templates[i].cols + 1;
		int result_rows = cubeSnippet.rows - templates[i].rows + 1; 
		result.create(result_rows, result_cols, CV_32FC1);
		cv::matchTemplate( cubeSnippet, templates[i], result, matchMethod);
		cv::normalize(result, result, 0, 1, NORM_MINMAX, -1, cv::Mat());		
		cv::minMaxLoc(result, &minVal, &maxVal, &minLoc, &maxLoc, cv::Mat());
		if (maxVal > curMaxVal) {
			matchLoc = maxLoc;
			bestMatchIndex = i; 
		} 
	} */
	//return corresponding[bestMatchIndex]; 
	return '1';
}

int main(int argc, char **argv)
{
		//load images into template library
		cv::Mat a, b, c, testb;
		char matchResult;
    a = cv::imread("/home/yl2908/baxter-abcs/src/vision/src/templates/a.png", 1);
		b = cv::imread("/home/yl2908/baxter-abcs/src/vision/src/templates/b.png", 1);
		c = cv::imread("/home/yl2908/baxter-abcs/src/vision/src/templates/c.png", 1);
		testb = cv::imread("/home/yl2908/baxter-abcs/src/vision/src/templates/testb.png", 1);
		templates[0] = a;
		templates[1] = b;
		templates[2] = c;
		corresponding[0] = 'a';
		corresponding[1] = 'b';
		corresponding[2] = 'c';
		matchResult = bestMatch(testb); 
		cout << "The tester match result: " << endl;
		cout << matchResult << endl; 

		/*
		string alpha ("abcdefghijklmnopqrstuvwxyz");
    string ext(".png"); 
		string name;
		DIR* dirp = null;
		struct dirent* dp = null;
		dirp = opendir(".");
		if (dirp == NULL) {
			cout << "yea you fucked up" ;
		}
		else {
			dp = readdir(dirp);
			cout << "\nDirectory name: " ;
			cout << dp->d_name ;
		}    
		for (int i=0; i<alpha.length(); ++i)
		{    
			name = alpha[i] + ext;
			dirp = opendir("templates");
			while ((dp = readdir(dirp)) != NULL) {
				//if (strcmp(dp->d_name,name.c_str()) == 0) { 
						//(void) closedir(dirp);
            //cout << "\nfound: " + name ;
				//}
				cout << dp->d_name;	
			}
			(void) closedir(dirp);
			cout << "\nnot found: " + name;
 		}
		*/
    //ros::init(argc, argv, "my_tf_broadcaster");
    ros::init(argc, argv, "kinect_listener");

    //if (argc != 2){ROS_ERROR("need turtle name as argument"); return -1;};
    //turtle_name = argv[1];

    ros::NodeHandle node;  // access to ROS system

    cv::namedWindow("view");
    cv::startWindowThread();
    image_transport::ImageTransport it(node);
    image_transport::Subscriber sub = it.subscribe("/kinect_mount/kinect_mount/rgb/image_raw", 1, kinectCallback);
    // TODO: find correct kinect topic
    //ros::Subscriber sub = node.subscribe(turtle_name+"/pose", 10, &kinectCallback);

    cout << "kinect????" << endl;
    
    BlockABC testA('a');

    ros::spin();
    cv::destroyWindow("view");

    return 0;
}
