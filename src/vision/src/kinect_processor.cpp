#include <iostream>
#include <vector>
#include <math.h>
#include <dirent.h>
#include "ros/ros.h"
#include "ros/package.h"
#include <tf/transform_broadcaster.h>

#include "turtlesim/Pose.h"

#include "std_msgs/String.h"

#include <sensor_msgs/PointCloud2.h> // for the kinect point cloud
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <image_transport/image_transport.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include "blockABC.h"

using namespace std;
using namespace cv;

int kcbCount = 0;
vector<cv::Mat> templates(3); 
vector<char> corresponding(3); //indices indicate the letter that the template image is


vector<BlockABC *> blocks;

ros::Publisher pub;
ros::Publisher toVisionService;
int pclCount = 0;
void ptCloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{

    // Container for original & filtered data
    pcl::PCLPointCloud2 *cloud = new pcl::PCLPointCloud2;
    pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
    pcl::PCLPointCloud2 cloud_filtered;

    // Convert to PCL data type
    pcl_conversions::toPCL(*cloud_msg, *cloud);

    // Downsample the points
    pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
    sor.setInputCloud(cloudPtr);
    sor.setLeafSize(0.01, 0.01, 0.01);
    sor.filter(cloud_filtered);

    // convert to PointCloud old type to use passthrough filter
    pcl::PointCloud<pcl::PointXYZ> *cloudv1 = new pcl::PointCloud<pcl::PointXYZ>;
    pcl::PointCloud<pcl::PointXYZ>::Ptr old_cloud(cloudv1);
    pcl::PointCloud<pcl::PointXYZ>::Ptr old_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(cloud_filtered, *cloudv1);

    //cout << old_cloud->points[0].x << " " << old_cloud->points[0].y << " " << old_cloud->points[0].z << endl;
    
    pcl::PassThrough<pcl::PointXYZ> pass; //TODO maybe there's a passthrough filter for pointCloud2?
    pass.setInputCloud(old_cloud);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(1.0, 1.17);
    pass.filter(*old_filtered);

    // convert back to PointCloud2...
    pcl::PCLPointCloud2 new_cloud;
    pcl::toPCLPointCloud2(*old_filtered, new_cloud);
    //pcl::PCLPointCloud2 = ROS message type replacing sensors_msgs::PointCloud2

    // Convert to ROS data type
    sensor_msgs::PointCloud2 output;
    pcl_conversions::fromPCL(new_cloud, output);//cloud_filtered, output);

    // publish the filtered data, should only be cubes in space now without table!
    pub.publish(output);

}

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
    if (kcbCount == 0)
    {
        //use point cloud data to find location of blocks and create block objects
        /* for (int i=0; i<blocks.size(); i++) {
        //for each block, call bestMatch (make sure the sub-image is larger than the templates!
        //set the type to the right letter, make sure the 2d positioning is accurate and being used  
        } 
        */
        cout << "kcbCount = 0" << endl;
    }
    else 
    {
        //cout << "kcbCount: ";
        //cout << kcbCount << endl; 	
    }

    try
    {
        cv::Mat tempImg, grayImg;
        tempImg = cv_bridge::toCvCopy(msg, "bgr8")->image;
        //cv::cvtColor(tempImg, grayImg, CV_BGR2GRAY);
        //cv::imshow("view", tempImg);

//        cv::imshow("view", cv_bridge::toCvCopy(msg, "bgr8")->image);

        //cv::waitKey(30);
        //writing doesn't seem to work no matter what I try... ugh. It's not too important.
        //grayImg.convertTo(grayImg, CV_8UC3, 255.0);
        //cv::imwrite("test.bmp", grayImg);
        /*				cout << "kinect callback count: " <<endl;
                        cout << kcbCount << endl; 
                        cout << "i only call you when it's half past five" << endl;
                        */
    }
    catch(cv_bridge::Exception& e)
    {
        ROS_ERROR("could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }

    kcbCount++;
}

//cubesnippet will be the image within the bounds of the cube found by the point cloud data
char bestMatch(cv::Mat cubeSnippet)
{ 

    double curMaxVal;
    int bestMatchIndex;
    Point matchLoc; 
    int matchMethod = CV_TM_CCORR_NORMED;

    //use each image in the template library as the template to match against cubesnippet

    for (int i=0; i<templates.size(); i++){ 
        cv::Mat result;
        double minVal, maxVal; 
        Point minLoc, maxLoc;

        int result_cols = cubeSnippet.cols - templates[i].cols + 1;
        int result_rows = cubeSnippet.rows - templates[i].rows + 1; 
        /*
           cout << "result columns" << endl;
           cout << result_cols << endl;
           cout << "result rows" << endl;
           cout << result_rows << endl; 
           */
        result.create(result_rows, result_cols, CV_32FC1);
        cv::matchTemplate( cubeSnippet, templates[i], result, matchMethod);
        //cv::normalize(result, result, 0, 1, NORM_MINMAX, -1, Mat());		
        cv::minMaxLoc(result, &minVal, &maxVal, &minLoc, &maxLoc, Mat());
        /*
           cout << "current max value: " ;
           cout << curMaxVal << endl; 
           cout << "max value: " ;
           cout << maxVal << endl;
           cout << "min value: " ;
           cout << minVal << endl; 
           */

        if (maxVal > curMaxVal) {
            curMaxVal = maxVal;
            matchLoc = maxLoc;
            bestMatchIndex = i;
            /*
               cout << "best match index" << endl;
               cout << bestMatchIndex << endl;
               */   
        }

    }
    return corresponding[bestMatchIndex]; 
}

int main(int argc, char **argv)
{
    //load images into template library
    cv::Mat a, b, c, testb;
    char matchResult;
    a = cv::imread("/home/yl2908/baxter-abcs/src/vision/src/templates/a.png", 1);
    b = cv::imread("/home/yl2908/baxter-abcs/src/vision/src/templates/b.png", 1);
    c = cv::imread("/home/yl2908/baxter-abcs/src/vision/src/templates/c.png", 1);
    //testb = cv::imread("/home/yl2908/baxter-abcs/src/vision/src/templates/testb.png", 1);
    templates[0] = a;
    templates[1] = b;
    templates[2] = c;
    corresponding[0] = 'a';
    corresponding[1] = 'b';
    corresponding[2] = 'c';

    /* works!
       matchResult = bestMatch(testb); 
       cout << "The tester match result: " << endl;
       cout << matchResult << endl; 
       */

    cout << kcbCount << endl; 
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

    //cv::namedWindow("view");
    //cv::startWindowThread();
    image_transport::ImageTransport it(node);
    image_transport::Subscriber sub = it.subscribe("/kinect_mount/kinect_mount/rgb/image_raw", 1, kinectCallback);

    ros::Subscriber sub2 = node.subscribe<sensor_msgs::PointCloud2>("/kinect_mount/kinect_mount/rgb/points", 1, ptCloudCallback);


    // publisher for modified cloud
    pub = node.advertise<sensor_msgs::PointCloud2>("/kinect_mount/kinect_mount/kinect_object_cloud_filtered", 100);
    cout << "kinect????" << endl;
		//testing talking to vision_service 
		toVisionService = node.advertise<std_msgs::String>("service_test_topic",1);
		std_msgs::String str;
		str.data = "testing the vision service!";
		toVisionService.publish(str); 

    BlockABC testA('a');

    ros::spin();
    //cv::destroyWindow("view");

    return 0;
}

/* deletes all blocks to reset */
// deleteBlocks();
void deleteBlocks() {
    cout << "deleting blocks vector..." << endl;
    if (blocks.size() == 0)
        return;

    while (blocks.size() > 0) {
        BlockABC *_block = blocks.back();
        delete _block;
        blocks.pop_back();
    }
}

/* adds a block to vector of blocks */
// eg: addBlock('a');
void addBlock(char type) {
    BlockABC *block = new BlockABC(type);
    blocks.push_back(block);
}
