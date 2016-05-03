#include <iostream>
#include <string>

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
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

#include <image_transport/image_transport.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include "blockABC.h"
#include "vision/GetXYZFromABC.h"

using namespace std;
using namespace cv;

vector<cv::Mat> templates(5); 
vector<char> corresponding(5); //indices indicate the letter that the template image is

Mutex blockLock;
vector<cv::Rect *> blockBounds;
vector<pcl::PointXYZ *> blockPositions;
Mutex matLock;
cv::Mat rgbImg;

vector<BlockABC *> blocks;
char bestMatch(cv::Mat cubeSnippet);
void addBlock(char type); 
void deleteBlocks();

ros::Publisher pub;

bool stopUpdating = false;

void ptCloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{

    if (!stopUpdating) {
    // Container for original & filtered data
    pcl::PCLPointCloud2 *cloud = new pcl::PCLPointCloud2;
    pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
    pcl::PCLPointCloud2 cloud_filtered;

    // cloud_msg->point_step = 32 = length of a point in bytes = 4x8byte float? XYZRGB
    // cloud_msg->row_step = 9830400 = 32 x 640 x 480
    // kinect point cloud data is one array (cloud_msg->height = 1) of 307200 (cloud_msg->width = 640 x 480)
    // cloud_msg->data = array of uint8 (size = row_step * height)
    //cout << "------->" << (float)cloud_msg->data[0] << endl;

    // Convert to PCL data type
    pcl_conversions::toPCL(*cloud_msg, *cloud);
    // cloud: 307200 width x 1 height
    pcl::PointCloud<pcl::PointXYZ> original_cloud;
    pcl::fromROSMsg(*cloud_msg, original_cloud);
    // loop thru 307200 points:

    // Downsample the points using leaf size of 1cm
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

    // euclidean cluster extraction
    // create KdTree object for search method of extraction
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(old_filtered);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(0.02); // 2cm
    ec.setMinClusterSize(10);
    ec.setMaxClusterSize(25000);
    ec.setSearchMethod(tree);
    ec.setInputCloud(old_filtered);
    ec.extract(cluster_indices);


    blockLock.lock();
    for (int b = 0; b < blockBounds.size(); b++) {
        delete blockBounds[b];
        delete blockPositions[b];
    }
    blockBounds.clear();
    blockPositions.clear();

    //cout << "getting each point of each cluster" << endl;
    // old_filtered size will be the total number of cluster points. cluster indices index into this
    // to get the points for each cluster
    int j = 0;
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
    {
        // for each cluster:
        //pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
        //cout << "cloud " << j << ": " << old_filtered->points.size() << endl; // total number of points overall
        float centerX = 0.0, centerY = 0.0, centerZ = 0.0;

        float minCX = 50, minCY = 50;
        float maxCX = -50, maxCY = -50;

        float numP = 0.0;

        for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit) {

            float x = old_filtered->points[*pit].x;
            float y = old_filtered->points[*pit].y;
            float z = old_filtered->points[*pit].z;

            if (x < minCX) minCX = x;
            if (x > maxCX) maxCX = x;
            if (y < minCY) minCY = y;
            if (y > maxCY) maxCY = y;

            centerX += x;
            centerY += y;
            centerZ += z;

            numP++;
        }
        centerX = centerX / numP;
        centerY = centerY / numP;
        centerZ = centerZ / numP;
        // store the xyz location before multiplying by strange error for 2d image space:
        pcl::PointXYZ *pt_xyz = new pcl::PointXYZ(centerX, centerY, centerZ);
        
        float weirdo_error = 1.8;
        centerX *= weirdo_error;
        centerY *= weirdo_error;

        //cout << "cluster " << j << " center: " << centerX << ", " << centerY << ", " << centerZ << endl;
        //cout << "width: " << maxCX - minCX << ", height: " << maxCY - minCY << endl;

        int px = 0;
        int pxMin = 0;
        int pxMax = 0;
        for (int ct = 0; ct < 640; ct++) {
            pcl::PointXYZ pt = original_cloud[ct];

            if (centerX < pt.x) {
                pxMin = px;
                break;
            }

            px++;
        }

        px = 0;
        int pyMin = 0;
        for (int ct = 0; ct < 307200; ct += 640) {
            pcl::PointXYZ pt = original_cloud[ct];

            if (centerY < pt.y) {
                pyMin = px;
                break;
            }
            px++;
        }

        float rx = pxMin - 40;
        float ry = pyMin - 40;
        float rw = 80;
        float rh = 80;

        if (rx < 0) rx = 0;
        if (ry < 0) ry = 0;

        if (rx + rw > 639) rw = 639 - rx;
        if (ry + rh > 479) rh = 479 - ry;

        cv::Rect *rect = new cv::Rect(rx, ry, rw, rh);
        blockBounds.push_back(rect);
        blockPositions.push_back(pt_xyz);

        j++;
    }
    blockLock.unlock();
    //cout << ">>> there were " << j << " clusters found" << endl;

    // convert back to PointCloud2 for display via Ros message
    pcl::PCLPointCloud2 new_cloud;
    pcl::toPCLPointCloud2(*old_filtered, new_cloud);

    // Convert to ROS data type
    sensor_msgs::PointCloud2 output;
    pcl_conversions::fromPCL(new_cloud, output);//cloud_filtered, output);

    // publish the filtered data, should only be cubes in space now without table!
    pub.publish(output);
    
    }
}

void kinectCallback(const sensor_msgs::ImageConstPtr& msg)
{

    if (!stopUpdating) {
    try
    {
        matLock.lock();
        rgbImg = cv_bridge::toCvCopy(msg, "bgr8")->image;
        matLock.unlock();
        //cv::cvtColor(tempImg, grayImg, CV_BGR2GRAY);
        //cv::imshow("view", tempImg);


        // keep for testing, continuous checking
        /*blockLock.lock();
        matLock.lock();
        for (int b = 0; b < blockBounds.size(); b++) {
            cv::Rect *rect = blockBounds[b];
            cv::Mat letterImg;
            //cout << "----> imshow " << b << ": " << rect->x << ", " << rect->y << ", " << rect->width << ", " << rect->height << endl;
            letterImg = rgbImg(*rect);

            cv::Mat grayImg;
            cv::cvtColor(letterImg, grayImg, CV_BGR2GRAY);
            cv::imshow("view", grayImg);

            char result;
            result = bestMatch(grayImg);
            cout << "the best match " << b << ": " << result << endl;

            sleep(1);
        }
        matLock.unlock();
        blockLock.unlock();*/
    }
    catch(cv_bridge::Exception& e)
    {
        ROS_ERROR("could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
    }
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

bool getXYZ_ABC(vision::GetXYZFromABC::Request &req,
        vision::GetXYZFromABC::Response &res)
{

    stopUpdating = true;

    char reqChar = req.letter.c_str()[0];

    blockLock.lock();
    matLock.lock();

    //deleteBlocks();
    for (int b = 0; b < blockBounds.size(); b++) {
        cv::Rect *rect = blockBounds[b];
        cv::Mat letterImg;
        //cout << "----> imshow " << b << ": " << rect->x << ", " << rect->y << ", " << rect->width << ", " << rect->height << endl;
        letterImg = rgbImg(*rect);
        
        cv::Mat grayImg;
        cv::cvtColor(letterImg, grayImg, CV_BGR2GRAY);
        cv::imshow("view", grayImg);

        char result = bestMatch(grayImg);
        cout << "the best match: " << result << endl;

        if (result == reqChar) {
            
            pcl::PointXYZ *pt = blockPositions[b];
            cout << "x: " << pt->x << ", " << pt->y << ", " << pt->z << endl;

            res.x = pt->x;
            res.y = pt->y;
            res.z = pt->z;

            break;
        }
        //addBlock(result, pt->x, pt->y, pt->z);
        sleep(1);
    }
    //cv::imshow("view", rgbImg);
    matLock.unlock();
    blockLock.unlock();
    
    ROS_INFO("request: %s", req.letter.c_str());
    ROS_INFO("sending back %lf %lf %lf", (double)res.x, (double)res.y, (double)res.z);

    return true;
}

void loadABC(cv::Mat *a, cv::Mat *b, cv::Mat *c, cv::Mat *d, cv::Mat *e)
{
    string path = "__PATH__" + "/src/vision/src/templates/";
    }

    string pathA = path + "a.png";
    *a = cv::imread(pathA, CV_LOAD_IMAGE_GRAYSCALE);

    string pathB = path + "b.png";
    *b = cv::imread(pathB, CV_LOAD_IMAGE_GRAYSCALE);

    string pathC = path + "c.png";
    *c = cv::imread(pathC, CV_LOAD_IMAGE_GRAYSCALE);

    string pathD = path + "d.png";
    *d = cv::imread(pathD, CV_LOAD_IMAGE_GRAYSCALE);

    string pathE = path + "e.png";
    *e = cv::imread(pathE, CV_LOAD_IMAGE_GRAYSCALE);

}
int main(int argc, char **argv)
{
    //load images into template library
    cv::Mat a, b, c, d, e, testb;
    char matchResult;
    
    loadABC(&a, &b, &c, &d, &e);
    templates[0] = a;
    templates[1] = b;
    templates[2] = c;
    templates[3] = d;
    templates[4] = e;
    corresponding[0] = 'a';
    corresponding[1] = 'b';
    corresponding[2] = 'c';
    corresponding[3] = 'd';
    corresponding[4] = 'e'; 


    char buff[PATH_MAX];
    ssize_t len = ::readlink("/proc/self/exe", buff, sizeof(buff) - 1);
    if (len != -1) {
        buff[len] = '\0';
        cout << std::string(buff) << endl;

        std::string delimiter = "baxter-abcs";
    }
    ros::init(argc, argv, "kinect_listener"); //initialize the node

    ros::NodeHandle node;  // access to ROS system

    cv::namedWindow("view");
    cv::startWindowThread();
    image_transport::ImageTransport it(node);
    image_transport::Subscriber sub = it.subscribe("/kinect_mount/kinect_mount/rgb/image_raw", 1, kinectCallback);

    ros::Subscriber sub2 = node.subscribe<sensor_msgs::PointCloud2>("/kinect_mount/kinect_mount/rgb/points", 1, ptCloudCallback);

    // publisher for modified cloud
    pub = node.advertise<sensor_msgs::PointCloud2>("/kinect_mount/kinect_mount/kinect_object_cloud_filtered", 100);

    // get XYZ service
    ros::ServiceServer service = node.advertiseService("get_xyz_from_abc", getXYZ_ABC);
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
void addBlock(char type, float x, float y, float z) {
    BlockABC *block = new BlockABC(type);
    block->setPosition(x, y, z);
    blocks.push_back(block);
}
