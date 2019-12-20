#pragma once
#include <iostream>
#include "opencv2/opencv.hpp"
#include "opencv2/highgui.hpp"
#include <opencv2/xfeatures2d.hpp>
#include <pcl-1.8/pcl/io/pcd_io.h>
#include <pcl-1.8/pcl/io/ply_io.h>
#include <pcl-1.8/pcl/point_types.h>
#include <pcl-1.8/pcl/point_cloud.h>
#include <pcl-1.8/pcl/visualization/pcl_visualizer.h>
#include <vector>


using namespace cv;
using namespace std;
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<pcl::PointXYZ> PointCloudT;
class MYREALSENSE;
class FEATURE_EXTRACT
{
      friend MYREALSENSE;
public:
      FEATURE_EXTRACT();
      virtual ~FEATURE_EXTRACT(){};  
      void getdsp();
      void goodmatcher();
      void get_homography();
      void printmatrix();
      void sutura_detect(Mat skull_pic);
      void get_mask(Mat mask_pic);
      Mat imgL;
      Mat imgR;
      PointCloudT::Ptr cloud_sutura;
      vector<Point> vec_sutura;
private:
      vector<KeyPoint> keypointsL,keypointsR;
      Mat descriptorsL,descriptorsR;
      vector<std::vector<DMatch>> match;
      vector<DMatch> goomatch;
      vector<Point2f> psL,psR;
      vector<DMatch> goodmatch;
      Mat matchimg;
      Mat homo;
      
      const int pic_width=640;
      const int pic_height=480;
};