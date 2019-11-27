#pragma once

#include <iostream>
#include <sstream>
#include <fstream>
#include <algorithm>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <librealsense2/rs.hpp>
#include <librealsense2/rsutil.h>

#include <pcl-1.8/pcl/io/pcd_io.h>
#include <pcl-1.8/pcl/io/ply_io.h>
#include <pcl-1.8/pcl/point_types.h>
#include <pcl-1.8/pcl/point_cloud.h>
#include <pcl-1.8/pcl/visualization/pcl_visualizer.h>

#include "example.hpp"

using namespace std;
using namespace cv;

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;


class MYREALSENSE
{
public:
    /* data */
    Mat depth;
    Mat color;
    Mat result;
    Mat depth_color;
    PointCloudT::Ptr targetcloud;

    const char* depth_win="depth_Image";
    const char* color_win="color_Image";
  
    int depth_w=640;
    int depth_h=480;
    int color_w=640;
    int color_h=480;
 
    const float depth_clipping_distance=1.f;
    rs2::pipeline_profile profile;
    rs2::pipeline pipe;
public:
    MYREALSENSE(/* args */);
    ~MYREALSENSE();
    float get_depth_scale(rs2::device dev);
    Mat align_Depth2Color();
    int get_pointcloud();
    void view_pointcloud();
private:
    PointCloudT::Ptr out_pointcloud;
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
    const int screen_width=2560;
    const int screen_height=1080;
    const string WindowName="Realsense Output PointCloud";
};


