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


#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>


#include "example.hpp"
#include "feature_extract.h"

using namespace std;
using namespace cv;

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;


class MYREALSENSE
{
public:
    /* data */
    Mat dMat_left;
    Mat dMat_right;
    Mat dMat_depth;
    Mat dMat_color;
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
    void view_pointcloud(PointCloudT::Ptr cloud);
    int get_LR();
    int extract_target();
private:
    PointCloudT::Ptr cloud_realsense;
    PointCloudT::Ptr cloud_filtered;
    PointCloudT::Ptr cloud_plane;
    PointCloudT::Ptr cloud_f;
    PointCloudT::Ptr target;
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
    const int screen_width=2560;
    const int screen_height=1080;
    const string WindowName="Realsense Output PointCloud";
    const int pic_width=640;
    const int pic_height=480;
};


