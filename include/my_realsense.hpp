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

#include "example.hpp"

using namespace std;
using namespace cv;

class MYREALSENSE
{
public:
    /* data */
    Mat depth;
    Mat color;
    Mat result;
    int depth_width;
    int depth_height;
    int color_width;
    int color_height;
    const float depth_clipping_distance=1.f;
    rs2::pipeline_profile profile;
    rs2::pipeline pipe;
public:
    MYREALSENSE(/* args */);
    ~MYREALSENSE();
    float get_depth_scale(rs2::device dev);
    Mat align_Depth2Color();
    int get_pointcloud();
};


