#include "feature_extract.h"
#include "get_depth.hpp"
#include "pointcloud.hpp"
#include "my_realsense.hpp"

int main()
{
    MYREALSENSE firstone;
    //firstone.get_LR();
    firstone.extract_target();
    return 0;

    rs2::colorizer c; 
    namedWindow(firstone.depth_win,WINDOW_AUTOSIZE);
    namedWindow(firstone.color_win,WINDOW_AUTOSIZE);
    auto depth_stream=firstone.profile.get_stream(RS2_STREAM_DEPTH).as<rs2::video_stream_profile>();
    auto color_stream=firstone.profile.get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>();
    auto intrinDepth=depth_stream.get_intrinsics();
    auto intrinColor=color_stream.get_intrinsics();
 
    //直接获取从深度摄像头坐标系到彩色摄像头坐标系的欧式变换矩阵
    auto  extrinDepth2Color=depth_stream.get_extrinsics_to(color_stream);
 
    while (cvGetWindowHandle(firstone.depth_win)&&cvGetWindowHandle(firstone.color_win)) // Application still alive?
    {
        //堵塞程序直到新的一帧捕获
        rs2::frameset frameset = firstone.pipe.wait_for_frames();
        //取深度图和彩色图
        rs2::frame color_frame = frameset.get_color_frame();//processed.first(align_to);
        rs2::frame depth_frame = frameset.get_depth_frame();
        rs2::frame depth_frame_4_show = frameset.get_depth_frame().apply_filter(c);
        //获取宽高
        firstone.depth_w=depth_frame.as<rs2::video_frame>().get_width();
        firstone.depth_h=depth_frame.as<rs2::video_frame>().get_height();
        firstone.color_w=color_frame.as<rs2::video_frame>().get_width();
        firstone.color_h=color_frame.as<rs2::video_frame>().get_height();
 
        //创建OPENCV类型 并传入数据
        firstone.dMat_depth=Mat(Size(firstone.depth_w,firstone.depth_h),
                                CV_16U,(void*)depth_frame.get_data(),Mat::AUTO_STEP);
        firstone.depth_color=Mat(Size(firstone.depth_w,firstone.depth_h),
                                CV_8UC3,(void*)depth_frame_4_show.get_data(),Mat::AUTO_STEP);
        firstone.dMat_color=Mat(Size(firstone.color_w,firstone.color_h),
                                CV_8UC3,(void*)color_frame.get_data(),Mat::AUTO_STEP);
        //实现深度图对齐到彩色图
        firstone.result=firstone.align_Depth2Color();
        imshow(firstone.depth_win,firstone.depth_color);
        imshow(firstone.color_win,firstone.dMat_color);
        imshow("result",firstone.result);
        //imwrite("/home/yons/projects/realsense/res/depth.JPG",firstone.depth_color);
        //imwrite("/home/yons/projects/realsense/res/color.JPG",firstone.dMat_color);
        waitKey(10);
    }
    return 0;
}

