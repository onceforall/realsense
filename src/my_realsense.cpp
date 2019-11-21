#include "my_realsense.hpp"


MYREALSENSE::MYREALSENSE(/* args */)
{
    
 
    rs2::config cfg;
    cfg.enable_stream(RS2_STREAM_INFRARED, 1, 640, 480, RS2_FORMAT_Y8, 30);
    cfg.enable_stream(RS2_STREAM_INFRARED, 2, 640, 480, RS2_FORMAT_Y8, 30);
    cfg.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 30);
    cfg.enable_stream(RS2_STREAM_COLOR,640,480,RS2_FORMAT_RGB8,30);
    profile=pipe.start(cfg);
    //pipe.start();
    depth=Mat(Size(640,480),CV_16UC1);
    color=Mat(Size(640,480),CV_8UC3);
}

MYREALSENSE::~MYREALSENSE()
{
}

float MYREALSENSE::get_depth_scale(rs2::device dev)
{
    for(rs2::sensor& sensor : dev.query_sensors())
    {
        if(rs2::depth_sensor dpt=sensor.as<rs2::depth_sensor>())
        {
            return dpt.get_depth_scale();
        }
    }
    throw std::runtime_error("Device does not have a depth sensor");
}

Mat MYREALSENSE::align_Depth2Color()
{
    auto depth_stream=profile.get_stream(RS2_STREAM_DEPTH).as<rs2::video_stream_profile>();
    auto color_stream=profile.get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>();
    const auto intrinDepth=depth_stream.get_intrinsics();
    const auto intrinColor=color_stream.get_intrinsics();

    rs2_extrinsics extrinDepth2Color;
    rs2_error *error;
    rs2_get_extrinsics(depth_stream,color_stream,&extrinDepth2Color,&error);

    float pd_uv[2],pc_uv[2];
    float Pdc3[3],Pcc3[3];

    float depth_scale=get_depth_scale(profile.get_device());

    Mat result=Mat::zeros(color.rows,color.cols,CV_8UC3);
    int y=0,x=0;
    for(int row=0;row<depth.rows;row++)
    {
        for(int col=0;col<depth.cols;col++)
        {
            pd_uv[0]=col;
            pd_uv[1]=row;
            uint16_t depth_value=depth.at<uint16_t>(row,col);
            float depth_in_meter=depth_value*depth_scale;
            rs2_deproject_pixel_to_point(Pdc3,&intrinDepth,pd_uv,depth_in_meter);
            rs2_transform_point_to_point(Pcc3,&extrinDepth2Color,Pdc3);
            rs2_project_point_to_pixel(pc_uv,&intrinColor,Pcc3);

            x=(int)pc_uv[0];
            y=(int)pc_uv[1];

            x=x<0?0:x;
            x=x>depth.cols-1?depth.cols-1:x;
            y=y<0?0:y;
            y=y>depth.rows-1?depth.rows-1:y;

            for(int k=0;k<3;k++)
            {
                if(depth_in_meter<1)
                    result.at<cv::Vec3b>(y,x)[k]=color.at<cv::Vec3b>(y,x)[k];
            }
        }
    }
    return result;

}

int MYREALSENSE::get_pointcloud()
try
{
    window app(640,480,"pointcloud example");
    glfw_state app_state;
    register_glfw_callbacks(app,app_state);

    rs2::pointcloud pc;
    rs2::points points;
    while(app)
    {
        rs2::frameset  frames=pipe.wait_for_frames();
        rs2::frame color_frame=frames.get_color_frame();

        if(!color_frame)
            color_frame=frames.get_infrared_frame();
        pc.map_to(color_frame);

        rs2::frame depth_frame=frames.get_depth_frame();

        points=pc.calculate(depth_frame);

        app_state.tex.upload(color_frame);

        draw_pointcloud(app.width(),app.height(),app_state,points);
    }
    return EXIT_SUCCESS;
}
catch (const rs2::error & e)
{
    std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
    return EXIT_FAILURE;
}
catch (const std::exception & e)
{
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
}