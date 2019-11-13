#include "get_depth.hpp"
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/core.hpp>

using namespace cv;

int get_depth()
try
{
    rs2::pipeline p;
    rs2::config cfg;
    cfg.enable_stream(RS2_STREAM_INFRARED, 1, 1280, 720, RS2_FORMAT_Y8, 30);
    cfg.enable_stream(RS2_STREAM_INFRARED, 2, 1280, 720, RS2_FORMAT_Y8, 30);
    cfg.enable_stream(RS2_STREAM_DEPTH, 1280, 720, RS2_FORMAT_Z16, 30);
    rs2::pipeline pipe;
    p.start(cfg);
    
    while (true)
    {
        rs2::frameset frames=p.wait_for_frames();
        rs2::depth_frame depth=frames.get_depth_frame();
        rs2::video_frame ir_frame_left = frames.get_infrared_frame(1);
		rs2::video_frame ir_frame_right = frames.get_infrared_frame(2);

		cv::Mat dMat_left = cv::Mat(cv::Size(1280, 720), CV_8UC1, (void*)ir_frame_left.get_data());
		cv::Mat dMat_right = cv::Mat(cv::Size(1280, 720), CV_8UC1, (void*)ir_frame_right.get_data());

		cv::imshow("img_l", dMat_left);
		cv::imshow("img_r", dMat_right);
		char c = cv::waitKey(1);
        float width=depth.get_width();
        float height=depth.get_height();

        float dist_to_center=depth.get_distance(width/2,height/2);

        std::cout<<"The camera is facing an object "<<dist_to_center<<" meters away \r";
      
    }
    return EXIT_SUCCESS;
}
catch(const rs2::error & e)
{
    std::cerr << "Realsense error calling " <<e.get_failed_function()<<"("<<e.get_failed_args()<<"):\n   "<<e.what()<< '\n';
    return EXIT_FAILURE;
}
catch(const std::exception & e)
{
    std::cerr<<e.what()<<std::endl;
    return EXIT_FAILURE;
}