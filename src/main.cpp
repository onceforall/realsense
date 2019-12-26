#include "feature_extract.hpp"
#include "my_realsense.hpp"
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"


void metadata_to_csv(const rs2::frame& frm, const std::string& filename)
{
    std::ofstream csv;

    csv.open(filename);

    //    std::cout << "Writing metadata to " << filename << endl;
    csv << "Stream," << rs2_stream_to_string(frm.get_profile().stream_type()) << "\nMetadata Attribute,Value\n";

    // Record all the available metadata attributes
    for (size_t i = 0; i < RS2_FRAME_METADATA_COUNT; i++)
    {
        if (frm.supports_frame_metadata((rs2_frame_metadata_value)i))
        {
            csv << rs2_frame_metadata_to_string((rs2_frame_metadata_value)i) << ","
                << frm.get_frame_metadata((rs2_frame_metadata_value)i) << "\n";
        }
    }

    csv.close();
}

int main()
{
    int num=0;
    MYREALSENSE firstone;
    
    //firstone.get_depth();
   
    //#if 0
    string test_path="/home/yons/projects/pycharms/Mask_RCNN/Out_Mask/color.png";
    firstone.mask_pic=imread(test_path,0);
    Mat* mask=&firstone.mask_pic;
    firstone.feature_extract.get_mask(mask);
    //firstone.feature_extract.sutura_detect(test);
    //return 0;
    //firstone.get_LR();
    //firstone.result=firstone.align_Depth2Color();
    return 0;

    //#endif
    rs2::colorizer c; 
    namedWindow(firstone.depth_win,WINDOW_AUTOSIZE);
    namedWindow(firstone.color_win,WINDOW_AUTOSIZE);
   
    //std::stringstream png_file;
    //std::stringstream csv_file;
    //png_file << "/home/yons/projects/realsense/res/depth" << ".png";
    //csv_file << "/home/yons/projects/realsense/res/depth"<< "-metadata.csv";

    #if 0
    PointCloudT::Ptr cloud=PointCloudT::Ptr(new PointCloudT);
    PointCloudT::Ptr part=PointCloudT::Ptr(new PointCloudT);
    string inputcloudfile="/home/yons/projects/realsense/res/pointcloud.ply";
    string partcloudfile="/home/yons/projects/realsense/res/part.ply";
    pcl::io::loadPLYFile(inputcloudfile, *cloud);
    pcl::io::loadPLYFile(partcloudfile,*part);
    firstone.feature_extract.cloud_sutura=part;
    firstone.view_pointcloud(cloud);
    return 0;
    #endif 

    std::ofstream outfile("/home/yons/projects/realsense/res/depth.txt",ios::out | ios::binary);
    while (cvGetWindowHandle(firstone.depth_win)&&cvGetWindowHandle(firstone.color_win)) // Application still alive?
    {
        
        //堵塞程序直到新的一帧捕获
        for (auto i = 0; i < 30; ++i) 
            firstone.pipe.wait_for_frames();
        rs2::frameset frameset = firstone.pipe.wait_for_frames();

        #if 0
        for (auto&& frame : frameset)
        {
            auto vf= frame.as<rs2::video_frame>();
            auto stream=frame.get_profile().stream_type();
            if(vf.is<rs2::depth_frame>()) 
            {
                //vf = c.process(frame);
            
                stbi_write_png(png_file.str().c_str(), vf.get_width(), vf.get_height(),
                vf.get_bytes_per_pixel(), vf.get_data(), vf.get_stride_in_bytes());
            
                metadata_to_csv(vf, csv_file.str());
                cout<<"saved depth!"<<endl;
            }
        }
        #endif
      
        //取深度图和彩色图
        
        rs2::depth_frame depth = frameset.get_depth_frame();

        rs2::frame color_frame = frameset.get_color_frame();//processed.first(align_to);
        rs2::frame depth_frame = frameset.get_depth_frame();
        rs2::frame depth_frame_4_show = frameset.get_depth_frame().apply_filter(c);
        //获取宽高
        
        firstone.depth_w=depth_frame.as<rs2::video_frame>().get_width();
        firstone.depth_h=depth_frame.as<rs2::video_frame>().get_height();
        firstone.color_w=color_frame.as<rs2::video_frame>().get_width();
        firstone.color_h=color_frame.as<rs2::video_frame>().get_height();
        //outfile.write(depth_frame.get_distance(width / 2, height / 2))
        //创建OPENCV类型 并传入数据
        
        firstone.dMat_depth=Mat(Size(firstone.depth_w,firstone.depth_h),
                                CV_16UC1,(void*)depth_frame.get_data(),Mat::AUTO_STEP);
        firstone.depth_color=Mat(Size(firstone.depth_w,firstone.depth_h),
                                CV_8UC3,(void*)depth_frame_4_show.get_data(),Mat::AUTO_STEP);
        firstone.dMat_color=Mat(Size(firstone.color_w,firstone.color_h),
                                CV_8UC3,(void*)color_frame.get_data(),Mat::AUTO_STEP);
        //实现深度图对齐到彩色图
        for(int row=0;row<firstone.depth_h;row++)
        {
            for(int col=0;col<firstone.depth_w;col++)
            {
                uint16_t depth_value=firstone.dMat_depth.at<uint16_t>(row,col);
                //firstone.vec_depth[row][col]=firstone.dMat_depth.at<uint16_t>(col,row);
                //cout<<firstone.dMat_depth.at<uint16_t>(row,col)<<' ';
                outfile<<depth_value<<' ';
            }
            outfile<<endl;
        }
        outfile.close();
        
        //imwrite("/home/yons/projects/realsense/res/depth.png",firstone.dMat_depth);
        imwrite("/home/yons/projects/realsense/res/color.png",firstone.dMat_color);
        //firstone.feature_extract.sutura_detect(firstone.dMat_color);
        //firstone.feature_extract.get_mask();
        //firstone.align_Depth2Color();
       
        imshow(firstone.depth_win,firstone.depth_color);
        imshow(firstone.color_win,firstone.dMat_color);
        waitKey(10);
        
        #if 0
        char key_board=waitKey(10);
        if(key_board=='s' || key_board=='S')
        {
            string savepath="/home/yons/projects/realsense/dataset/"+to_string(num++)+".jpg";
            imwrite(savepath,firstone.dMat_color);
            cout<<"Save one pic to "<<savepath<<" sucessful!"<<endl;
        }
            
        if(key_board=='q' || key_board=='Q')
        {
            destroyAllWindows();
            cout<<"Quiting ..."<<endl;
            return 0;
        }
        #endif 
    }
    return 0;
}

