#include "my_realsense.hpp"
#include "feature_extract.hpp"
#include <iostream>


MYREALSENSE::MYREALSENSE(/* args */)
{
    rs2::config cfg;
    cfg.enable_stream(RS2_STREAM_INFRARED, 1, pic_width, pic_height, RS2_FORMAT_Y8, 30);
    cfg.enable_stream(RS2_STREAM_INFRARED, 2, pic_width, pic_height, RS2_FORMAT_Y8, 30);
    cfg.enable_stream(RS2_STREAM_DEPTH, pic_width, pic_height, RS2_FORMAT_Z16, 30);
    cfg.enable_stream(RS2_STREAM_COLOR,pic_width,pic_height,RS2_FORMAT_RGB8,30);
    
    profile=pipe.start(cfg);

    auto sensor=profile.get_device().first<rs2::depth_sensor>();                                        //find the first device you use
    sensor.set_option(rs2_option::RS2_OPTION_VISUAL_PRESET,rs2_rs400_visual_preset::RS2_RS400_VISUAL_PRESET_HIGH_ACCURACY);  //prest the visual mode to HIGH_DENSITY
    //pipe.start();
    dMat_depth=Mat(Size(pic_width,pic_height),CV_16UC1);
    dMat_color=Mat(Size(pic_width,pic_height),CV_8UC3);
    
    cloud_filtered=PointCloudT::Ptr(new PointCloudT);
    cloud_plane=PointCloudT::Ptr(new PointCloudT);
    cloud_f=PointCloudT::Ptr(new PointCloudT);
    target=PointCloudT::Ptr(new PointCloudT);
    cloud_blob=pcl::PCLPointCloud2::Ptr(new pcl::PCLPointCloud2);
    cloud_blob_filtered=pcl::PCLPointCloud2::Ptr(new pcl::PCLPointCloud2);
    cloud_filtered_backup=PointCloudT::Ptr(new PointCloudT);
}

MYREALSENSE::~MYREALSENSE()
{
   
}

int MYREALSENSE::extract_target()
{
    view_pointcloud(cloud_realsense);
    std::cerr << "PointCloud before filtering: " << cloud_realsense->points.size()<< " data points." << std::endl;
    // Create the filtering object: downsample the dataset using a leaf size of 1cm
    pcl::VoxelGrid<PointT> sor;
    sor.setInputCloud (cloud_realsense);
    sor.setLeafSize (0.1f, 0.1f, 0.1f);
    sor.filter (*cloud_filtered);
    
    std::cerr<<"Pointcloud after filtering: "<<cloud_filtered->points.size()<<std::endl;
   
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());

    //segmentation
    pcl::SACSegmentation<PointT> seg;
    // Optional
    seg.setOptimizeCoefficients (true);
    // Mandatory
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    //seg.setMaxIterations (1000);
    seg.setDistanceThreshold (5);

    //pcl::PLYWriter writer;

    //extract plane
    pcl::ExtractIndices<PointT> extract;

    int i = 0, nr_points = (int) cloud_filtered->points.size ();
    // While 30% of the original cloud is still there
    while (cloud_filtered->points.size () > 0.3 * nr_points)
    { 
        // Segment the largest planar component from the remaining cloud
        seg.setInputCloud (cloud_filtered);
        seg.segment (*inliers, *coefficients);
        if (inliers->indices.size () == 0)
        {
            std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
            break;
        }
        // Extract the inliers
        extract.setInputCloud (cloud_filtered);
        extract.setIndices (inliers);
        extract.setNegative (false);
        extract.filter (*cloud_plane);
        std::cerr << "PointCloud representing the planar component: " << cloud_plane->width * cloud_plane->height << " data points." << std::endl;
        // Create the filtering object
        extract.setNegative (true);
        extract.filter (*cloud_f);
        cloud_filtered.swap (cloud_f);
        i++;
    }
    
    view_pointcloud(cloud_plane);
    view_pointcloud(cloud_filtered);
    //writer.write<PointT> ("/home/yons/projects/realsense/res/target.pcd", *target, false);

  return (0);

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
    cloud_realsense=PointCloudT::Ptr (new PointCloudT);
    feature_extract.cloud_sutura=PointCloudT::Ptr (new PointCloudT);
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

    result=Mat::zeros(dMat_color.rows,dMat_color.cols,CV_8UC3);
    int y=0,x=0;
   
    for(int row=0;row<dMat_depth.rows;row++)
    {
        for(int col=0;col<dMat_depth.cols;col++)
        {
            pd_uv[0]=col;
            pd_uv[1]=row;
            uint16_t depth_value=dMat_depth.at<uint16_t>(row,col);
            float depth_in_meter=depth_value*depth_scale;
            rs2_deproject_pixel_to_point(Pdc3,&intrinDepth,pd_uv,depth_in_meter);
            rs2_transform_point_to_point(Pcc3,&extrinDepth2Color,Pdc3);
            rs2_project_point_to_pixel(pc_uv,&intrinColor,Pcc3);
            //std::cout<<Pdc3[0]<<' '<<Pdc3[1]<<' '<<Pdc3[2]<<std::endl;
            cloud_realsense->points.push_back(PointT(Pdc3[0]*1000,Pdc3[1]*1000,Pdc3[2]*1000));
            x=(int)pc_uv[0];
            y=(int)pc_uv[1];

            x=x<0?0:x;
            x=x>dMat_depth.cols-1?dMat_depth.cols-1:x;
            y=y<0?0:y;
            y=y>dMat_depth.rows-1?dMat_depth.rows-1:y;
            if(find(feature_extract.vec_sutura.begin(),feature_extract.vec_sutura.end(),Point(x,y))!=feature_extract.vec_sutura.end())
                feature_extract.cloud_sutura->points.push_back(PointT(Pdc3[0]*1000,Pdc3[1]*1000,Pdc3[2]*1000));
            
            for(int k=0;k<3;k++)
            {
                if(depth_in_meter<1)
                    result.at<cv::Vec3b>(y,x)[k]=dMat_color.at<cv::Vec3b>(y,x)[k];
            }
        }
    }
    
     //显示 
    feature_extract.cloud_sutura->height = 1;
    feature_extract.cloud_sutura->width = feature_extract.cloud_sutura->points.size();
    feature_extract.cloud_sutura->is_dense = false;
    cloud_realsense->height = 1;
    cloud_realsense->width = cloud_realsense->points.size();
    cloud_realsense->is_dense = false;
    view_pointcloud(cloud_realsense);
    //pcl::io::savePLYFileASCII("/home/yons/projects/realsense/res/pointcloud.ply", *cloud_realsense); 
    //extract_target();
    
    return result;

}

int MYREALSENSE::get_pointcloud()
try
{
    window app(pic_width,pic_width,"pointcloud example");
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



void MYREALSENSE::view_pointcloud(PointCloudT::Ptr cloud)
{
    string suturaWindow="Sutura";
    cout<<feature_extract.cloud_sutura->points.size()<<' '<<cloud->points.size()<<endl;
	viewer = boost::shared_ptr<pcl::visualization::PCLVisualizer>(new pcl::visualization::PCLVisualizer(WindowName));
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> red(feature_extract.cloud_sutura, 255, 0, 0);
    viewer->addPointCloud(feature_extract.cloud_sutura, red, suturaWindow);
	viewer->addPointCloud(cloud, WindowName);
	viewer->resetCameraViewpoint(WindowName);
    viewer->resetCameraViewpoint(suturaWindow);
	viewer->addCoordinateSystem(1);
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, WindowName);
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 7, suturaWindow);
	//viewer->setFullScreen(true); // Visualiser window size
	viewer->setSize(screen_width,screen_height);
	while (!viewer->wasStopped())
	{
		viewer->spin();
		//boost::this_thread::sleep(boost::posix_time::microseconds(10));
        //viewer->close();
	}
}

#if 0
void MYREALSENSE::view_pointcloud(PointCloudT::Ptr cloud)
{
	
	viewer = boost::shared_ptr<pcl::visualization::PCLVisualizer>(new pcl::visualization::PCLVisualizer(WindowName)); //定义窗口共享指针
	int v1 ; //定义两个窗口v1，v2，窗口v1用来显示初始位置，v2用以显示配准过程
	int v2 ;
	viewer->setSize(screen_width,screen_height);
	viewer->resetCameraViewpoint(WindowName);
	viewer->addCoordinateSystem(1);

	viewer->createViewPort(0.0,0.0,0.5,1.0,v1);  //四个窗口参数分别对应x_min,y_min,x_max.y_max.
	viewer->createViewPort(0.5,0.0,1.0,1.0,v2);

	viewer->setBackgroundColor(0.0,0.05,0.05,v1); //设着两个窗口的背景色
	viewer->setBackgroundColor(0.05,0.05,0.05,v2);

	pcl::visualization::PointCloudColorHandlerCustom<PointT> sources_cloud_color(cloud,250,0,0); //设置源点云的颜色为红色
	viewer->addPointCloud(cloud,sources_cloud_color,"The Whole PointCloud",v1);
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,2,"The Whole PointCloud");  //设置显示点的大小
	

	pcl::visualization::PointCloudColorHandlerCustom<PointT>  res_cloud(feature_extract.cloud_sutura,0,255,0);  //设置配准结果为白色
	viewer->addPointCloud(feature_extract.cloud_sutura,res_cloud,"Sutura Cloud",v2);
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,2,"Sutura Cloud");

	viewer->addText("The Whole PointCloud",200,30,18,0.9,0.9,0.9,"Before",v1);
	viewer->addText("Sutura PointCloud",200,30,18,0.9,0.9,0.9,"After",v2);

	while(!viewer->wasStopped())
	{
		viewer->spinOnce();  //运行视图
	}
}
#endif

int MYREALSENSE::get_LR()
try
{
    
    while (true)
    {
        rs2::frameset frames=pipe.wait_for_frames(30);
        rs2::frame color_frame=frames.get_color_frame();
        rs2::depth_frame depth_frame=frames.get_depth_frame();
        rs2::video_frame ir_frame_left = frames.get_infrared_frame(1);
		rs2::video_frame ir_frame_right = frames.get_infrared_frame(2);

		dMat_left = cv::Mat(cv::Size(pic_width, pic_height), CV_8UC1, (void*)ir_frame_left.get_data());
		dMat_right = cv::Mat(cv::Size(pic_width, pic_height), CV_8UC1, (void*)ir_frame_right.get_data());
        dMat_depth=cv::Mat(cv::Size(pic_width,pic_height),CV_8UC1,(void*)depth_frame.get_data());
        dMat_color=cv::Mat(cv::Size(pic_width,pic_height),CV_8UC3,(void*)color_frame.get_data());
		
        cv::imshow("img_color", dMat_color);
		//cv::imshow("img_r", dMat_right);
        cv::imshow("depth",dMat_depth);

        cv::imwrite("/home/yons/projects/realsense/res/color.jpg",dMat_color);
        //cv::imwrite("/home/yons/projects/realsense/res/right.jpg",dMat_right);
        //feature_extractor.imgL=cv::imread("/home/yons/projects/realsense/res/left.jpg");
        dMat_color.copyTo(feature_extract.imgL);
        feature_extract.imgR=cv::imread("/home/yons/Documents/Wzx/before/preprocessed/2.jpg");
        cv::resize(feature_extract.imgR,feature_extract.imgR,feature_extract.imgL.size());
        cout<<feature_extract.imgL.size()<<' '<<feature_extract.imgR.size()<<endl;
        //feature_extractor.imgR=dMat_right;
        feature_extract.getdsp();
        feature_extract.goodmatcher();
        feature_extract.get_homography();
		char c = cv::waitKey(30);

        //float width=depth.get_width();
        //float height=depth.get_height();
        
        //float dist_to_center=depth.get_distance(width/2,height/2);
        //std::cout<<"The camera is facing an object "<<dist_to_center<<" meters away \r";
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