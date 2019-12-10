#include "my_realsense.hpp"
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
    depth=Mat(Size(pic_width,pic_height),CV_16UC1);
    color=Mat(Size(pic_width,pic_height),CV_8UC3);

    cloud_realsense=PointCloudT::Ptr (new PointCloudT);
    cloud_filtered=PointCloudT::Ptr(new PointCloudT);
    cloud_plane=PointCloudT::Ptr(new PointCloudT);
    cloud_f=PointCloudT::Ptr(new PointCloudT);
    target=PointCloudT::Ptr(new PointCloudT);
}

MYREALSENSE::~MYREALSENSE()
{
   
}

int MYREALSENSE::extract_target()
{
    pcl::PCLPointCloud2::Ptr cloud_blob (new pcl::PCLPointCloud2), cloud_filtered_blob (new pcl::PCLPointCloud2);

    pcl::PCDReader reader;
    reader.read ("/home/yons/projects/realsense/data/table_scene_lms400.pcd", *cloud_blob);

    std::cerr << "PointCloud before filtering: " << cloud_blob->width * cloud_blob->height << " data points." << std::endl;

    // Create the filtering object: downsample the dataset using a leaf size of 1cm
    pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
    sor.setInputCloud (cloud_blob);
    sor.setLeafSize (0.01f, 0.01f, 0.01f);
    sor.filter (*cloud_filtered_blob);

    // Convert to the templated PointCloud
    pcl::fromPCLPointCloud2 (*cloud_filtered_blob, *cloud_filtered);

    std::cerr<<"Pointcloud after filtering: "<<cloud_filtered->width*cloud_filtered->height<<std::endl;

    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
    pcl::PointIndices::Ptr indices_in  (new pcl::PointIndices ()); 
    pcl::PointIndices indices_rem;
    //segmentation
    //optional
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    // Optional
    seg.setOptimizeCoefficients (true);
    // Mandatory
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (1000);
    seg.setDistanceThreshold (0.01);

    pcl::PCDWriter writer;
    //extract plane
    pcl::ExtractIndices<PointT> extract;
    PointCloudT::Ptr cloud_filtered_backup=PointCloudT::Ptr(new PointCloudT);
    for(int i=0;i<cloud_filtered->points.size();i++)
        cloud_filtered_backup->points.push_back(cloud_filtered->points[i]);
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
        for(int i=0;i<inliers->indices.size();++i)
            indices_in->indices.push_back(inliers->indices[i]);
        std::stringstream ss;
        ss << "table_scene_lms400_plane_" << i << ".pcd";
        writer.write<pcl::PointXYZ> (ss.str (), *cloud_plane, false);
        view_pointcloud(cloud_plane);
        // Create the filtering object
        extract.setNegative (true);
        extract.filter (*cloud_f);
        cloud_filtered.swap (cloud_f);
        i++;
    }

    
    pcl::ExtractIndices<PointT> eifilter (true); // Initializing with true will allow us to extract the removed indices
	eifilter.setInputCloud (cloud_filtered_backup);
	eifilter.setIndices (indices_in);
    eifilter.filter(*cloud_plane);
	eifilter.getRemovedIndices (indices_rem);
	cout<<"the target pointcloud size: "<<' '<<indices_rem.indices.size()<<endl;
	if(indices_rem.indices.size()!=0)
	{
		for (int i = 0; i < indices_rem.indices.size(); ++i)
		{
			target->points.push_back(cloud_filtered_backup->points.at(indices_rem.indices[i]));
		}
	}
	else
	{
        cout<<"nothing"<<endl;
		target=cloud_filtered;
	}
    view_pointcloud(cloud_filtered_backup);
    view_pointcloud(target);
    writer.write<pcl::PointXYZ> ("/home/yons/projects/realsense/res/target.pcd", *target, false);

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

    result=Mat::zeros(color.rows,color.cols,CV_8UC3);
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
            //std::cout<<Pdc3[0]<<' '<<Pdc3[1]<<' '<<Pdc3[2]<<std::endl;
            cloud_realsense->points.push_back(PointT(Pdc3[0]*1000,Pdc3[1]*1000,Pdc3[2]*1000));
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
     //显示 
    extract_target();
    //pcl::io::savePLYFileASCII("/home/yons/projects/realsense/res/pointcloud.ply", *cloud_realsense); 
    //view_pointcloud(cloud_realsense);
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
	viewer = boost::shared_ptr<pcl::visualization::PCLVisualizer>(new pcl::visualization::PCLVisualizer(WindowName));
	viewer->addPointCloud(cloud, WindowName);
	viewer->resetCameraViewpoint(WindowName);
	viewer->addCoordinateSystem(10);
	//viewer->setFullScreen(true); // Visualiser window size
	viewer->setSize(screen_width,screen_height);
	while (!viewer->wasStopped())
	{
		viewer->spin();
		//boost::this_thread::sleep(boost::posix_time::microseconds(10));
        viewer->close();
	}
}


int MYREALSENSE::get_LR()
try
{
    FEATURE_EXTRACT feature_extractor;
    while (true)
    {
        rs2::frameset frames=pipe.wait_for_frames();
        rs2::depth_frame depth=frames.get_depth_frame();
        rs2::video_frame ir_frame_left = frames.get_infrared_frame(1);
		rs2::video_frame ir_frame_right = frames.get_infrared_frame(2);

		cv::Mat dMat_left = cv::Mat(cv::Size(pic_width, pic_height), CV_8UC1, (void*)ir_frame_left.get_data());
		cv::Mat dMat_right = cv::Mat(cv::Size(pic_width, pic_height), CV_8UC1, (void*)ir_frame_right.get_data());
        cv::Mat dMat_depth=cv::Mat(cv::Size(pic_width,pic_height),CV_8UC1,(void*)depth.get_data());

		cv::imshow("img_l", dMat_left);
		cv::imshow("img_r", dMat_right);
        cv::imshow("depth",dMat_depth);
        //cv::imwrite("/home/yons/projects/realsense/res/left.jpg",dMat_left);
        //cv::imwrite("/home/yons/projects/realsense/res/right.jpg",dMat_right);
        //feature_extractor.imgL=cv::imread("/home/yons/projects/realsense/res/left.jpg");
        //feature_extractor.imgR=cv::imread("/home/yons/projects/realsense/res/right.jpg");
        feature_extractor.imgL=dMat_left;
        feature_extractor.imgR=dMat_right;
        feature_extractor.goodmatcher();
        feature_extractor.get_homography();
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