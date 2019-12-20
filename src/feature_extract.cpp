#include "feature_extract.hpp"
#include "my_realsense.hpp"

using namespace cv;
using namespace std;
FEATURE_EXTRACT::FEATURE_EXTRACT()
{
    imgL=Mat(Size(pic_width,pic_height),CV_8UC3);
    imgR=Mat(Size(pic_width,pic_height),CV_8UC3);
    
}
 void FEATURE_EXTRACT::getdsp()
 {
     Mat imgl,imgr;
     GaussianBlur(imgL,imgl,Size(3,3),0.5);
     GaussianBlur(imgR,imgr,Size(3,3),0.5);
    
    Ptr<Feature2D> sift=xfeatures2d::SIFT::create();
    sift->detect(imgl,keypointsL);
    sift->detect(imgr,keypointsR);
    sift->compute(imgl,keypointsL,descriptorsL); 
    sift->compute(imgr,keypointsR,descriptorsR);
}


void  FEATURE_EXTRACT::goodmatcher()
{
    getdsp();
    if(descriptorsL.empty() || descriptorsR.empty() || keypointsL.empty() || keypointsR.empty())
        return;

    const float ratio=0.6;//1.f/1.5f;
    BFMatcher matcher(NORM_L2,false);
    matcher.knnMatch(descriptorsL,descriptorsR,match,2);
    
    for(size_t i=0;i<match.size();i++)
    {
        const DMatch& bestmatch=match[i][0];
        const DMatch& bettermatch=match[i][1];
        float distanceRatio=bestmatch.distance/bettermatch.distance;
        if(distanceRatio<ratio)
            goodmatch.push_back(bestmatch);
    }
    sort(goodmatch.begin(), goodmatch.end());

    if(goodmatch.size()>4)
    {
        for(size_t i=0;i<goodmatch.size();i++)
        {
            psL.push_back(keypointsL[goodmatch[i].queryIdx].pt);
            psR.push_back(keypointsR[goodmatch[i].trainIdx].pt);
        }
        //goodmatch.clear();
        drawMatches(imgL,keypointsL,imgR,keypointsR,goodmatch,matchimg,(0,255,0),(255,0,0));
        imwrite("/home/yons/projects/realsense/res/match.png",matchimg);
    }
    goodmatch.clear();
}

Mat src,dst;
Mat res,gray,filtered;

int s1=0,s2=0,s3=3;
void trackBar(int,void*)
{
    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;
    Mat imageContours=Mat::zeros(Size(640,480),CV_8UC1);
    Mat Contours=Mat::zeros(Size(640,480),CV_8UC1);
    Canny(filtered,dst,s1,s2,3);
    dilate(dst,dst,Mat(),Point(-1,-1),1);
    erode(dst,dst,Mat(),Point(-1,-1),2);
    if(s3%2!=1) s3+=1;
    GaussianBlur(dst,dst,Size(s3,s3),0);
    findContours(dst,contours,hierarchy,CV_RETR_EXTERNAL, CHAIN_APPROX_SIMPLE, Point(0, 0));
    
    for (int i = 0; i < contours.size(); i++)
    {
        //compute contour area
        double area = contourArea(contours[i]);
        
        //remove contours whose area less than 1000
        if (area < 300.0 || area>2000.0)
            continue;
        else
        {
            for(int j=0;j<contours[i].size();j++)
            {
                Point p=Point(contours[i][j].x,contours[i][j].y);
                Contours.at<uchar>(p)=255;
                //vec_sutura.push_back(Point(contours[i][j].x,contours[i][j].y));
                cout<<"contour "<<i<<" area: "<<area<<endl;
            }     
        }
        //char ch[256];  
        //sprintf(ch,"%d",i);  
        //string str=ch;  
        //cout<<"向量hierarchy的第" <<str<<" 个元素内容为："<<endl<<hierarchy[i]<<endl<<endl;  
        //绘制轮廓  
        drawContours(imageContours,contours,i,Scalar(255),1,8,hierarchy);  
    }  
    //imshow("gray",gray);
    //imshow("res",res);
    imshow("output", dst);
    imshow("Contours Image",imageContours); //轮廓  
    imshow("Point of Contours",Contours);   //向量contours内保存的所有轮廓点集  
}

void FEATURE_EXTRACT::get_mask(Mat mask_pic)
{
    vector<vector<Point2i>> Ctour;
    if(mask_pic.empty())
    {
        printf("can't load image \n");
        return;
    }
    Mat pic_contour=Mat::zeros(Size(mask_pic.cols,mask_pic.rows),CV_8UC1);
    int rowNumber = mask_pic.rows;    //行数
	int colNumber = mask_pic.cols*mask_pic.channels();   //列数*通道数=每一行元素的个数
 
	for(int i = 0; i < rowNumber; i++)  //行循环，可根据需要换成rowNumber
	{
		uchar* data = mask_pic.ptr<uchar>(i);  //获取第i行的首地址
		for(int j = 0; j < colNumber; j++)  //列循环，同理
		{
			int intensity = data[j];
            if(intensity==100)
            { 
                vector<Point> pointse;
                vec_sutura.push_back(Point(i,j));
                if(j>=1 && j<colNumber-1 && data[j-1]<100 && data[j+1]==100)
                    pointse.push_back(Point(i,j));
                if(j>=1 && j<colNumber-1 && data[j-1]==100 && data[j+1]<100)
                    pointse.push_back(Point(i,j));
                if(pointse.size()>0)
                    Ctour.push_back(pointse);
            }
		}
	}
    cout<<"index done"<<endl;
    for(auto end:Ctour)
    {
        for(int i=0;i<end.size();i++)
            pic_contour.at<uchar>(end[i].x,end[i].y)=100;
    }
    imshow("contours",pic_contour);
    waitKey(0);
}

void FEATURE_EXTRACT::sutura_detect(Mat skull_pic)
{
    if(skull_pic.empty())
    {
        printf("can not load image \n");
        return;
    }
    if(skull_pic.channels()==3)
        cvtColor(skull_pic,gray,COLOR_BGR2GRAY);
    else
    {
        gray=skull_pic.clone();
    }
    
    threshold(gray,res,100,255,THRESH_BINARY+THRESH_OTSU);
    GaussianBlur(res,filtered,Size(s3,s3),0);
 
    cvNamedWindow("input",CV_WINDOW_AUTOSIZE);
    imshow("input",skull_pic);

    cvNamedWindow("output",CV_WINDOW_AUTOSIZE);
    createTrackbar("canny1","output",&s1,255,trackBar);
    createTrackbar("canny2", "output", &s2, 255, trackBar);
    createTrackbar("gauss","output",&s3,9,trackBar);
    //GaussianBlur(src,src,Size(3,3),0);
    waitKey(0);

    #if 0
    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;
    Mat imageContours=Mat::zeros(Size(640,480),CV_8UC1);
    Mat Contours=Mat::zeros(Size(640,480),CV_8UC1);
    findContours(dst,contours,hierarchy,CV_RETR_EXTERNAL, CHAIN_APPROX_SIMPLE, Point(0, 0));
    
    for (int i = 0; i < contours.size(); i++)
    {
        //compute contour area
        double area = contourArea(contours[i]);
        
        //remove contours whose area less than 1000
        if (area < 300.0 || area>2000.0)
            continue;
        else
        {
            for(int j=0;j<contours[i].size();j++)
            {
                Point p=Point(contours[i][j].x,contours[i][j].y);
                Contours.at<uchar>(p)=255;
                //vec_sutura.push_back(Point(contours[i][j].x,contours[i][j].y));
                cout<<"contour "<<i<<" area: "<<area<<endl;
            }     
        }
        //char ch[256];  
        //sprintf(ch,"%d",i);  
        //string str=ch;  
        //cout<<"向量hierarchy的第" <<str<<" 个元素内容为："<<endl<<hierarchy[i]<<endl<<endl;  
        //绘制轮廓  
        //drawContours(imageContours,contours,i,Scalar(255),1,8,hierarchy);  
    }  
    //imshow("gray",gray);
    //imshow("res",res);
    imshow("Contours Image",imageContours); //轮廓  
    imshow("Point of Contours",Contours);   //向量contours内保存的所有轮廓点集  
    #endif 
}

void FEATURE_EXTRACT::printmatrix()
{
    int rows=homo.size[0];
    int cols=homo.size[1];
    if(rows==0 || cols==0)
    {
        cout<<"invalid homography"<<endl;
        return;
    }
    for(int i=0;i<rows;i++)
    {
        for(int j=0;j<cols;j++)
            cout<<homo.at<float>(i,j)<<' ';
        cout<<endl;
    }    
    cout<<endl;
}


void FEATURE_EXTRACT::get_homography()
{
    homo=findHomography(psL,psR,RANSAC);
    printmatrix();
    warpPerspective(imgL,imgR,homo,imgR.size());
    imwrite("/home/yons/projects/realsense/res/homo.png",imgR);
}



