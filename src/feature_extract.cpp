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

void FEATURE_EXTRACT::get_mask(Mat* mask_pic)
{
    int dx[]={-1,0,1},dy[]={-1,0,1};  //find outmost border,if there exists a 0-pixel in 8-neighbors,it's border point 
    //vector<vector<Point2i>> Ctour;
    Mat binary;
    if(mask_pic->empty())
    {
        printf("can not load image \n");
        return;
    }
    if(mask_pic->channels()==3)
        cvtColor(*mask_pic,gray,COLOR_BGR2GRAY);
    else
    {
        gray=mask_pic->clone();
    }
    
    threshold(gray,binary,0,255,THRESH_BINARY+THRESH_OTSU);
    //GaussianBlur(binary,binary,Size(s3,s3),0);

    //closure operation
    Mat morphImage;
    Mat kernel = getStructuringElement(MORPH_RECT, Size(3, 3), Point(-1, -1));
    morphologyEx(binary, morphImage, MORPH_CLOSE, kernel, Point(-1, -1), 2);
    
    Mat pic_contour=Mat::zeros(Size(morphImage.cols,morphImage.rows),CV_8UC1);
    int rowNumber = mask_pic->rows;    //行数
	int colNumber = mask_pic->cols*mask_pic->channels();   //列数*通道数=每一行元素的个数
 
	for(int row = 0; row < rowNumber; row++)  //行循环，可根据需要换成rowNumber
	{
        //vector<Point> pointse;
		//uchar* data = morphImage.ptr<uchar>(row);  //获取第i行的首地址
		for(int col = 0; col < colNumber; col++)  //列循环，同理
		{
			//int intensity = data[col];
            if(morphImage.at<uchar>(row,col))
            {
                for(int i=0;i<3;i++)
                {
                    int x=row+dx[i];
                    for(int j=0;j<3;j++)
                    {
                        int y=col+dy[j];
                        if(x>=0 && x<rowNumber && y>=0 && y<colNumber)
                        {
                            //uchar* curdata=morphImage.ptr<uchar>(x);
                            //if(curdata[y]==0)
                            if(morphImage.at<uchar>(x,y)==0)
                            {
                                vec_sutura.push_back(Point(row,col));
                                break;
                            } 
                        }
                    }
                }         
            } 
		}
        //if(pointse.size()>0)
            //Ctour.push_back(pointse);
	}
    cout<<"index done"<<endl;
    for(auto end:vec_sutura)
    {
        pic_contour.at<uchar>(end.x,end.y)=255;
    }
    *mask_pic=pic_contour.clone();
    //imshow("contours",pic_contour);
    //waitKey(0);
}

void FEATURE_EXTRACT::sutura_detect(Mat skull_pic)
{
    Mat binary;
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
    
    threshold(gray,binary,0,255,THRESH_BINARY+THRESH_OTSU);
    //GaussianBlur(binary,filtered,Size(s3,s3),0);
 
    cvNamedWindow("input",CV_WINDOW_AUTOSIZE);
    imshow("input",skull_pic);

    cvNamedWindow("output",CV_WINDOW_AUTOSIZE);

    //closure operation
    Mat morphImage;
    Mat kernel = getStructuringElement(MORPH_RECT, Size(3, 3), Point(-1, -1));
    morphologyEx(binary, morphImage, MORPH_CLOSE, kernel, Point(-1, -1), 2);
    imshow("morphology", morphImage);

    // find the max contour
    vector<vector<Point>> contours;
    vector<Vec4i> hireachy;

    vector<Point> Maxcontour;
    int Maxindex=-1;
    double Maxarea=0;
    
    findContours(morphImage, contours, hireachy, CV_RETR_EXTERNAL, CHAIN_APPROX_SIMPLE, Point());
    Mat connImage = Mat::zeros(skull_pic.size(), CV_8UC3);
    for (size_t t = 0; t < contours.size(); t++){
        Rect rect = boundingRect(contours[t]);
       
        double area = contourArea(contours[t]);
        double len = arcLength(contours[t], true);
        if(area>Maxarea)
        {
            Maxcontour=contours[t];
            Maxarea=area;
            Maxindex=t;
        }
    }
    drawContours(connImage, contours, Maxindex, Scalar(0, 0, 255), 1, 8, hireachy);
    imshow("output", connImage);

    if(Maxindex!=-1)
    {
        for(auto point:Maxcontour)
            vec_sutura.push_back(Point(point.x,point.y));  
    }
    
    //createTrackbar("canny1","output",&s1,255,trackBar);
    //createTrackbar("canny2", "output", &s2, 255, trackBar);
    //createTrackbar("gauss","output",&s3,9,trackBar);
    //GaussianBlur(src,src,Size(3,3),0);
    waitKey(0);
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



