#include "feature_extract.h"

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
    using namespace std;
    const int k=2;
    const float ratio=0.75;//1.f/1.5f;
    BFMatcher matcher(NORM_L2,false);
    vector<DMatch> goodmatch;
    Mat matchimg;
    matcher.knnMatch(descriptorsL,descriptorsR,match,2);
    for(size_t i=0;i<match.size();i++)
    {
        const DMatch& bestmatch=match[i][0];
        const DMatch& bettermatch=match[i][1];
        float distanceRatio=bestmatch.distance/bettermatch.distance;
        if(distanceRatio<ratio)
            goodmatch.push_back(bestmatch);
    }
    std::sort(goodmatch.begin(), goodmatch.end());
    if(goodmatch.size()>4)
    {
        for(size_t i=0;i<goodmatch.size()*0.3;i++)
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



