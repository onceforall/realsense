#pragma once
#include <iostream>
#include "opencv2/opencv.hpp"
#include "opencv2/highgui.hpp"
#include "/usr/local/include/opencv2/xfeatures2d.hpp"
#include <vector>

using namespace cv;
class FEATURE_EXTRACT
{
public:
      FEATURE_EXTRACT();
      virtual ~FEATURE_EXTRACT(){};  
      void getdsp();
      void goodmatcher();
      void get_homography();
      void printmatrix();
      Mat imgL;
      Mat imgR;
private:
      std::vector<KeyPoint> keypointsL,keypointsR;
      Mat descriptorsL,descriptorsR;
      std::vector<std::vector<DMatch>> match;
      std::vector<DMatch> goomatch;
      std::vector<Point2f> psL,psR;
      Mat homo;
      const int pic_width=640;
      const int pic_height=480;
};