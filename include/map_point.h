#ifndef MAP_POINT_H
#define MAP_POINT_H

#include "common.h"

namespace Simple_ORB_SLAM
{ 

class MapPoint
{
public:
	MapPoint(const cv::Point3f& point, const cv::Mat& descriptor);
	cv::Point3f GetPose();
	cv::Mat GetDescriptor();

private:
	cv::Point3f mPose;
	cv::Mat mDescriptor;



};


 }




#endif 
