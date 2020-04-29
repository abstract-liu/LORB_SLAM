#include "../include/map_point.h"
#include <opencv2/core/types.hpp>

namespace Simple_ORB_SLAM
{

	
MapPoint::MapPoint(const cv::Point3f& mp, const cv::Mat& descriptor)
{
	mPose = mp;
	mDescriptor = descriptor.clone();
}		


cv::Point3f MapPoint::GetPose()
{
	return mPose;
}

cv::Mat MapPoint::GetDescriptor()
{
	cv::Mat descriptor = mDescriptor.clone();
	return descriptor;

}



}
