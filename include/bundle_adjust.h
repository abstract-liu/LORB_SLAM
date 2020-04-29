#ifndef BUNDLE_ADJUST_H
#define BUNDLE_ADJUST_H

#include "common.h"
#include "frame.h"
#include "camera.h"
#include <opencv2/core/mat.hpp>

namespace Simple_ORB_SLAM
{ 

class BA
{
public:
	BA();
	void static PnpPoseAdjust(const vector<cv::Point3f>& kps3d, const vector<cv::Point2f>& kps2d, cv::Mat& R, cv::Mat& T, Camera* camera);
	void static LocalPoseAdjust(const vector<cv::Point3f>& kps3d, const vector<cv::Point2f>& kps2d, cv::Mat& R, cv::Mat& T, Camera* camera);


};


}





#endif 
