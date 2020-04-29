#ifndef FRAME_H
#define FRAME_H

#include "common.h"
#include "map_point.h"
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <vector>
#include "camera.h"


namespace Simple_ORB_SLAM
{ 

class Frame
{
public:
	Frame();
	Frame(const cv::Mat& imgLeft, const cv::Mat& imgRight, Camera* camera);

	//detect feature
	void DetectFeature(const cv::Mat& imgLeft, const cv::Mat& imgRight);

	//stereo
	void Stereo(const cv::Mat& imgLeft, const cv::Mat& imgRight);
	
	//judge is visible
	bool IsVisible(MapPoint* point, cv::Point2f ptInImg);


	//set or get pose
	void SetPose(const cv::Mat& Tcw);
	cv::Mat GetPose();
	cv::Mat GetInvPose();
	
	//get Kps
	std::vector<cv::Point2f> GetKps2d();
	std::vector<cv::Point3f> GetKps3d();
	cv::Mat GetDescriptors();

public:
	size_t mN;


private:
	//camera parameter	
	Camera* mpCamera;

	//image's orb feature 
	std::vector<cv::KeyPoint> mKps, mKpsRight;
	std::vector<cv::Point2f> mKps2d, mKpsRight2d;
	std::vector<cv::Point3f> mKps3d;
	cv::Mat mDescriptors, mDescriptorsRight;
	
	//key frame tag 
	bool isKeyFrame;

	//camera's pose
	cv::Mat mTcw, mTwc;

};




 }




#endif 
