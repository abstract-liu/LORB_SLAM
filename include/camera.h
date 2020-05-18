#ifndef CAMERA_H
#define CAMERA_H 

#include "common.h"
#include <opencv2/core/mat.hpp>

namespace Simple_ORB_SLAM
{ 


class Camera
{
public:
	Camera(const string& strParaFile);
	void RectifyL(const cv::Mat& img_src, cv::Mat& img_rec);
	void RectifyR(const cv::Mat& img_src, cv::Mat& img_rec);
	bool Project(const Point3f& pt_3d, Point2f& pt_2d);


public:

	//camera settings
	cv::Mat K_l, K_r, P_l, P_r, R_l, R_r, D_l, D_r;
	int rows_l, rows_r, cols_l, cols_r;
	cv::Mat M1l,M2l,M1r,M2r;
	float bf, cx, cy, fx, fy;
	cv::Mat pi;
	float fps;

	int mMinPNPN;
	float mMaxNorm;

	//view point
	float mViewpointF, mViewpointX, mViewpointY, mViewpointZ;


	//draw settings
	float mKeyFrameSize, mKeyFrameLineWidth, mPointSize, mCameraSize, mCameraLineWidth;

	int mnFeatures,mnLevels,mfIniThFAST,mfMinThFAST;
	float mfScaleFactor;

};

 }











#endif 
