#ifndef VISUAL_ODOMETRY_H
#define VISUAL_ODOMETRY_H 

#include "common.h"
#include "camera.h"
#include "frame.h"
#include "map.h"
#include "bundle_adjust.h"

#include <opencv2/core/types.hpp>

namespace Simple_ORB_SLAM
{  

class VisualOdometry
{
public:
	VisualOdometry(const string& strParaFile, Camera* camera, Map* map);
	~VisualOdometry();

	void Run(const string& timeStamp, const cv::Mat& imgLeft, const cv::Mat& imgRight);

	//api
	cv::Mat GetCurrInvPos();

protected:
	
	void Initialize();
	void PnPMatchFeature();
	bool EstimatePosePnP();
	void UpdatePnP();

	void LocalMatchFeature();
	void UpdateMap();

	void Rotation2Quaternion();
	void SaveTrajectory();

	void EstimatePoseMotion();


public:
	Map* mpGlobalMap;
	Map* mpLocalMap;

private:
	
	Camera* mpCamera;

	bool isFistFrame;
	
	//last frame and current frame
	cv::Mat mCurrImg, mPrevImg;
	Frame* mPrevFrame; 
	Frame* mCurrFrame;
	
	//last frame and current frame key points
	vector<cv::Point2f> mPrevKps2d;
	vector<cv::Point2f> mCurrKps2d;
	vector<cv::Point3f> mPrevKps3d;

	//pose variables
	Mat mTcw, mTcc, mTwt;
	Mat mRvec, mTvec;
	float mTUM[7];

	//pose lock
	std::mutex mPosLock;

	//ofstream
	ofstream mOutFile;

	string mTimestamp;

};

}






#endif 
