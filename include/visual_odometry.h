#ifndef VISUAL_ODOMETRY_H
#define VISUAL_ODOMETRY_H 

#include "common.h"
#include "camera.h"
#include "frame.h"
#include "map.h"
#include "bundle_adjust.h"
#include "matcher.h"

#include <opencv2/core/types.hpp>
#include <vector>

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
	
	//track
	bool EstimatePoseMotion();
	bool EstimatePosePnP();
	bool EstimatePoseLocal();

	//aid function
	void UpdatePrevFrame();
	void UpdateLocalMap();

	//add key frame
	void AddKeyFrame();

	void UpdateMap();

	void Rotation2Quaternion();
	void SaveTrajectory();



public:
	Map* mpMap;

private:
	
	Camera* mpCamera;

	bool isFistFrame;
	
	//last frame and current frame
	Frame* mPrevFrame; 
	Frame* mCurrFrame;
	Frame* mReferFrame;

	//local map
	std::vector<Frame*> mpLocalKeyFrames;
	std::set<MapPoint*> mpLocalMapPoints;
		

	//pose variables
	cv::Mat mTcc;
	float mTUM[7];

	//pose lock
	std::mutex mPosLock;

	//ofstream
	ofstream mOutFile;

	string mTimestamp;

};

}






#endif 
