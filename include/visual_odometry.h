#ifndef VISUAL_ODOMETRY_H
#define VISUAL_ODOMETRY_H 

#include "common.h"
#include "camera.h"
#include "frame.h"
#include "map.h"
#include "map_point.h"
#include "bundle_adjust.h"
#include "matcher.h"
#include "local_mapping.h"


namespace Simple_ORB_SLAM
{  

class VisualOdometry
{
public:
	VisualOdometry(LocalMapping* lcMapper, Camera* camera, Map* map);
	~VisualOdometry();

	void Run(const string& timeStamp, const cv::Mat& imgLeft, const cv::Mat& imgRight);
	
	cv::Mat GetCurrInvPos();

protected:
	
	void Initialize();
	
	//track
	bool EstimatePoseMotion(Frame* pF);
	bool EstimatePosePnP();
	bool EstimatePoseLocal();

	//aid function
	void UpdateFrame(Frame* pF);
	void UpdateLocalMap();

	//add key frame
	void AddKeyFrame();

	void UpdateMap();

	void SaveTrajectory();



public:
	Map* mpMap;

private:
	
	Camera* mpCamera;
	LocalMapping* mpLocalMapper;	

	bool isFistFrame;
	
	//last frame and current frame
	Frame* mPrevFrame; 
	Frame* mCurrFrame;
	Frame* mReferFrame;

	//local map
	std::vector<Frame*> mpLocalKeyFrames;
	std::set<MapPoint*> mpLocalMapPoints;
		

	//pose variables
	cv::Mat mTcc, mTwc;
	float mTUM[7];

	//pose lock
	std::mutex mPosLock;

	//ofstream
	ofstream mOutFile;

	string mTimestamp;

};

}






#endif 
