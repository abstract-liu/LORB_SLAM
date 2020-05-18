#ifndef MAP_POINT_H
#define MAP_POINT_H

#include "common.h"
#include "frame.h"
#include "map.h"

namespace Simple_ORB_SLAM
{ 

class Frame;
class Map;

class MapPoint
{
public:
	
	MapPoint(cv::Point3f pt, Frame* pF, Map* pMap);
	MapPoint(cv::Point3f pt, Frame* pF, Map* pMap, size_t idx);
	
	//pose
	void SetWorldPos(cv::Point3f kp3d);
	cv::Point3f GetPos();
	
	//descriptor
	cv::Mat GetDescriptor();
	void ComputeDescriptor();


	//observations operation
	void AddObservation(Frame* pF, size_t id);
	void EraseObservation(Frame* pF);
	std::map<Frame*, size_t> GetObservations();
	
	void IncreaseFound(size_t n = 1);
	void IncreaseVisible(size_t n = 1);
	float GetFoundRatio();
	

	bool IsInFrame(Frame* pF);
	
	//bad
	bool IsBad();
	void SetBadFlag();
	
	float GetMinDistanceInvariance();
	float GetMaxDistanceInvariance();
	cv::Mat GetNormal();
	int PredictScale(const float &currentDist, Frame* pKF);

public:
	size_t mnObs;
	size_t mnFirstFId;
	
	// Variables used by the tracking
    float mTrackProjX;
    float mTrackProjY;
    float mTrackProjXR;
    int mnTrackScaleLevel;
    float mTrackViewCos;
    // TrackLocalMap - SearchByProjection中决定是否对该点进行投影的变量
    // mbTrackInView==false的点有几种：
    // a 已经和当前帧经过匹配（TrackReferenceKeyFrame，TrackWithMotionModel）但在优化过程中认为是外点
    // b 已经和当前帧经过匹配且为内点，这类点也不需要再进行投影
    // c 不在当前相机视野中的点（即未通过isInFrustum判断）
    bool mbTrackInView;
	size_t mnLastFrameSeen;
	float mfMinDistance, mfMaxDistance;
	// Mean viewing direction
    // 该MapPoint平均观测方向
    cv::Mat mNormalVector;


private:

	Map* mpMap;

	cv::Point3f mWorldPos;
	cv::Mat mDescriptor;
	
	std::map<Frame*, size_t> mObservations;

	bool mbBadFlag;
	
	size_t mnVisible;
	size_t mnFound;


};


 }




#endif 
