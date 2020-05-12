#ifndef MAP_POINT_H

#define MAP_POINT_H

#include "common.h"
#include "frame.h"
#include "map.h"

namespace Simple_ORB_SLAM
{ 

class MapPoint
{
public:
	MapPoint(const cv::Point3f& point, const Frame* pF, Map* pMap);
	
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
	
	void IncreaseFound(size_t n);
	void IncreaseVisible(size_t n);
	float GetFoundRatio();
	

	bool IsInFrame(Frame* pF);
	
	//bad
	bool IsBad();
	void SetBadFlag();
	



public:
	size_t mnObs;
	size_t mnFirstFId;


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
