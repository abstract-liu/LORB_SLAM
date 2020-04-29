#ifndef MAP_H
#define MAP_H

#include "common.h"
#include "map_point.h"
#include "frame.h"
#include <mutex>
#include <vector>

namespace Simple_ORB_SLAM
{ 

class Map
{
public:
	Map();

	void AddMapPoint(MapPoint* mapPoint);
	void AddKeyFrame(Frame* keyFrame);
	
	std::vector<Frame*> GetKeyFrames();
	std::vector<MapPoint*> GetPoints();

	void EraseBadPoints();

	size_t mKFN;
	size_t mMPN;

public:
	std::vector<int> mUnSeenTimes;


private:
	std::vector<MapPoint*> mpMapPoints;
	std::vector<Frame*> mpKeyFrames;

	std::mutex mPointLock;
	std::mutex mFrameLock;
};




}



#endif 
