
#include "../include/map.h"
#include <mutex>
#include <vector>

namespace Simple_ORB_SLAM
{

Map::Map()
{
	mKFN = 0;
	mMPN = 0;
}

void Map::AddMapPoint(MapPoint* mapPoint)
{	
	std::unique_lock<std::mutex> lock(mPointLock);
	mpMapPoints.push_back(mapPoint);

	mMPN += 1;
	mUnSeenTimes.push_back(0);
}

void Map::AddKeyFrame(Frame* keyFrame)
{
	std::unique_lock<std::mutex> lock(mFrameLock);
	mpKeyFrames.push_back(keyFrame);
	mKFN += 1;
}


std::vector<Frame*> Map::GetKeyFrames()
{
	std::unique_lock<std::mutex> lock(mFrameLock);
	return mpKeyFrames;
}


std::vector<MapPoint*> Map::GetPoints()
{
	std::unique_lock<std::mutex> lock(mPointLock);
	return mpMapPoints;
}

void Map::EraseBadPoints()
{

	std::unique_lock<std::mutex> lock(mPointLock);
	for(size_t i=0; i<mMPN; i++)
	{
		//warning 
		if(mUnSeenTimes[i] > 5)
		{
			mpMapPoints.erase(mpMapPoints.begin()+i);
			mMPN -= 1;
		}
	}
}

}
