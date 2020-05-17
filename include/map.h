#ifndef MAP_H
#define MAP_H

#include "common.h"
#include "map_point.h"
#include "frame.h"

namespace Simple_ORB_SLAM
{ 

class MapPoint;
class Frame;

class Map
{
public:
	Map();

	//map point
	void AddMapPoint(MapPoint* pMP);
	void EraseMapPoint(MapPoint* pMP);
	std::set<MapPoint*> GetPoints();
	size_t GetMapPointsNum();	

	//key frame
	void AddFrame(Frame* pF);
	void EraseFrame(Frame* pF);
	std::set<Frame*> GetFrames();
	size_t GetFramesNum();


public:


private:
	std::set<MapPoint*> mspMapPoints;
	std::set<Frame*> mspFrames;

	std::mutex mPointLock;
	std::mutex mFrameLock;
};




}



#endif 
