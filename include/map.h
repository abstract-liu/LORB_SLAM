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

	//map point
	void AddMapPoint(MapPoint* pMP);
	void EraseMapPoint(MapPoint* pMP);
	std::set<MapPoint*> GetPoints();
	

	//key frame
	void AddFrame(Frame* pF);
	void EraseFrame(Frame* pF);
	std::set<Frame*> GetFrames();



public:


private:
	std::set<MapPoint*> mspMapPoints;
	std::set<Frame*> mspFrames;

	std::mutex mPointLock;
	std::mutex mFrameLock;
};




}



#endif 
