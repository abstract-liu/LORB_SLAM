#ifndef LOCALMAPPING_H
#define LOCALMAPPING_H

#include "frame.h"
#include "bundle_adjust.h"
#include "map_point.h"
#include "map.h"
#include <list>
#include <vector>

namespace Simple_ORB_SLAM
{



class LocalMapping
{

public:
	LocalMapping();

	void Run();

	bool CheckNewFrames();

	void ProcessNewFrames();

	void MapPointsCulling();

	void KeyFramesCulling();




private:
	Frame* mpCurrFrame;
	Map* mpMap;

	std::list<Frame*> mlpNewFrames;
	std::vector<MapPoint*> mvpRecentAddPoints;
};



}






#endif
