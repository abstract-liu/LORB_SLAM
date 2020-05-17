#ifndef LOCALMAPPING_H
#define LOCALMAPPING_H

#include "frame.h"
#include "bundle_adjust.h"
#include "map_point.h"
#include "map.h"
#include "common.h"

namespace Simple_ORB_SLAM
{



class LocalMapping
{

public:
	LocalMapping(Map* pMap);

	void InsertKeyFrame(Frame* pF);

	void Run();

	bool CheckNewFrame();

	void ProcessNewFrames();

	void MapPointsCulling();

	void KeyFramesCulling();


protected:

	std::mutex mFrameLock;

private:


	Frame* mpCurrFrame;
	Map* mpMap;

	std::list<Frame*> mlpNewFrames;
	std::vector<MapPoint*> mvpRecentAddPoints;
};



}






#endif
