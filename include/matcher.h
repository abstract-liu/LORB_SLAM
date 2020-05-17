#ifndef MATCHER_H
#define MATCHER_H


#include "common.h"
#include "frame.h"
#include <vector>

namespace Simple_ORB_SLAM
{

class Matcher
{
public:
	Matcher();
	
	size_t static SearchByProjection(Frame* curr, Frame* prev);
	
	size_t static SearchLocalPoints(Frame* curr, std::set<MapPoint*> vpMPs);

};



}

#endif 
