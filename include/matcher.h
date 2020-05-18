#ifndef MATCHER_H
#define MATCHER_H


#include "common.h"
#include "frame.h"
#include "map_point.h"

namespace Simple_ORB_SLAM
{

class Frame;
class MapPoint;

class Matcher
{
public:
	Matcher();
	
	size_t static SearchByProjection(Frame* curr, Frame* prev);
	size_t static SearchByProjection(Frame* CurrentFrame, Frame* LastFrame, const float th);
	
	size_t static SearchLocalPoints(Frame* curr, std::set<MapPoint*> vpMPs);	
	size_t static SearchByProjection(Frame* F, const std::set<MapPoint*> &vpMapPoints, const float th);
	
	int static DescriptorDistance(const cv::Mat &a, const cv::Mat &b);
	float static RadiusByViewingCos(const float &viewCos);
	void static ComputeThreeMaxima(vector<int>* histo, const int L, int &ind1, int &ind2, int &ind3);

public:
    static const int TH_LOW;
    static const int TH_HIGH;
    static const int HISTO_LENGTH;


};



}

#endif 
