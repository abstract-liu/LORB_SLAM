#include "../include/matcher.h"
#include <opencv2/features2d.hpp>
#include <vector>

namespace Simple_ORB_SLAM
{



//naive version
//need more strategies in future

size_t Matcher::SearchByProjection(Frame* currFrame, Frame* prevFrame)
{
	std::vector<MapPoint*> prevMPs;

	cv::Mat prevDescriptors;
	cv::Mat currDescriptors = currFrame->GetDescriptors();

	//get prev map points
	for(size_t i=0; i<prevFrame->mN; i++)
	{
		MapPoint* pMP = prevFrame->mvpMapPoints[i];
		cv::Point2f pt2f;

		//null
		if(!pMP)
			continue;
		//not in frustum
		bool bTag =	currFrame->IsVisible(pMP, pt2f); 
		if(bTag == false)
			continue;

		prevMPs.push_back(pMP);
		prevDescriptors.push_back(pMP->GetDescriptor());
	}

	//brute force match
	cv::BFMatcher matcher(NORM_HAMMING, true);
	std::vector<DMatch> matches;

	matcher.match(prevDescriptors, currDescriptors, matches);

	//reject outliers
	double minDist = 9999;
	for(size_t i=0; i<matches.size(); i++)
	{
		if(matches[i].distance < minDist)
			minDist = matches[i].distance;
	}

	size_t nPoints = 0;
	for(size_t i=0; i<matches.size(); i++)
	{
		if(matches[i].distance > max(2*minDist, 30.0))
			continue;
		currFrame->mvpMapPoints[matches[i].trainIdx] = prevMPs[matches[i].queryIdx];
		nPoints += 1;
	}
	
	return nPoints;

}





size_t Matcher::SearchLocalPoints(Frame* currFrame, std::vector<MapPoint*> vpMPs)
{
	std::vector<MapPoint*> prevMPs;

	cv::Mat prevDescriptors;
	cv::Mat currDescriptors = currFrame->GetDescriptors();

	//get prev map points
	for(size_t i=0; i<vpMPs.size(); i++)
	{
		MapPoint* pMP = vpMPs[i];
		cv::Point2f pt2f;

		//null
		if(!pMP)
			continue;
		//not in frustum
		bool bTag =	currFrame->IsVisible(pMP, pt2f); 
		if(bTag == false)
			continue;

		prevMPs.push_back(pMP);
		prevDescriptors.push_back(pMP->GetDescriptor());
	}

	//brute force match
	cv::BFMatcher matcher(NORM_HAMMING, true);
	std::vector<DMatch> matches;

	matcher.match(prevDescriptors, currDescriptors, matches);

	//reject outliers
	double minDist = 9999;
	for(size_t i=0; i<matches.size(); i++)
	{
		if(matches[i].distance < minDist)
			minDist = matches[i].distance;
	}

	size_t nPoints = 0;
	for(size_t i=0; i<matches.size(); i++)
	{
		if(matches[i].distance > max(2*minDist, 30.0))
			continue;
		currFrame->mvpMapPoints[matches[i].trainIdx] = prevMPs[matches[i].queryIdx];
		nPoints += 1;
	}
	
	return nPoints;

}





}
