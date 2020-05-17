#include "../include/matcher.h"

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
	for(size_t i=0; i<prevFrame->mnMapPoints; i++)
	{
		MapPoint* pMP = prevFrame->mvpMapPoints[i];
		cv::Point2f pt2f;

		//null
		if(pMP == NULL)
			continue;
		prevMPs.push_back(pMP);
		prevDescriptors.push_back(pMP->GetDescriptor());
	}
	
	//cout << "prev size " << prevMPs.size() << endl;

	//brute force match
	cv::BFMatcher matcher(NORM_HAMMING, true);
	std::vector<DMatch> matches;

	matcher.match(currDescriptors, prevDescriptors, matches);

	//reject outliers
	double minDist = DBL_MAX;
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
		currFrame->mvpMapPoints[matches[i].queryIdx] = prevMPs[matches[i].trainIdx];
		nPoints += 1;
	}
	
	//cout << "match size " << nPoints << endl;

	return nPoints;

}





size_t Matcher::SearchLocalPoints(Frame* currFrame, std::set<MapPoint*> vpMPs)
{
	std::vector<MapPoint*> prevMPs;

	cv::Mat prevDescriptors;
	cv::Mat currDescriptors = currFrame->GetDescriptors();

	//get prev map points
	for(std::set<MapPoint*>::iterator it = vpMPs.begin(); it != vpMPs.end(); it++)
	{
		MapPoint* pMP = *it;
		cv::Point2f pt2f;

		//null
		if(pMP == NULL)
			continue;

		prevMPs.push_back(pMP);
		prevDescriptors.push_back(pMP->GetDescriptor());
	}


	//brute force match
	cv::BFMatcher matcher(NORM_HAMMING, true);
	std::vector<DMatch> matches;

	matcher.match(currDescriptors, prevDescriptors, matches);

	//reject outliers
	double minDist = DBL_MAX;
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
		currFrame->mvpMapPoints[matches[i].queryIdx] = prevMPs[matches[i].trainIdx];
		nPoints += 1;
	}
	
	return nPoints;

}





}
