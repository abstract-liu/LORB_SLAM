#include "../include/map_point.h"
#include <opencv2/core/types.hpp>
#include <vector>

namespace Simple_ORB_SLAM
{

	
MapPoint::MapPoint(cv::Point3f pt, Frame* pF, Map* pMap)
{
	mWorldPos = pt;
	
	mnFirstFId = pF->mnId;
	mnObs = 0;

	mbBadFlag = false;
	
	mnVisible = 0;
	mnFound = 0;
	
	mpMap = pMap;
}		

MapPoint::MapPoint(cv::Point3f pt, Frame* pF, Map* pMap, size_t idx)
{
	mWorldPos = pt;

	mnFirstFId = pF->mnId;
	mnObs = 0;

	mbBadFlag = false;
	
	mnVisible = 0;
	mnFound = 0;
	mDescriptor = pF->GetDescriptor(idx);
	
	mpMap = pMap;

}


void MapPoint::SetWorldPos(cv::Point3f kp3d)
{
	mWorldPos = kp3d;
}

cv::Point3f MapPoint::GetPos()
{
	return mWorldPos;
}

cv::Mat MapPoint::GetDescriptor()
{
	cv::Mat descriptor = mDescriptor.clone();
	return descriptor;

}

void MapPoint::ComputeDescriptor()
{
	std::vector<cv::Mat> vDescriptors;
	

	for(std::map<Frame*, size_t>::iterator it = mObservations.begin(); it != mObservations.end(); it++)
	{	
		Frame* pF = it->first;
		if(pF->IsBad() == true)
			continue;
		cv::Mat descriptor = pF->GetDescriptor(it->second);
		vDescriptors.push_back(descriptor);
	}
	
	//warning vector need to resize first
	std::vector<std::vector<int>> distances;
	distances.resize(vDescriptors.size(), std::vector<int>(vDescriptors.size(),0));

	for(size_t i=0; i<vDescriptors.size(); i++)
	{
		distances[i][i] = 0;
		for(size_t j=i+1; j<vDescriptors.size(); j++)
		{

			const int *pa = vDescriptors[i].ptr<int32_t>();
    		const int *pb = vDescriptors[j].ptr<int32_t>();
			int dist=0;

			for(size_t k=0; k<8; k++, pa++, pb++)
			{
        		unsigned  int v = *pa ^ *pb;
        		v = v - ((v >> 1) & 0x55555555);
        		v = (v & 0x33333333) + ((v >> 2) & 0x33333333);
        		dist += (((v + (v >> 4)) & 0xF0F0F0F) * 0x1010101) >> 24;
    		}


			distances[i][j] = dist;
			distances[j][i] = dist;
		}
	}

	int bestIdx = 0;
	int bestMedian = INT_MAX;
	for(size_t i=0; i<vDescriptors.size(); i++)
	{
		std::vector<int> vDist(distances[i].begin(), distances[i].end());
		std::sort(vDist.begin(), vDist.end());

		int median = vDist[0.5*(vDescriptors.size()-1)];
		if(median < bestMedian)
		{
			bestMedian = median;
			bestIdx = i;
		}
	}

	mDescriptor = vDescriptors[bestIdx].clone();


}



void MapPoint::AddObservation(Frame* frame, size_t id)
{
	if(mObservations.count(frame))
		return;
	mObservations[frame] = id;
	mnObs += 1;
}

void MapPoint::EraseObservation(Frame* pF)
{
	if(mObservations.count(pF) == true)
	{
		mObservations.erase(pF);
		mnObs -= 1;
	}
	if(mnObs <= 2)
		SetBadFlag();
}

std::map<Frame*, size_t> MapPoint::GetObservations()
{
	return mObservations;
}


bool MapPoint::IsInFrame(Frame* pF)
{
	return mObservations.count(pF);
}


void MapPoint::IncreaseVisible(size_t n)
{
	mnVisible += n;
}

void MapPoint::IncreaseFound(size_t n)
{
	mnFound += n;
}

float MapPoint::GetFoundRatio()
{
	return static_cast<float>(mnFound)/mnVisible;
}



void MapPoint::SetBadFlag()
{
	mbBadFlag = true;

	std::map<Frame*, size_t> obs;
	obs = mObservations;
	mObservations.clear();

	for(std::map<Frame*, size_t>::iterator it = obs.begin(); it != obs.end(); it++)
	{
		Frame* pF = it->first;
		pF->EraseMapPoint(it->second);
	}

	mpMap->EraseMapPoint(this);
}



bool MapPoint::IsBad()
{
	return mbBadFlag;	
}






}
