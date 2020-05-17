#include "../include/local_mapping.h"
#include <mutex>
#include <vector>

namespace Simple_ORB_SLAM
{

LocalMapping::LocalMapping(Map* pMap)
{
	mpMap = pMap;
}

void LocalMapping::InsertKeyFrame(Frame* pF)
{
	std::unique_lock<std::mutex> lock(mFrameLock);
	mlpNewFrames.push_back(pF);
}

void LocalMapping::Run()
{
	while(true)
	{
		
		if(CheckNewFrame() == true )
		{
			//ProcessNewFrames();

			//MapPointsCulling();
			
			if( CheckNewFrame() == false)
			{
				//BA::LocalPoseOptimization(mpCurrFrame);

				//KeyFramesCulling();
	
			}
		}
	}

}

bool LocalMapping::CheckNewFrame()
{
	std::unique_lock<std::mutex> lock(mFrameLock);
	return ( mlpNewFrames.empty() == false);
}


void LocalMapping::ProcessNewFrames()
{
	{
		mpCurrFrame = mlpNewFrames.front();
		mlpNewFrames.pop_front();
	}
/*
	//step 1 
	for(size_t i=0; i<mpCurrFrame->mnMapPoints; i++)
	{
		MapPoint* pMP = mpCurrFrame->mvpMapPoints[i];
		if(pMP->IsBad())
			continue;
		if(pMP->IsInFrame(mpCurrFrame) == false)
		{
			pMP->AddObservation(mpCurrFrame, i);
		}
		else
		{
			mvpRecentAddPoints.push_back(pMP);
		}
	}

	//step 2
	mpCurrFrame->UpdateConnections();
*/
	//step 3
	mpMap->AddFrame(mpCurrFrame);

}



void LocalMapping::MapPointsCulling()
{
	
	for(std::vector<MapPoint*>::iterator it = mvpRecentAddPoints.begin(); it != mvpRecentAddPoints.end(); it++)
	{
		MapPoint* pMP = *it;

		if(pMP->IsBad())
			it = mvpRecentAddPoints.erase(it);
		
		if(pMP->GetFoundRatio() < 0.25f)
		{
			pMP->SetBadFlag();
			it = mvpRecentAddPoints.erase(it);
		}

		if((mpCurrFrame->mnId - pMP->mnFirstFId)>=2 && pMP->mnObs < 3)
		{
			pMP->SetBadFlag();
			it = mvpRecentAddPoints.erase(it);
		}

		if((mpCurrFrame->mnId-pMP->mnFirstFId)>=3)
			it = mvpRecentAddPoints.erase(it);
	}

}

void LocalMapping::KeyFramesCulling()
{

}


}
