#include "../include/local_mapping.h"
#include <vector>

namespace Simple_ORB_SLAM
{


void LocalMapping::Run()
{
	while(true)
	{
		if(CheckNewFrames() == true)
		{
			ProcessNewFrames();

			MapPointsCulling();
			
			if(CheckNewFrames() == false)
			{
				BA::LocalPoseOptimization(mpCurrFrame);

				KeyFramesCulling();
	
			}
		}
	}

}

bool LocalMapping::CheckNewFrames()
{
	return (!mlpNewFrames.empty());
}

void LocalMapping::ProcessNewFrames()
{
	{
		mpCurrFrame = mlpNewFrames.front();
		mlpNewFrames.pop_front();
	}

	//step 1 
	for(size_t i=0; i<mpCurrFrame->mN; i++)
	{
		MapPoint* pMP = mpCurrFrame->mvpMapPoints[i];
		if(pMP->IsBad())
			continue;
		if(!pMP->IsInFrame(mpCurrFrame))
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

		if((mpCurrFrame->mId - pMP->mFirstFId)>=2 && pMP->mN < 3)
		{
			pMP->SetBadFlag();
			it = mvpRecentAddPoints.erase(it);
		}

		if((mpCurrFrame->mId-pMP->mFirstFId)>=3)
			it = mvpRecentAddPoints.erase(it);
	}

}




}
