#include "../include/map_point.h"
#include <opencv2/core/types.hpp>
#include <vector>

namespace Simple_ORB_SLAM
{

	
MapPoint::MapPoint(cv::Point3f pt, Frame* pF, Map* pMap)
{
	mWorldPos = pt;
	mnFirstFId = pF->mnId;
	mnLastFrameSeen = 0;
	mnObs = 0;
	mbBadFlag = false;
	mnVisible = 1;
	mnFound = 1;
	mpMap = pMap;

	mfMaxDistance = 0;
	mfMinDistance = 0;
	mNormalVector = cv::Mat::zeros(3,1,CV_32F);
	mpRefKF = pF;
}		

MapPoint::MapPoint(cv::Point3f pt, Frame* pF, Map* pMap, size_t idx)
{
	mWorldPos = pt;
	mnFirstFId = pF->mnId;
	mnLastFrameSeen = 0;
	mnObs = 0;
	mbBadFlag = false;
	mnVisible = 1;
	mnFound = 1;
	mDescriptor = pF->GetDescriptor(idx);
	mpMap = pMap;

	cv::Point3f Ow = pF->GetCameraCenter();
	cv::Point3f PC = pt - Ow;
	mNormalVector = cv::Mat(pt-Ow);// 世界坐标系下相机到3D点的向量
    mNormalVector = mNormalVector/cv::norm(mNormalVector);// 世界坐标系下相机到3D点的单位向量
	const float dist = cv::norm(PC);
    const int level = pF->mvKeysUn[idx].octave;
    const float levelScaleFactor =  pF->mvScaleFactors[level];
    const int nLevels = pF->mnScaleLevels;
	mfMaxDistance = dist*levelScaleFactor;
    mfMinDistance = mfMaxDistance/pF->mvScaleFactors[nLevels-1];
	pF = static_cast<Frame*>(NULL);
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
		if(mpRefKF==pF)
            mpRefKF=mObservations.begin()->first;

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


float MapPoint::GetMinDistanceInvariance()
{
    return 0.8f*mfMinDistance;
}

float MapPoint::GetMaxDistanceInvariance()
{
    return 1.2f*mfMaxDistance;
}

cv::Mat MapPoint::GetNormal()
{
    return mNormalVector.clone();
}

void MapPoint::UpdateNormalAndDepth()
{
    map<Frame*,size_t> observations;
    Frame* pRefKF;
    cv::Point3f Pos;
    {
        if(mbBadFlag)
            return;

        observations=mObservations; // 获得观测到该3d点的所有关键帧
        pRefKF=mpRefKF;             // 观测到该点的参考关键帧
        Pos = mWorldPos;    // 3d点在世界坐标系中的位置
    }

    if(observations.empty())
        return;

    cv::Mat normal = cv::Mat::zeros(3,1,CV_32F);
    int n=0;
    for(map<Frame*,size_t>::iterator mit=observations.begin(), mend=observations.end(); mit!=mend; mit++)
    {
        Frame* pKF = mit->first;
        cv::Point3f Owi = pKF->GetCameraCenter();
        cv::Point3f normali = mWorldPos - Owi;
        normal = normal + cv::Mat(normali)/cv::norm(normali); // 对所有关键帧对该点的观测方向归一化为单位向量进行求和
        n++;
    } 

    cv::Point3f PC = Pos - pRefKF->GetCameraCenter(); // 参考关键帧相机指向3D点的向量（在世界坐标系下的表示）
    const float dist = cv::norm(PC); // 该点到参考关键帧相机的距离
    const int level = pRefKF->mvKeysUn[observations[pRefKF]].octave;
    const float levelScaleFactor =  pRefKF->mvScaleFactors[level];
    const int nLevels = pRefKF->mnScaleLevels; // 金字塔层数

    {
        // 另见PredictScale函数前的注释
        mfMaxDistance = dist*levelScaleFactor;                           // 观测到该点的距离最大值
        mfMinDistance = mfMaxDistance/pRefKF->mvScaleFactors[nLevels-1]; // 观测到该点的距离最小值
        mNormalVector = normal/n;                                        // 获得平均的观测方向
    }
}


int MapPoint::PredictScale(const float &currentDist, Frame* pKF)
{
    float ratio;
    {
        // mfMaxDistance = ref_dist*levelScaleFactor为参考帧考虑上尺度后的距离
        // ratio = mfMaxDistance/currentDist = ref_dist/cur_dist
        ratio = mfMaxDistance/currentDist;
    }

    // 同时取log线性化
    int nScale = ceil(log(ratio)/pKF->mfLogScaleFactor);
    if(nScale<0)
        nScale = 0;
    else if(nScale>=pKF->mnScaleLevels)
        nScale = pKF->mnScaleLevels-1;

    return nScale;
}


}
