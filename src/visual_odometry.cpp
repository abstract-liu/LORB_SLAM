#include "../include/visual_odometry.h"
#include <algorithm>
#include <cmath>
#include <mutex>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/core/base.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/features2d.hpp>
#include <vector>

namespace Simple_ORB_SLAM 
{
VisualOdometry::VisualOdometry(LocalMapping* lcMapper, Camera* camera, Map* map)
{
	mpCamera = camera;
	mpMap = map;
	mpLocalMapper = lcMapper;
	
	mTwc = cv::Mat::eye(4,4,CV_32F);

	isFistFrame = true;

	mOutFile.open("trajectory.csv");

}

VisualOdometry::~VisualOdometry()
{
	mOutFile.close();
}




void VisualOdometry::Run(const string& timeStamp, const cv::Mat& imgLeft, const cv::Mat& imgRight)
{
	// construct frame
	mCurrFrame = new Frame(imgLeft, imgRight, mpCamera);

	if(isFistFrame == true)
	{
		Initialize();
		return;
	}
	
	bool tag = EstimatePoseMotion(mPrevFrame);
	
	if(tag == false)
	{
		cout << "---- motion mode fail" << endl;
		tag = EstimatePoseMotion(mReferFrame);
	}

	//local map track
	if(tag == false)
	{
		cout << "---- reference frame fail" << endl;
	}
	
	tag = EstimatePoseLocal();	

	//update pose and velocity
	mTcc = mPrevFrame->GetInvPose() * mCurrFrame->GetPose();
	mTwc = mCurrFrame->GetInvPose();

	//add key frame and do local bundle
	//AddKeyFrame();
	mpMap->AddFrame(mCurrFrame);
	
	//update prev frame and refer frame
	mPrevFrame = mCurrFrame;

}

void VisualOdometry::Initialize()
{
	
	if(mCurrFrame->mnMapPoints < 300)
		return;

	//set pose
	mCurrFrame->SetPose(cv::Mat::eye(4,4,CV_32F));

	for(size_t i=0; i<mCurrFrame->mnMapPoints; i++)
	{
		if(mCurrFrame->mvDepth[i] < 0)
			continue;
		
		cv::Point3f kp3d = mCurrFrame->UnprojectStereo(i);
		MapPoint* pMP = new MapPoint(kp3d, mCurrFrame, mpMap);
		pMP->AddObservation(mCurrFrame, i);
		pMP->ComputeDescriptor();
		pMP->UpdateNormalAndDepth();

		mCurrFrame->AddMapPoint(pMP, i);
		mpMap->AddMapPoint(pMP);
	}
	cout << "create new map with " << mpMap->GetMapPointsNum() << " points" << endl;
	
	//add to local map
	mpLocalMapper->InsertKeyFrame(mCurrFrame);


	mPrevFrame = mCurrFrame;
	mReferFrame = mCurrFrame;
	mRelocateFrame = mCurrFrame;

	isFistFrame = false;

	mTcc = cv::Mat::eye(4,4,CV_32F);

}


bool VisualOdometry::EstimatePoseMotion(Frame* pF)
{
	mCurrFrame->SetPose(pF->GetPose()*mTcc);

	UpdateFrame(pF);
	
	//match points
	size_t nMatches = Matcher::SearchByProjection(mCurrFrame, pF, 15);

	if(nMatches <= 20)
	{
		std::fill(mCurrFrame->mvpMapPoints.begin(), mCurrFrame->mvpMapPoints.end(), static_cast<MapPoint*>(NULL));
		nMatches = Matcher::SearchByProjection(mCurrFrame, pF, 30);
	}
	if(nMatches <= 20)
		return false; 

	//optimise
	BA::ProjectPoseOptimization(mCurrFrame);

	return true;
}




bool VisualOdometry::EstimatePoseLocal()
{
	//step 1
	UpdateLocalMap();


	//step 2
	// Do not search map points already matched
    // 步骤1：遍历当前帧的mvpMapPoints，标记这些MapPoints不参与之后的搜索
    // 因为当前的mvpMapPoints一定在当前帧的视野中
    for(vector<MapPoint*>::iterator vit=mCurrFrame->mvpMapPoints.begin(); vit!=mCurrFrame->mvpMapPoints.end(); vit++)
    {
        MapPoint* pMP = *vit;
        if(pMP)
        {
            if(pMP->IsBad())
            {
                *vit = static_cast<MapPoint*>(NULL);
            }
            else
            {
                // 更新能观测到该点的帧数加1
                pMP->IncreaseVisible();
				pMP->mnLastFrameSeen = mCurrFrame->mnId;
                // 标记该点将来不被投影，因为已经匹配过
                pMP->mbTrackInView = false;
            }
        }
    }

    int nToMatch=0;
    // Project points in frame and check its visibility
    // 步骤2：将所有局部MapPoints投影到当前帧，判断是否在视野范围内，然后进行投影匹配
    for(std::set<MapPoint*>::iterator vit=mpLocalMapPoints.begin(); vit!=mpLocalMapPoints.end(); vit++)
    {
        MapPoint* pMP = *vit;

        // 已经被当前帧观测到MapPoint不再判断是否能被当前帧观测到
        if(pMP->mnLastFrameSeen == mCurrFrame->mnId)
            continue;
        if(pMP->IsBad())
            continue;
        
        // Project (this fills MapPoint variables for matching)
        // 步骤2.1：判断LocalMapPoints中的点是否在在视野内
        if(mCurrFrame->IsInFrustum(pMP,0.5))
        {
        	// 观测到该点的帧数加1，该MapPoint在某些帧的视野范围内
            pMP->IncreaseVisible();
            // 只有在视野范围内的MapPoints才参与之后的投影匹配
            nToMatch++;
        }
    }

	size_t nMatches = 0;
    if(nToMatch>0)
    {
        // 步骤2.2：对视野范围内的MapPoints通过投影进行特征点匹配
        nMatches = Matcher::SearchByProjection(mCurrFrame,mpLocalMapPoints,1);
    }

	if(nMatches <= 20)
		return false;

	BA::ProjectPoseOptimization(mCurrFrame);

	return true;
}




void VisualOdometry::UpdateFrame(Frame* pF)
{
	
	for(size_t i=0; i<pF->mnMapPoints; i++)
	{
		MapPoint* pMP = pF->mvpMapPoints[i];
		
		if(pMP != NULL)
			continue;
		
		cv::Point3f kp3d = pF->UnprojectStereo(i);
		MapPoint* pNewMP = new MapPoint(kp3d, pF, mpMap, i);
		pF->AddMapPoint(pNewMP, i);
	}

}



void VisualOdometry::UpdateLocalMap()
{
	//step 1 update local key frames
	
	std::map<Frame*, size_t> frameCounter;

	//step 1.1 
	for(size_t i =0; i<mCurrFrame->mnMapPoints; i++)
	{
		MapPoint* pMP = mCurrFrame->mvpMapPoints[i];
		if(pMP != NULL)
		{
			if(pMP->IsBad() == false)
			{
				const std::map<Frame*, size_t> observations = pMP->GetObservations();
				for(std::map<Frame*, size_t>::const_iterator it = observations.begin(); it != observations.end(); it++)
				{
					frameCounter[it->first] += 1;
				}
			}
			else
			{
				mCurrFrame->mvpMapPoints[i] = NULL;
			}
		}
	}

	
	//step 1.2
	mpLocalKeyFrames.clear();

	//step 1.3.1
	size_t nMax = 0;
	Frame* pFMax = static_cast<Frame*>(NULL);
	for(std::map<Frame*, size_t>::const_iterator it = frameCounter.begin(); it != frameCounter.end(); it++)
	{
		Frame* pFrame = it->first;
		if(pFrame->IsBad() == true)
			continue;
		
		if(nMax < it->second)
		{
			nMax = it->second;
			pFMax = pFrame;
		}
		mpLocalKeyFrames.push_back(pFrame);
	}

	for(std::vector<Frame*>::const_iterator it = mpLocalKeyFrames.begin(); it != mpLocalKeyFrames.end(); it++)
	{
		
		if(mpLocalKeyFrames.size() > 10)
			break;

		Frame* pFrame = *it;

		//1.3.2.1 best covisible frames 
		const std::vector<Frame*> bestCovisibleFrames = pFrame->GetBestCovisibleFrames(10);
		for(std::vector<Frame*>::const_iterator it = bestCovisibleFrames.begin(); it != bestCovisibleFrames.end(); it++)
		{
			Frame* pBestFrame = *it;
			if(pBestFrame->IsBad() == false)
			{
				std::vector<Frame*>::iterator res;
				res = std::find(mpLocalKeyFrames.begin(), mpLocalKeyFrames.end(), pBestFrame);
				if(res == mpLocalKeyFrames.end())
				{
					mpLocalKeyFrames.push_back(pBestFrame);
					break;
				}
			}
		}


	}
	
	//update reference frame
	if(pFMax != NULL)
		mReferFrame = pFMax;

	//step 2 update local points
	mpLocalMapPoints.clear();
	for(std::vector<Frame*>::const_iterator it = mpLocalKeyFrames.begin(); it != mpLocalKeyFrames.end(); it++)
	{
		Frame* pFrame = *it;
		const std::vector<MapPoint*> vMPs = pFrame->GetMapPoints();

		for(std::vector<MapPoint*>::const_iterator it = vMPs.begin(); it != vMPs.end(); it++)
		{
			MapPoint* pMP = *it;
			if(pMP == NULL)
				continue;
			if(pMP->IsBad() == true)
				continue;
			
			std::set<MapPoint*>::iterator res;
			res = std::find(mpLocalMapPoints.begin(), mpLocalMapPoints.end(), pMP);
			if(res == mpLocalMapPoints.end())
			{
				mpLocalMapPoints.insert(pMP);
			}
		}
	}
	
	
	
}















void VisualOdometry::AddKeyFrame()
{

	//step 1.2
	size_t nMap = 0;
	for(size_t i=0; i<mCurrFrame->mnMapPoints; i++)
	{
		MapPoint* pMP = mCurrFrame->mvpMapPoints[i];
		if(pMP != NULL)
		{
			nMap += 1;
		}
	}
	const float ratioMap = (float)nMap / mCurrFrame->mnMapPoints;
	float theMapRatio = 0.35;
	if(ratioMap > theMapRatio)
		return;


	//step 2 create new key frame
	//step 2.1 create new map points
	
	std::vector<cv::Point3f> kps3d = mCurrFrame->GetKps3d();

	for(size_t i=0; i<mCurrFrame->mnMapPoints; i++)
	{
		MapPoint* pMP = mCurrFrame->mvpMapPoints[i];
		if(pMP != NULL)
			continue;
		
		cv::Mat kp3d = (cv::Mat_<float>(4,1) << kps3d[i].x, kps3d[i].y, kps3d[i].z, 1);
		cv::Mat kpWorld = mPrevFrame->GetInvPose() * kp3d;
		cv::Point3f ptWorld(kpWorld.at<float>(0), kpWorld.at<float>(1), kpWorld.at<float>(2));

		pMP = new MapPoint(ptWorld, mCurrFrame, mpMap);
		pMP->AddObservation(mCurrFrame, i);
		pMP->ComputeDescriptor();
		pMP->UpdateNormalAndDepth();

		mCurrFrame->AddMapPoint(pMP, i);
		mpMap->AddMapPoint(pMP);

	}

	//step 2.2
	mpLocalMapper->InsertKeyFrame(mCurrFrame);
	mReferFrame = mCurrFrame;

}




void VisualOdometry::SaveTrajectory()
{
	mOutFile << mTimestamp << "e-8" << " " << mTUM[4] << " " << mTUM[5] << " " << mTUM[6] << " " << mTUM[0] << " " << mTUM[1] << " " << mTUM[2] << " " << mTUM[3] << endl;

}


cv::Mat VisualOdometry::GetCurrInvPos()
{
	cv::Mat invPose = mTwc.clone();
	return invPose;
}


bool VisualOdometry::EstimatePosePnP()
{
	//fist ransac pnp
	vector<int> inliers;
	vector<Point3f> referKps3d, tempKps3d;
	vector<Point2f> currKps2d, tempKps2d;

	//increate map points
	UpdateFrame(mReferFrame);

	mCurrFrame->SetPose(mPrevFrame->GetPose());

	fill(mCurrFrame->mvpMapPoints.begin(), mCurrFrame->mvpMapPoints.end(), static_cast<MapPoint*>(NULL));

	int nMatches = Matcher::SearchByProjection(mCurrFrame, mReferFrame);

	if(nMatches < 15)
	{
		cout << "not enough points for pnp" << endl;
		return false;
	}
	
	//trans to point type
	for(size_t i=0; i<mCurrFrame->mnMapPoints; i++)
	{
		MapPoint* pMP = mCurrFrame->mvpMapPoints[i];
		if(pMP == NULL)
			continue;
		cv::Point2f kp2d = mCurrFrame->GetKp2d(i);

		referKps3d.push_back(pMP->GetPos());
		currKps2d.push_back(kp2d);
	}

	
	cv::Mat Rvec, Tvec;
	
	solvePnPRansac(referKps3d, currKps2d, mpCamera->pi, Mat(), Rvec, Tvec, false, 500, 2.0f, 0.999, inliers, SOLVEPNP_ITERATIVE);

	//reject outliers pnp
	for(size_t i=0; i<inliers.size(); i++)
	{
		tempKps3d.push_back( referKps3d[inliers[i]] );
		tempKps2d.push_back( currKps2d[inliers[i]] );
	}

	referKps3d = tempKps3d;
	currKps2d = tempKps2d;

	if(referKps3d.size() < 15)
	{
		cout << "not enough points for pnp" << endl;
		return false;
	}
	
	solvePnP(referKps3d, currKps2d, mpCamera->pi, Mat(), Rvec, Tvec );
	mCurrFrame->SetPose(Tvec, Rvec);
	

	return true;
}




 }
