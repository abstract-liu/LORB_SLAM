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
	AddKeyFrame();
	mpMap->AddFrame(mCurrFrame);
	
	//update prev frame and refer frame
	mPrevFrame = mCurrFrame;

}

void VisualOdometry::Initialize()
{
	
	if(mCurrFrame->mnMapPoints < 100)
		return;

	//set pose
	mCurrFrame->SetPose(cv::Mat::eye(4,4,CV_32F));

	//add frame to global map
	mpMap->AddFrame(mCurrFrame);

	//add points to global
	std::vector<cv::Point3f> kps3d = mCurrFrame->GetKps3d();

	for(size_t i=0; i<mCurrFrame->mnMapPoints; i++)
	{
		MapPoint* pMP = new MapPoint(kps3d[i], mCurrFrame, mpMap);
		pMP->AddObservation(mCurrFrame, i);
		pMP->ComputeDescriptor();

		mCurrFrame->AddMapPoint(pMP, i);
		mpMap->AddMapPoint(pMP);
	}
	cout << "create new map with " << mCurrFrame->mnMapPoints << " points" << endl;
	
	//add to local map
	mpLocalMapper->InsertKeyFrame(mCurrFrame);


	mPrevFrame = mCurrFrame;
	mReferFrame = mCurrFrame;

	isFistFrame = false;

	mTcc = cv::Mat::eye(4,4,CV_32F);

}




bool VisualOdometry::EstimatePoseMotion(Frame* pF)
{
	mCurrFrame->SetPose(pF->GetPose()*mTcc);

	UpdateFrame(pF);
	//match points
	size_t nMatches = Matcher::SearchByProjection(mCurrFrame, pF);
	

	if(nMatches <= 10)
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
	size_t nMatches = Matcher::SearchLocalPoints(mCurrFrame, mpLocalMapPoints);
	
	if(nMatches <= 10)
		return false;

	BA::ProjectPoseOptimization(mCurrFrame);

	return true;
}




void VisualOdometry::UpdateFrame(Frame* pF)
{
	
	std::vector<cv::Point3f> kps3d = pF->GetKps3d();

	for(size_t i=0; i<pF->mnMapPoints; i++)
	{
		MapPoint* pMP = pF->mvpMapPoints[i];
		
		if(pMP != NULL)
			continue;
		
		cv::Mat kp3d = (cv::Mat_<float>(4,1) << kps3d[i].x, kps3d[i].y, kps3d[i].z, 1);
		cv::Mat kpWorld = mPrevFrame->GetInvPose() * kp3d;
		cv::Point3f ptWorld(kpWorld.at<float>(0), kpWorld.at<float>(1), kpWorld.at<float>(2));

		MapPoint* pNewMP = new MapPoint(ptWorld, pF, mpMap);
		pNewMP->AddObservation(pF, i);
		pNewMP->ComputeDescriptor();

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
