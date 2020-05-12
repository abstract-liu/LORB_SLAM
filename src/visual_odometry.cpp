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
VisualOdometry::VisualOdometry(const string& strParaFile, Camera* camera, Map* map)
{
	mpCamera = camera;
	mpMap = map;

	isFistFrame = true;

	std::unique_lock<std::mutex> lock(mPosLock);

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
	mTimestamp = timeStamp;

	if(isFistFrame == true)
	{
		Initialize();
		return;
	}
	
	bool tag;

	//track with motion
	tag = EstimatePoseMotion();
	
	if(tag == false)
		tag = EstimatePosePnP();
	
	//local map track
	if(tag == true)
		tag = EstimatePoseLocal();

	//add key frame and do local bundle
	AddKeyFrame();
}

void VisualOdometry::Initialize()
{

	if(mCurrFrame->mnMapPoints < 300)
		return;

	//add frame to global map
	mpMap->AddFrame(mCurrFrame);

	//add points to global
	std::vector<cv::Point3f> kps3d = mCurrFrame->GetKps3d();
	cv::Mat descriptors = mCurrFrame->GetDescriptors();

	for(size_t i=0; i<mCurrFrame->mnMapPoints; i++)
	{
		MapPoint* pMP = new MapPoint(kps3d[i], mCurrFrame, mpMap );
		pMP->AddObservation(mCurrFrame, i);
		pMP->ComputeDescriptor();

		mCurrFrame->AddMapPoint(pMP, i);
		mpMap->AddMapPoint(pMP);

	}
	
	//add to local map
	mpLocalKeyFrames.push_back(mCurrFrame);
	mpLocalMapPoints = mpMap->GetPoints();

	mTcc = Mat::eye(4,4,CV_32F);
	
	mPrevFrame = mCurrFrame;
	mReferFrame = mCurrFrame;

	isFistFrame = false;
}




bool VisualOdometry::EstimatePoseMotion()
{
	//add points
	UpdatePrevFrame();

	mCurrFrame->SetPose(mPrevFrame->GetPose()*mTcc);
	mCurrFrame->UpdatePoseVector();

	fill(mCurrFrame->mvpMapPoints.begin(), mCurrFrame->mvpMapPoints.end(), static_cast<MapPoint*>(NULL));

	//match points
	size_t nMatches = Matcher::SearchByProjection(mCurrFrame, mPrevFrame);

	//optimise
	BA::ProjectPoseOptimization(mCurrFrame);
	mCurrFrame->UpdatePoseMat();

	//reject outliers
	//
	
	return nMatches >= 10;
}



bool VisualOdometry::EstimatePosePnP()
{
	//fist ransac pnp
	vector<int> inliers;
	vector<Point3f> prevKps3d, tempKps3d;
	vector<Point2f> currKps2d, tempKps2d;

	//increate map points
	UpdatePrevFrame();

	mCurrFrame->SetPose(mPrevFrame->GetPose()*mTcc);
	mCurrFrame->UpdatePoseVector();

	fill(mCurrFrame->mvpMapPoints.begin(), mCurrFrame->mvpMapPoints.end(), static_cast<MapPoint*>(NULL));

	int nMatches = Matcher::SearchByProjection(mCurrFrame, mPrevFrame);

	if(nMatches < 4)
	{
		cout << "not enough points for pnp" << endl;
		return false;
	}
	
	//trans to point type
	for(size_t i=0; i<mCurrFrame->mnMapPoints; i++)
	{
		MapPoint* pMP = mCurrFrame->mvpMapPoints[i];
		if(!pMP)
			continue;

		prevKps3d.push_back(pMP->GetPose());
	}
	currKps2d = mCurrFrame->GetKps2d();

	solvePnPRansac(prevKps3d, currKps2d, mpCamera->pi, Mat(), mCurrFrame->mRvec, mCurrFrame->mTvec, false, 500, 2.0f, 0.999, inliers, SOLVEPNP_ITERATIVE);

	//reject outliers pnp
	for(size_t i=0; i<inliers.size(); i++)
	{
		tempKps3d.push_back( prevKps3d[inliers[i]] );
		tempKps2d.push_back( currKps2d[inliers[i]] );
	}

	prevKps3d = tempKps3d;
	currKps2d = tempKps2d;

	if(prevKps3d.size() < 4 )
	{
		cout << "not enough points for pnp" << endl;
		return false;
	}


	solvePnP(prevKps3d, currKps2d, mpCamera->pi, Mat(), mCurrFrame->mRvec, mCurrFrame->mTvec );
	mCurrFrame->UpdatePoseMat();
	

	return true;
}





bool VisualOdometry::EstimatePoseLocal()
{
	//step 1 
	UpdateLocalMap();

	//step 2
	size_t nMatches = Matcher::SearchLocalPoints(mCurrFrame, mpLocalMapPoints);

	BA::ProjectPoseOptimization(mCurrFrame);

	//update found 
	for(size_t i=0; i<mCurrFrame->mnMapPoints; i++)
	{
		MapPoint* pMP = mCurrFrame->mvpMapPoints[i];
		if(!pMP)
			pMP->mnMapPoints += 1;
	}
	
	return true;
}






void VisualOdometry::UpdatePrevFrame()
{
	
	std::vector<cv::Point3f> kps3d = mCurrFrame->GetKps3d();
	cv::Mat descriptors = mCurrFrame->GetDescriptors();

	size_t nPoints = 0;
	for(size_t i=0; i<mPrevFrame->mnMapPoints; i++)
	{
		bool createNewTag;
		
		MapPoint* pMP = mPrevFrame->mvpMapPoints[i];
		
		if(!pMP)
			createNewTag = true;
		else if(pMP->mnMapPoints < 1)
			createNewTag = true;

		if(createNewTag == true)
		{
			MapPoint* pNewMP = new MapPoint(kps3d[i], descriptors.row(i));
			pNewMP->AddObservation(mPrevFrame, i);
			mPrevFrame->mvpMapPoints[i] = pNewMP;
		}
		
		nPoints += 1;

		if(nPoints > 100)
			break;
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
		if(pMP)
		{
			if(! pMP->IsBad())
			{
				const std::map<Frame*, size_t> observations = pMP->GetObservations();
				for(std::map<Frame*, size_t>::const_iterator it = observations.begin(); it != observations.end(); it++)
				{
					frameCounter[it->first] += 1;
				}
			}
			else
			{
				//fbi warning
				mCurrFrame->mvpMapPoints[i] = NULL;
			}
		}
	}

	
	//step 1.2
	mpLocalKeyFrames.clear();
	mpLocalKeyFrames.reserve(3*frameCounter.size());

	//step 1.3.1
	for(std::map<Frame*, size_t>::const_iterator it = frameCounter.begin(); it != frameCounter.end(); it++)
	{
		Frame* pFrame = it->first;
		if(pFrame->IsBad())
			continue;
		mpLocalKeyFrames.push_back(pFrame);
	}

	//step 1.3.2
	for(std::vector<Frame*>::const_iterator it = mpLocalKeyFrames.begin(); it != mpLocalKeyFrames.end(); it++)
	{
		
		//
		if(mpLocalKeyFrames.size() > 80)
			break;

		Frame* pFrame = *it;

		//1.3.2.1
		const std::vector<Frame*> bestCovisibleFrames = pFrame->GetBestCovisibility(10);
		for(std::vector<Frame*>::const_iterator it = bestCovisibleFrames.begin(); it != bestCovisibleFrames.end(); it++)
		{
			Frame* pBestFrame = *it;
			if(!pBestFrame->IsBad())
			{
				std::vector<Frame*>::iterator res;
				res = std::find(mpLocalKeyFrames.begin(), mpLocalKeyFrames.end(), pBestFrame);
				if(res != mpLocalKeyFrames.end())
				{
					mpLocalKeyFrames.push_back(pBestFrame);
					break;
				}
			}
		}


		//1.3.2.2
		const std::vector<Frame*> vpChildren = pFrame->GetChildren();
		for(std::vector<Frame*>::const_iterator it = vpChildren.begin(); it != vpChildren.end(); it++)
		{
			Frame* pChildFrame = *it;
			if(!pChildFrame->IsBad())
			{
				std::vector<Frame*>::iterator res;
				res = std::find(mpLocalKeyFrames.begin(), mpLocalKeyFrames.end(), pChildFrame);
				if(res != mpLocalKeyFrames.end())
				{
					mpLocalKeyFrames.push_back(pChildFrame);
					break;
				}
			}
		}


		//1.3.2.3
		Frame* pParent = pFrame->GetParent();
		if(!pParent->IsBad())
		{
			std::vector<Frame*>::iterator res;
			res = std::find(mpLocalKeyFrames.begin(), mpLocalKeyFrames.end(), pParent);
			if(res != mpLocalKeyFrames.end())
			{
				mpLocalKeyFrames.push_back(pParent);
				break;
			}
		}


	}

	

	//step 2 update local points
	mpLocalMapPoints.clear();
	for(std::vector<Frame*>::const_iterator it = mpLocalKeyFrames.begin(); it != mpLocalKeyFrames.end(); it++)
	{
		Frame* pFrame = *it;
		const std::vector<MapPoint*> vMPs = pFrame->GetMapPoints();

		for(std::vector<MapPoint*>::const_iterator it = vMPs.begin(); it != vMPs.end(); it++)
		{
			MapPoint* pMP = *it;
			if(!pMP)
				continue;
			if(pMP->IsBad())
				continue;
			
			std::vector<MapPoint*>::iterator res;
			res = std::find(mpLocalMapPoints.begin(), mpLocalMapPoints.end(), pMP);
			if(res != mpLocalMapPoints.end())
			{
				mpLocalMapPoints.push_back(pMP);
				break;
			}
		}
	}
	
	
	
}















void VisualOdometry::AddKeyFrame()
{
	//judge if need to add new key frame
	//step 1.1
	if(mCurrFrame->mId < mReferFrame->mId + mFPS && mpMap->mKFN > mFPS)
		return;

	//step 1.2
	size_t nMap = 0;
	for(size_t i=0; i<mCurrFrame->mnMapPoints; i++)
	{
		MapPoint* pMP = mCurrFrame->mvpMapPoints[i];
		if(pMP)
		{
			//why??????????
			if(pMP->mnMapPoints > 0)
				nMap += 1;
		}
	}
	const float ratioMap = (float)nMap / mCurrFrame->mnMapPoints;
	float theMapRatio = 0.35;
	


	//step 2 create new key frame
	//step 2.1 create new map points
	//sort ????
	
	std::vector<cv::Point3f> kps3d = mCurrFrame->GetKps3d();
	cv::Mat descriptors = mCurrFrame->GetDescriptors();
	size_t nPoints = 0;

	for(size_t i=0; i<mCurrFrame->mnMapPoints; i++)
	{
		MapPoint* pMP = mCurrFrame->mvpMapPoints[i];
		if(pMP)
			continue;
		if(pMP->mnMapPoints > 1)
			continue;
		
		pMP = new MapPoint(kps3d[i], descriptors.row(i));
		pMP->AddObservation(mCurrFrame, i);

		mCurrFrame->mvpMapPoints[i] = pMP;
		mpMap->AddMapPoint(pMP);

		nPoints += 1;

		if(nPoints > 100)
			break;

	}

	//stemp 2.2
	mpLocalMapper->InsertKeyFrame(mCurrFrame);
	mReferFrame = mCurrFrame;

}






void VisualOdometry::Rotation2Quaternion()
{
    double trace = mTcw.at<float>(0,0) + mTcw.at<float>(1,1) + mTcw.at<float>(2,2);
 
    if (trace > 0.0) 
    {
        float s = sqrt(trace + 1.0);
        mTUM[3] = (s * 0.5);
        s = 0.5 / s;
        mTUM[0] = ((mTcw.at<float>(2,1) - mTcw.at<float>(1,2)) * s);
        mTUM[1] = ((mTcw.at<float>(0,2) - mTcw.at<float>(2,0)) * s);
        mTUM[2] = ((mTcw.at<float>(1,0) - mTcw.at<float>(0,1)) * s);
    } 
    
    else 
    {
        int i = mTcw.at<float>(0,0) < mTcw.at<float>(1,1) ? (mTcw.at<float>(1,1) < mTcw.at<float>(2,2) ? 2 : 1) : (mTcw.at<float>(0,0) < mTcw.at<float>(2,2) ? 2 : 0); 
        int j = (i + 1) % 3;  
        int k = (i + 2) % 3;

        float s = sqrt(mTcw.at<float>(i, i) - mTcw.at<float>(j,j) - mTcw.at<float>(k,k) + 1.0);
        mTUM[i] = s * 0.5;
        s = 0.5 / s;

        mTUM[3] = (mTcw.at<float>(k,j) - mTcw.at<float>(j,k)) * s;
        mTUM[j] = (mTcw.at<float>(j,i) + mTcw.at<float>(i,j)) * s;
        mTUM[k] = (mTcw.at<float>(k,i) + mTcw.at<float>(i,k)) * s;
    }

	mTUM[4] = mTcw.at<float>(0,3);
	mTUM[5] = mTcw.at<float>(1,3);
	mTUM[6] = mTcw.at<float>(2,3);

}


void VisualOdometry::SaveTrajectory()
{
	Rotation2Quaternion();
	
	mOutFile << mTimestamp << "e-8" << " " << mTUM[4] << " " << mTUM[5] << " " << mTUM[6] << " " << mTUM[0] << " " << mTUM[1] << " " << mTUM[2] << " " << mTUM[3] << endl;

}



 }
