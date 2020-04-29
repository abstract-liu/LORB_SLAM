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
	mpGlobalMap = map;
	mpLocalMap = new Map(); 

	isFistFrame = true;

	std::unique_lock<std::mutex> lock(mPosLock);
	mTcw = Mat::eye(4,4,CV_32F);
	mTcc = Mat::eye(4,4,CV_32F);

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
	mCurrImg = imgLeft.clone();
	mTimestamp = timeStamp;

	if(isFistFrame == true)
	{
		isFistFrame = false;
		Initialize();
		return;
	}
	
	bool tag;

	//pnp track
	PnPMatchFeature();	
	tag = EstimatePosePnP();
	if(tag == false)	
	{ 
		EstimatePoseMotion();
	}
	BA::PnpPoseAdjust(mPrevKps3d, mCurrKps2d, mRvec, mTvec, mpCamera);
	UpdatePnP();

	//local map track
	LocalMatchFeature();
	BA::LocalPoseAdjust(mPrevKps3d, mCurrKps2d, mRvec, mTvec, mpCamera);	
	UpdateMap();

	//save data
	SaveTrajectory();	

}

void VisualOdometry::Initialize()
{
	Frame* pKF = mCurrFrame;
	
	//add frame to global and local map
	mpGlobalMap->AddKeyFrame(pKF);
	mpLocalMap->AddKeyFrame(pKF);

	//add points to global and local map
	std::vector<cv::Point3f> kps3d = pKF->GetKps3d();
	cv::Mat descriptors = pKF->GetDescriptors();

	for(size_t i=0; i<pKF->mN; i++)
	{
		cv::Mat ptInCam = (Mat_<float>(4,1) << kps3d[i].x, kps3d[i].y,kps3d[i].z, 1);
		cv::Mat ptInWorld =	mTcw.inv() * ptInCam;
		cv::Point3f tempPt(ptInWorld.at<float>(0), ptInWorld.at<float>(1), ptInWorld.at<float>(2));

		MapPoint* pMP = new MapPoint(tempPt, descriptors.row(i));
		mpGlobalMap->AddMapPoint(pMP);
		mpLocalMap->AddMapPoint(pMP);

	}

	mCurrFrame->SetPose(mTcw);
	mPrevFrame = mCurrFrame;


}







void VisualOdometry::PnPMatchFeature()
{

	mPrevKps3d = mPrevFrame->GetKps3d();
	mCurrKps2d = mCurrFrame->GetKps2d();
	vector<Point2f> tempKps2d;
	vector<Point3f> tempKps3d;

	cv::Mat prevDescriptors = mPrevFrame->GetDescriptors();
	cv::Mat currDescriptors = mCurrFrame->GetDescriptors();
	
	cv::BFMatcher matcher(NORM_HAMMING, true);
	vector<DMatch> matches;



	matcher.match(currDescriptors, prevDescriptors, matches);

	double minDist = 9999;
	for(size_t i=0; i<matches.size(); i++)
	{
		if(matches[i].distance < minDist)
			minDist = matches[i].distance;
	}


	for(size_t i=0; i<matches.size(); i++)
	{
		if(matches[i].distance > max(2*minDist, 30.0))
			continue;
		tempKps2d.push_back( mCurrKps2d[matches[i].queryIdx]);
		tempKps3d.push_back( mPrevKps3d[matches[i].trainIdx]);
	}
	
	mPrevKps3d = tempKps3d;
	mCurrKps2d = tempKps2d;
	


}


bool VisualOdometry::EstimatePosePnP()
{
	//fist ransac pnp
	vector<int> inliers;
	vector<Point3f> tempKps3d;
	vector<Point2f> tempKps2d;


	if(mPrevKps3d.size() < 4)
	{
		cout << "not enough points for pnp" << endl;
		return false;
	}
	
	solvePnPRansac(mPrevKps3d, mCurrKps2d, mpCamera->pi, Mat(),mRvec, mTvec, false, 500, 2.0f, 0.999, inliers, SOLVEPNP_ITERATIVE);

	//reject outliers pnp
	for(size_t i=0; i<inliers.size(); i++)
	{
		tempKps3d.push_back( mPrevKps3d[inliers[i]] );
		tempKps2d.push_back( mCurrKps2d[inliers[i]] );
	}

	mPrevKps3d = tempKps3d;
	mCurrKps2d = tempKps2d;

	if(mPrevKps3d.size() < 4 )
	{
		cout << "not enough points for pnp" << endl;
		return false;
	}


	solvePnP(mPrevKps3d, mCurrKps2d, mpCamera->pi, Mat(), mRvec, mTvec );
	
	mRvec.convertTo(mRvec, CV_32F);
	mTvec.convertTo(mTvec, CV_32F);	

	if(cv::norm(mTvec, cv::NORM_L2) > mpCamera->mMaxNorm)
	{
		cout << cv::norm(mTvec, cv::NORM_L2) << endl;
		return false;
	}

	return true;
}


void VisualOdometry::EstimatePoseMotion()
{
	cv::Mat R;
	mTcc.rowRange(0,3).colRange(0,3).copyTo(R);
	mTcc.rowRange(0,3).col(3).copyTo(mTvec);
	cv::Rodrigues(R, mRvec);
}



void VisualOdometry::UpdatePnP()
{
	std::unique_lock<std::mutex> lock(mPosLock);
	mPrevFrame = mCurrFrame;
	
	//transform se3 to SE3
	cv::Mat R;
	cv::Rodrigues(mRvec, R);
	
	mTcc = cv::Mat::eye(4,4,mTcw.type());
    R.copyTo(mTcc.rowRange(0,3).colRange(0,3));
    mTvec.copyTo(mTcc.rowRange(0,3).col(3));

	//update Tcw Rvec Tvec (wold -> camera)
	cv::Mat newTcw;
	newTcw = mTcw * mTcc;
	newTcw.rowRange(0,3).colRange(0,3).copyTo(R);
	newTcw.rowRange(0,3).col(3).copyTo(mTvec);
	cv::Rodrigues(R,mRvec);

	mCurrFrame->SetPose(newTcw);

}


cv::Mat VisualOdometry::GetCurrInvPos()
{
	std::unique_lock<std::mutex> lock(mPosLock);
	cv::Mat invPose = mTcw.inv();
	return invPose;
}


void VisualOdometry::LocalMatchFeature()
{
	//find visible points

	//initialize
	std::vector<cv::Point2f> visibleKps2d;
	std::vector<MapPoint*> visibleKps3d = mpLocalMap->GetPoints();
	cv::Mat visibleDescriptors;


	//find visible points
	size_t k =0;
	for(size_t i=0; i<mpLocalMap->mMPN; i++)
	{
		cv::Point2f visibleKp2d;
		MapPoint* visibleKp3d = visibleKps3d[i];

		bool tag = mCurrFrame->IsVisible(visibleKp3d, visibleKp2d);
		if(tag == true)
		{
			visibleKps3d[k] = visibleKp3d;
			visibleKps2d.push_back(visibleKp2d);
			visibleDescriptors.push_back(visibleKp3d->GetDescriptor());

			k += 1;
		}
		else
		{
			mpLocalMap->mUnSeenTimes[i] += 1;
		}
	}
	visibleKps3d.resize(k);

	//match 3d points and 2d points
	
	//initialize
	mPrevKps3d.clear();
	mCurrKps2d.clear();

	cv::BFMatcher matcher(NORM_HAMMING, true);
	std::vector<DMatch> matches;
	cv::Mat currDescriptors = mCurrFrame->GetDescriptors();
	


	matcher.match(currDescriptors, visibleDescriptors, matches);

	double minDist = 9999, maxDist = 0;
	for(size_t i=0; i<matches.size(); i++)
	{
		double dist = matches[i].distance;
		if(dist < minDist )
			minDist = dist;
		if( dist > maxDist )
			maxDist = dist;
	}
	

	std::vector<cv::Point2f> currKps2d = mCurrFrame->GetKps2d();
	for(size_t i=0; i<matches.size(); i++)
	{
		if(matches[i].distance <= std::max(2*minDist, 30.0))
		{
			mPrevKps3d.push_back(visibleKps3d[matches[i].trainIdx]->GetPose());
			mCurrKps2d.push_back(currKps2d[matches[i].queryIdx]);
		}
	}


}



//add current frame all points
//delete points that are not seen over 3 times
//every frame is key frame

void VisualOdometry::UpdateMap()
{
{
	std::unique_lock<std::mutex> lock(mPosLock);
	// update pose message
	cv::Mat R, newTcw, newTcc;
	cv::Rodrigues(mRvec, R);
	
	newTcw = cv::Mat::eye(4,4,mTcw.type());
    R.copyTo(newTcw.rowRange(0,3).colRange(0,3));
    mTvec.copyTo(newTcw.rowRange(0,3).col(3));
	

	newTcc = mTcw.inv() * newTcw;
	newTcc.rowRange(0,3).col(3).copyTo(mTvec);

	if(cv::norm(mTvec, cv::NORM_L2) > mpCamera->mMaxNorm )
	{
		cout << "bundle " << cv::norm(mTvec, cv::NORM_L2) << endl; 
		mTcw = mCurrFrame->GetPose();
	}
	else
	{
		mTcc = newTcc.clone();
		mTcw = newTcw.clone();
	}

	mCurrFrame->SetPose(mTcw);
}

	//add new points to global and local map
	std::vector<cv::Point3f> kps3d = mCurrFrame->GetKps3d();
	cv::Mat descriptors = mCurrFrame->GetDescriptors();

	for(size_t i=0; i<mCurrFrame->mN; i++)
	{
		cv::Mat ptInCam = (Mat_<float>(4,1) << kps3d[i].x, kps3d[i].y,kps3d[i].z, 1);
		cv::Mat ptInWorld =	mTcw.inv() * ptInCam;
		cv::Point3f tempPt(ptInWorld.at<float>(0), ptInWorld.at<float>(1), ptInWorld.at<float>(2));

		MapPoint* pMP = new MapPoint(tempPt, descriptors.row(i));
		mpGlobalMap->AddMapPoint(pMP);
		mpLocalMap->AddMapPoint(pMP);
	}

	//add new key frames to global map
	mpGlobalMap->AddKeyFrame(mCurrFrame);


	//delete bad points in local map
	mpLocalMap->EraseBadPoints();


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
