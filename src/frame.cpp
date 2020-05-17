#include "../include/frame.h"
#include <opencv2/highgui.hpp>
#include <vector>


namespace Simple_ORB_SLAM
{ 

size_t Frame::Idx = 0;

Frame::Frame()
{
}


Frame::Frame(const cv::Mat& imgLeft, const cv::Mat& imgRight, Camera* camera)
{
	mpCamera = camera;
	
	mTcw = cv::Mat::eye(4,4,CV_32F);
	
	mnId = Idx++;

	mbBadFlag = false;


	//extract orb feature
	DetectFeature(imgLeft, imgRight);
	
	
	//stereo match
	Stereo(imgLeft, imgRight);
	
	mvpMapPoints = std::vector<MapPoint*>(mnMapPoints, static_cast<MapPoint*>(NULL));

	//show orb feature map
	cv::Point kPs;
	Mat imgLeftCopy = imgLeft.clone();
	for(size_t i=0; i<mnMapPoints; i++)
	{
		kPs.x = int(mKps2d[i].x);
		kPs.y = int(mKps2d[i].y);
		cv::circle(imgLeftCopy, kPs, 4,  CV_RGB(0, 255, 0));
	}
	cv::imshow("Simple_ORB_SLAM: Current Frame",imgLeftCopy);
	cv::waitKey(250/mpCamera->fps);
	//cv::waitKey(0);
}




void Frame::DetectFeature(const cv::Mat& imgLeft, const cv::Mat& imgRight)
{
	Ptr<cv::FeatureDetector> detector = ORB::create();
	Ptr<cv::DescriptorExtractor> descriptor = ORB::create();

	detector->detect(imgLeft, mKps);
	detector->detect(imgRight, mKpsRight);

	descriptor->compute(imgLeft, mKps, mDescriptors);
	descriptor->compute(imgRight, mKpsRight, mDescriptorsRight);
	
	mnMapPoints = mKps.size();
	cv::KeyPoint::convert(mKps, mKps2d);
	cv::KeyPoint::convert(mKpsRight, mKpsRight2d);
	
}

void Frame::Stereo(const cv::Mat& imgLeft, const cv::Mat& imgRight)
{
	vector<DMatch> matches;
	cv::Mat newDescriptors;
	cv::BFMatcher matcher(NORM_HAMMING,true);


	matcher.match(mDescriptors, mDescriptorsRight, matches);
	vector<Point2f> tempKps2d;

	double min_dist = 9999, maxDist = 0;
	for(size_t i=0; i<matches.size(); i++)
	{
		if(matches[i].distance < min_dist)
			min_dist = matches[i].distance;
		if(matches[i].distance > maxDist)
			maxDist = matches[i].distance;
	}


	size_t k =0;
	for(size_t i=0; i<matches.size(); i++)
	{
		if(abs(mKps2d[matches[i].queryIdx].y - mKpsRight2d[matches[i].trainIdx].y) > 2)
			continue;
		float disparity = mKps2d[matches[i].queryIdx].x - mKpsRight2d[matches[i].trainIdx].x;
		if((disparity<5) || disparity>150)
			continue;
		if(matches[i].distance < max(min_dist,30.0))
			continue;

		Point3f point;
		point.z = mpCamera->bf / disparity ;
		point.x = (mKps2d[matches[i].queryIdx].x - mpCamera->cx) * point.z/ mpCamera->fx;
		point.y = (mKps2d[matches[i].queryIdx].y- mpCamera->cy) * point.z/ mpCamera->fy;
		
		tempKps2d.push_back(mKps2d[matches[i].queryIdx]);
		mKps3d.push_back(point);
		newDescriptors.push_back(mDescriptors.row(matches[i].queryIdx));

		k++;

	}
    
	mDescriptors = newDescriptors.clone();
    mKps2d = tempKps2d;
	mnMapPoints = k; 
	


}


void Frame::SetPose(const cv::Mat& Tcw)
{
	mTcw = Tcw.clone();
	
	UpdatePoseVector();
}

void Frame::SetPose(const cv::Mat& Tvec, const cv::Mat& Rvec)
{
	mTvec = Tvec.clone();
	mRvec = Rvec.clone();

	UpdatePoseMat();
}

void Frame::UpdatePoseMat()
{
	cv::Mat rotMat;
	cv::Rodrigues(mRvec, rotMat);
	rotMat.copyTo(mTcw.rowRange(0,3).colRange(0,3));
	mTvec.copyTo(mTcw.rowRange(0,3).col(3));

}


void Frame::UpdatePoseVector()
{
	cv::Mat rotMat;
	mTcw.rowRange(0,3).colRange(0,3).copyTo(rotMat);
	cv::Rodrigues(rotMat,mRvec);
	mTcw.rowRange(0,3).col(3).copyTo(mTvec);
}



cv::Mat Frame::GetPose()
{
	cv::Mat pose = mTcw.clone();
	return pose;
}


cv::Mat Frame::GetInvPose()
{
	cv::Mat invPose = mTcw.inv();
	return invPose;
}




cv::Point2f Frame::GetKp2d(size_t idx)
{
	return mKps2d[idx];
}

std::vector<cv::Point2f> Frame::GetKps2d()
{
	return mKps2d;
}


std::vector<cv::Point3f> Frame::GetKps3d()
{
	return mKps3d;
}





cv::Mat Frame::GetDescriptor(size_t idx)
{
	cv::Mat descriptor = mDescriptors.row(idx).clone();
	return descriptor;
}

cv::Mat Frame::GetDescriptors()
{
	cv::Mat descriptors = mDescriptors.clone();
	return descriptors;
}



// project to the frame 
bool Frame::IsVisible(MapPoint* point, cv::Point2f ptInImg)
{	
	cv::Point3f tPoint = point->GetPos(), ptInCam3d;
	cv::Mat ptInCam;
	cv::Mat ptInWorld = (cv::Mat_<float>(4,1) << tPoint.x, tPoint.y, tPoint.z , 1);
	
	ptInCam = mTcw * ptInWorld;
	
	ptInCam3d.x = ptInCam.at<float>(0);
	ptInCam3d.y = ptInCam.at<float>(1);
	ptInCam3d.z = ptInCam.at<float>(2);

	bool tag = mpCamera->Project(ptInCam3d, ptInImg);
	
	return tag;

}



bool Frame::IsBad()
{
	return mbBadFlag;	
}


void Frame::SetBadFlag()
{
	if(mnId == 0)
		return;

	//erase connected key frames
	for(std::map<Frame*, size_t>::iterator it = mConnectedKeyFrameWeights.begin(); it != mConnectedKeyFrameWeights.end(); it++)
	{
		it->first->EraseConnection(this);
	}
	
	//erase connected map points
	for(size_t i=0; i<mnMapPoints; i++)
	{
		MapPoint* pMP = mvpMapPoints[i];
		if(pMP != NULL)
		{
			pMP->EraseObservation(this);
		}
	}

	//clear vectors
	mConnectedKeyFrameWeights.clear();
	mvpOrderedKeyFrames.clear();

	mbBadFlag = true;

	mpMap->EraseFrame(this);


}














std::vector<Frame*> Frame::GetCovisibleFrames()
{
	return mvpOrderedKeyFrames;
}


std::vector<Frame*> Frame::GetBestCovisibleFrames(size_t N)
{
	if(mvpOrderedKeyFrames.size() < N)
		return mvpOrderedKeyFrames;
	else
		return std::vector<Frame*>(mvpOrderedKeyFrames.begin(), mvpOrderedKeyFrames.begin()+N);
}


size_t Frame::GetWeight(Frame* pF)
{
	if(mConnectedKeyFrameWeights.count(pF))
		return mConnectedKeyFrameWeights[pF];
	else
		return 0 ;
}



void Frame::UpdateBestCovisibleFrames()
{
	std::vector<std::pair<size_t, Frame*>> vPairs;
	for(std::map<Frame*,size_t>::iterator it = mConnectedKeyFrameWeights.begin(); it != mConnectedKeyFrameWeights.end(); it++)
	{
		vPairs.push_back(std::make_pair(it->second, it->first));
	}

	std::sort(vPairs.begin(), vPairs.end());

	//clear vector
	mvpOrderedKeyFrames.clear();
	mvOrderedWeights.clear();
	
	//write vector
	for(size_t i=0; i<vPairs.size(); i++)
	{
		mvpOrderedKeyFrames.push_back(vPairs[i].second);
		mvOrderedWeights.push_back(vPairs[i].first);
	}
}


void Frame::AddConnection(Frame* pF, size_t weight)
{
	if(mConnectedKeyFrameWeights.count(pF) == false)
		mConnectedKeyFrameWeights[pF] = weight;
	else if(mConnectedKeyFrameWeights[pF] != weight)
		mConnectedKeyFrameWeights[pF] = weight;
	else 
		return;

	UpdateBestCovisibleFrames();

}


void Frame::EraseConnection(Frame* pF)
{
	if(mConnectedKeyFrameWeights.count(pF))
	{
		mConnectedKeyFrameWeights.erase(pF);
		UpdateBestCovisibleFrames();
	}
}


void Frame::UpdateConnections()
{
	
	std::map<Frame*, size_t> frameCounter;
	std::vector<MapPoint*> vpMps = mvpMapPoints;

	//step 1 all map points
	for(size_t i=0; i<vpMps.size(); i++)
	{
		MapPoint* pMP = vpMps[i];
		if(pMP == NULL)
			continue;
		if(pMP->IsBad())
			continue;
//map is empty!!!!!!!!!!!!!!
		std::map<Frame* ,size_t> observations = pMP->GetObservations();
		for(std::map<Frame*, size_t>::iterator it = observations.begin(); it != observations.end(); it++)
		{
			if(it->first->mnId != mnId)
				frameCounter[it->first] += 1;
		}
	}

	//step 2 add connections
	size_t nMax = -1;
	Frame* pMax = NULL;
	std::vector<std::pair< size_t, Frame*>> vPairs;
	for(std::map<Frame*, size_t>::iterator it = frameCounter.begin(); it != frameCounter.end(); it++)
	{
		//memorize max
		if(it->second  > nMax)
		{
			nMax = it->second;
			pMax = it->first;
		}

		//over threshold
		if(it->second >= 3)
		{
			vPairs.push_back(std::make_pair(it->second, it->first));
			(it->first)->AddConnection(this, it->second);
		}
	}

	if(vPairs.empty() == true)
	{
		vPairs.push_back(std::make_pair(nMax,pMax));
		pMax->AddConnection(this, nMax);
	}

	std::sort(vPairs.begin(), vPairs.end());

	//update vectors
	cout << "3" << endl;
	mConnectedKeyFrameWeights = frameCounter;
	mvpOrderedKeyFrames.clear();
	mvOrderedWeights.clear();
	for(size_t i=0; i<vPairs.size(); i++)
	{
		mvpOrderedKeyFrames.push_back(vPairs[i].second);
		mvOrderedWeights.push_back(vPairs[i].first);
	}

	if(mbFirstConnection == true && mnId != 0)
	{
		mbFirstConnection = false;
	}

}





















void Frame::AddMapPoint(MapPoint* pMP, size_t i)
{
	mvpMapPoints[i] = pMP;
}

void Frame::EraseMapPoint(size_t id)
{
	mvpMapPoints[id] = static_cast<MapPoint*>(NULL);
}


std::vector<MapPoint*> Frame::GetMapPoints()
{
	return mvpMapPoints; 
}









}
