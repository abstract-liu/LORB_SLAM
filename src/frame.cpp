#include "../include/frame.h"
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <vector>


namespace Simple_ORB_SLAM
{ 

Frame::Frame()
{
}


Frame::Frame(const cv::Mat& imgLeft, const cv::Mat& imgRight, Camera* camera)
{
	mpCamera = camera;
	
	mTcw = Mat::eye(4,4,CV_32F);
	mTwc = Mat::eye(4,4,CV_32F);

	//extract orb feature
	DetectFeature(imgLeft, imgRight);
	
	
	//stereo match
	Stereo(imgLeft, imgRight);


	//show orb feature map
	cv::Point kPs;
	Mat imgLeftCopy = imgLeft.clone();
	for(size_t i=0; i<mN; i++)
	{
		kPs.x = int(mKps2d[i].x);
		kPs.y = int(mKps2d[i].y);
		cv::circle(imgLeftCopy, kPs, 4,  CV_RGB(0, 255, 0));
	}
	cv::imshow("Simple_ORB_SLAM: Current Frame",imgLeftCopy);
	//cv::waitKey(0);
	cv::waitKey(250/mpCamera->fps);

}




void Frame::DetectFeature(const cv::Mat& imgLeft, const cv::Mat& imgRight)
{
	Ptr<cv::FeatureDetector> detector = ORB::create();
	Ptr<cv::DescriptorExtractor> descriptor = ORB::create();

	detector->detect(imgLeft, mKps);
	detector->detect(imgRight, mKpsRight);

	descriptor->compute(imgLeft, mKps, mDescriptors);
	descriptor->compute(imgRight, mKpsRight, mDescriptorsRight);
	
	mN = mKps.size();
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
	mN = k; 
	


}


void Frame::SetPose(const cv::Mat& Tcw)
{
	mTcw = Tcw.clone();
	mTwc = Tcw.inv();
}


cv::Mat Frame::GetPose()
{
	cv::Mat pose = mTcw.clone();
	return pose;
}


cv::Mat Frame::GetInvPose()
{
	cv::Mat invPose = mTwc.clone();
	return invPose;
}


std::vector<cv::Point2f> Frame::GetKps2d()
{
	return mKps2d;
}

std::vector<cv::Point3f> Frame::GetKps3d()
{
	return mKps3d;
}

cv::Mat Frame::GetDescriptors()
{
	cv::Mat descriptors = mDescriptors.clone();
	return descriptors;
}

// project to the frame 
bool Frame::IsVisible(MapPoint* point, cv::Point2f ptInImg)
{	
	cv::Point3f tPoint = point->GetPose(), ptInCam3d;
	cv::Mat ptInCam;
	cv::Mat ptInWorld = (cv::Mat_<float>(4,1) << tPoint.x, tPoint.y, tPoint.z , 1);
	
	ptInCam = mTcw * ptInWorld;
	
	ptInCam3d.x = ptInCam.at<float>(0);
	ptInCam3d.y = ptInCam.at<float>(1);
	ptInCam3d.z = ptInCam.at<float>(2);

	bool tag = mpCamera->Project(ptInCam3d, ptInImg);
	
	return tag;

}




 }
