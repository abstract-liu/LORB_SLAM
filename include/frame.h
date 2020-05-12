#ifndef FRAME_H
#define FRAME_H

#include "common.h"
#include "map.h"
#include "map_point.h"
#include "camera.h"


namespace Simple_ORB_SLAM
{ 

class Frame
{
public:
	Frame();
	Frame(const cv::Mat& imgLeft, const cv::Mat& imgRight, Camera* camera);
	
	//judge is visible
	bool IsVisible(MapPoint* point, cv::Point2f ptInImg);


	//pose
	void SetPose(const cv::Mat& Tcw);
	void SetPose(const cv::Mat& Tvec, const cv::Mat& Rvec);
	void UpdatePoseMat();
	void UpdatePoseVector();
	cv::Mat GetPose();
	cv::Mat GetInvPose();
	
	//Kps
	std::vector<cv::Point2f> GetKps2d();
	std::vector<cv::Point3f> GetKps3d();

	//descriptors
	cv::Mat GetDescriptor(size_t idx);
	cv::Mat GetDescriptors();

	//get MapPoints
	void AddMapPoint(MapPoint* pMP, size_t i );
	void EraseMapPoint(size_t id);
	std::vector<MapPoint*> GetMapPoints();


	//bad flag
	bool IsBad();
	void SetBadFlag();


	//covisibility graph 
	std::vector<Frame*> GetBestCovisibleFrames(size_t N);
	std::vector<Frame*> GetCovisibleFrames();
	void UpdateBestCovisibleFrames();
	size_t GetWeight(Frame* pF);

	void EraseConnection(Frame* pF);
	void UpdateConnections();
	void AddConnection(Frame* pF, size_t weight);


	//spanning tree
	void EraseChild(Frame* pF);
	void AddChild(Frame* pF);
	std::set<Frame*> GetChildren();
	void ChangeParent(Frame* pF);
	Frame* GetParent();




public:
	static size_t Idx;
	size_t mnMapPoints;
	size_t mnId;

	std::vector<MapPoint*> mvpMapPoints;

	//camera parameter	
	Camera* mpCamera;

	//camera's pose
	cv::Mat mTcw, mTwc;
	cv::Mat mTvec, mRvec;



private:

	//detect feature
	void DetectFeature(const cv::Mat& imgLeft, const cv::Mat& imgRight);

	//stereo
	void Stereo(const cv::Mat& imgLeft, const cv::Mat& imgRight);


private:

	//image's orb feature 
	std::vector<cv::KeyPoint> mKps, mKpsRight;
	std::vector<cv::Point2f> mKps2d, mKpsRight2d;
	std::vector<cv::Point3f> mKps3d;
	cv::Mat mDescriptors, mDescriptorsRight;

	Map* mpMap;

	//bool tag
	bool mbBadFlag;
	bool mbFirstConnection;

	//covisibility graph
	std::map<Frame*, size_t> mConnectedKeyFrameWeights;
	std::vector<Frame*> mvpOrderedKeyFrames;
	std::vector<size_t> mvOrderedWeights;

	//spanning tree
	Frame* mpParent;
	std::set<Frame*> mspChildren;

};




 }




#endif 
