#ifndef FRAME_H
#define FRAME_H

#include "common.h"
#include "map.h"
#include "map_point.h"
#include "camera.h"
#include "ORBextractor.h"
#include "matcher.h"

namespace Simple_ORB_SLAM
{ 
const int FRAME_GRID_ROWS = 48;
const int FRAME_GRID_COLS = 64;


class MapPoint;
class Map;
class Mather;

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
	
	//key points
	cv::Point2f GetKp2d(size_t i);
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


    cv::Point3f UnprojectStereo(const int &i);
	std::vector<size_t> GetFeaturesInArea(const float &x, const float  &y, const float  &r, const int minLevel=-1, const int maxLevel=-1) const;
	bool IsInFrustum(MapPoint *pMP, float viewingCosLimit);
	cv::Point3f GetCameraCenter();

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
	cv::Point3f mptCameraCenter;

	std::vector<float> mvuRight;
    std::vector<float> mvDepth;
	std::vector<bool> mvbOutlier;
	std::vector<cv::KeyPoint> mvKeysUn;
	std::vector<cv::KeyPoint> mvKeys, mvKeysRight;

    float mbf, mb, cx, cy, fx, fy;

	float mnMinX;
    float mnMaxX;
    float mnMinY;
    float mnMaxY;

 	// Scale
    int mnScaleLevels;
    float mfScaleFactor;
    float mfLogScaleFactor;
    vector<float> mvScaleFactors;
    vector<float> mvInvScaleFactors;
    vector<float> mvLevelSigma2;
    vector<float> mvInvLevelSigma2;

private:

	//detect feature
	void DetectFeature(const cv::Mat& imgLeft, const cv::Mat& imgRight);
	void ComputeImageBounds(const cv::Mat &imLeft);
	void AssignFeaturesToGrid();
	bool PosInGrid(const cv::KeyPoint &kp, int &posX, int &posY);
	void ExtractORB(int flag, const cv::Mat &im);
	void UndistortKeyPoints();

	//stereo
	void Stereo(const cv::Mat& imgLeft, const cv::Mat& imgRight);
	void ComputeStereoMatches();

private:

	//image's orb feature 
	std::vector<cv::KeyPoint> mKps, mKpsRight;
	std::vector<cv::Point2f> mKps2d, mKpsRight2d;
	std::vector<cv::Point3f> mKps3d;
	cv::Mat mDescriptors, mDescriptorsRight;
	
	ORBextractor* mpORBextractorLeft, *mpORBextractorRight;
	std::vector<std::size_t> mGrid[FRAME_GRID_COLS][FRAME_GRID_ROWS];


	float mfGridElementWidthInv;
    float mfGridElementHeightInv;





	Map* mpMap;

	//bool tag
	bool mbBadFlag;
	bool mbFirstConnection;

	//covisibility graph
	std::map<Frame*, size_t> mConnectedKeyFrameWeights;
	std::vector<Frame*> mvpOrderedKeyFrames;
	std::vector<size_t> mvOrderedWeights;

};




 }




#endif 
