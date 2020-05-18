#include "../include/camera.h"
#include <opencv2/core/persistence.hpp>
#include <opencv2/imgproc.hpp>

namespace Simple_ORB_SLAM
{ 
Camera::Camera(const string& strParaFile)
{	
	cv::FileStorage file(strParaFile+"EuRoC.yaml", cv::FileStorage::READ);
	
	// 每一帧提取的特征点数 1000
   	mnFeatures = file["ORBextractor.nFeatures"];
    // 图像建立金字塔时的变化尺度 1.2
    mfScaleFactor = file["ORBextractor.scaleFactor"];
    // 尺度金字塔的层数 8
    mnLevels = file["ORBextractor.nLevels"];
    // 提取fast特征点的默认阈值 20
    mfIniThFAST = file["ORBextractor.iniThFAST"];
    // 如果默认阈值提取不出足够fast特征点，则使用最小阈值 8
    mfMinThFAST = file["ORBextractor.minThFAST"];

	mMinPNPN = file["MinPNPN"];
	mMaxNorm = file["MaxNorm"];

	mKeyFrameSize = file["Viewer.KeyFrameSize"];
	mKeyFrameLineWidth = file["Viewer.KeyFrameLineWidth"];
	mPointSize = file["Viewer.PointSize"];
	mCameraSize = file["Viewer.CameraSize"];
	mCameraLineWidth = file["Viewer.CameraLineWidth"];

	fps = file["Camera.fps"];
	
	mViewpointF = file["Viewer.ViewpointF"];
	mViewpointX = file["Viewer.ViewpointX"];
	mViewpointY = file["Viewer.ViewpointY"];
	mViewpointZ = file["Viewer.ViewpointZ"];

	fx = file["Camera.fx"];
	fy = file["Camera.fy"];
	cx = file["Camera.cx"];
	cy = file["Camera.cy"];
	bf = file["Camera.bf"];

	file["LEFT.K"] >> K_l;
	file["RIGHT.K"] >> K_r;

	file["LEFT.P"] >> P_l;
	file["RIGHT.P"] >> P_r;

	file["LEFT.R"] >> R_l;
	file["RIGHT.R"] >> R_r;

	file["LEFT.D"] >> D_l;
	file["RIGHT.D"] >> D_r;

	rows_l = file["LEFT.height"];
	cols_l = file["LEFT.width"];
	rows_r = file["RIGHT.height"];
	cols_r = file["RIGHT.width"];


    if(K_l.empty() || K_r.empty() || P_l.empty() || P_r.empty() || R_l.empty() || R_r.empty() || D_l.empty() || D_r.empty() ||
            rows_l==0 || rows_r==0 || cols_l==0 || cols_r==0)
    {
        cerr << "ERROR: Calibration parameters to rectify stereo are missing!" << endl;
    }

    cv::initUndistortRectifyMap(K_l,D_l,R_l,P_l.rowRange(0,3).colRange(0,3),cv::Size(cols_l,rows_l), CV_32F,M1l,M2l);
    cv::initUndistortRectifyMap(K_r,D_r,R_r,P_r.rowRange(0,3).colRange(0,3),cv::Size(cols_r,rows_r),CV_32F,M1r,M2r);

	pi = (Mat_<float>(3,3)<<fx, 0.0f, cx, 0.0f, fy, cy, 0.0f, 0.0f, 1.0f);

}
void Camera::RectifyL(const cv::Mat& img_src, cv::Mat& img_rec)
{
	cv::remap(img_src, img_rec, M1l,M2l,cv::INTER_LINEAR);	
}


void Camera::RectifyR(const cv::Mat& img_src, cv::Mat& img_rec)
{
	cv::remap(img_src, img_rec, M1r,M2r, cv::INTER_LINEAR);
}


bool Camera::Project(const Point3f& pt_3d, Point2f& pt_2d)
{
	float min_u = 3, max_u = cols_l-3;
	float min_v = 3, max_v = rows_l-3;
	float u, v;
	u = fx*pt_3d.x/pt_3d.z + cx;
	v = fy*pt_3d.y/pt_3d.z + cy;
	
	if( u<min_u || u>max_u || v<min_v || v>max_v)
		return false;
	
	pt_2d.x = u;
	pt_2d.y = v;
	return true;
	

}
 }
