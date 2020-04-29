#include "../include/bundle_adjust.h"
#include <ceres/autodiff_cost_function.h>
#include <ceres/cost_function.h>
#include <ceres/problem.h>
#include <ceres/solver.h>
#include <ceres/types.h>
#include <ceres/rotation.h>


namespace Simple_ORB_SLAM
{

struct PnPCost
{
	PnPCost(cv::Point3f kps3d, cv::Point2f kps2d, Camera* camera)
	{
		mKps3d = kps3d;
		mKps2d = kps2d;;
		mpCamera = camera;
	}

	template< typename T>
	bool operator()(const T* const tempR, const T* const tempT, T* residual) const
	{
		T tKps3d[3], tRKps3d[3];
		T tKps2d[2];
			
		tKps3d[0] = T(mKps3d.x);
		tKps3d[1] = T(mKps3d.y);
		tKps3d[2] = T(mKps3d.z);

		tKps2d[0] = T(mKps2d.x);
		tKps2d[1] = T(mKps2d.y);
		
		ceres::AngleAxisRotatePoint(tempR, tKps3d, tRKps3d);

		tRKps3d[0] += tempT[0];
		tRKps3d[1] += tempT[1];
		tRKps3d[2] += tempT[2];

		T u = tRKps3d[0] / tRKps3d[2] * T(mpCamera->fx) + T(mpCamera->cx);
		T v = tRKps3d[1] / tRKps3d[2] * T(mpCamera->fx) + T(mpCamera->cy);
			
		residual[0] = u - tKps2d[0];
		residual[1] = v - tKps2d[1];
		
		//wo shi da sb
		return true;

	}
	
	cv::Point3f mKps3d;
	cv::Point2f mKps2d;
	Camera* mpCamera;
};


struct LocalCost
{
	LocalCost(cv::Point3f kps3d, cv::Point2f kps2d, Camera* camera)
	{
		mKps3d = kps3d;
		mKps2d = kps2d;;
		mpCamera = camera;
	}

	template< typename T>
	bool operator()(const T* const tempR, const T* const tempT, T* residual) const
	{
		T tKps3d[3], tRKps3d[3];
		T tKps2d[2];
			
		tKps3d[0] = T(mKps3d.x);
		tKps3d[1] = T(mKps3d.y);
		tKps3d[2] = T(mKps3d.z);

		tKps2d[0] = T(mKps2d.x);
		tKps2d[1] = T(mKps2d.y);
		
		ceres::AngleAxisRotatePoint(tempR, tKps3d, tRKps3d);

		tRKps3d[0] += tempT[0];
		tRKps3d[1] += tempT[1];
		tRKps3d[2] += tempT[2];

		T u = tRKps3d[0] / tRKps3d[2] * T(mpCamera->fx) + T(mpCamera->cx);
		T v = tRKps3d[1] / tRKps3d[2] * T(mpCamera->fx) + T(mpCamera->cy);
			
		residual[0] = u - tKps2d[0];
		residual[1] = v - tKps2d[1];
		
		//wo shi da sb
		return true;

	}
	
	cv::Point3f mKps3d;
	cv::Point2f mKps2d;
	Camera* mpCamera;
};

BA::BA()
{

}


void BA::PnpPoseAdjust(const vector<cv::Point3f>& kps3d, const vector<cv::Point2f>& kps2d, cv::Mat& R, cv::Mat& T, Camera* camera)
{


	// intial value
	double initialR[3], initialT[3];
	for(size_t i=0; i<3; i++)
	{	
		initialT[i] = T.at<float>(i);
		initialR[i] = R.at<float>(i);
	}

	//define problem
	ceres::Problem problem;
	for(size_t i=0; i<kps3d.size(); i++)
	{
		ceres::CostFunction* tempCost = new ceres::AutoDiffCostFunction<PnPCost, 2,3,3>(
				new PnPCost(kps3d[i], kps2d[i], camera));
		problem.AddResidualBlock(tempCost, NULL, initialR, initialT);
	
	}

	//solve problem
	ceres::Solver::Options option;
	
	option.linear_solver_type = ceres::DENSE_SCHUR;

	ceres::Solver::Summary summary;

	ceres::Solve(option, &problem, &summary);
	
	//update R and T
	cv::Mat newR = (cv::Mat_<float>(3,1)<<initialR[0],initialR[1],initialR[2]);
	cv::Mat newT = ( cv::Mat_<float>(3,1)<<initialT[0],initialT[1],initialT[2]);
	
	R = newR.clone();
	T = newT.clone();

}



void BA::LocalPoseAdjust(const vector<cv::Point3f>& kps3d, const vector<cv::Point2f>& kps2d, cv::Mat& R, cv::Mat& T, Camera* camera)
{


	// intial value
	double initialR[3], initialT[3];
	for(size_t i=0; i<3; i++)
	{	
		initialT[i] = T.at<float>(i);
		initialR[i] = R.at<float>(i);
	}

	//define problem
	ceres::Problem problem;
	for(size_t i=0; i<kps3d.size(); i++)
	{
		ceres::CostFunction* tempCost = new ceres::AutoDiffCostFunction<LocalCost, 2,3,3>(
				new LocalCost(kps3d[i], kps2d[i], camera));
		problem.AddResidualBlock(tempCost, NULL, initialR, initialT);
	
	}

	//solve problem
	ceres::Solver::Options option;

	option.linear_solver_type = ceres::DENSE_SCHUR;

	ceres::Solver::Summary summary;

	ceres::Solve(option, &problem, &summary);
	
	//update R and T
	cv::Mat newR = (cv::Mat_<float>(3,1)<<initialR[0],initialR[1],initialR[2]);
	cv::Mat newT = ( cv::Mat_<float>(3,1)<<initialT[0],initialT[1],initialT[2]);
	
	R = newR.clone();
	T = newT.clone();

}


}
