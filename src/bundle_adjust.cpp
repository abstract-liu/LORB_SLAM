#include "../include/bundle_adjust.h"
#include <ceres/autodiff_cost_function.h>
#include <ceres/cost_function.h>
#include <ceres/problem.h>
#include <ceres/solver.h>
#include <ceres/types.h>
#include <ceres/rotation.h>
#include <opencv2/core/types.hpp>
#include <vector>


namespace Simple_ORB_SLAM
{


BA::BA()
{

}


struct PoseCost
{
	PoseCost(cv::Point3f kps3d, cv::Point2f kps2d, Camera* camera)
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



struct MPCost
{
	MPCost(cv::Mat Tvec, cv::Mat Rvec, cv::Point2f kps2d, Camera* pCamera)
	{
		mTvec = Tvec.clone();
		mRvec = Rvec.clone();
		mKps2d = kps2d;
		mpCamera = pCamera;
	}

	
	template<typename T>
	bool operator()(const T* const tempMP, T* residual) const
	{
		T tKps2d[2], tKps3d[3], tRKps3d[3];
		T tRvec[3];

		//change tRvec
		for(size_t i=0; i<3; i++)
			tRvec[i] = T(mRvec.at<float>(i) );

		//change kps3d
		tKps2d[0] = T(mKps2d.x);
		tKps2d[1] = T(mKps2d.y);

		//change camera to world coordinate
		for(size_t i=0; i<3; i++)
			tKps3d[i] = tempMP[i];
		ceres::AngleAxisRotatePoint(tRvec, tKps3d, tRKps3d);
		for(size_t i=0; i<3; i++)
			tRKps3d[i] += T(mTvec.at<float>(i));

		T u = tRKps3d[0] / tRKps3d[2] * T(mpCamera->fx) + T(mpCamera->cx);
		T v = tRKps3d[1] / tRKps3d[2] * T(mpCamera->fy) + T(mpCamera->cy);

		residual[0] = u - tKps2d[0];
		residual[1] = v - tKps2d[1];
		
		return true;
	}

	cv::Point2f mKps2d;
	cv::Mat mTvec, mRvec;
	Camera* mpCamera;

};


struct	PoseMPCost
{
	PoseMPCost(cv::Point2f Kps2d, Camera* pCamera)
	{
		mKps2d = Kps2d;
		mpCamera = pCamera;
	}

	template<typename T>
	bool operator()(const T* const tempMP, const T* const tempPose, T* residual ) const
	{
		T tKps2d[2], tKps3d[3], tRKps3d[3];

		tKps2d[0] = T(mKps2d.x);
		tKps2d[1] = T(mKps2d.y);
		
		for(size_t i=0; i<3; i++)
			tKps3d[i] = tempMP[i];

		ceres::AngleAxisRotatePoint(tempPose, tKps3d, tRKps3d);
		for(size_t i=0; i<3; i++)
			tRKps3d[i] += tempPose[i+3];

		T u = tRKps3d[0] / tRKps3d[2] * T(mpCamera->fx) + T(mpCamera->cx);
		T v = tRKps3d[1] / tRKps3d[2] * T(mpCamera->fy) + T(mpCamera->cy);

		residual[0] = u - tKps2d[0];
		residual[1] = v - tKps2d[1];

		return true;
	}

	cv::Point2f mKps2d;
	Camera* mpCamera;

};






void BA::ProjectPoseOptimization(Frame* pCurrFrame)
{


	// intial value
	double initialR[3], initialT[3];
	for(size_t i=0; i<3; i++)
	{	
		initialT[i] = pCurrFrame->mTvec.at<float>(i);
		initialR[i] = pCurrFrame->mRvec.at<float>(i);
	}

	std::vector<cv::Point2f> kps2d = pCurrFrame->GetKps2d();

	//define problem
	ceres::Problem problem;
	for(size_t i=0; i<pCurrFrame->mnMapPoints; i++)
	{
		MapPoint* pMP = pCurrFrame->mvpMapPoints[i];
		if(pMP == NULL)
			continue;
		
		cv::Point2f kp2d = kps2d[i];
		cv::Point3f kp3d = pMP->GetPos();

		ceres::CostFunction* tempCost = new ceres::AutoDiffCostFunction<PoseCost, 2,3,3>(new PoseCost(kp3d, kp2d, pCurrFrame->mpCamera));
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
	
	pCurrFrame->SetPose(newT, newR);
}




void BA::LocalPoseOptimization(Frame* pCurrFrame)
{
	//get covisible frames
	std:vector<Frame*> vpLocalFrames;
	vpLocalFrames.push_back(pCurrFrame);
	std::vector<Frame*> vpCovisibleFrames = pCurrFrame->GetCovisibleFrames();
	for(size_t i=0; i<vpCovisibleFrames.size(); i++)
	{
		Frame* pF = vpCovisibleFrames[i];
		if(pF->IsBad() == false)
		{
			vpLocalFrames.push_back(pF);
		}
	}


	//get covisible map points
	std::vector<MapPoint*> vpLocalMapPoints;
	for(size_t i=0; i<vpLocalFrames.size(); i++)
	{
		std::vector<MapPoint*> vpMPs = vpLocalFrames[i]->GetMapPoints();
		for(size_t i=0; i<vpMPs.size(); i++)
		{
			MapPoint* pMP = vpMPs[i];
			if(pMP == NULL)
				continue;
			if(pMP->IsBad() == true)
				continue;
			std::vector<MapPoint*>::iterator it = std::find(vpLocalMapPoints.begin(), vpLocalMapPoints.end(), pMP);
			if(it != vpLocalMapPoints.end())
				continue;
			
			vpLocalMapPoints.push_back(pMP);
		}
	}

	//store initial value
	double framesPose[vpLocalFrames.size()][6];
	double mpsPose[vpLocalMapPoints.size()][3];
	for(size_t i=0; i<vpLocalFrames.size(); i++)
	{
		Frame* pF = vpLocalFrames[i];
		
		framesPose[i][0] = pF->mRvec.at<float>(0);
		framesPose[i][1] = pF->mRvec.at<float>(1);
		framesPose[i][2] = pF->mRvec.at<float>(2);
		framesPose[i][3] = pF->mTvec.at<float>(0);
		framesPose[i][4] = pF->mTvec.at<float>(1);
		framesPose[i][5] = pF->mTvec.at<float>(2);
	}
	for(size_t i=0; i<vpLocalMapPoints.size(); i++)
	{
		MapPoint* pMP = vpLocalMapPoints[i];
		cv::Point3f kps3d = pMP->GetPos();

		mpsPose[i][0] = kps3d.x;
		mpsPose[i][1] = kps3d.y;
		mpsPose[i][2] = kps3d.z;
	}


	//set residual block
	ceres::Problem problem;
	for(size_t i=0; i<vpLocalMapPoints.size(); i++)
	{
		MapPoint* pMP = vpLocalMapPoints[i];
		std::map<Frame*, size_t> observations = pMP->GetObservations();
		
		for(std::map<Frame*, size_t>::const_iterator it = observations.begin(); it != observations.end(); it++)
		{
			Frame* pF = it->first;
			if(pF->IsBad() == true)
				continue;
			std::vector<Frame*>::iterator vit = std::find(vpLocalFrames.begin(), vpLocalFrames.end(), pF);
			
			//optimize  mp
			if(vit == vpLocalFrames.end())
			{
				std::vector<cv::Point2f> kps2d = pF->GetKps2d();
				cv::Point2f kp2d = kps2d[it->second];

				ceres::CostFunction* tempCost = new ceres::AutoDiffCostFunction<MPCost, 2, 3>( new MPCost(pF->mTvec, pF->mRvec, kp2d, pCurrFrame->mpCamera) );
				problem.AddResidualBlock(tempCost, NULL, mpsPose[i]);
			}

			//optimize mp and pose
			else
			{	
				std::vector<cv::Point2f> kps2d = pF->GetKps2d();
				cv::Point2f kp2d = kps2d[it->second];
				

				ceres::CostFunction* tempCost = new ceres::AutoDiffCostFunction<PoseMPCost, 2, 3, 6>( new PoseMPCost(kp2d, pCurrFrame->mpCamera) );
				problem.AddResidualBlock(tempCost, NULL, mpsPose[i], framesPose[vit-vpLocalFrames.begin()]);
			}
		}
	}



	//solve problem
	ceres::Solver::Options option;
	
	option.linear_solver_type = ceres::DENSE_SCHUR;

	ceres::Solver::Summary summary;

	ceres::Solve(option, &problem, &summary);

	//update 
	for(size_t i=0; i<vpLocalFrames.size(); i++)
	{
		Frame* pF = vpLocalFrames[i];
		cv::Mat Rvec = (cv::Mat_<float>(3,1) << framesPose[i][0], framesPose[i][1], framesPose[i][2]);
		cv::Mat Tvec = (cv::Mat_<float>(3,1) << framesPose[i][3], framesPose[i][4], framesPose[i][5] );
		pF->SetPose(Tvec, Rvec);
	}
	for(size_t i=0; i<vpLocalMapPoints.size(); i++)
	{
		MapPoint* pMP = vpLocalMapPoints[i];
		cv::Point3f kp3d(mpsPose[i][0], mpsPose[i][1], mpsPose[i][2]);
		pMP->SetWorldPos(kp3d);
	}
}


}
