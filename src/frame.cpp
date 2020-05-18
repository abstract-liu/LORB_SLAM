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

	//initialize
	ComputeImageBounds(imgLeft);
  
    // tracking过程都会用到mpORBextractorLeft作为特征点提取器
    mpORBextractorLeft = new ORBextractor(mpCamera->mnFeatures, mpCamera->mfScaleFactor, mpCamera->mnLevels, mpCamera->mfIniThFAST, mpCamera->mfMinThFAST);
	mpORBextractorRight = new ORBextractor(mpCamera->mnFeatures, mpCamera->mfScaleFactor, mpCamera->mnLevels, mpCamera->mfIniThFAST, mpCamera->mfMinThFAST);

	// Scale Level Info
    mnScaleLevels = mpORBextractorLeft->GetLevels();
    mfScaleFactor = mpORBextractorLeft->GetScaleFactor();
    mfLogScaleFactor = log(mfScaleFactor);
    mvScaleFactors = mpORBextractorLeft->GetScaleFactors();
    mvInvScaleFactors = mpORBextractorLeft->GetInverseScaleFactors();
    mvLevelSigma2 = mpORBextractorLeft->GetScaleSigmaSquares();
    mvInvLevelSigma2 = mpORBextractorLeft->GetInverseScaleSigmaSquares();


	//extract orb feature
	thread threadLeft(&Frame::ExtractORB,this,0,imgLeft);
    thread threadRight(&Frame::ExtractORB,this,1,imgRight);
    threadLeft.join();
    threadRight.join();
	mnMapPoints = mvKeys.size();
	UndistortKeyPoints();


	//stereo match
    ComputeStereoMatches();
	
	mvpMapPoints = std::vector<MapPoint*>(mnMapPoints, static_cast<MapPoint*>(NULL));
    mvbOutlier = vector<bool>(mnMapPoints,false);

	//assign feature to grid
    AssignFeaturesToGrid();

	//show orb feature map
	cv::imshow("Simple_ORB_SLAM: Current Frame",imgLeft);
	cv::waitKey(250/mpCamera->fps);
	//cv::waitKey(0);
}






void Frame::ComputeImageBounds(const cv::Mat &imLeft)
{
    mbf = mpCamera->bf;
	mb = mbf/mpCamera->fx;
    cx = mpCamera->cx;
    cy = mpCamera->cy;
    fx = mpCamera->fx;
    fy = mpCamera->fy;

    mnMinX = 0.0f;
    mnMaxX = imLeft.cols;
    mnMinY = 0.0f;
    mnMaxY = imLeft.rows;
    mfGridElementWidthInv=static_cast<float>(FRAME_GRID_COLS)/(mnMaxX-mnMinX);
    mfGridElementHeightInv=static_cast<float>(FRAME_GRID_ROWS)/(mnMaxY-mnMinY);
}

void Frame::AssignFeaturesToGrid()
{
    int nReserve = 0.5f*mnMapPoints/(FRAME_GRID_COLS*FRAME_GRID_ROWS);
    for(unsigned int i=0; i<FRAME_GRID_COLS;i++)
        for (unsigned int j=0; j<FRAME_GRID_ROWS;j++)
            mGrid[i][j].reserve(nReserve);

    // 在mGrid中记录了各特征点
    for(int i=0;i<mnMapPoints;i++)
    {
        const cv::KeyPoint &kp = mvKeysUn[i];

        int nGridPosX, nGridPosY;
        if(PosInGrid(kp,nGridPosX,nGridPosY))
            mGrid[nGridPosX][nGridPosY].push_back(i);
    }
}

bool Frame::PosInGrid(const cv::KeyPoint &kp, int &posX, int &posY)
{
    posX = round((kp.pt.x-mnMinX)*mfGridElementWidthInv);
    posY = round((kp.pt.y-mnMinY)*mfGridElementHeightInv);

    //Keypoint's coordinates are undistorted, which could cause to go out of the image
    if(posX<0 || posX>=FRAME_GRID_COLS || posY<0 || posY>=FRAME_GRID_ROWS)
        return false;

    return true;
}

void Frame::ExtractORB(int flag, const cv::Mat &im)
{
    if(flag==0)
        (*mpORBextractorLeft)(im,cv::Mat(),mvKeys,mDescriptors);
    else
        (*mpORBextractorRight)(im,cv::Mat(),mvKeysRight,mDescriptorsRight);
}

void Frame::ComputeStereoMatches()
{
    mvuRight = vector<float>(mnMapPoints,-1.0f);
    mvDepth = vector<float>(mnMapPoints,-1.0f);

    const int thOrbDist = (Matcher::TH_HIGH+Matcher::TH_LOW)/2;

    const int nRows = mpORBextractorLeft->mvImagePyramid[0].rows;

    //Assign keypoints to row table
    // 步骤1：建立特征点搜索范围对应表，一个特征点在一个带状区域内搜索匹配特征点
    // 匹配搜索的时候，不仅仅是在一条横线上搜索，而是在一条横向搜索带上搜索,简而言之，原本每个特征点的纵坐标为1，这里把特征点体积放大，纵坐标占好几行
    // 例如左目图像某个特征点的纵坐标为20，那么在右侧图像上搜索时是在纵坐标为18到22这条带上搜索，搜索带宽度为正负2，搜索带的宽度和特征点所在金字塔层数有关
    // 简单来说，如果纵坐标是20，特征点在图像第20行，那么认为18 19 20 21 22行都有这个特征点
    // vRowIndices[18]、vRowIndices[19]、vRowIndices[20]、vRowIndices[21]、vRowIndices[22]都有这个特征点编号
    vector<vector<size_t> > vRowIndices(nRows,vector<size_t>());

    for(int i=0; i<nRows; i++)
        vRowIndices[i].reserve(200);

    const int Nr = mvKeysRight.size();

    for(int iR=0; iR<Nr; iR++)
    {
        // !!在这个函数中没有对双目进行校正，双目校正是在外层程序中实现的
        const cv::KeyPoint &kp = mvKeysRight[iR];
        const float &kpY = kp.pt.y;
        // 计算匹配搜索的纵向宽度，尺度越大（层数越高，距离越近），搜索范围越大
        // 如果特征点在金字塔第一层，则搜索范围为:正负2
        // 尺度越大其位置不确定性越高，所以其搜索半径越大
        const float r = 2.0f*mvScaleFactors[mvKeysRight[iR].octave];
        const int maxr = ceil(kpY+r);
        const int minr = floor(kpY-r);

        for(int yi=minr;yi<=maxr;yi++)
            vRowIndices[yi].push_back(iR);
    }

    // Set limits for search
    const float minZ = mb;        // NOTE bug mb没有初始化，mb的赋值在构造函数中放在ComputeStereoMatches函数的后面
    const float minD = 0;        // 最小视差, 设置为0即可
    const float maxD = mbf/minZ;  // 最大视差, 对应最小深度 mbf/minZ = mbf/mb = mbf/(mbf/fx) = fx (wubo???)

    // For each left keypoint search a match in the right image
    vector<pair<int, int> > vDistIdx;
    vDistIdx.reserve(mnMapPoints);

    // 步骤2：对左目相机每个特征点，通过描述子在右目带状搜索区域找到匹配点, 再通过SAD做亚像素匹配
    // 注意：这里是校正前的mvKeys，而不是校正后的mvKeysUn
    // KeyFrame::UnprojectStereo和Frame::UnprojectStereo函数中不一致
    // 这里是不是应该对校正后特征点求深度呢？(wubo???)
    for(int iL=0; iL<mnMapPoints; iL++)
    {
        const cv::KeyPoint &kpL = mvKeys[iL];
        const int &levelL = kpL.octave;
        const float &vL = kpL.pt.y;
        const float &uL = kpL.pt.x;

        // 可能的匹配点
        const vector<size_t> &vCandidates = vRowIndices[vL];

        if(vCandidates.empty())
            continue;

        const float minU = uL-maxD; // 最小匹配范围
        const float maxU = uL-minD; // 最大匹配范围

        if(maxU<0)
            continue;

        int bestDist = Matcher::TH_HIGH;
        size_t bestIdxR = 0;

        // 每个特征点描述子占一行，建立一个指针指向iL特征点对应的描述子
        const cv::Mat &dL = mDescriptors.row(iL);

        // Compare descriptor to right keypoints
        // 步骤2.1：遍历右目所有可能的匹配点，找出最佳匹配点（描述子距离最小）
        for(size_t iC=0; iC<vCandidates.size(); iC++)
        {
            const size_t iR = vCandidates[iC];
            const cv::KeyPoint &kpR = mvKeysRight[iR];

            // 仅对近邻尺度的特征点进行匹配
            if(kpR.octave<levelL-1 || kpR.octave>levelL+1)
                continue;

            const float &uR = kpR.pt.x;

            if(uR>=minU && uR<=maxU)
            {
                const cv::Mat &dR = mDescriptorsRight.row(iR);
                const int dist = Matcher::DescriptorDistance(dL,dR);

                if(dist<bestDist)
                {
                    bestDist = dist;
                    bestIdxR = iR;
                }
            }
        }
        // 最好的匹配的匹配误差存在bestDist，匹配点位置存在bestIdxR中

        // Subpixel match by correlation
        // 步骤2.2：通过SAD匹配提高像素匹配修正量bestincR
        if(bestDist<thOrbDist)
        {
            // coordinates in image pyramid at keypoint scale
            // kpL.pt.x对应金字塔最底层坐标，将最佳匹配的特征点对尺度变换到尺度对应层 (scaleduL, scaledvL) (scaleduR0, )
            const float uR0 = mvKeysRight[bestIdxR].pt.x;
            const float scaleFactor = mvInvScaleFactors[kpL.octave];
            const float scaleduL = round(kpL.pt.x*scaleFactor);
            const float scaledvL = round(kpL.pt.y*scaleFactor);
            const float scaleduR0 = round(uR0*scaleFactor);

            // sliding window search
            const int w = 5; // 滑动窗口的大小11*11 注意该窗口取自resize后的图像
            cv::Mat IL = mpORBextractorLeft->mvImagePyramid[kpL.octave].rowRange(scaledvL-w,scaledvL+w+1).colRange(scaleduL-w,scaleduL+w+1);
            IL.convertTo(IL,CV_32F);
            IL = IL - IL.at<float>(w,w) * cv::Mat::ones(IL.rows,IL.cols,CV_32F);//窗口中的每个元素减去正中心的那个元素，简单归一化，减小光照强度影响

            int bestDist = INT_MAX;
            int bestincR = 0;
            const int L = 5;
            vector<float> vDists;
            vDists.resize(2*L+1); // 11

            // 滑动窗口的滑动范围为（-L, L）,提前判断滑动窗口滑动过程中是否会越界
            const float iniu = scaleduR0+L-w; //这个地方是否应该是scaleduR0-L-w (wubo???)
            const float endu = scaleduR0+L+w+1;
            if(iniu<0 || endu >= mpORBextractorRight->mvImagePyramid[kpL.octave].cols)
                continue;

            for(int incR=-L; incR<=+L; incR++)
            {
                // 横向滑动窗口
                cv::Mat IR = mpORBextractorRight->mvImagePyramid[kpL.octave].rowRange(scaledvL-w,scaledvL+w+1).colRange(scaleduR0+incR-w,scaleduR0+incR+w+1);
                IR.convertTo(IR,CV_32F);
                IR = IR - IR.at<float>(w,w) * cv::Mat::ones(IR.rows,IR.cols,CV_32F);//窗口中的每个元素减去正中心的那个元素，简单归一化，减小光照强度影响

                float dist = cv::norm(IL,IR,cv::NORM_L1); // 一范数，计算差的绝对值
                if(dist<bestDist)
                {
                    bestDist =  dist;// SAD匹配目前最小匹配偏差
                    bestincR = incR; // SAD匹配目前最佳的修正量
                }

                vDists[L+incR] = dist; // 正常情况下，这里面的数据应该以抛物线形式变化
            }

            if(bestincR==-L || bestincR==L) // 整个滑动窗口过程中，SAD最小值不是以抛物线形式出现，SAD匹配失败，同时放弃求该特征点的深度
                continue;

            // Sub-pixel match (Parabola fitting)
            // 步骤2.3：做抛物线拟合找谷底得到亚像素匹配deltaR
            // (bestincR,dist) (bestincR-1,dist) (bestincR+1,dist)三个点拟合出抛物线
            // bestincR+deltaR就是抛物线谷底的位置，相对SAD匹配出的最小值bestincR的修正量为deltaR
            const float dist1 = vDists[L+bestincR-1];
            const float dist2 = vDists[L+bestincR];
            const float dist3 = vDists[L+bestincR+1];

            const float deltaR = (dist1-dist3)/(2.0f*(dist1+dist3-2.0f*dist2));

            // 抛物线拟合得到的修正量不能超过一个像素，否则放弃求该特征点的深度
            if(deltaR<-1 || deltaR>1)
                continue;

            // Re-scaled coordinate
            // 通过描述子匹配得到匹配点位置为scaleduR0
            // 通过SAD匹配找到修正量bestincR
            // 通过抛物线拟合找到亚像素修正量deltaR
            float bestuR = mvScaleFactors[kpL.octave]*((float)scaleduR0+(float)bestincR+deltaR);

            // 这里是disparity，根据它算出depth
            float disparity = (uL-bestuR);

            if(disparity>=minD && disparity<maxD) // 最后判断视差是否在范围内
            {
                if(disparity<=0)
                {
                    disparity=0.01;
                    bestuR = uL-0.01;
                }
                // depth 是在这里计算的
                // depth=baseline*fx/disparity
                mvDepth[iL]=mbf/disparity;   // 深度
                mvuRight[iL] = bestuR;       // 匹配对在右图的横坐标
                vDistIdx.push_back(pair<int,int>(bestDist,iL)); // 该特征点SAD匹配最小匹配偏差
            }
        }
    }

    // 步骤3：剔除SAD匹配偏差较大的匹配特征点
    // 前面SAD匹配只判断滑动窗口中是否有局部最小值，这里通过对比剔除SAD匹配偏差比较大的特征点的深度
    sort(vDistIdx.begin(),vDistIdx.end()); // 根据所有匹配对的SAD偏差进行排序, 距离由小到大
    const float median = vDistIdx[vDistIdx.size()/2].first;
    const float thDist = 1.5f*1.4f*median; // 计算自适应距离, 大于此距离的匹配对将剔除

    for(int i=vDistIdx.size()-1;i>=0;i--)
    {
        if(vDistIdx[i].first<thDist)
            break;
        else
        {
            mvuRight[vDistIdx[i].second]=-1;
            mvDepth[vDistIdx[i].second]=-1;
        }
    }
}

cv::Point3f Frame::UnprojectStereo(const int &i)
{
    // KeyFrame::UnprojectStereo
    // mvDepth是在ComputeStereoMatches函数中求取的
    // mvDepth对应的校正前的特征点，可这里却是对校正后特征点反投影
    // KeyFrame::UnprojectStereo中是对校正前的特征点mvKeys反投影
    // 在ComputeStereoMatches函数中应该对校正后的特征点求深度？？ (wubo???)
    const float z = mvDepth[i];
    if(z>0)
    {
        const float u = mvKeysUn[i].pt.x;
        const float v = mvKeysUn[i].pt.y;
        const float x = (u-mpCamera->cx)*z/mpCamera->fx;
        const float y = (v-mpCamera->cy)*z/mpCamera->fy;
        cv::Mat x3Dc = (cv::Mat_<float>(4,1) << x, y, z, 1);
		cv::Mat xInWorld = mTcw.inv() * x3Dc;
		cv::Point3f res(xInWorld.at<float>(0), xInWorld.at<float>(1), xInWorld.at<float>(2));
        return res;
    }
    else
        return cv::Point3f();
}

void Frame::UndistortKeyPoints()
{
    // 如果没有图像是矫正过的，没有失真
    mvKeysUn=mvKeys;
}

cv::Point2f Frame::GetKp2d(size_t idx)
{
    cv::Point2f res(mvKeysUn[idx].pt.x, mvKeysUn[idx].pt.y);
	return res;
}

vector<size_t> Frame::GetFeaturesInArea(const float &x, const float  &y, const float  &r, const int minLevel, const int maxLevel) const
{
    vector<size_t> vIndices;
    vIndices.reserve(mnMapPoints);

    const int nMinCellX = max(0,(int)floor((x-mnMinX-r)*mfGridElementWidthInv));
    if(nMinCellX>=FRAME_GRID_COLS)
        return vIndices;

    const int nMaxCellX = min((int)FRAME_GRID_COLS-1,(int)ceil((x-mnMinX+r)*mfGridElementWidthInv));
    if(nMaxCellX<0)
        return vIndices;

    const int nMinCellY = max(0,(int)floor((y-mnMinY-r)*mfGridElementHeightInv));
    if(nMinCellY>=FRAME_GRID_ROWS)
        return vIndices;

    const int nMaxCellY = min((int)FRAME_GRID_ROWS-1,(int)ceil((y-mnMinY+r)*mfGridElementHeightInv));
    if(nMaxCellY<0)
        return vIndices;

    const bool bCheckLevels = (minLevel>0) || (maxLevel>=0);

    for(int ix = nMinCellX; ix<=nMaxCellX; ix++)
    {
        for(int iy = nMinCellY; iy<=nMaxCellY; iy++)
        {
            const vector<size_t> vCell = mGrid[ix][iy];
            if(vCell.empty())
                continue;

            for(size_t j=0, jend=vCell.size(); j<jend; j++)
            {
                const cv::KeyPoint &kpUn = mvKeysUn[vCell[j]];
                if(bCheckLevels)
                {
                    if(kpUn.octave<minLevel)
                        continue;
                    if(maxLevel>=0)
                        if(kpUn.octave>maxLevel)
                            continue;
                }

                const float distx = kpUn.pt.x-x;
                const float disty = kpUn.pt.y-y;

                if(fabs(distx)<r && fabs(disty)<r)
                    vIndices.push_back(vCell[j]);
            }
        }
    }

    return vIndices;
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
