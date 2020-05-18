#include "../include/matcher.h"

namespace Simple_ORB_SLAM
{

const int Matcher::TH_HIGH = 100;
const int Matcher::TH_LOW = 50;
const int Matcher::HISTO_LENGTH = 30;

//naive version
//need more strategies in future

size_t Matcher::SearchByProjection(Frame* currFrame, Frame* prevFrame)
{
	std::vector<MapPoint*> prevMPs;

	cv::Mat prevDescriptors;
	cv::Mat currDescriptors = currFrame->GetDescriptors();

	//get prev map points
	for(size_t i=0; i<prevFrame->mnMapPoints; i++)
	{
		MapPoint* pMP = prevFrame->mvpMapPoints[i];
		cv::Point2f pt2f;

		//null
		if(pMP == NULL)
			continue;
		prevMPs.push_back(pMP);
		prevDescriptors.push_back(pMP->GetDescriptor());
	}
	
	//cout << "prev size " << prevMPs.size() << endl;

	//brute force match
	cv::BFMatcher matcher(NORM_HAMMING, true);
	std::vector<DMatch> matches;

	matcher.match(currDescriptors, prevDescriptors, matches);

	//reject outliers
	double minDist = DBL_MAX;
	for(size_t i=0; i<matches.size(); i++)
	{
		if(matches[i].distance < minDist)
			minDist = matches[i].distance;
	}

	size_t nPoints = 0;
	for(size_t i=0; i<matches.size(); i++)
	{
		if(matches[i].distance > max(2*minDist, 30.0))
			continue;
		currFrame->mvpMapPoints[matches[i].queryIdx] = prevMPs[matches[i].trainIdx];
		nPoints += 1;
	}
	
	//cout << "match size " << nPoints << endl;

	return nPoints;

}

size_t Matcher::SearchByProjection(Frame* CurrentFrame, Frame* LastFrame, const float th)
{
    int nmatches = 0;

    // Rotation Histogram (to check rotation consistency)
    vector<int> rotHist[HISTO_LENGTH];
    for(int i=0;i<HISTO_LENGTH;i++)
        rotHist[i].reserve(500);
    const float factor = HISTO_LENGTH/360.0f;

    const cv::Mat Rcw = CurrentFrame->mTcw.rowRange(0,3).colRange(0,3);
    const cv::Mat tcw = CurrentFrame->mTcw.rowRange(0,3).col(3);

    const cv::Mat twc = -Rcw.t()*tcw; // twc(w)

    const cv::Mat Rlw = LastFrame->mTcw.rowRange(0,3).colRange(0,3);
    const cv::Mat tlw = LastFrame->mTcw.rowRange(0,3).col(3); // tlw(l)

    // vector from LastFrame to CurrentFrame expressed in LastFrame
    const cv::Mat tlc = Rlw*twc+tlw; // Rlw*twc(w) = twc(l), twc(l) + tlw(l) = tlc(l)

    // 判断前进还是后退，并以此预测特征点在当前帧所在的金字塔层数
    const bool bForward = tlc.at<float>(2)>CurrentFrame->mb ; // 非单目情况，如果Z大于基线，则表示前进
    const bool bBackward = -tlc.at<float>(2)>CurrentFrame->mb ; // 非单目情况，如果Z小于基线，则表示前进

    for(size_t i=0; i<LastFrame->mnMapPoints; i++)
    {
        MapPoint* pMP = LastFrame->mvpMapPoints[i];

        if(pMP)
        {
            if(!LastFrame->mvbOutlier[i])
            {
                // 对上一帧有效的MapPoints进行跟踪
                // Project
				cv::Point3f pt3d = pMP->GetPos();
                cv::Mat x3Dw = (cv::Mat_<float>(3,1)<<pt3d.x, pt3d.y, pt3d.z);
                cv::Mat x3Dc = Rcw*x3Dw+tcw;

                const float xc = x3Dc.at<float>(0);
                const float yc = x3Dc.at<float>(1);
                const float invzc = 1.0/x3Dc.at<float>(2);

                if(invzc<0)
                    continue;

                float u = CurrentFrame->fx*xc*invzc+CurrentFrame->cx;
                float v = CurrentFrame->fy*yc*invzc+CurrentFrame->cy;

                if(u<CurrentFrame->mnMinX || u>CurrentFrame->mnMaxX)
                    continue;
                if(v<CurrentFrame->mnMinY || v>CurrentFrame->mnMaxY)
                    continue;

                int nLastOctave = LastFrame->mvKeys[i].octave;

                // Search in a window. Size depends on scale
                float radius = th*CurrentFrame->mvScaleFactors[nLastOctave]; // 尺度越大，搜索范围越大

                vector<size_t> vIndices2;

                // NOTE 尺度越大,图像越小
                // 以下可以这么理解，例如一个有一定面积的圆点，在某个尺度n下它是一个特征点
                // 当前进时，圆点的面积增大，在某个尺度m下它是一个特征点，由于面积增大，则需要在更高的尺度下才能检测出来
                // 因此m>=n，对应前进的情况，nCurOctave>=nLastOctave。后退的情况可以类推
                if(bForward) // 前进,则上一帧兴趣点在所在的尺度nLastOctave<=nCurOctave
                    vIndices2 = CurrentFrame->GetFeaturesInArea(u,v, radius, nLastOctave);
                else if(bBackward) // 后退,则上一帧兴趣点在所在的尺度0<=nCurOctave<=nLastOctave
                    vIndices2 = CurrentFrame->GetFeaturesInArea(u,v, radius, 0, nLastOctave);
                else // 在[nLastOctave-1, nLastOctave+1]中搜索
                    vIndices2 = CurrentFrame->GetFeaturesInArea(u,v, radius, nLastOctave-1, nLastOctave+1);

                if(vIndices2.empty())
                    continue;

                const cv::Mat dMP = pMP->GetDescriptor();

                int bestDist = 256;
                int bestIdx2 = -1;

                // 遍历满足条件的特征点
                for(vector<size_t>::const_iterator vit=vIndices2.begin(), vend=vIndices2.end(); vit!=vend; vit++)
                {
                    // 如果该特征点已经有对应的MapPoint了,则退出该次循环
                    const size_t i2 = *vit;
                    if(CurrentFrame->mvpMapPoints[i2])
                        if(CurrentFrame->mvpMapPoints[i2]->mnObs>0)
                            continue;

                    if(CurrentFrame->mvuRight[i2]>0)
                    {
                        // 双目和rgbd的情况，需要保证右图的点也在搜索半径以内
                        const float ur = u - CurrentFrame->mbf*invzc;
                        const float er = fabs(ur - CurrentFrame->mvuRight[i2]);
                        if(er>radius)
                            continue;
                    }

                    const cv::Mat &d = CurrentFrame->GetDescriptor(i2);

                    const int dist = DescriptorDistance(dMP,d);

                    if(dist<bestDist)
                    {
                        bestDist=dist;
                        bestIdx2=i2;
                    }
                }

                // 详见SearchByBoW(KeyFrame* pKF,Frame &F, vector<MapPoint*> &vpMapPointMatches)函数步骤4
                if(bestDist<=TH_HIGH)
                {
                    CurrentFrame->mvpMapPoints[bestIdx2]=pMP; // 为当前帧添加MapPoint
                    nmatches++;

                    if(true)
                    {
                        float rot = LastFrame->mvKeysUn[i].angle-CurrentFrame->mvKeysUn[bestIdx2].angle;
                        if(rot<0.0)
                            rot+=360.0f;
                        int bin = round(rot*factor);
                        if(bin==HISTO_LENGTH)
                            bin=0;
                        assert(bin>=0 && bin<HISTO_LENGTH);
                        rotHist[bin].push_back(bestIdx2);
                    }
                }
            }
        }
    }

    //Apply rotation consistency
    if(true)
    {
        int ind1=-1;
        int ind2=-1;
        int ind3=-1;

        ComputeThreeMaxima(rotHist,HISTO_LENGTH,ind1,ind2,ind3);

        for(int i=0; i<HISTO_LENGTH; i++)
        {
            if(i!=ind1 && i!=ind2 && i!=ind3)
            {
                for(size_t j=0, jend=rotHist[i].size(); j<jend; j++)
                {
                    CurrentFrame->mvpMapPoints[rotHist[i][j]]=static_cast<MapPoint*>(NULL);
                    nmatches--;
                }
            }
        }
    }

    return nmatches;
}

size_t Matcher::SearchByProjection(Frame* F, const std::set<MapPoint*> &vpMapPoints, const float th)
{
    int nmatches=0;

    const bool bFactor = th!=1.0;

    for(std::set<MapPoint*>::iterator it = vpMapPoints.begin(); it != vpMapPoints.end(); it++)
    {
        MapPoint* pMP = *it;

        // 判断该点是否要投影
        if(!pMP->mbTrackInView)
            continue;

        if(pMP->IsBad())
            continue;
            
        // step1：通过距离预测特征点所在的金字塔层数
        const int &nPredictedLevel = pMP->mnTrackScaleLevel;

        // The size of the window will depend on the viewing direction
        // step2：根据观测到该3D点的视角确定搜索窗口的大小, 若相机正对这该3D点则r取一个较小的值（mTrackViewCos>0.998?2.5:4.0）
        float r = RadiusByViewingCos(pMP->mTrackViewCos);
        
        if(bFactor)
            r*=th;

        // (pMP->mTrackProjX, pMP->mTrackProjY)：图像特征点坐标
        // r*F.mvScaleFactors[nPredictedLevel]：搜索范围
        // nPredictedLevel-1：miniLevel
        // nPredictedLevel：maxLevel
        // step3：在2D投影点附近一定范围内搜索属于miniLevel~maxLevel层的特征点 ---> vIndices
        const vector<size_t> vIndices =
                F->GetFeaturesInArea(pMP->mTrackProjX,pMP->mTrackProjY,r*F->mvScaleFactors[nPredictedLevel],nPredictedLevel-1,nPredictedLevel);

        if(vIndices.empty())
            continue;

        const cv::Mat MPdescriptor = pMP->GetDescriptor();

        int bestDist=256;
        int bestLevel= -1;
        int bestDist2=256;
        int bestLevel2 = -1;
        int bestIdx =-1 ;

        // Get best and second matches with near keypoints
        // step4：在vIndices内找到最佳匹配与次佳匹配，如果最优匹配误差小于阈值，且最优匹配明显优于次优匹配，则匹配3D点-2D特征点匹配关联成功
        for(vector<size_t>::const_iterator vit=vIndices.begin(), vend=vIndices.end(); vit!=vend; vit++)
        {
            const size_t idx = *vit;

            // 如果Frame中的该兴趣点已经有对应的MapPoint了，则退出该次循环
            if(F->mvpMapPoints[idx])
                if(F->mvpMapPoints[idx]->mnObs>0)
                    continue;

            if(F->mvuRight[idx]>0)
            {
                const float er = fabs(pMP->mTrackProjXR-F->mvuRight[idx]);
                if(er>r*F->mvScaleFactors[nPredictedLevel])
                    continue;
            }

            const cv::Mat &d = F->GetDescriptor(idx);

            const int dist = DescriptorDistance(MPdescriptor,d);
            
            // 记录最优匹配和次优匹配
            if(dist<bestDist)
            {
                bestDist2=bestDist;
                bestDist=dist;
                bestLevel2 = bestLevel;
                bestLevel = F->mvKeysUn[idx].octave;
                bestIdx=idx;
            }
            else if(dist<bestDist2)
            {
                bestLevel2 = F->mvKeysUn[idx].octave;
                bestDist2=dist;
            }
        }

        // Apply ratio to second match (only if best and second are in the same scale level)
        if(bestDist<=TH_HIGH)
        {
            if(bestLevel==bestLevel2 && bestDist>0.8*bestDist2)
                continue;

            F->mvpMapPoints[bestIdx]=pMP; // 为Frame中的兴趣点增加对应的MapPoint
            nmatches++;
        }
    }

    return nmatches;
}


size_t Matcher::SearchLocalPoints(Frame* currFrame, std::set<MapPoint*> vpMPs)
{
	std::vector<MapPoint*> prevMPs;

	cv::Mat prevDescriptors;
	cv::Mat currDescriptors = currFrame->GetDescriptors();

	//get prev map points
	for(std::set<MapPoint*>::iterator it = vpMPs.begin(); it != vpMPs.end(); it++)
	{
		MapPoint* pMP = *it;
		cv::Point2f pt2f;

		//null
		if(pMP == NULL)
			continue;

		prevMPs.push_back(pMP);
		prevDescriptors.push_back(pMP->GetDescriptor());
	}


	//brute force match
	cv::BFMatcher matcher(NORM_HAMMING, true);
	std::vector<DMatch> matches;

	matcher.match(currDescriptors, prevDescriptors, matches);

	//reject outliers
	double minDist = DBL_MAX;
	for(size_t i=0; i<matches.size(); i++)
	{
		if(matches[i].distance < minDist)
			minDist = matches[i].distance;
	}

	size_t nPoints = 0;
	for(size_t i=0; i<matches.size(); i++)
	{
		if(matches[i].distance > max(2*minDist, 30.0))
			continue;
		currFrame->mvpMapPoints[matches[i].queryIdx] = prevMPs[matches[i].trainIdx];
		nPoints += 1;
	}
	
	return nPoints;

}


int Matcher::DescriptorDistance(const cv::Mat &a, const cv::Mat &b)
{
    const int *pa = a.ptr<int32_t>();
    const int *pb = b.ptr<int32_t>();

    int dist=0;

    for(int i=0; i<8; i++, pa++, pb++)
    {
        unsigned  int v = *pa ^ *pb;
        v = v - ((v >> 1) & 0x55555555);
        v = (v & 0x33333333) + ((v >> 2) & 0x33333333);
        dist += (((v + (v >> 4)) & 0xF0F0F0F) * 0x1010101) >> 24;
    }

    return dist;
}

void Matcher::ComputeThreeMaxima(vector<int>* histo, const int L, int &ind1, int &ind2, int &ind3)
{
    int max1=0;
    int max2=0;
    int max3=0;

    for(int i=0; i<L; i++)
    {
        const int s = histo[i].size();
        if(s>max1)
        {
            max3=max2;
            max2=max1;
            max1=s;
            ind3=ind2;
            ind2=ind1;
            ind1=i;
        }
        else if(s>max2)
        {
            max3=max2;
            max2=s;
            ind3=ind2;
            ind2=i;
        }
        else if(s>max3)
        {
            max3=s;
            ind3=i;
        }
    }

    if(max2<0.1f*(float)max1)
    {
        ind2=-1;
        ind3=-1;
    }
    else if(max3<0.1f*(float)max1)
    {
        ind3=-1;
    }
}

float Matcher::RadiusByViewingCos(const float &viewCos)
{
    if(viewCos>0.998)
        return 2.5;
    else
        return 4.0;
}

}
