#ifndef BUNDLE_ADJUST_H
#define BUNDLE_ADJUST_H

#include "common.h"
#include "frame.h"
#include "camera.h"
#include <opencv2/core/mat.hpp>

namespace Simple_ORB_SLAM
{ 

class BA
{
public:
	BA();
	
	void static ProjectPoseOptimization(Frame* curr);
	
	void static LocalPoseOptimization(Frame* pCurrFrame);

};


}





#endif 
