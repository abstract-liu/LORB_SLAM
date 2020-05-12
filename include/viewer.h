#ifndef VIEWER_H
#define VIEWER_H

#include "common.h"
#include "camera.h"
#include "map.h"
#include "visual_odometry.h"
#include <pangolin/pangolin.h>

namespace Simple_ORB_SLAM
{ 

class Viewer
{
public:
	Viewer(Camera* camera, Map* map, VisualOdometry* vo);
	void Run();

protected:
	void DrawKeyFrames();
	void DrawCurrCamera();
	void DrawMapPoints();


private:
	Camera* mpCamera;
	Map* mpMap;
	VisualOdometry* mpVO;

	//draw settings
	float mKeyFrameSize;
	float mKeyFrameLineWidth;
	float mCameraSize;
	float mCameraLineWidth;
	float mPointSize;
};

}
#endif
