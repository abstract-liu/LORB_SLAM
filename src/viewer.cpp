#include "../include/viewer.h"
#include <opencv2/highgui.hpp>
#include <pangolin/display/display.h>
#include <vector>

namespace Simple_ORB_SLAM
{

Viewer::Viewer(Camera* camera, Map* map, VisualOdometry* vo)
{
	mpCamera = camera;	
	mpMap = map;
	mpVO = vo;

	mKeyFrameSize = camera->mKeyFrameSize;
	mKeyFrameLineWidth = camera->mKeyFrameLineWidth;
	mCameraSize = camera->mCameraSize;
	mCameraLineWidth = camera->mCameraLineWidth;
	mPointSize = camera->mPointSize;

}

void Viewer::Run()
{
	pangolin::CreateWindowAndBind("Simple-ORB-SLAM", 1024, 768);
	
	glEnable(GL_DEPTH_TEST);
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	
	//set menu
	pangolin::CreatePanel("menu").SetBounds(0.0,1.0,0.0,pangolin::Attach::Pix(175));
    pangolin::Var<bool> menuFollowCamera("menu.Follow Camera",true,true);
    pangolin::Var<bool> menuShowPoints("menu.Show Points",true,true);
    pangolin::Var<bool> menuShowKeyFrames("menu.Show KeyFrames",true,true);

	//set camera
	pangolin::OpenGlRenderState s_cam(pangolin::ProjectionMatrix(1024,768, mpCamera->mViewpointF,mpCamera->mViewpointF,512,389,0.1,1000),pangolin::ModelViewLookAt(mpCamera->mViewpointX,mpCamera->mViewpointY,mpCamera->mViewpointZ, 0,0,0,0.0,-1.0, 0.0) );
	
	//set view
	pangolin::View& d_cam = pangolin::CreateDisplay()
            .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1024.0f/768.0f)
            .SetHandler(new pangolin::Handler3D(s_cam));
	
	//set transform mat
    pangolin::OpenGlMatrix Twc;
    Twc.SetIdentity();

	//set opencv window
    cv::namedWindow("Simple_ORB_SLAM: Current Frame");
	



	while(!pangolin::ShouldQuit())
	{
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
		d_cam.Activate(s_cam);

		//set background
		glClearColor(1.0f, 1.0f, 1.0f, 1.0f);

		DrawCurrCamera();
			
		if(menuShowPoints)
			DrawMapPoints();

		if(menuShowKeyFrames)
			DrawKeyFrames();

		pangolin::FinishFrame();	
	}



}

// opengl learning!!!
void Viewer::DrawKeyFrames()
{
	std::vector<Frame*> keyFrames = mpMap->GetKeyFrames();
	
	const float w = mKeyFrameSize;
    const float h = w*0.75;
    const float z = w*0.6;

	for(size_t i=0; i<keyFrames.size(); i++)
	{
		glPushMatrix();
		
		cv::Mat Twc = keyFrames[i]->GetInvPose().t();
		glMultMatrixf(Twc.ptr<GLfloat>(0));
	
		glLineWidth(mKeyFrameLineWidth);
            //设置当前颜色为蓝色(关键帧图标显示为蓝色)
        glColor3f(0.0f,0.0f,1.0f);
            //用线将下面的顶点两两相连
        glBegin(GL_LINES);
        glVertex3f(0,0,0);
        glVertex3f(w,h,z);
        glVertex3f(0,0,0);
        glVertex3f(w,-h,z);
        glVertex3f(0,0,0);
        glVertex3f(-w,-h,z);
        glVertex3f(0,0,0);
        glVertex3f(-w,h,z);

        glVertex3f(w,h,z);
        glVertex3f(w,-h,z);

        glVertex3f(-w,h,z);
        glVertex3f(-w,-h,z);

        glVertex3f(-w,h,z);
        glVertex3f(w,h,z);

        glVertex3f(-w,-h,z);
        glVertex3f(w,-h,z);
        glEnd();
		

		glPopMatrix();
	}



}


void Viewer::DrawCurrCamera()
{
	const float w = mCameraSize;
    const float h = w*0.75;
    const float z = w*0.6;

	glPushMatrix();
	
	//warning null !!!!!!!!!!!!!!!! 
	cv::Mat Twc = mpVO->GetCurrInvPos().t();
	glMultMatrixf(Twc.ptr<GLfloat>(0));
	
	glLineWidth(mCameraLineWidth);
            //设置当前颜色为蓝色(关键帧图标显示为蓝色)
    glColor3f(0.0f,1.0f,0.0f);
            //用线将下面的顶点两两相连
    glBegin(GL_LINES);
    glVertex3f(0,0,0);
    glVertex3f(w,h,z);
    glVertex3f(0,0,0);
    glVertex3f(w,-h,z);
    glVertex3f(0,0,0);
    glVertex3f(-w,-h,z);
    glVertex3f(0,0,0);
    glVertex3f(-w,h,z);

    glVertex3f(w,h,z);
    glVertex3f(w,-h,z);

    glVertex3f(-w,h,z);
    glVertex3f(-w,-h,z);

    glVertex3f(-w,h,z);
    glVertex3f(w,h,z);

    glVertex3f(-w,-h,z);
    glVertex3f(w,-h,z);
    glEnd();
		

	glPopMatrix();
	


}


void Viewer::DrawMapPoints()
{
	std::vector<MapPoint*> globalMapPoints = mpMap->GetPoints();
	std::vector<MapPoint*> localMapPoints = mpVO->mpLocalMap->GetPoints();

	//draw global points
	glPointSize(mPointSize);
    glBegin(GL_POINTS);
    glColor3f(0.0,0.0,0.0);

	for(size_t i=0; i<globalMapPoints.size(); i++)
	{
		cv::Point3f mapPoint = globalMapPoints[i]->GetPose();
		glVertex3f(mapPoint.x, mapPoint.y, mapPoint.z);
	}
	glEnd();

	//draw local points
	glPointSize(mPointSize);
    glBegin(GL_POINTS);
    glColor3f(1.0,0.0,0.0);

	for(size_t i=0; i<localMapPoints.size(); i++)
	{
		cv::Point3f mapPoint = localMapPoints[i]->GetPose();
		glVertex3f(mapPoint.x,mapPoint.y, mapPoint.z);
	}
	glEnd();


}


}
