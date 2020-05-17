#include "../include/common.h"
#include "../include/frame.h"
#include "../include/camera.h"
#include "../include/visual_odometry.h"
#include "../include/viewer.h"
#include <Eigen/src/Core/Matrix.h>
#include <opencv2/core/types.hpp>
#include <opencv2/highgui.hpp>
#include <vector>
#include <thread>

void LoadImg(const string& strParaFile, vector<string>& img, vector<string>& timeStamp, const char& camera);


int main(int argc, char* argv[])
{
	//read parameter and image
	if(argc != 2)
	{
		cout << "wrong parameter" << endl;
		return 1;
	}

	std::vector<string> strImgLeft, strImgRight, timeStampLeft, timeStampRight;
	LoadImg(argv[1], strImgLeft, timeStampLeft, 'L');
	LoadImg(argv[1], strImgRight, timeStampRight, 'R');
	size_t frameNum = strImgLeft.size();
	cout << "----load " << frameNum << " frames" <<endl;

	Simple_ORB_SLAM::Camera euroc(argv[1]);
	Simple_ORB_SLAM::Map map;
	Simple_ORB_SLAM::LocalMapping localMapper(&map);
	Simple_ORB_SLAM::VisualOdometry vo(&localMapper, &euroc, &map);
	Simple_ORB_SLAM::Viewer viewer(&euroc, &map, &vo);
	
	std::thread* tViewer = new std::thread(&Simple_ORB_SLAM::Viewer::Run, &viewer);
	std::thread* tMapper = new std::thread(&Simple_ORB_SLAM::LocalMapping::Run, &localMapper);	
	for(size_t i=0; i<frameNum; i++)
	{
		cv::Mat imgLeft, imgRight;
		cv::Mat imgLeftRec, imgRightRec;

		imgLeft = cv::imread(strImgLeft[i]);
		imgRight = cv::imread(strImgRight[i]);
		
		if(imgLeft.empty() || imgRight.empty())
		{
			cout << "----empty image" << endl;
			return 1;
		}

		euroc.RectifyL(imgLeft, imgLeftRec);
		euroc.RectifyR(imgRight, imgRightRec);
		
		
		vo.Run(timeStampLeft[i], imgLeftRec, imgRightRec);
		
	}
	
	

	return 0;
}




void LoadImg(const string& strParaFile, vector<string>& img, vector<string>& timeStamp, const char& camera)
{
	string img_name = strParaFile;
	if(camera == 'L')
	{
		img_name += "cam0/data.csv";
	}
	else
	{
		img_name += "cam1/data.csv";
	}
	
	string oneline,part_of_str;
	vector<string> line_of_str;
	bool is_fist_row = true;

	ifstream img_input(img_name.c_str());
	while(getline(img_input, oneline))
	{
		istringstream read_str(oneline);
		if(is_fist_row == true )
		{
			is_fist_row = false;
			continue;
		}

		while(getline(read_str, part_of_str, ','))
		{	
			part_of_str.erase(part_of_str.end()-1);
			line_of_str.push_back(part_of_str);
		}	
	}

	for(size_t i=0; i<line_of_str.size(); i+=2 )
	{
		if(camera == 'L')
			img.push_back(strParaFile + "cam0/data/" +line_of_str[i+1]);
		else
			img.push_back(strParaFile + "cam1/data/" + line_of_str[i+1]);

		timeStamp.push_back(line_of_str[i]);
	}
}






