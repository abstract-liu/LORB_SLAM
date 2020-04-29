#include "../include/common.h"


int main(  )
{
	cv::Mat a =  (cv::Mat_<float>(3,3) << 1,2,3,4,5,6,7,8,9);
	cv::Mat b;
	b.push_back(a.row(0));
	b.push_back(a.row(2));
	b.push_back(a);

	cout << b <<endl;	
}
