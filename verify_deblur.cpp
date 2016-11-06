//测试去模糊算法，参考SIGGRAPH 2009年Sunyung Cho的Fast motion deblur

#include <opencv2/opencv.hpp>
#include "opencv2/objdetect/objdetect.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/imgproc/imgproc_c.h"
#include "opencv2/imgproc/imgproc.hpp"  
#include "opencv2/nonfree/features2d.hpp"
#include <opencv2/legacy/legacy.hpp>
#include <string>
#include <array>
#include <vector>
#include <deque>
#include <set>
#include <algorithm>
#include <iostream>
#include <process.h>  
#include <windows.h>  
#include <fstream>
#include "Homography_from_trj.h"

using namespace cv;
using namespace std;

//冲击滤波
void shock(Mat src, Mat out, int iters, double dt)
{
	int rows = src.rows;
	int cols = src.cols;
	Mat src_copy;//=src.clone();
	src.convertTo(src_copy, CV_32F);
	while(iters--)
	{
		//先计算一阶及二阶导数		
		Mat I_right_shift = Mat::zeros(rows, cols, src_copy.type());
		Mat I_left_shift = Mat::zeros(rows, cols, src_copy.type());
		cout<<CV_32F<<" "<<I_left_shift.type()<<endl;
		src_copy.col(cols-1).copyTo(I_left_shift.col(cols-1));
		src_copy.colRange(1, cols).copyTo(I_left_shift.colRange(0, cols-1));
		src_copy.col(0).copyTo(I_right_shift.col(0));
		src_copy.colRange(0, cols-1).copyTo(I_right_shift.colRange(1, cols));
		Mat I_x = (I_left_shift - I_right_shift)/2.f;
		//Mat I_xx = (I_right_shift + I_left_shift - 2*src_copy)/2.f;
		
		Mat I_up_shift = Mat::zeros(rows, cols, src_copy.type());
		Mat I_down_shift = Mat::zeros(rows, cols, src_copy.type());
		src_copy.row(rows-1).copyTo(I_down_shift.row(rows-1));
		I_down_shift.rowRange(1, rows) = src_copy.rowRange(0, rows-1);
		src_copy.row(0).copyTo(I_up_shift.row(0));
		I_up_shift.rowRange(0, rows-1) = src_copy.rowRange(1, rows);
		Mat I_y = (I_up_shift - I_down_shift)/2;
		
		//再计算冲击滤波
		Mat kernel = (Mat_<float>(3, 3) << 1, 1, 1, 1, -8, 1, 1, 1, 1);
		//CvMat ker(kernel);
		Mat I_Laplace;
		filter2D(src_copy, I_Laplace, src_copy.depth(), kernel);
		Mat I_grad_value = (I_x.mul(I_x)+I_y.mul(I_y));
		float temp = 0;
		for (int i=0; i<rows; i++)
		{
			for (int j=0; j<cols; j++)
			{
				temp = ((float*)I_Laplace.data)[i*cols+j];
				((float*)src_copy.data)[i*cols+j] -= (temp>0 ? 1 : (temp<0 ? -1 : 0))*sqrt(((float*)I_grad_value.data)[i*cols+j])*dt;
			}
		}
	}
	//将滤波后图像转换回原址
	src_copy.convertTo(src, src.type());
	//ofstream out_src("orig.txt");
	//out_src<<src<<endl;
	//ofstream out_after("after_shock.txt");
	//out_after<<src_copy<<endl;
	//imshow("original", src);
	//src_copy.convertTo(src, src.type());
	//imshow("after shock", src);
	//waitKey(0);
	//cout<<"after shock"<<endl;
}

int main()
{
	//双边滤波
	Mat blurred_img = imread("blurred.png");
	Mat gray;
	cvtColor(blurred_img, gray, COLOR_BGR2GRAY);
	Mat after_bilateral;
	bilateralFilter(gray, after_bilateral, 5, 2, 0.5 );

	//imshow("original", gray);
	//imshow("after_bilateral", after_bilateral);
	//cvWaitKey(0);

	//冲击滤波
	Mat after_shock;
	shock(after_bilateral, after_shock, 10, 0.1);

	//

	return 0;
}