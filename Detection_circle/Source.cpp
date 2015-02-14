#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <math.h>
#include <cstdio>
#include <Windows.h>
#include "Labeling.h"


/*
 * 赤青黄の各色の閾値設定
 */
#define RED_HUE_THRESH_UPPER 6
#define RED_HUE_THRESH_LOWER 170
#define RED_SATURATION_THRESH 60
#define RED_VALUE_THRESH 80

#define BLUE_HUE_THRESH_UPPER 120
#define BLUE_HUE_THRESH_LOWER 100
#define BLUE_SATURATION_THRESH_UPPER 255
#define BLUE_SATURATION_THRESH_LOWER 245
#define BLUE_VALUE_THRESH_UPPER 255
#define BLUE_VALUE_THRESH_LOWER 0

#define YELLOW_HUE_THRESH_UPPER 35
#define YELLOW_HUE_THRESH_LOWER 20
#define YELLOW_SATURATION_THRESH_UPPER 190
#define YELLOW_SATURATION_THRESH_LOWER 40
#define YELLOW_VALUE_THRESH_UPPER 255
#define YELLOW_VALUE_THRESH_LOWER 110


/* PIXELES_THRESH以下の画素の塊をはじく */
#define PIXELES_THRESH 250

/* 画像の解像度設定 */
#define IMAGE_QUALITY 0.37



class RobotPoint{
public:
	int x, y;
};

class RobotView{
private:
	int mWidth, mHeight;
	int mCenterX, mCenterY;
	int mViewAngle;
public:
	RobotView();
	RobotView(int width, int height, int viewAngle);
	void setViewAngle(int viewAngle);
	void setArea(int width, int height);
	void convertCvPoint2RobotPoint(const cv::Point src, RobotPoint dst);
};

RobotView::RobotView(int width, int height, int viewAngle){
	mViewAngle = viewAngle;
	setArea(width, height);
}

void RobotView::setViewAngle(int viewAngle){
	mViewAngle = viewAngle;
}

void RobotView::setArea(int width, int height){
	mWidth = width;
	mHeight = height;
	mCenterX = width / 2;
	mCenterY = height / 2;
}

void RobotView::convertCvPoint2RobotPoint(const cv::Point src, RobotPoint dst){
	dst.x = src.x - mCenterX;
	dst.y = (src.y - mCenterY) * -1;
}


/*
 * マウスで移動距離を測るクラス
 */
class RobotMouse{
private:
	int mBasisPosX, mBasisPosY;
public:
	RobotMouse(const int basisPosX, const int basisPosY);
	bool getDistOfMovement(int& x, int& y);
};

RobotMouse::RobotMouse(const int basisPosX, const int basisPosY){
	mBasisPosX = basisPosX;
	mBasisPosY = basisPosY;
	SetCursorPos(mBasisPosX, mBasisPosY);
}

bool RobotMouse::getDistOfMovement(int& x, int& y){
	POINT cursorPos;
	int diffX, diffY;
	if (GetCursorPos(&cursorPos)){
		diffX = cursorPos.x - mBasisPosX;
		diffY = cursorPos.y - mBasisPosY;
		if ((diffX | diffY) != 0){
			SetCursorPos(mBasisPosX, mBasisPosY);
		}
		x = diffX;
		y = diffY;
		return true;
	}
	return false;
}


/*
 * 設定した閾値から赤青黄の各色を抽出
 * 結果をそれぞれのcv::Matに格納
 */
void threshold(const cv::Mat& src, cv::Mat& dst_red, cv::Mat& dst_blue, cv::Mat& dst_yellow){
	std::vector<cv::Mat> channel;
	cv::Mat hue1, hue2, hue, saturation1, saturation2, saturation,  value1, value2, value, hue_saturation;
	cv::split(src, channel);

	cv::threshold(channel[0], hue1, RED_HUE_THRESH_UPPER, 255, CV_THRESH_BINARY_INV);
	cv::threshold(channel[0], hue2, RED_HUE_THRESH_LOWER, 255, CV_THRESH_BINARY);
	cv::threshold(channel[1], saturation2, RED_SATURATION_THRESH, 255, CV_THRESH_BINARY);
	cv::threshold(channel[2], value2, RED_VALUE_THRESH, 255, CV_THRESH_BINARY);
	cv::bitwise_or(hue1, hue2, hue);
	cv::bitwise_and(hue, saturation2, hue_saturation);
	cv::bitwise_and(hue_saturation, value2, dst_red);

	cv::threshold(channel[0], hue1, BLUE_HUE_THRESH_UPPER, 255, CV_THRESH_BINARY_INV);
	cv::threshold(channel[0], hue2, BLUE_HUE_THRESH_LOWER, 255, CV_THRESH_BINARY);
	cv::threshold(channel[1], saturation1, BLUE_SATURATION_THRESH_UPPER, 255, CV_THRESH_BINARY_INV);
	cv::threshold(channel[1], saturation2, BLUE_SATURATION_THRESH_LOWER, 255, CV_THRESH_BINARY);
	cv::threshold(channel[2], value1, BLUE_VALUE_THRESH_UPPER, 255, CV_THRESH_BINARY_INV);
	cv::threshold(channel[2], value2, BLUE_VALUE_THRESH_LOWER, 255, CV_THRESH_BINARY);
	cv::bitwise_and(hue1, hue2, hue);
	//cv::bitwise_and(value1, value2, value);
	//cv::bitwise_and(hue, value, dst_blue);
	cv::bitwise_and(saturation1, saturation2, saturation);
	cv::bitwise_and(value1, value2, value);
	cv::bitwise_and(hue, saturation, hue_saturation);
	cv::bitwise_and(hue_saturation, value, dst_blue);

	cv::threshold(channel[0], hue1, YELLOW_HUE_THRESH_UPPER, 255, CV_THRESH_BINARY_INV);
	cv::threshold(channel[0], hue2, YELLOW_HUE_THRESH_LOWER, 255, CV_THRESH_BINARY);
	cv::threshold(channel[1], saturation1, YELLOW_SATURATION_THRESH_UPPER, 255, CV_THRESH_BINARY_INV);
	cv::threshold(channel[1], saturation2, YELLOW_SATURATION_THRESH_LOWER, 255, CV_THRESH_BINARY);
	cv::threshold(channel[2], value1, YELLOW_VALUE_THRESH_UPPER, 255, CV_THRESH_BINARY_INV);
	cv::threshold(channel[2], value2, YELLOW_VALUE_THRESH_LOWER, 255, CV_THRESH_BINARY);
	cv::bitwise_and(hue1, hue2, hue);
	cv::bitwise_and(saturation1, saturation2, saturation);
	cv::bitwise_and(value1, value2, value);
	cv::bitwise_and(hue, saturation, hue_saturation);
	cv::bitwise_and(hue_saturation, value, dst_yellow);
}

/*
 * 白黒画像の白の部分をラベリング
 * ラベリングした部分を指定した色の矩形で囲む
 */
void labeling(const cv::Mat& src, cv::Mat& resultimg, const cv::Scalar color){
	cv::Mat label_img(src.size(), CV_16SC1);
	LabelingBS label;
	char buf[32];
	label.Exec(src.data, (short *)label_img.data, src.cols, src.rows, true, PIXELES_THRESH);
	int numOfRegion = label.GetNumOfResultRegions();
	for (int i = 0; i < numOfRegion; i++){
		RegionInfoBS *regioninfo = label.GetResultRegionInfo(i);
		int x1, y1, x2, y2;
		regioninfo->GetMin(x1, y1);
		regioninfo->GetMax(x2, y2);
		cv::rectangle(resultimg, cv::Point(x1, y1), cv::Point(x2, y2), color);
		sprintf(buf, "(%4d, %4d)", x2 - x1, y2 - y1);
		cv::putText(resultimg, std::string(buf), cv::Point(x1+10, y1+10), cv::FONT_HERSHEY_SIMPLEX, 0.25, color);
	}
	std::cout << numOfRegion << std::endl;
}


/*
 * 画面の中心画素のHSV値を標準出力に出力
 */
void test(cv::Mat& img){
	cv::Mat hsvimg;
	static int radius = 4;
	static int center_x = (img.cols / 2);
	static int center_y = (img.rows / 2);
	static int centerpos = center_y * img.cols + center_x;
	cv::cvtColor(img, hsvimg, CV_BGR2HSV);
	std::vector<cv::Mat> channel;
	cv::split(hsvimg, channel);
	printf("%4d, %4d, %4d\n", 
		channel[0].data[centerpos], 
		channel[1].data[centerpos], 
		channel[2].data[centerpos]
	);
	cv::line(img, cv::Point(center_x, center_y - 30), cv::Point(center_x, center_y - 15), cv::Scalar(0, 0, 255));
	cv::line(img, cv::Point(center_x, center_y + 30), cv::Point(center_x, center_y + 15), cv::Scalar(0, 0, 255));
	cv::line(img, cv::Point(center_x - 30, center_y), cv::Point(center_x - 15, center_y), cv::Scalar(0, 0, 255));
	cv::line(img, cv::Point(center_x + 30, center_y), cv::Point(center_x + 15, center_y), cv::Scalar(0, 0, 255));
	cv::namedWindow("test", cv::WINDOW_AUTOSIZE);
	cv::imshow("test", img);
	Sleep(100);
}

/*
 * 引数srcからボールを検出する関数
 */
void detect(cv::Mat& src){
	cv::Mat img, blurimg, hsvimg, hue1, hue2,
		hue, saturation, hue_saturation, value,
		erodeimg, dilateimg,
		threshold_red, threshold_blue, threshold_yellow,
		labeling_result,
		display_img;
	static const double IMAGE_ORIGIN_QUALITY = 1.0 / IMAGE_QUALITY;

	cv::resize(src, img, img.size(), IMAGE_QUALITY, IMAGE_QUALITY);
	//cv::GaussianBlur(img, blurimg, cv::Size(3, 3), 2, 2);
	cv::cvtColor(img, hsvimg, CV_BGR2HSV);
	threshold(hsvimg, threshold_red, threshold_blue, threshold_yellow);

	/* red */
	cv::erode(threshold_red, erodeimg, cv::Mat(), cv::Point(-1, -1), 1);
	cv::dilate(erodeimg, dilateimg, cv::Mat(), cv::Point(-1, -1), 1);
	labeling(dilateimg, img, cv::Scalar(0, 0, 255));

	/* blue */
	cv::erode(threshold_blue, erodeimg, cv::Mat(), cv::Point(-1, -1), 1);
	cv::dilate(erodeimg, dilateimg, cv::Mat(), cv::Point(-1, -1), 1);
	labeling(dilateimg, img, cv::Scalar(255, 0, 0));

	/* yellow */
	cv::erode(threshold_yellow, erodeimg, cv::Mat(), cv::Point(-1, -1), 1);
	cv::dilate(erodeimg, dilateimg, cv::Mat(), cv::Point(-1, -1), 1);
	labeling(dilateimg, img, cv::Scalar(0, 255, 255));

	/* display result */
	cv::resize(img, display_img, display_img.size(), IMAGE_ORIGIN_QUALITY, IMAGE_ORIGIN_QUALITY);
	cv::namedWindow("result", cv::WINDOW_AUTOSIZE);
	imshow("result", display_img);

	/* display red, blue and yellow area */
	cv::namedWindow("red", cv::WINDOW_AUTOSIZE);
	imshow("red", threshold_red);
	cv::namedWindow("blue", cv::WINDOW_AUTOSIZE);
	imshow("blue", threshold_blue);
	cv::namedWindow("yellow", cv::WINDOW_AUTOSIZE);
	imshow("yellow", threshold_yellow);
}





int main(int argc, const char* argv[]){
	cv::Mat img, gray;
	RobotMouse mouse(500, 500);
	int x, y, integX = 0, integY = 0;
	cv::VideoCapture cap(0);
	if (!cap.isOpened()){
		return -1;
	}

	while (true){
		cap >> img;
		//test(img);
		detect(img);
		/*if (!mouse.getDistOfMovement(x, y)){
			break;
		}
		integX += x;
		integY += y;
		printf("%4d, %4d\n", integX, integY);*/
		if (cv::waitKey(30) >= 0) break;
	}
	
	return 0;
}