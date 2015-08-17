#ifndef STEREO_CALIBRATION
#define STEREO_CALIBRATION

/*
THIS CLASS IS COMPATIBLE WITH OPENCV 3.0.0

*/

#include <opencv2\opencv.hpp>
#include <vector>
#include "NewStructTypeDefine.h"
typedef std::vector<cv::Mat> ImageList;

class StereoCalibration{
public:
	//variables
	ImageList listMat[2];//left and right
	float squareSize;
	cv::Size boardSize;

	//constructors
	StereoCalibration(float squareSize, cv::Size boardSize, const ImageList &l, const ImageList &r);
	StereoCalibration(float squareSize, cv::Size boardSize, const cv::Mat &oneImageL, const cv::Mat &oneImageR);
	~StereoCalibration();

	//functions
	void readimagePointsFromMatlab(const char* fileName);
	void compute();
	void showRectifyImage();
	void addTwoImages(const cv::Mat &oneImageL, const cv::Mat &oneImageR);
	void saveCalibResult(const char* fileName);

private:
	bool useMatlabDetectResult;
	//variables
	int numGoodImages;
	ImageList listMat_good[2];
	std::vector< std::vector<cv::Point2f> > imagePoints[2];
	std::vector< std::vector<cv::Point3f> > objectPoints;
	cv::Mat cameraMatrix[2], distCoeffs[2];
	cv::Mat R, T, E, F;
	cv::Size imageSize;

	//variables for rectifying images
	cv::Mat R1, R2, P1, P2, Q;
	cv::Rect validRoi[2];
	cv::Mat rmap[2][2];

	//important parameters are stocked in this struct
	//including calib_time, corner_dist, S_00, K_00, D_00, R_00, T_00, ROI_00, S_rect_00, R_rect_00, P_rect_00
	//and 01
	Calib_Data_Type calibData;

	//functions
	void runCalib();
	int chooseGoodImages();//return num of images
	bool checkTwoListSize() { return listMat[0].size() == listMat[1].size(); } inline//same return true, not same return false
	bool checkTwoGoodListSize() { return listMat_good[0].size() == listMat_good[1].size(); } inline//same return true, not same return false
	void initPrivateVariables();
	bool forFindingChessboardCorners(const cv::Mat &image, std::vector<cv::Point2f> &corners);
	void savedInStruct();
};

#endif