#include "StereoCalibration.h"
#include <iostream>
#include <ctime>
using namespace std;
using namespace cv;


StereoCalibration::StereoCalibration(float _squareSize, Size _boardSize, const ImageList &l, const ImageList &r)
{
	listMat[0] = l;
	listMat[1] = r;
	squareSize = _squareSize;
	boardSize = _boardSize;
	initPrivateVariables();
}

StereoCalibration::StereoCalibration(float _squareSize, Size _boardSize, const cv::Mat &oneImageL, const cv::Mat &oneImageR)
{
	listMat[0].push_back(oneImageL);
	listMat[1].push_back(oneImageR);
	squareSize = _squareSize;
	boardSize = _boardSize;
	initPrivateVariables();
}

StereoCalibration::~StereoCalibration(){

}

void StereoCalibration::addTwoImages(const cv::Mat &oneImageL, const cv::Mat &oneImageR)
{
	listMat[0].push_back(oneImageL);
	listMat[1].push_back(oneImageR);
}

void StereoCalibration::initPrivateVariables(){
	listMat_good[0].clear();
	listMat_good[1].clear();
	useMatlabDetectResult = false;
}

void StereoCalibration::readimagePointsFromMatlab(const char* fileName){
	ifstream in(fileName);
	if (!in.is_open())
	{
		cout << "open file error :  matlab corner points data from " << fileName << endl;
		return;
	}
	in >> numGoodImages;
	int numOfCornerPointsPerImage = 0;
	in >> numOfCornerPointsPerImage;
	if (boardSize.height * boardSize.width != numOfCornerPointsPerImage)
	{
		cout << "something wrong happens in readimagePointsFromMatlab of file " << fileName << endl;
		return;
	}
	imagePoints[0].resize(numGoodImages);
	imagePoints[1].resize(numGoodImages);

	for (int indexImage = 0; indexImage < numGoodImages; indexImage++)
	{
		string imageFile;
		//first image
		in >> imageFile;
		Mat img = imread(imageFile, 0);
		listMat_good[0].push_back(img);

		//seconde image
		in >> imageFile;
		img = imread(imageFile, 0);
		listMat_good[1].push_back(img);

		//first set of corner points
		for (int indexPoint = 0; indexPoint < numOfCornerPointsPerImage; indexPoint++)
		{
			float x, y;
			in >> x >> y;
			imagePoints[0][indexImage].push_back(Point2f(x, y));
		}

		cornerSubPix(listMat_good[0][indexImage],imagePoints[0][indexImage], Size(3, 3), Size(-1, -1),
			TermCriteria(TermCriteria::EPS,
			30, 1e-5));
		
		//seconde set of corner points
		for (int indexPoint = 0; indexPoint < numOfCornerPointsPerImage; indexPoint++)
		{
			float x, y;
			in >> x >> y;
			imagePoints[1][indexImage].push_back(Point2f(x, y));
		}

		cornerSubPix(listMat_good[1][indexImage], imagePoints[1][indexImage], Size(3, 3), Size(-1, -1),
			TermCriteria(TermCriteria::EPS,
			30, 1e-5));
		
		Mat cimg;
		cvtColor(listMat_good[0][indexImage], cimg, CV_GRAY2BGR);
		drawChessboardCorners(cimg, boardSize, imagePoints[0][indexImage], true);
		imshow("corners_left", cimg);

		cvtColor(listMat_good[1][indexImage], cimg, CV_GRAY2BGR);
		drawChessboardCorners(cimg, boardSize, imagePoints[1][indexImage], true);
		imshow("corners_right", cimg);
		waitKey(0);
	}

	useMatlabDetectResult = true;
}

void StereoCalibration::compute(){
	if (!useMatlabDetectResult)
	{
		if (!checkTwoListSize())
		{
			cout << "Error: the image list different number of elements\n";
			return;
		}
		//use opencv function to detect corner points.
		//choose images with nice chessboards.
		numGoodImages = chooseGoodImages();
		if (numGoodImages < 2)
		{
			cout << "Error: too little pairs to run the calibration\n";
			return;
		}
	}
	else
	{
		//read corner points from matlab. We have alread done in @func readimagePointsFromMatlab.
	}

	if (!checkTwoGoodListSize())
	{
		cout << "Error: the good image list contains different number of elements\n";
		return;
	}

	runCalib();
}

void StereoCalibration::runCalib(){

	//generate chessboard corner points coordinates for each good image
 	objectPoints.resize(numGoodImages);
	for (int nImage = 0; nImage < numGoodImages; nImage++)
	{
		for (int j = 0; j < boardSize.height; j++)
			for (int i = 0; i < boardSize.width; i++)
				objectPoints[nImage].push_back(Point3f(i*squareSize, j*squareSize, 0));
	}

	cout << "Good pairs : " << numGoodImages << endl;
	cout << "Running stereo calibration ...\n";

	cameraMatrix[0] = Mat::eye(3, 3, CV_64F);
	cameraMatrix[1] = Mat::eye(3, 3, CV_64F);
	imageSize = listMat_good[0][0].size();

	//Mono camera calibration at first
	vector<Mat> rvecs, tvecs;
	cout << calibrateCamera(objectPoints, imagePoints[0], imageSize, cameraMatrix[0], distCoeffs[0], rvecs, tvecs,
		CV_CALIB_FIX_K3) << endl;
	cout << calibrateCamera(objectPoints, imagePoints[1], imageSize, cameraMatrix[1], distCoeffs[1], rvecs, tvecs,
		CV_CALIB_FIX_K3) << endl;

	for (int nImage = 0; nImage < numGoodImages; nImage++)
	{
		Mat img;
		undistort(listMat_good[0][nImage], img, cameraMatrix[0], distCoeffs[0]);
		imshow("img_undistort_left", img);
		undistort(listMat_good[1][nImage], img, cameraMatrix[1], distCoeffs[1]);
		imshow("img_undistort_right", img);
		waitKey(10);
	}

	//stereo camera calibration
	double rms = stereoCalibrate(objectPoints, imagePoints[0], imagePoints[1],
		cameraMatrix[0], distCoeffs[0],
		cameraMatrix[1], distCoeffs[1],
		imageSize, R, T, E, F,
		//CALIB_USE_INTRINSIC_GUESS + CV_CALIB_FIX_K1);
		CV_CALIB_FIX_INTRINSIC);
	cout << "done with RMS error=" << rms << endl;
	

	stereoRectify(cameraMatrix[0], distCoeffs[0],
		cameraMatrix[1], distCoeffs[1],
		imageSize, R, T, R1, R2, P1, P2, Q,
		CV_CALIB_ZERO_DISPARITY, -1, imageSize, &validRoi[0], &validRoi[1]);

	savedInStruct();
}

void StereoCalibration::showRectifyImage(){
	//imageSize = listMat[0][0].size();

	//Precompute maps for cv::remap()
	initUndistortRectifyMap(cameraMatrix[0], distCoeffs[0], R1, P1, imageSize, CV_32FC1, rmap[0][0], rmap[0][1]);
	initUndistortRectifyMap(cameraMatrix[1], distCoeffs[1], R2, P2, imageSize, CV_32FC1, rmap[1][0], rmap[1][1]);
	
	//
	Mat canvas;
	double sf = 600. / MAX(imageSize.width, imageSize.height);
	int w = cvRound(imageSize.width*sf);
	int h = cvRound(imageSize.height*sf);
	canvas.create(h, w * 2, CV_8UC3);

	for (int nImage = 0; nImage < numGoodImages; nImage++)
	{
		for (int k = 0; k < 2; k++)
		{
			Mat img = listMat_good[k][nImage];
			Mat rimg, cimg;
			remap(img, rimg, rmap[k][0], rmap[k][1], INTER_LINEAR);
			cvtColor(rimg, cimg, COLOR_GRAY2BGR);
			Mat canvasPart = canvas(Rect(w*k, 0, w, h));
			resize(cimg, canvasPart, canvasPart.size(), 0, 0, INTER_AREA);

			Rect vroi(cvRound(validRoi[k].x*sf), cvRound(validRoi[k].y*sf),
				cvRound(validRoi[k].width*sf), cvRound(validRoi[k].height*sf));
			rectangle(canvasPart, vroi, Scalar(0, 0, 255), 3, 8);
			
		}
		for (int j = 0; j < canvas.rows; j += 16)
			line(canvas, Point(0, j), Point(canvas.cols, j), Scalar(0, 255, 0), 1, 8);
		
		imshow("rectified", canvas);
		char c = (char)waitKey(10);
		if (c == 27 || c == 'q' || c == 'Q')
			break;
	}
}

int StereoCalibration::chooseGoodImages(){
	int numImages = listMat[0].size();
	int numGoodImage = 0;
	imagePoints[0].resize(numImages);
	imagePoints[1].resize(numImages);
	for (int nImage = 0; nImage < numImages; nImage+=1)
	{
		int k = 0;
		bool found = false;
		for (k = 0; k < 2; k++)
		{
			Mat img = listMat[k][nImage];
			if (img.empty())
				break;

			vector<Point2f>& corners = imagePoints[k][numGoodImage];
			found = forFindingChessboardCorners(img, corners);
			if (!found) break;

			//Mat cimg;
			//cvtColor(img, cimg, CV_GRAY2BGR);
			//drawChessboardCorners(cimg, boardSize, corners, found); 
			//imshow("corners", cimg);
			//waitKey(0);
		}
		if (k == 2)
		{
			listMat_good[0].push_back(listMat[0][nImage]);
			listMat_good[1].push_back(listMat[1][nImage]);
			numGoodImage++;
			cout << "chosed image: " << nImage << endl;
		}

		
	}

	imagePoints[0].resize(numGoodImage);
	imagePoints[1].resize(numGoodImage);
	return numGoodImage;
}

bool StereoCalibration::forFindingChessboardCorners(const Mat &image, vector<Point2f> &corners){
	bool found = false;

	Mat copy_img, img; 
	image.copyTo(copy_img);
	//threshold(copy_img, copy_img, 128, 255, THRESH_TOZERO);

	found = findChessboardCorners(copy_img, boardSize, corners,
		CALIB_CB_ADAPTIVE_THRESH + CALIB_CB_NORMALIZE_IMAGE);
	if (!found) return found;

	cornerSubPix(image, corners, Size(3, 3), Size(-1, -1),
		TermCriteria(TermCriteria::EPS,
		30, 1e-5));

	return true;
}

void StereoCalibration::savedInStruct(){
	time_t t = time(0);
	char tmp[64];
	strftime(tmp, sizeof(tmp), "calib_time: %d-%b-%Y %X", localtime(&t));
	calibData.calib_time = tmp;

	calibData.corner_dist = squareSize;

	calibData.S_00[0] = listMat_good[0][0].size().width, calibData.S_00[1] = listMat_good[0][0].size().height;
	calibData.S_01[0] = listMat_good[0][0].size().width, calibData.S_01[1] = listMat_good[0][0].size().height;

	memcpy(calibData.K_00, (double*)cameraMatrix[0].data, sizeof(calibData.K_00));
	memcpy(calibData.K_01, (double*)cameraMatrix[1].data, sizeof(calibData.K_01));

	memcpy(calibData.D_00, (double*)distCoeffs[0].data, sizeof(calibData.D_00));
	memcpy(calibData.D_01, (double*)distCoeffs[1].data, sizeof(calibData.D_01));

	calibData.ROI_00[0] = validRoi[0].x, calibData.ROI_00[1] = validRoi[0].y;
	calibData.ROI_00[2] = validRoi[0].width, calibData.ROI_00[3] = validRoi[0].height;

	calibData.ROI_01[0] = validRoi[1].x, calibData.ROI_01[1] = validRoi[1].y;
	calibData.ROI_01[2] = validRoi[1].width, calibData.ROI_01[3] = validRoi[1].height;
	
	calibData.S_rect_00[0] = validRoi[0].width, calibData.S_rect_00[1] = validRoi[0].height;
	//S_rect_01 is the same as S_rect_00
	//changed to not same 17/08/2015
	calibData.S_rect_01[0] = validRoi[1].width, calibData.S_rect_01[1] = validRoi[1].height;

	memcpy(calibData.T_01, (double*)T.data, sizeof(calibData.T_01));

	memcpy(calibData.R_01, (double*)R.data, sizeof(calibData.R_01));

	memcpy(calibData.R_rect_00, (double*)R1.data, sizeof(calibData.R_rect_00));
	memcpy(calibData.P_rect_00, (double*)P1.data, sizeof(calibData.P_rect_00));

	memcpy(calibData.R_rect_01, (double*)R2.data, sizeof(calibData.R_rect_01));
	memcpy(calibData.P_rect_01, (double*)P2.data, sizeof(calibData.P_rect_01));
}


void StereoCalibration::saveCalibResult(const char* fileName){
	ofstream out(fileName);
	out << calibData;
	out.close();
	cout << fileName << " saved successfully!" << endl;
}

//read in
std::ifstream& operator >> (std::ifstream &in, Calib_Data_Type &calib_data)
{
	char firstLine[SIZE_TIME];
	in.getline(firstLine, SIZE_TIME);
	calib_data.calib_time = firstLine;
	//in.getline(calib_data.corner_dist, SIZE_CORNER_DIST);

	while (!in.eof())
	{
		std::string prefix;
		in >> prefix;

		if (prefix == "corner_dist:")
		{
			for (int i = 0; i < 1; i++)
				in >> calib_data.corner_dist;
		}
		if (prefix == "S_00:")
		{
			for (int i = 0; i < SIZE_S; i++)
				in >> calib_data.S_00[i];
		}
		if (prefix == "K_00:")
		{
			for (int i = 0; i < SIZE_K; i++)
				in >> calib_data.K_00[i];
		}
		if (prefix == "D_00:")
		{
			for (int i = 0; i < SIZE_D; i++)
				in >> calib_data.D_00[i];
		}
		if (prefix == "S_01:")
		{
			for (int i = 0; i < SIZE_S; i++)
				in >> calib_data.S_01[i];
		}
		if (prefix == "K_01:")
		{
			for (int i = 0; i < SIZE_K; i++)
				in >> calib_data.K_01[i];
		}
		if (prefix == "D_01:")
		{
			for (int i = 0; i < SIZE_D; i++)
				in >> calib_data.D_01[i];
		}
		if (prefix == "R_01:")
		{
			for (int i = 0; i < SIZE_R; i++)
				in >> calib_data.R_01[i];
		}
		if (prefix == "T_01:")
		{
			for (int i = 0; i < SIZE_T; i++)
				in >> calib_data.T_01[i];
		}

		if (prefix == "ROI_00:")
		{
			for (int i = 0; i < SIZE_ROI; i++)
				in >> calib_data.ROI_00[i];
		}
		if (prefix == "S_rect_00:")
		{
			for (int i = 0; i < SIZE_S; i++)
				in >> calib_data.S_rect_00[i];
		}

		if (prefix == "R_rect_00:")
		{
			for (int i = 0; i < SIZE_R; i++)
				in >> calib_data.R_rect_00[i];
		}

		if (prefix == "P_rect_00:")
		{
			for (int i = 0; i < SIZE_P; i++)
				in >> calib_data.P_rect_00[i];
		}

		if (prefix == "ROI_01:")
		{
			for (int i = 0; i < SIZE_ROI; i++)
				in >> calib_data.ROI_01[i];
		}

		if (prefix == "S_rect_01:")
		{
			for (int i = 0; i < SIZE_S; i++)
				in >> calib_data.S_rect_01[i];
		}

		if (prefix == "R_rect_01:")
		{
			for (int i = 0; i < SIZE_R; i++)
				in >> calib_data.R_rect_01[i];
		}

		if (prefix == "P_rect_01:")
		{
			for (int i = 0; i < SIZE_P; i++)
				in >> calib_data.P_rect_01[i];
		}
	}

	return in;
}

//write out
std::ofstream& operator << (std::ofstream &of, const Calib_Data_Type &calib_data)
{
	of << calib_data.calib_time << "\n";

	of << "corner_dist: ";
	for (int i = 0; i < 1; i++)
		of << calib_data.corner_dist << "\n";

	of << "S_00: ";
	for (int i = 0; i < SIZE_S; i++)
		of << calib_data.S_00[i] << " ";
	of << "\n";

	of << "K_00: ";
	for (int i = 0; i < SIZE_K; i++)
		of << calib_data.K_00[i] << " ";
	of << "\n";

	of << "D_00: ";
	for (int i = 0; i < SIZE_D; i++)
		of << calib_data.D_00[i] << " ";
	of << "\n";

	of << "ROI_00: ";
	for (int i = 0; i < SIZE_ROI; i++)
		of << calib_data.ROI_00[i] << " ";
	of << "\n";

	of << "S_rect_00: ";
	for (int i = 0; i < SIZE_S; i++)
		of << calib_data.S_rect_00[i] << " ";
	of << "\n";

	of << "R_rect_00: ";
	for (int i = 0; i < SIZE_R; i++)
		of << calib_data.R_rect_00[i] << " ";
	of << "\n";

	of << "P_rect_00: ";
	for (int i = 0, n = 0; i < SIZE_P; i++)
	{
		of << calib_data.P_rect_00[i] << " ";
	}
	of << "\n";

	of << "S_01: ";
	for (int i = 0; i < SIZE_S; i++)
		of << calib_data.S_01[i] << " ";
	of << "\n";

	of << "K_01: ";
	for (int i = 0; i < SIZE_K; i++)
		of << calib_data.K_01[i] << " ";
	of << "\n";

	of << "D_01: ";
	for (int i = 0; i < SIZE_D; i++)
		of << calib_data.D_01[i] << " ";
	of << "\n";

	of << "R_01: ";
	for (int i = 0; i < SIZE_R; i++)
		of << calib_data.R_01[i] << " ";
	of << "\n";

	of << "T_01: ";
	for (int i = 0; i < SIZE_T; i++)
		of << calib_data.T_01[i] << " ";
	of << "\n";

	of << "ROI_01: ";
	for (int i = 0; i < SIZE_ROI; i++)
		of << calib_data.ROI_01[i] << " ";
	of << "\n";

	of << "S_rect_01: ";
	for (int i = 0; i < SIZE_S; i++)
		of << calib_data.S_rect_01[i] << " ";
	of << "\n";

	of << "R_rect_01: ";
	for (int i = 0; i < SIZE_R; i++)
		of << calib_data.R_rect_01[i] << " ";
	of << "\n";

	of << "P_rect_01: ";
	for (int i = 0, n = 0; i < SIZE_P; i++)
	{
		of << calib_data.P_rect_01[i] << " ";
	}
	of << "\n";

	return of;
}
