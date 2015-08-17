#include "StereoCalibration.h"

using namespace std;
using namespace cv;



void parse(string &cornerDataInputFileName, string &calibResultOutputFileName,
	int &boardSize1, int &boardSize2, float &squareSize)
{
	ifstream in("config.txt");
	in >> cornerDataInputFileName;
	in >> calibResultOutputFileName;
	in >> boardSize1 >> boardSize2;
	in >> squareSize;
	in.close();
}


int main(){
	int boardSize1, boardSize2;
	float squareSize;//mm
	string cornerDataFileName, calibFileName;

	//read information from config.txt
	parse(cornerDataFileName, calibFileName, boardSize1, boardSize2,
		squareSize);

	ImageList imageLists[2];
	Size boardSize = Size(boardSize1, boardSize2);
	StereoCalibration sC(squareSize, boardSize, imageLists[0], imageLists[1]);
	sC.readimagePointsFromMatlab(cornerDataFileName.c_str());
	sC.compute();
	sC.showRectifyImage();
	sC.saveCalibResult(calibFileName.c_str());
}