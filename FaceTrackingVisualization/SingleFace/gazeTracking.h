#ifndef GAZE_TRACKING_H
#define GAZE_TRACKING_H

#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <string>
//#define ROUND(x) ((int)(x+0.5))

class GazeTracking{
public:
	GazeTracking();
	~GazeTracking();

	bool initialize(cv::String xmlFile);

	void process(cv::Mat& frame);
	void process(IplImage* image);

	cv::Point getLeftPupil();
	cv::Point getRightPupil();
	cv::Point getLeftPupilInFaceRect();
	cv::Point getRightPupilInFaceRect();

	void getLeftPupilXY(int& x, int& y);
	void getRightPupilXY(int& x, int& y);
	void getLeftPupilXYInFaceRect(int& x, int& y);
	void getRightPupilXYInFaceRect(int& x, int& y);

	cv::Mat& getFace(){return faceROI;}
	cv::Rect& GetFaceRect(){return faceRect;}
	bool isFindFace(){return findFace;}

	void setLeftPupilXY(int x, int y);
	void setRightPupilXY(int x, int y);

private:
	void findPupils(cv::Mat& frameGray, cv::Rect& face);

	cv::Point findEyeCenter(cv::Mat face, cv::Rect eye, std::string debugWindow);

	cv::Point unscalePoint(cv::Point p, cv::Rect origSize);
	// returns a mask
	cv::Mat floodKillEdges(cv::Mat &mat);

	bool floodShouldPushPoint(const cv::Point &np, const cv::Mat &mat);

	bool inMat(cv::Point p,int rows,int cols);

	void testPossibleCentersFormula(int x, int y, unsigned char weight,double gx, double gy, cv::Mat &out);

	double computeDynamicThreshold(const cv::Mat &mat, double stdDevFactor);

	cv::Mat matrixMagnitude(const cv::Mat &matX, const cv::Mat &matY);

	void scaleToFastSize(const cv::Mat &src,cv::Mat &dst);

	cv::Mat computeMatXGradient(const cv::Mat &mat);

	int round(double x);

	cv::CascadeClassifier faceCascade;
	cv::Mat faceROI;
	cv::Point leftPupil;
	cv::Point rightPupil;

	cv::Rect faceRect;

	bool findFace;

	// Size constants
	const int kEyePercentTop;
	const int kEyePercentSide;
	const int kEyePercentHeight;
	const int kEyePercentWidth; 

	//Preprocessing
	const bool kSmoothFaceImage;
	const float kSmoothFaceFactor;

	// Algorithm Parameters
	const int kFastEyeWidth;
	const int kWeightBlurSize;
	const float kWeightDivisor;
	const double kGradientThreshold;

	//Postprocessing
	const bool kEnablePostProcess;
	const float kPostProcessThreshold;

	// Eye Corner
	const bool kEnableEyeCorner;
};

#endif