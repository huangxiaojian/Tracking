#include "stdafx.h"
#include "gazeTracking.h"
#include <vector>
#include <queue>

GazeTracking::GazeTracking():kEyePercentTop(25),kEyePercentSide(13),
	kEyePercentHeight(30),kEyePercentWidth(35), 
	kFastEyeWidth(50), kWeightBlurSize(5),
	kWeightDivisor(150.0), kGradientThreshold(50.0),
	kPostProcessThreshold(0.97), kSmoothFaceImage(false),
	kSmoothFaceFactor(0.005),kEnableEyeCorner(false),
	kEnablePostProcess(true)
{

}

GazeTracking::~GazeTracking()
{

}

bool GazeTracking::initialize(cv::String xmlFile)
{
	return faceCascade.load(xmlFile);
}

void GazeTracking::process(IplImage* image)
{
	cv::Mat frame(image, true);
	process(frame);
}

void GazeTracking::process(cv::Mat& frame)
{
	std::vector<cv::Rect> faces;
	std::vector<cv::Mat> rgbChannels(frame.channels());
	cv::split(frame, rgbChannels);
	cv::Mat frameGray = rgbChannels[2];

	findFace = false;
	faceCascade.detectMultiScale( frameGray, faces, 1.1, 2, 0|CV_HAAR_SCALE_IMAGE|CV_HAAR_FIND_BIGGEST_OBJECT, cv::Size(150, 150) );
	if(faces.size() > 0)
	{
		findFace = true;
		findPupils(frameGray, faces[0]);
	}
}

void GazeTracking::getLeftPupilXY(int& x, int& y)
{
	x = leftPupil.x + faceRect.x;
	y = leftPupil.y + faceRect.y;
}

void GazeTracking::getRightPupilXY(int& x, int& y)
{
	x = rightPupil.x + faceRect.x;
	y = rightPupil.y + faceRect.y;
}

cv::Point GazeTracking::getLeftPupil()
{
	cv::Point point;
	point.x = leftPupil.x + faceRect.x;
	point.y = leftPupil.y + faceRect.y;
	return point;
}

cv::Point GazeTracking::getRightPupil()
{
	cv::Point point;
	point.x = rightPupil.x + faceRect.x;
	point.y = rightPupil.y + faceRect.y;
	return point;
}

void GazeTracking::getLeftPupilXYInFaceRect(int& x, int& y)
{
	x = leftPupil.x;
	y = leftPupil.y;
}

void GazeTracking::getRightPupilXYInFaceRect(int& x, int& y)
{
	x = rightPupil.x;
	y = rightPupil.y;
}

cv::Point GazeTracking::getLeftPupilInFaceRect()
{
	return leftPupil;
}

cv::Point GazeTracking::getRightPupilInFaceRect()
{
	return rightPupil;
}

int GazeTracking::round(double x)
{
	return (int)(x+0.5);
}

void GazeTracking::findPupils(cv::Mat& frameGray, cv::Rect& face)
{
	faceRect = face;
	faceROI = frameGray(face);
	if (kSmoothFaceImage) {
		double sigma = kSmoothFaceFactor * face.width;
		GaussianBlur( faceROI, faceROI, cv::Size( 0, 0 ), sigma);
	}
	//-- Find eye regions and draw them
	int eyeRegionWidth = face.width * (kEyePercentWidth/100.0);
	int eyeRegionHeight = face.width * (kEyePercentHeight/100.0);
	int eyeRegionTop = face.height * (kEyePercentTop/100.0);
	cv::Rect leftEyeRegion(face.width*(kEyePercentSide/100.0),
		eyeRegionTop,eyeRegionWidth,eyeRegionHeight);
	cv::Rect rightEyeRegion(face.width - eyeRegionWidth - face.width*(kEyePercentSide/100.0),
		eyeRegionTop,eyeRegionWidth,eyeRegionHeight);

	//-- Find Eye Centers
	leftPupil = findEyeCenter(faceROI,leftEyeRegion,"Left Eye");
	rightPupil = findEyeCenter(faceROI,rightEyeRegion,"Right Eye");
	// get corner regions
	cv::Rect leftCornerRegion(leftEyeRegion);
	leftCornerRegion.width -= leftPupil.x;
	leftCornerRegion.x += leftPupil.x;
	leftCornerRegion.height /= 2;
	leftCornerRegion.y += leftCornerRegion.height / 2;
	cv::Rect rightCornerRegion(rightEyeRegion);
	rightCornerRegion.width = rightPupil.x;
	rightCornerRegion.height /= 2;
	rightCornerRegion.y += rightCornerRegion.height / 2;
	// change eye centers to face coordinates
	rightPupil.x += rightEyeRegion.x;
	rightPupil.y += rightEyeRegion.y;
	leftPupil.x += leftEyeRegion.x;
	leftPupil.y += leftEyeRegion.y;

	//-- Find Eye Corners
	/*if (kEnableEyeCorner) {
		cv::Point leftCorner = findEyeCorner(faceROI(leftCornerRegion), false);
		leftCorner.x += leftCornerRegion.x;
		leftCorner.y += leftCornerRegion.y;
		cv::Point rightCorner = findEyeCorner(faceROI(leftCornerRegion), false);
		rightCorner.x += rightCornerRegion.x;
		rightCorner.y += rightCornerRegion.y;
	}*/
}

cv::Point GazeTracking::findEyeCenter(cv::Mat face, cv::Rect eye, std::string debugWindow)
{
	cv::Mat eyeROIUnscaled = face(eye);
	cv::Mat eyeROI;
	scaleToFastSize(eyeROIUnscaled, eyeROI);

	//-- Find the gradient
	cv::Mat gradientX = computeMatXGradient(eyeROI);
	cv::Mat gradientY = computeMatXGradient(eyeROI.t()).t();

	//-- Normalize and threshold the gradient
	// compute all the magnitudes
	cv::Mat mags = matrixMagnitude(gradientX, gradientY);

	//compute the threshold
	double gradientThresh = computeDynamicThreshold(mags, kGradientThreshold);

	//normalize
	for (int y = 0; y < eyeROI.rows; ++y) {
		double *Xr = gradientX.ptr<double>(y), *Yr = gradientY.ptr<double>(y);
		const double *Mr = mags.ptr<double>(y);
		for (int x = 0; x < eyeROI.cols; ++x) {
			double gX = Xr[x], gY = Yr[x];
			double magnitude = Mr[x];
			if (magnitude > gradientThresh) {
				Xr[x] = gX/magnitude;
				Yr[x] = gY/magnitude;
			} else {
				Xr[x] = 0.0;
				Yr[x] = 0.0;
			}
		}
	}

	//-- Create a blurred and inverted image for weighting
	cv::Mat weight;
	GaussianBlur( eyeROI, weight, cv::Size( kWeightBlurSize, kWeightBlurSize ), 0, 0 );
	for (int y = 0; y < weight.rows; ++y) {
		unsigned char *row = weight.ptr<unsigned char>(y);
		for (int x = 0; x < weight.cols; ++x) {
			row[x] = (255 - row[x]);
		}
	}

	//-- Run the algorithm!
	cv::Mat outSum = cv::Mat::zeros(eyeROI.rows,eyeROI.cols,CV_64F);
	// for each possible center
	//printf("Eye Size: %ix%i\n",outSum.cols,outSum.rows);
	for (int y = 0; y < weight.rows; ++y) {
		const unsigned char *Wr = weight.ptr<unsigned char>(y);
		const double *Xr = gradientX.ptr<double>(y), *Yr = gradientY.ptr<double>(y);
		for (int x = 0; x < weight.cols; ++x) {
			double gX = Xr[x], gY = Yr[x];
			if (gX == 0.0 && gY == 0.0) {
				continue;
			}
			testPossibleCentersFormula(x, y, Wr[x], gX, gY, outSum);
		}
	}

	// scale all the values down, basically averaging them
	double numGradients = (weight.rows*weight.cols);
	cv::Mat out;
	outSum.convertTo(out, CV_32F,1.0/numGradients);
	//imshow(debugWindow,out);
	//-- Find the maximum point
	cv::Point maxP;
	double maxVal;
	cv::minMaxLoc(out, NULL,&maxVal,NULL,&maxP);
	//-- Flood fill the edges
	if(kEnablePostProcess) {
		cv::Mat floodClone;
		//double floodThresh = computeDynamicThreshold(out, 1.5);
		double floodThresh = maxVal * kPostProcessThreshold;
		cv::threshold(out, floodClone, floodThresh, 0.0f, cv::THRESH_TOZERO);
		cv::Mat mask = floodKillEdges(floodClone);
		//imshow(debugWindow + " Mask",mask);
		//imshow(debugWindow,out);
		// redo max
		cv::minMaxLoc(out, NULL,&maxVal,NULL,&maxP,mask);
	}
	return unscalePoint(maxP,eye);
}

cv::Point GazeTracking::unscalePoint(cv::Point p, cv::Rect origSize) 
{
	float ratio = (((float)kFastEyeWidth)/origSize.width);
	/*int x = round(p.x / ratio);
	int y = round(p.y / ratio);*/
	int x = round(p.x / ratio);
	int y = round(p.y / ratio);
	return cv::Point(x,y);
}

// returns a mask
cv::Mat GazeTracking::floodKillEdges(cv::Mat &mat) 
{
	rectangle(mat,cv::Rect(0,0,mat.cols,mat.rows),255);

	cv::Mat mask(mat.rows, mat.cols, CV_8U, 255);
	std::queue<cv::Point> toDo;
	toDo.push(cv::Point(0,0));
	while (!toDo.empty()) {
		cv::Point p = toDo.front();
		toDo.pop();
		if (mat.at<float>(p) == 0.0f) {
			continue;
		}
		// add in every direction
		cv::Point np(p.x + 1, p.y); // right
		if (floodShouldPushPoint(np, mat)) toDo.push(np);
		np.x = p.x - 1; np.y = p.y; // left
		if (floodShouldPushPoint(np, mat)) toDo.push(np);
		np.x = p.x; np.y = p.y + 1; // down
		if (floodShouldPushPoint(np, mat)) toDo.push(np);
		np.x = p.x; np.y = p.y - 1; // up
		if (floodShouldPushPoint(np, mat)) toDo.push(np);
		// kill it
		mat.at<float>(p) = 0.0f;
		mask.at<uchar>(p) = 0;
	}
	return mask;
}
bool GazeTracking::floodShouldPushPoint(const cv::Point &np, const cv::Mat &mat) 
{
	return inMat(np, mat.rows, mat.cols);
}

bool GazeTracking::inMat(cv::Point p,int rows,int cols) 
{
	return p.x >= 0 && p.x < cols && p.y >= 0 && p.y < rows;
}

void GazeTracking::testPossibleCentersFormula(int x, int y, unsigned char weight,double gx, double gy, cv::Mat &out) 
{
	// for all possible centers
	for (int cy = 0; cy < out.rows; ++cy) {
		double *Or = out.ptr<double>(cy);
		for (int cx = 0; cx < out.cols; ++cx) {
			if (x == cx && y == cy) {
				continue;
			}
			// create a vector from the possible center to the gradient origin
			double dx = x - cx;
			double dy = y - cy;
			// normalize d
			double magnitude = sqrt((dx * dx) + (dy * dy));
			dx = dx / magnitude;
			dy = dy / magnitude;
			double dotProduct = dx*gx + dy*gy;
			dotProduct = std::max(0.0,dotProduct);
			// square and multiply by the weight
			Or[cx] += dotProduct * dotProduct * (weight/kWeightDivisor);
		}
	}
}

double GazeTracking::computeDynamicThreshold(const cv::Mat &mat, double stdDevFactor) 
{
	cv::Scalar stdMagnGrad, meanMagnGrad;
	cv::meanStdDev(mat, meanMagnGrad, stdMagnGrad);
	double stdDev = stdMagnGrad[0] / sqrt((long double)mat.rows*mat.cols);
	return stdDevFactor * stdDev + meanMagnGrad[0];
}

cv::Mat GazeTracking::matrixMagnitude(const cv::Mat &matX, const cv::Mat &matY) {
	cv::Mat mags(matX.rows,matX.cols,CV_64F);
	for (int y = 0; y < matX.rows; ++y) {
		const double *Xr = matX.ptr<double>(y), *Yr = matY.ptr<double>(y);
		double *Mr = mags.ptr<double>(y);
		for (int x = 0; x < matX.cols; ++x) {
			double gX = Xr[x], gY = Yr[x];
			double magnitude = sqrt((gX * gX) + (gY * gY));
			Mr[x] = magnitude;
		}
	}
	return mags;
}

void GazeTracking::scaleToFastSize(const cv::Mat &src,cv::Mat &dst) 
{
	cv::resize(src, dst, cv::Size(kFastEyeWidth,(((float)kFastEyeWidth)/src.cols) * src.rows));
}

cv::Mat GazeTracking::computeMatXGradient(const cv::Mat &mat) 
{
	cv::Mat out(mat.rows,mat.cols,CV_64F);

	for (int y = 0; y < mat.rows; ++y) {
		const uchar *Mr = mat.ptr<uchar>(y);
		double *Or = out.ptr<double>(y);

		Or[0] = Mr[1] - Mr[0];
		for (int x = 1; x < mat.cols - 1; ++x) {
			Or[x] = (Mr[x+1] - Mr[x-1])/2.0;
		}
		Or[mat.cols-1] = Mr[mat.cols-1] - Mr[mat.cols-2];
	}

	return out;
}
