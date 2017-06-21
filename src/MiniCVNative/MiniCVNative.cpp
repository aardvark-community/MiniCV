// MiniCVNative.cpp : Defines the exported functions for the DLL application.
//

#include "stdafx.h"
#include <stdio.h>

#include <vector>
#include <opencv2\opencv.hpp>
#include <opencv2\calib3d.hpp>

using namespace cv;
using namespace std;


//runtime identify cv::Mat type and channel count
string type2str(int type) {
	string r;

	uchar depth = type & CV_MAT_DEPTH_MASK;
	uchar chans = 1 + (type >> CV_CN_SHIFT);

	switch (depth) {
	case CV_8U:  r = "8U"; break;
	case CV_8S:  r = "8S"; break;
	case CV_16U: r = "16U"; break;
	case CV_16S: r = "16S"; break;
	case CV_32S: r = "32S"; break;
	case CV_32F: r = "32F"; break;
	case CV_64F: r = "64F"; break;
	default:     r = "User"; break;
	}

	r += "C";
	r += (chans + '0');

	return r;
}


typedef struct {

	double FocalLength;
	Point2d PrincipalPoint;
	double Probability;
	double InlierThreshold;

} RecoverPoseConfig;

DllExport(void) cvRecoverPoses(const RecoverPoseConfig* config, const int N, const Point2d* pa, const Point2d* pb, Matx33d& rMat1, Matx33d& rMat2, Vec3d& tVec, uint8_t* ms) {
	vector<Point2d> a(pa, pa + N);
	vector<Point2d> b(pb, pb + N);

	Mat E;
	Mat mask;

	E = findEssentialMat(a, b, config->FocalLength, config->PrincipalPoint, RANSAC, config->Probability, config->InlierThreshold, mask);

	for (int i = 0; i < mask.rows; i++) {
		ms[i] = mask.at<uint8_t>(i);
		//printf("fufu1 %d  ", mask.at<uint8_t>(i));
		//printf("fufu2 %d  \n", ms[i]);
	}

	decomposeEssentialMat(E, rMat1, rMat2, tVec);
}


DllExport(int) cvRecoverPose(const RecoverPoseConfig* config, const int N, const Point2d* pa, const Point2d* pb, Matx33d& rMat, Vec3d& tVec, uint8_t* ms) {
	vector<Point2d> a(pa, pa + N);
	vector<Point2d> b(pb, pb + N);

	Mat E;
	Mat mask;

	E = findEssentialMat(a, b, config->FocalLength, config->PrincipalPoint, RANSAC, config->Probability, config->InlierThreshold, mask);

	for (int i = 0; i < mask.rows; i++) {
		ms[i] = mask.at<uint8_t>(i);
		//printf("fufu1 %d  ", mask.at<uint8_t>(i));
		//printf("fufu2 %d  \n", ms[i]);
	}

	auto res = recoverPose(E, a, b, rMat, tVec, config->FocalLength, config->PrincipalPoint, mask);

	return res;
}

