// MiniCVNative.cpp : Defines the exported functions for the DLL application.
//

#include "stdafx.h"
#include <stdio.h>

#include <vector>
#include <opencv2\opencv.hpp>
#include <opencv2\calib3d.hpp>

using namespace cv;
using namespace std;


typedef struct {

	double FocalLength;
	Point2d PrincipalPoint;
	double Probability;
	double InlierThreshold;

} RecoverPoseConfig;

DllExport(int) cvRecoverPose(const RecoverPoseConfig* config, const int N, const Point2d* pa, const Point2d* pb, Matx33d& rMat, Vec3d& tVec) {
	vector<Point2d> a(pa, pa + N);
	vector<Point2d> b(pb, pb + N);

	Mat E;
	Mat mask;

	E = findEssentialMat(a, b, config->FocalLength, config->PrincipalPoint, RANSAC, config->Probability, config->InlierThreshold, mask);
	return recoverPose(E, a, b, rMat, tVec, config->FocalLength, config->PrincipalPoint, mask);
}