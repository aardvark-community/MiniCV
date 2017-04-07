// MiniCVNative.cpp : Defines the exported functions for the DLL application.
//

#include "stdafx.h"
#include <stdio.h>

#include <vector>
#include <opencv2\opencv.hpp>
#include <opencv2\calib3d.hpp>

using namespace cv;
using namespace std;


DllExport(int) cvRecoverPose(const int N, const Point2d* pa, const Point2d* pb, Matx33d& rMat, Vec3d& tVec) {
	vector<Point2d> a(pa, pa + N);
	vector<Point2d> b(pb, pb + N);

	Mat E;
	Mat mask;

	E = findEssentialMat(a, b, 1.0, Point2d(0,0), RANSAC, 0.99999999999999, 0.000000001, mask);
	return recoverPose(E, a, b, rMat, tVec, 1.0, Point2d(0,0), mask);
}