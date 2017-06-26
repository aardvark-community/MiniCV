// MiniCVNative.cpp : Defines the exported functions for the DLL application.
//

#include "stdafx.h"
#include <stdio.h>
#include <string>
#include <iostream>

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

DllExport(bool) cvRecoverPoses(const RecoverPoseConfig* config, const int N, const Point2d* pa, const Point2d* pb, Matx33d& rMat1, Matx33d& rMat2, Vec3d& tVec, uint8_t* ms) {
	
	if (N < 5) return false;

	Mat E;
	Mat mask;
	
	Mat aa(N, 1, CV_64FC2, (void*)pa);
	Mat ba(N, 1, CV_64FC2, (void*)pb);

	//printf("%d vs %d (%s vs %s)\n", aa.checkVector(2), ba.checkVector(2), type2str(aa.type()).c_str(), type2str(aa.type()).c_str());

	E = findEssentialMat(aa, ba, config->FocalLength, config->PrincipalPoint, RANSAC, config->Probability, config->InlierThreshold, mask);

	for (int i = 0; i < mask.rows; i++) {
		ms[i] = mask.at<uint8_t>(i);
		//printf("fufu1 %d  ", mask.at<uint8_t>(i));
		//printf("fufu2 %d  \n", ms[i]);
	}
	if (E.rows == 3 && E.cols == 3)
	{
		decomposeEssentialMat(E, rMat1, rMat2, tVec);
		return true;
	}
	else
	{
		return false;
	}
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

void cvCornerSubPix(const cv::Mat img, const vector<Vec2d> corners) {
	cornerSubPix(img, corners, Size(11, 11), Size(-1, -1), TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 30, 0.001));
}

DllExport(void) cvDoStuff(std::string* imgs1, int ct, std::string* repr1, int rct, std::string* oFilenames1) {
	auto op = vector<Vec3d>();
	for (int x = 0; x < 7; x++) {
		for (int y = 0; y < 6; y++) {
			op.push_back(Vec3d(x, y, 0));
		}
	}

	string imgs[13] = 
	{
		"d:\\bla2\\calib\\small\\dsc02030_small.jpg",
		"d:\\bla2\\calib\\small\\dsc02031_small.jpg",
		"d:\\bla2\\calib\\small\\dsc02032_small.jpg",
		"d:\\bla2\\calib\\small\\dsc02033_small.jpg",
		"d:\\bla2\\calib\\small\\dsc02034_small.jpg",
		"d:\\bla2\\calib\\small\\dsc02035_small.jpg",
		"d:\\bla2\\calib\\small\\dsc02036_small.jpg",
		"d:\\bla2\\calib\\small\\dsc02037_small.jpg",
		"d:\\bla2\\calib\\small\\dsc02038_small.jpg",
		"d:\\bla2\\calib\\small\\dsc02039_small.jpg",
		"d:\\bla2\\calib\\small\\dsc02040_small.jpg",
		"d:\\bla2\\calib\\small\\dsc02041_small.jpg",
		"d:\\bla2\\calib\\small\\dsc02042_small.jpg"
	};

	string repr[6] =
	{
		"d:\\bla2\\buch\\small\\1\\dsc02043_small.jpg",
		"d:\\bla2\\buch\\small\\1\\dsc02044_small.jpg",
		"d:\\bla2\\buch\\small\\1\\dsc02045_small.jpg",
		"d:\\bla2\\buch\\small\\1\\dsc02047_small.jpg",
		"d:\\bla2\\buch\\small\\1\\dsc02048_small.jpg",
		"d:\\bla2\\buch\\small\\1\\dsc02049_small.jpg"
	};

	string oFilenames[6] =
	{
		"D:\\bla2\\buch\\small\\1\\undistorted\\dsc02043_small.jpg",
		"D:\\bla2\\buch\\small\\1\\undistorted\\dsc02044_small.jpg",
		"D:\\bla2\\buch\\small\\1\\undistorted\\dsc02045_small.jpg",
		"D:\\bla2\\buch\\small\\1\\undistorted\\dsc02047_small.jpg",
		"D:\\bla2\\buch\\small\\1\\undistorted\\dsc02048_small.jpg",
		"D:\\bla2\\buch\\small\\1\\undistorted\\dsc02049_small.jpg"
	};





	auto size = Size();
	
	auto objectPoints = vector<vector<Vec3d>>();
	auto imagePoints = vector<vector<Vec2d>>();

	printf("Finding calibration target in %d images...\n", ct);
	for (int i = 0; i < ct; i++) {
		auto filename = imgs[i];
		printf("attempt %d with name: %s", i, (filename.c_str()));
		auto color = imread(filename.c_str());
		auto img = Mat();
		cvtColor(color, img, COLOR_BGR2GRAY);
		size = Size(img.rows, img.cols);
		auto corners = vector<Vec2d>();

		bool success = findChessboardCorners(img, Size(7, 6), corners);

		if (success) {
			objectPoints.push_back(op);

			cvCornerSubPix(img, corners);

			imagePoints.push_back(corners);
			printf("Found %d target points in img No. %d\n", corners.size(), i);
		}
	}

	printf("Estimating calibration ...\n");
	auto kMat = Matx33d();
	auto dist = vector<double>();
	calibrateCamera(objectPoints, imagePoints, size, kMat, dist, noArray(), noArray());
	printf("fx = %d\n", kMat(0, 0));
	printf("fy = %d\n", kMat(1, 1));
	printf("cx = %d\n", kMat(2, 0));
	printf("cy = %d\n", kMat(2, 1));

	printf("k1 = %d\n", dist.at(0));
	printf("k2 = %d\n", dist.at(1));
	printf("p1 = %d\n", dist.at(2));
	printf("p2 = %d\n", dist.at(3));
	printf("k3 = %d\n", dist.at(4));

	printf("Undistorting %d imgs ...\n", rct);
	for (int i = 0; i < rct; i++) {
		auto img = imread(repr[i]);

		auto dst = Mat();

		undistort(img, dst, kMat, dist);

		printf("Undistorted img No. %d\n", i);

		auto fn = oFilenames[i];
		imwrite(fn, dst);
	}
	printf("finished\n");
}