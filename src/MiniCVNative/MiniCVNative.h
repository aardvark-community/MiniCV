#pragma once

#include "stdafx.h"
#include <stdio.h>


#include <vector>
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>
#include "fivepoint.h"
#include "ap3p.h"
#include "aruco_nano.h"
#include <opencv2/xfeatures2d.hpp>

typedef struct KeyPoint2d_ {
	cv::Point2f pt;
	float size;
	float angle;
	float response; 
	int octave;
	int class_id;
} KeyPoint2d;

typedef struct DetectorResult_ {
	int PointCount;
	int DescriptorEntries;
	KeyPoint2d* Points;
	uchar* Descriptors;
} DetectorResult;

typedef struct {
	float X;
	float Y;
} V2f;

typedef struct ArucoMarkerInfo_ {
	int Id;
	V2f P0;
	V2f P1;
	V2f P2;
	V2f P3;
} ArucoMarkerInfo;


#define FEATURE_MODE_AKAZE 1
#define FEATURE_MODE_ORB 2
#define FEATURE_MODE_BRISK 3
#define FEATURE_MODE_SIFT 4
#define FEATURE_MODE_SURF 5
#define FEATURE_MODE_MSD 6
#define FEATURE_MODE_VGG 7