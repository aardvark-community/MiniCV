#pragma once

#include "stdafx.h"
#include <stdio.h>

#include <vector>
#include <opencv2\opencv.hpp>
#include <opencv2\calib3d.hpp>

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

#define FEATURE_MODE_AKAZE 1
#define FEATURE_MODE_ORB 2
#define FEATURE_MODE_BRISK 3
