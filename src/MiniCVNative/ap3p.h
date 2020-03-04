#pragma once

#include "stdafx.h"
#include <stdio.h>

#include <vector>
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>

DllExport(int) solveAp3p(cv::Matx33d* R, cv::Vec3d* t, double mu0, double mv0, double X0, double Y0, double Z0, double mu1,
	double mv1, double X1, double Y1, double Z1, double mu2, double mv2, double X2, double Y2, double Z2, double inv_fx, double inv_fy, double cx_fx, double cy_fy);