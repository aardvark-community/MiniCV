#pragma once

#include "stdafx.h"
#include <stdio.h>

#include <vector>
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>

int runFivepoint(cv::InputArray _m1, cv::InputArray _m2, cv::OutputArray _model);