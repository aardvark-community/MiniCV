// MiniCVNative.cpp : Defines the exported functions for the DLL application.
//

#include "stdafx.h"
#include "MiniCVNative.h"

using namespace cv;
using namespace std;


//runtime identify cv::Mat type and channel count
static string type2str(int type) {
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




DllExport(DetectorResult*) cvDetectFeatures(char* data, int width, int height, int channels, int mode = FEATURE_MODE_AKAZE) {

	int fmt;
	switch (channels)
	{
		case 1: 
			fmt = CV_8UC1;
			break;
		case 2:
			fmt = CV_8UC2;
			break;
		case 3:
			fmt = CV_8UC3;
			break;
		case 4:
			fmt = CV_8UC4;
			break;
		default:
			return nullptr;
	}

	Mat input(height, width, fmt, (void*)data);
	cv::Ptr<cv::FeatureDetector> detector;
	switch (mode)
	{
	case FEATURE_MODE_AKAZE:
		detector = cv::AKAZE::create();
		break;
	case FEATURE_MODE_ORB:
		detector = cv::ORB::create();
		break;
	case FEATURE_MODE_BRISK:
		detector = cv::BRISK::create();
		break;
	default:
		return nullptr;
	}

	Mat img;
	cv::cvtColor(input, img, CV_RGB2GRAY);

	vector<KeyPoint> points;
	vector<float> descriptors;
	detector->detectAndCompute(img, noArray(), points, descriptors);

	if (points.size() == 0)
	{
		auto res = new DetectorResult();
		res->PointCount = 0;
		res->DescriptorEntries = 0;
		res->Points = nullptr;
		res->Descriptors = nullptr;
		return res;
	}
	else
	{
		printf("descriptors: %d\n", descriptors.size());
		printf("points: %d\n", points.size());

		auto points1 = new KeyPoint2d[points.size()];
		auto descriptors1 = new float[descriptors.size()];

		for (int i = 0; i < points.size(); i++)
		{
			points1[i].angle = points[i].angle;
			points1[i].class_id = points[i].class_id;
			points1[i].octave = points[i].octave;
			points1[i].pt = points[i].pt;
			points1[i].response = points[i].response;
			points1[i].size = points[i].size;
		}

		std::copy(descriptors.begin(), descriptors.end(), descriptors1);
		//std::copy(descriptors.begin<float>(), descriptors.end<float>(), descriptors1);

		detector->clear();
		img.release();

		auto res = new DetectorResult();
		res->PointCount = (int)points.size();
		res->DescriptorEntries = (int)descriptors.size();
		res->Points = points1;
		res->Descriptors = descriptors1;

		return res;
	}
}

DllExport(void) cvFreeFeatures(DetectorResult* res)
{
	if (!res || !res->PointCount) return;

	if (res->Descriptors)
	{
		delete res->Descriptors;
		res->Descriptors = nullptr;
	}
	if (res->Points)
	{
		delete res->Points;
		res->Points = nullptr;
	}

	delete res;
}