// MiniCVNative.cpp : Defines the exported functions for the DLL application.
//

#include "stdafx.h"
#include "MiniCVNative.h"
#include <string>
#include <iostream>
#include <opencv2/imgproc/types_c.h>

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

DllExport(bool) cvSolvePnP(const Point2d* imgPoints, const Point3d* worldPoints, const int N, const Matx33d K, const double* distortionCoeffs, const int solverKind, Vec3d& tVec, Vec3d& rVec) {
	vector<Point2d> image(imgPoints, imgPoints + N);
	vector<Point3d> world(worldPoints, worldPoints + N);
	//vector<double> distortion(distortionCoeffs, distortionCoeffs + 6);
	
	int kind = 0;
	if (solverKind == 0) {
		kind = cv::SOLVEPNP_ITERATIVE;
	}
	else if (solverKind == 1) {
		kind = cv::SOLVEPNP_EPNP;
	}
	else if (solverKind == 2) {
		kind = cv::SOLVEPNP_P3P;
	}
	else if (solverKind == 3) {
		kind = cv::SOLVEPNP_DLS;
	}
	else if (solverKind == 4) {
		kind = cv::SOLVEPNP_UPNP;
	}
	else if (solverKind == 5) {
		kind = cv::SOLVEPNP_AP3P;
	}

	Mat intern(K);

	Mat distortion(1, 4, CV_64F, (void*)distortionCoeffs);

	Vec3d tOut;
	Vec3d rOut;
	bool suc = cv::solvePnP(world, image, intern, distortion, rOut, tOut, false, kind);
	if (suc) {
		tVec = tOut;
		rVec = rOut;
		return true;
	}
	else {
		return false;
	}
}

DllExport(bool) cvSolvePnPRansac(const Point2d* imgPoints, const Point3d* worldPoints, const int N, const Matx33d K, const double* distortionCoeffs, const int solverKind, Vec3d& tVec, Vec3d& rVec, int* inlierCount, int* outInliers) {
	vector<Point2d> image(imgPoints, imgPoints + N);
	vector<Point3d> world(worldPoints, worldPoints + N);
	//vector<double> distortion(distortionCoeffs, distortionCoeffs + 6);

	int kind = 0;
	if (solverKind == 0) {
		kind = cv::SOLVEPNP_ITERATIVE;
	}
	else if (solverKind == 1) {
		kind = cv::SOLVEPNP_EPNP;
	}
	else if (solverKind == 2) {
		kind = cv::SOLVEPNP_P3P;
	}
	else if (solverKind == 3) {
		kind = cv::SOLVEPNP_DLS;
	}
	else if (solverKind == 4) {
		kind = cv::SOLVEPNP_UPNP;
	}
	else if (solverKind == 5) {
		kind = cv::SOLVEPNP_AP3P;
	}

	Mat intern(K);

	Mat distortion(1, 4, CV_64F, (void*)distortionCoeffs);

	Vec3d tOut;
	Vec3d rOut;
	vector<int> inliers;
	bool suc = cv::solvePnPRansac(world, image, intern, distortion, rOut, tOut, false, 100, 0.002F, 0.99, inliers, kind);
	if (suc) {
		tVec = tOut;
		rVec = rOut;
		*inlierCount = inliers.size();
		for (int i = 0; i < inliers.size(); i++) {
			outInliers[i] = inliers[i];
		}
		return true;
	}
	else {
		*inlierCount = 0;
		return false;
	}
}
DllExport(void) cvRefinePnPLM(const Point2d* imgPoints, const Point3d* worldPoints, const int N, const Matx33d K, const double* distortionCoeffs, Vec3d& tVec, Vec3d& rVec) {
	vector<Point2d> image(imgPoints, imgPoints + N);
	vector<Point3d> world(worldPoints, worldPoints + N);
	Mat intern(K);
	Mat distortion(1, 4, CV_64F, (void*)distortionCoeffs);
	Vec3d tOut = tVec;
	Vec3d rOut = rVec;
	cv::solvePnPRefineLM(world, image, intern, distortion, rOut, tOut);
	tVec = tOut;
	rVec = rOut;
	return;
}
DllExport(void) cvRefinePnPVVS(const Point2d* imgPoints, const Point3d* worldPoints, const int N, const Matx33d K, const double* distortionCoeffs, Vec3d& tVec, Vec3d& rVec) {
	vector<Point2d> image(imgPoints, imgPoints + N);
	vector<Point3d> world(worldPoints, worldPoints + N);
	Mat intern(K);
	Mat distortion(1, 4, CV_64F, (void*)distortionCoeffs);
	Vec3d tOut = tVec;
	Vec3d rOut = rVec;
	cv::solvePnPRefineVVS(world, image, intern, distortion, rOut, tOut);
	tVec = tOut;
	rVec = rOut;
	return;
}

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

DllExport(DetectorResult*) cvDetectFeatures(char* data, int width, int height, int channels, int mode = FEATURE_MODE_AKAZE) {

	int fmt;
	int convert;
	switch (channels)
	{
		case 1: 
			fmt = CV_8UC1;
			convert = 0;
			break;
		case 2:
			fmt = CV_8UC2;
			convert = 0;
			break;
		case 3:
			fmt = CV_8UC3;
			convert = CV_RGB2GRAY;
			break;
		case 4:
			fmt = CV_8UC4;
			convert = CV_RGBA2GRAY;
			break;
		default:
			return nullptr;
	}
	cv::setBreakOnError(false);
	
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
	case FEATURE_MODE_SIFT:
		detector = cv::SIFT::create();
		break;
	default:
		return nullptr;
	}

	Mat img;
	if(convert != 0) cv::cvtColor(input, img, CV_RGB2GRAY);
	else img = input;

	std::vector<KeyPoint> points;
	Mat descriptorsM;
	detector->detect(img, points);
	detector->compute(img, points, descriptorsM);

	auto descriptorCount = descriptorsM.total();
	

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

		auto points1 = new KeyPoint2d[points.size()];
		auto descriptors1 = new uchar[descriptorCount];

		for (int i = 0; i < points.size(); i++)
		{
			points1[i].angle = points[i].angle;
			points1[i].class_id = points[i].class_id;
			points1[i].octave = points[i].octave;
			points1[i].pt = points[i].pt;
			points1[i].response = points[i].response;
			points1[i].size = points[i].size;
		}
		if (descriptorsM.elemSize() != 1)
		{
			printf("BAD DESCRIPTORS\n");
		}
		memcpy(descriptors1, descriptorsM.data, descriptorCount);
		
		detector->clear();
		img.release();

		auto res = new DetectorResult();
		res->PointCount = (int)points.size();
		res->DescriptorEntries = (int)descriptorCount;
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


DllExport(int) cvFivePoint(const Point2d* pa, const Point2d* pb, Matx33d* Es)
{
	Mat aa(5, 1, CV_64FC2, (void*)pa);
	Mat ba(5, 1, CV_64FC2, (void*)pb);
	Mat res;
	int cnt = runFivepoint(aa, ba, res);

	for (int i = 0; i < cnt; i++) 
	{
		auto e = res.rowRange(3 * i, 3 * i + 3);
		Matx33d m33((double*)e.ptr());
		Es[i] = m33;
	}
	return cnt;
}

DllExport(bool) cvDetectQRCode(char* data, int width, int height, int channels, int* positions, int* count)
{
	int fmt;
	int convert;
	switch (channels)
	{
	case 1:
		fmt = CV_8UC1;
		convert = 0;
		break;
	case 2:
		fmt = CV_8UC2;
		convert = 0;
		break;
	case 3:
		fmt = CV_8UC3;
		convert = CV_RGB2GRAY;
		break;
	case 4:
		fmt = CV_8UC4;
		convert = CV_RGBA2GRAY;
		break;
	default:
		return false;
	}
	cv::setBreakOnError(false);

	Mat input(height, width, fmt, (void*)data);
	cv::Ptr<cv::FeatureDetector> detector;
	
	Mat img;
	if (convert != 0) cv::cvtColor(input, img, CV_RGB2GRAY);
	else img = input;


	std::vector<cv::Point> points;
	cv::QRCodeDetector d;
	if (d.detect(img, points))
	{
		auto cnt = std::min((int)points.size(), *count);

		int o = 0;
		for (int i = 0; i < cnt; i++)
		{
			positions[o++] = points[i].x;
			positions[o++] = points[i].y;
		}
		*count = cnt;
		return true;
	}
	else return false;

}


DllExport(bool) cvDetectArucoMarkers(char* data, int width, int height, int channels, int* infoCount, ArucoMarkerInfo* infos)
{
	int fmt;
	int convert;
	switch (channels)
	{
	case 1:
		fmt = CV_8UC1;
		convert = 0;
		break;
	case 2:
		fmt = CV_8UC2;
		convert = 0;
		break;
	case 3:
		fmt = CV_8UC3;
		convert = CV_RGB2GRAY;
		break;
	case 4:
		fmt = CV_8UC4;
		convert = CV_RGBA2GRAY;
		break;
	default:
		return false;
	}
	cv::setBreakOnError(false);

	Mat input(height, width, fmt, (void*)data);
	Mat img;

	if (convert != 0) cv::cvtColor(input, img, convert);
	else img = input;

	aruconano::MarkerDetector detector;
	
	auto result = detector.detect(img);

	int pi = 0;
	int ii = 0;
	int ic = *infoCount;
	for (int i = 0; i < result.size(); i++)
	{
		auto marker = result[i];
		auto start = pi;
		V2f tmp[4];
		for (int ipi = 0; ipi < marker.size() && ipi < 4; ipi++)
		{
			auto pt = marker[ipi];
			tmp[ipi] = { pt.x, pt.y };
			pi++;
		}
		if (infos && ii < ic) infos[ii] = { marker.id, tmp[0], tmp[1], tmp[2], tmp[3] };
		ii++;
	}

	if (ii > 0 && ii <= ic)
	{
		*infoCount = ii;

		return true;
	}
	else return false;

}

DllExport(void) cvTest()
{
	double arr1[] = { 1.02736340896169,0.0824668492092278,0.399551267404442,-0.0921929548453944,1.03823834690651,0.12968385038247,-0.159700861517284,-0.0688464520524622,1 };
	// 1.02736340896169,0.0824668492092278,0.399551267404442,-0.0921929548453944,1.03823834690651,0.12968385038247,-0.159700861517284,-0.0688464520524622,1

	double arr[] = { arr1[0], arr1[3], arr1[6], arr1[1], arr1[4], arr1[7], arr1[2], arr1[5], arr1[8] };

	cv::Mat H(3, 3, CV_64FC1, (void*)arr);
	auto K = cv::Mat::eye(3, 3, CV_64FC1);

	std::vector< cv::Mat > rot;
	std::vector< cv::Mat > trans;
	std::vector< cv::Mat > norm;

	cv::Mat U(3, 3, CV_64FC1);
	cv::Mat S(3, 3, CV_64FC1);
	cv::Mat Vt(3, 3, CV_64FC1);
	cv::SVDecomp(H, S, U, Vt);
	cout << "S" << endl;
	cout << S << endl;
	cout << "U" << endl;
	cout << U << endl;
	cout << "Vt" << endl;
	cout << Vt << endl;

	cv::decomposeHomographyMat(H, K, rot, trans, norm);


	for (int i = 0; i < rot.size(); i++)
	{
		printf("%d: \n", i);
		auto R = rot[i];
		for (int r = 0; r < 3; r++)
		{
			for (int c = 0; c < 3; c++)
			{
				printf("%.8f ", R.at<double>(c, r));
			}
			printf("\n");
		}
	}



}
