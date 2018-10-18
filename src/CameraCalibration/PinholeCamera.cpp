#include "PinholeCamera.h"
#define _USE_MATH_DEFINES
#include <math.h>
#include <Common_GQ.h>

using namespace std;
using namespace cv;

PinholeCamera::PinholeCamera()
{
}


PinholeCamera::~PinholeCamera()
{
}

void PinholeCamera::InheritInitUndistortRectifyMap(const cv::Mat &R, const cv::Mat &K, const cv::Size &size, cv::Mat &mapx, cv::Mat &mapy)
{
	cv::initUndistortRectifyMap(m_K, m_D, R, K, size, CV_32FC1, mapx, mapy);
}

void PinholeCamera::InheritProjectPoints(const std::vector<cv::Point3f> &pts_3d, const cv::InputArray &r_vec, const cv::InputArray &t_vec, std::vector<cv::Point2f> &pts_2d)
{
	cv::projectPoints(pts_3d, r_vec, t_vec, m_K, m_D, pts_2d);
}

void PinholeCamera::InheritUndistortPoints(const std::vector<cv::Point2f> &pts_src, std::vector<cv::Point2f> &pts_dst)
{
	cv::undistortPoints(pts_src, pts_dst, m_K, m_D);
}

double PinholeCamera::InheritCalibrate(const std::vector<std::vector<cv::Point3f>> objects, const std::vector<std::vector<cv::Point2f>> &corners, std::vector<cv::Vec3d> &rotation_vectors, std::vector<cv::Vec3d> &translation_vectors, cv::TermCriteria critia, cv::Mat &idx)
{
	int flags = cv::CALIB_USE_INTRINSIC_GUESS;
	double rms = cv::calibrateCamera(objects, corners, m_raw_size, m_K, m_D, rotation_vectors, translation_vectors, flags, critia);
	size_t size = objects.size();
	idx = cv::Mat(1, size, CV_32S);
	for (size_t i = 0; i < size; i++)
		idx.at<int>(i) = i;
	return rms;
}

bool PinholeCamera::InheritSovlePnP(const std::vector<cv::Point3f> &object_pts, const std::vector<cv::Point2f> &corners, cv::Mat &R, cv::Mat &T)
{
	return cv::solvePnP(object_pts, corners, m_K, m_D, R, T, false, cv::SOLVEPNP_ITERATIVE);
}
