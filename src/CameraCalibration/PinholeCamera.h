#pragma once
#include "CameraBase.h"
class PinholeCamera :
	public CameraBase
{
public:
	PinholeCamera();
	~PinholeCamera();

protected:
	void InheritInitUndistortRectifyMap(const cv::Mat &R, const cv::Mat &K, const cv::Size &size, cv::Mat &mapx, cv::Mat &mapy) override;
	void InheritProjectPoints(const std::vector<cv::Point3f> &pts_3d, const cv::InputArray &r_vec, const cv::InputArray &t_vec, std::vector<cv::Point2f> &pts_2d) override;
	void InheritUndistortPoints(const std::vector<cv::Point2f> &pts_src, std::vector<cv::Point2f> &pts_dst) override;
	double InheritCalibrate(const std::vector<std::vector<cv::Point3f>> objects, const std::vector<std::vector<cv::Point2f>> &corners,
		std::vector<cv::Vec3d> &rotation_vectors, std::vector<cv::Vec3d> &translation_vectors,
		cv::TermCriteria critia, cv::Mat &idx) override;
	bool InheritSovlePnP(const std::vector<cv::Point3f> &object_pts, const std::vector<cv::Point2f> &corners, cv::Mat &R, cv::Mat &T) override;

private:

};

