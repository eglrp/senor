#pragma once
#include <opencv2/opencv.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "PlaneExtrinsicOptimization.h"
class LaserCalib
{
public:
	LaserCalib();
	~LaserCalib();

	bool Init(const std::vector<std::string> &left, const std::vector<std::string> &right);

	std::string GetLaserPath(int index,bool is_laser1) const;

	std::vector<int> GetPlanePoints(int index,bool is_laser1) const;

	void SetPlanePointsIndex(int index, const std::vector<int> &list,bool is_laser1);

	bool Calibrate(double ransac_thre=0.01); 

	bool SaveCalibResult(const char *path);

	bool LoadCalibResult(const char *path);

	bool MergePointCloud(int index,pcl::PointCloud<pcl::PointXYZI> &pc);
private:

	cv::Mat m_trans_mat;            //激光2到激光1的变换矩阵

	std::vector<std::string> m_laser1_list;            //激光1路径
	std::vector<std::string> m_laser2_list;            //激光2路径

	std::vector<std::vector<int>> m_plane_points_list_1;            //激光1选择的平面点集合
	std::vector<std::vector<int>> m_plane_points_list_2;            //激光2选择的平面点集合

	PlaneExtrinsicOptimization m_calib;
};

