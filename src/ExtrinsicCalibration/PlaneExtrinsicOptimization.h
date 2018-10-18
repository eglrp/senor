#pragma once
#include <opencv2/opencv.hpp>
class PlaneExtrinsicOptimization
{
public:
	PlaneExtrinsicOptimization();
	~PlaneExtrinsicOptimization();
	bool Calibrate(double src_ransac_thre = 0, double dst_ransac_thre = 0) ;
	void Init(int count=0);
	bool SaveCalibResult(const char *path);
	bool LoadCalibResult(const char *path);
	void SetSrcData(const std::vector<cv::Point3d> &data, int index = -1);
	void SetDstData(const std::vector<cv::Point3d> &data, int index = -1);
	bool GetSrcData(std::vector<cv::Point3d> &data, int index);
	bool GetDstData(std::vector<cv::Point3d> &data, int index);

	bool GetCalibrated() const;
	cv::Mat GetExtrinsicMat() const; //P(src)=m_trans_mat*P(dst)
	void SetExtrinsicMat(const cv::Mat &RT);

protected:
	std::vector<std::vector<cv::Point3d>> m_data_src;
	std::vector<std::vector<cv::Point3d>> m_data_dst;
	//存储旋转向量和平移向量
	double m_trans_vec[6];   
	bool m_calibrated;
//protected:
public:
	//Ransac筛选平面内点
	void PlanePointsOptimize(std::vector<cv::Point3d>& pts, double distance, double plane[4]);
	//平面拟合，计算平面方程
	void ComputePlaneCoeff(const std::vector<cv::Point3d> &points, double plane[4]);

	void Mat44ToVec(const cv::Mat &mat44, double R[3], double T[3]);
	cv::Mat VecToMat44(const double R[3], const double T[3]) const;
};

