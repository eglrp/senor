#pragma once
#include "CameraBase.h"
#define _USE_MATH_DEFINES
#include <math.h>

class OmniCamera :
	public CameraBase
{
public:
	OmniCamera();
	~OmniCamera();

	//************************************
	// Method:    GenerateOmniToPanoMap
	// FullName:  OmniCamera::GenerateOmniToPanoMap
	// Access:    public 
	// Returns:   bool
	// Qualifier: 生成去畸变后的全景图（存在镜像效果，但魏轶聪那边需要镜像全景图）
	// Parameter: cv::Mat & mapx
	// Parameter: cv::Mat & mapy
	// Parameter: int pano_width 全景长
	// Parameter: int pano_height 全景宽,应为长的一半
	// Parameter: double fov 垂直fov，180度内，默认127，手持设备的最佳值
	// Parameter: double radian_offset 水平初始视角的偏移量，默认为90度，这样用全景浏览器刚好0度代表手持设备的正方向
	//************************************
	bool GenerateOmniToPanoMap(cv::Mat &mapx, cv::Mat &mapy, int pano_width, int pano_height, double fov = 127.0, double radian_offset = M_PI / 2.0);
	
	//************************************
	// Method:    GeneratePanoToHexahedronMap
	// FullName:  OmniCamera::GeneratePanoToHexahedronMap
	// Access:    public 
	// Returns:   bool
	// Qualifier: 从全景图生成六面体图的映射表
	// Parameter: int pano_width 全景长
	// Parameter: int pano_height 全景宽
	// Parameter: bool type 六面体类型，true为3:2，false为6:1
	//************************************
	bool GeneratePanoToHexahedronMap(int pano_width, int pano_height, bool type=true);
	//************************************
	// Method:    RemapPanoToHexahedron
	// FullName:  OmniCamera::RemapPanoToHexahedron
	// Access:    public 
	// Returns:   bool
	// Qualifier: 映射全景图到六面体图上
	// Parameter: const cv::Mat & pano
	// Parameter: cv::Mat & output
	//************************************
	bool RemapPanoToHexahedron(const cv::Mat &pano, cv::Mat &output);
	// 投影指定3维点到球面全景图上
	//************************************
	// Method:    ProjectPointsToPano
	// FullName:  OmniCamera::ProjectPointsToPano
	// Access:    public 
	// Returns:   void
	// Qualifier: 投影指定点集到全景图上
	// Parameter: const std::vector<cv::Point3f> & pts_3d 三维点，即点在全景球坐标系下的三维坐标
	// Parameter: std::vector<cv::Point2f> & pts_2d 输出二维点
	// Parameter: int pano_width 全景宽
	// Parameter: int pano_height 全景高
	// Parameter: double radian_offset 水平初始视角的偏移量，默认为90度，这样用全景浏览器刚好0度代表手持设备的正方向
	//************************************
	void ProjectPointsToPano(const std::vector<cv::Point3f> &pts_3d, std::vector<cv::Point2f> &pts_2d, int pano_width, int pano_height,double radian_offset = M_PI / 2.0);

	bool LoadIntrinsicPara(const std::string &path) override;
	bool SaveIntrinsicPara(const std::string &path) override;

	double GetXi() const;

protected:
	void InheritInitUndistortRectifyMap(const cv::Mat &R, const cv::Mat &K, const cv::Size &size, cv::Mat &mapx, cv::Mat &mapy) override;
	void InheritProjectPoints(const std::vector<cv::Point3f> &pts_3d, const cv::InputArray &r_vec, const cv::InputArray &t_vec, std::vector<cv::Point2f> &pts_2d) override;
	void InheritUndistortPoints(const std::vector<cv::Point2f> &pts_src, std::vector<cv::Point2f> &pts_dst) override;
	double InheritCalibrate(const std::vector<std::vector<cv::Point3f>> objects, const std::vector<std::vector<cv::Point2f>> &corners,
		std::vector<cv::Vec3d> &rotation_vectors, std::vector<cv::Vec3d> &translation_vectors,
		cv::TermCriteria critia, cv::Mat &idx) override;
	bool InheritSovlePnP(const std::vector<cv::Point3f> &object_pts, const std::vector<cv::Point2f> &corners, cv::Mat &R, cv::Mat &T) override;

private:
	cv::Mat m_xi;

	cv::Mat m_hex_mapx;
	cv::Mat m_hex_mapy;

};

