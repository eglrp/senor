#pragma once
#include "PlaneExtrinsicOptimization.h"
#include <opencv2/opencv.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <CameraCalibration/CameraBase.h>
class CameraVelodyneCalib
{
public:
	CameraVelodyneCalib();
	~CameraVelodyneCalib(void);
	
	bool ComputeBoardPlanes(const std::vector<std::vector<cv::Point3f>> &objects_pts,const std::vector<cv::Vec3d> &rotation_vectors,const std::vector<cv::Vec3d> &translation_vectors);

	cv::Vec3d ReprojectImgToLaser(int index, const std::vector<cv::Point3f> &objects_pts,const cv::Vec3d &rotation_vector, const cv::Vec3d &translation_vector, pcl::PointCloud<pcl::PointXYZI> &pc);

	bool ReprojectLaserToImg(int index,cv::Mat &img,CameraBase *camera_model, pcl::PointCloud<pcl::PointXYZRGB> &pc);


	void Init(const std::vector<std::string> &path_list);

	//************************************
	// Method:    SaveCalibResult
	// FullName:  LadyBugVelodyneCalib::SaveCalibResult
	// Access:    public 
	// Returns:   bool
	// Qualifier: 保存激光选点中间结果和最终检校结果到yaml
	// Parameter: const char * path
	//************************************
	bool SaveCalibResult(const char *path);

	//************************************
	// Method:    LoadCalibResult
	// FullName:  LadyBugVelodyneCalib::LoadCalibResult
	// Access:    public 
	// Returns:   bool
	// Qualifier: 从yaml读取激光选点中间结果和最终检校结果
	// Parameter: const char * path
	//************************************
	bool LoadCalibResult(const char *path);

	//************************************
	// Method:    GetLaserPath
	// FullName:  LadyBugVelodyneCalib::GetLaserPath
	// Access:    public 
	// Returns:   std::string
	// Qualifier: const 获取指定组激光路径
	// Parameter: int index 组号
	//************************************
	std::string GetLaserPath(int index) const ;
	//************************************
	// Method:    GetPlanePoints
	// FullName:  LadyBugVelodyneCalib::GetPlanePoints
	// Access:    public 
	// Returns:   std::vector<int>
	// Qualifier: const 获取指定组激光选点结果
	// Parameter: int index 组号
	//************************************
	std::vector<int> GetPlanePoints(int index) const;
	//************************************
	// Method:    SetPlanePointsIndex
	// FullName:  LadyBugVelodyneCalib::SetPlanePointsIndex
	// Access:    public 
	// Returns:   void
	// Qualifier: 设置指定组激光选点结果
	// Parameter: int index 组号
	// Parameter: const std::vector<int> & list 选择的点云下标链表
	//************************************
	void SetPlanePointsIndex(int index,const std::vector<int> &list);

	//************************************
	// Method:    Calibrate
	// FullName:  LadyBugVelodyneCalib::Calibrate
	// Access:    public 
	// Returns:   bool
	// Qualifier: 开始进行检校，前提是激光选点步骤做完
	//************************************
	bool Calibrate(double ransac_thre=0.01);

	bool GetCalibrated() const;
private:
	bool m_calibrated;

	std::vector<std::string> m_laser_list;							//激光pcd路径
	std::vector<std::vector<int>> m_plane_points_list;            //激光选择的平面点集合

	PlaneExtrinsicOptimization m_optimizer;
};

