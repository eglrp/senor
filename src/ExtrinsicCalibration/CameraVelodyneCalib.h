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
	// Qualifier: ���漤��ѡ���м��������ռ�У�����yaml
	// Parameter: const char * path
	//************************************
	bool SaveCalibResult(const char *path);

	//************************************
	// Method:    LoadCalibResult
	// FullName:  LadyBugVelodyneCalib::LoadCalibResult
	// Access:    public 
	// Returns:   bool
	// Qualifier: ��yaml��ȡ����ѡ���м��������ռ�У���
	// Parameter: const char * path
	//************************************
	bool LoadCalibResult(const char *path);

	//************************************
	// Method:    GetLaserPath
	// FullName:  LadyBugVelodyneCalib::GetLaserPath
	// Access:    public 
	// Returns:   std::string
	// Qualifier: const ��ȡָ���鼤��·��
	// Parameter: int index ���
	//************************************
	std::string GetLaserPath(int index) const ;
	//************************************
	// Method:    GetPlanePoints
	// FullName:  LadyBugVelodyneCalib::GetPlanePoints
	// Access:    public 
	// Returns:   std::vector<int>
	// Qualifier: const ��ȡָ���鼤��ѡ����
	// Parameter: int index ���
	//************************************
	std::vector<int> GetPlanePoints(int index) const;
	//************************************
	// Method:    SetPlanePointsIndex
	// FullName:  LadyBugVelodyneCalib::SetPlanePointsIndex
	// Access:    public 
	// Returns:   void
	// Qualifier: ����ָ���鼤��ѡ����
	// Parameter: int index ���
	// Parameter: const std::vector<int> & list ѡ��ĵ����±�����
	//************************************
	void SetPlanePointsIndex(int index,const std::vector<int> &list);

	//************************************
	// Method:    Calibrate
	// FullName:  LadyBugVelodyneCalib::Calibrate
	// Access:    public 
	// Returns:   bool
	// Qualifier: ��ʼ���м�У��ǰ���Ǽ���ѡ�㲽������
	//************************************
	bool Calibrate(double ransac_thre=0.01);

	bool GetCalibrated() const;
private:
	bool m_calibrated;

	std::vector<std::string> m_laser_list;							//����pcd·��
	std::vector<std::vector<int>> m_plane_points_list;            //����ѡ���ƽ��㼯��

	PlaneExtrinsicOptimization m_optimizer;
};

