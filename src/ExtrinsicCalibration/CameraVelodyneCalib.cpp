#include <QDebug>
#include "CameraVelodyneCalib.h"
#include <qstring.h>
#include "ladybug.h"
#include "ladybuggeom.h"
#include "ladybugrenderer.h"
#include <fstream>
#include <qfiledialog.h>
#include <QDir>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <QBoxLayout>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/ransac.h>
#include <omp.h>
#include <Common_GQ.h>
using namespace std;
CameraVelodyneCalib::CameraVelodyneCalib():m_calibrated(false)
{
}


CameraVelodyneCalib::~CameraVelodyneCalib()
{

}

bool CameraVelodyneCalib::ComputeBoardPlanes(const std::vector<std::vector<cv::Point3f>> &objects_pts, const std::vector<cv::Vec3d> &rotation_vectors, const std::vector<cv::Vec3d> &translation_vectors)
{
	int count = objects_pts.size();
	if (count!=rotation_vectors.size()||count!=translation_vectors.size())
	{
		return false;
	}
	for (int i = 0; i < count; i++)
	{
		int pts_count = objects_pts[i].size();
		std::vector<cv::Point3d> data;
		if (rotation_vectors[i][0] == 0 && rotation_vectors[i][1] == 0 && rotation_vectors[i][2] == 0)
			;
		else
		{
			data.resize(pts_count);
			for (int j = 0; j < pts_count; j++)
			{
				const cv::Point3f &p = objects_pts[i][j];
				cv::Vec3d pt_cam(p.x, p.y, p.z);
				pt_cam = rotation_vectors[i] * pt_cam + translation_vectors[i];
				pt_cam /= 1000.0;            //毫米转米
				data[j].x = pt_cam[0];
				data[j].y = pt_cam[1];
				data[j].z = pt_cam[2];
			}
		}
		m_optimizer.SetSrcData(data, i);
	}
	return true;
}

//************************************
// Method:    ReprojectImgToLaser
// FullName:  CameraVelodyneCalib::ReprojectImgToLaser
// Access:    public 
// Returns:   cv::Vec3d
// Qualifier: 重投影图片到激光
// Parameter: int index---激光ID
// Parameter: const std::vector<cv::Point3f> & objects_pts
// Parameter: const cv::Vec3d & rotation_vector
// Parameter: const cv::Vec3d & translation_vector
// Parameter: pcl::PointCloud<pcl::PointXYZI> & pc
//************************************
cv::Vec3d CameraVelodyneCalib::ReprojectImgToLaser(int index, const std::vector<cv::Point3f> &objects_pts, const cv::Vec3d &rotation_vector, const cv::Vec3d &translation_vector, pcl::PointCloud<pcl::PointXYZI> &pc)
{
	if (!m_calibrated)
	{
		return false;
	}
	pc.clear();
	pcl::io::loadPCDFile(m_laser_list[index], pc);
	for (size_t i = 0; i < pc.size(); i++)
	{
		pc[i].intensity = 0;
	}
	Eigen::Matrix4d mat = Common_GQ::toMatrix4d(m_optimizer.GetExtrinsicMat().inv());
	Eigen::Vector4d vec(0, 0, 0, 1);
	pcl::PointXYZI p;
	p.intensity = 255;
	cv::Vec3d vec_3d;
	for (size_t i = 0; i < objects_pts.size(); i++)
	{
		const cv::Point3f & pt = objects_pts[i];
		vec_3d[0] = pt.x;
		vec_3d[1] = pt.y;
		vec_3d[2] = pt.z;
		vec_3d = rotation_vector*vec_3d + translation_vector;
		vec_3d /= 1000.0;
		vec[0] = vec_3d[0];
		vec[1] = vec_3d[1];
		vec[2] = vec_3d[2];
		vec = mat*vec;
		p.x = vec[0];
		p.y = vec[1];
		p.z = vec[2];
		pc.push_back(p);
	}
	return true;
}

//************************************
// Method:    ReprojectLaserToImg
// FullName:  CameraVelodyneCalib::ReprojectLaserToImg
// Access:    public 
// Returns:   bool
// Qualifier: 重投影激光到图片
// Parameter: int index
// Parameter: cv::Mat & img---图片
// Parameter: CameraBase * camera_model
// Parameter: pcl::PointCloud<pcl::PointXYZRGB> & pc
//************************************
bool CameraVelodyneCalib::ReprojectLaserToImg(int index, cv::Mat &img, CameraBase *camera_model, pcl::PointCloud<pcl::PointXYZRGB>& pc)
{
	if (!m_calibrated)
	{
		return false;
	}

	pc.clear();
	pcl::io::loadPCDFile(m_laser_list[index], pc);
	vector<cv::Point3f> pts_3d;
	vector<cv::Point2f> pts_2d;
	for (size_t i = 0; i < pc.size(); i++)
	{
		const pcl::PointXYZRGB &p = pc[i];
		pts_3d.push_back(cv::Point3f(p.x, p.y, p.z));
	}
	cout<<"外参：" << m_optimizer.GetExtrinsicMat()<<endl;
	camera_model->ProjectPoints(pts_3d, m_optimizer.GetExtrinsicMat(), pts_2d);
	
	// pack r/g/b into rgb
	uint8_t r = 0, g = 0, b = 0;    // Example: Red color

	for (size_t i = 0; i < pts_2d.size(); i++)
	{
		const cv::Vec3b &bgr_vec = CameraBase::GetBGRImgPixel(img, pts_2d[i]);
		r = bgr_vec[2];
		g = bgr_vec[1];
		b = bgr_vec[0];
		uint32_t rgb = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);
		pc[i].rgb = *reinterpret_cast<float*>(&rgb);
	}
	for (size_t i = 0; i < pts_2d.size(); i++)
		cv::circle(img, pts_2d[i], 1, CV_RGB(255, 0, 0));
	return true;
}

void CameraVelodyneCalib::Init(const std::vector<std::string> &path_list)
{
	m_laser_list = path_list;
	m_plane_points_list.resize(m_laser_list.size());
	m_calibrated = false;
	m_optimizer.Init(m_laser_list.size());
}

bool CameraVelodyneCalib::SaveCalibResult(const char *path)
{
	cv::FileStorage fs(path, cv::FileStorage::WRITE);
	fs << "TransMatFromLaserToPano" << m_optimizer.GetExtrinsicMat();
	for (int i = 0; i < m_plane_points_list.size();i++)
	{
		std::string name = "PlanePoints" + string(QString::number(i+1).toLocal8Bit());
		fs << name << "[:";
		for (int j = 0; j < m_plane_points_list[i].size();j++)
			fs << m_plane_points_list[i].at(j);
		fs << "]";
	}
	return true;
}

//************************************
// Method:    LoadCalibResult
// FullName:  CameraVelodyneCalib::LoadCalibResult
// Access:    public 
// Returns:   bool
// Qualifier:从文件中加载平面信息
// Parameter: const char * path---文件路径
//************************************
bool CameraVelodyneCalib::LoadCalibResult(const char *path)
{
	cv::FileStorage fs(path, cv::FileStorage::READ);
	if (fs.isOpened()==false)
	{
		return false;
	}
	m_plane_points_list.clear();
	//点云数据的ID
	std::vector<int> list;
	for (int i = 0; i < m_laser_list.size(); i++)
	{
		std::string name = "PlanePoints" + string(QString::number(i + 1).toLocal8Bit());
		fs[name] >> list;
		m_plane_points_list.push_back(list);
	}
	//RT为空
	cv::Mat RT;
	fs["TransMatFromLaserToPano"] >> RT;
	m_optimizer.SetExtrinsicMat(RT);
	m_calibrated = true;
	return true;
}


std::string CameraVelodyneCalib::GetLaserPath(int index) const
{
	return m_laser_list[index];
}

std::vector<int> CameraVelodyneCalib::GetPlanePoints(int index) const
{
	return m_plane_points_list[index];
}

void CameraVelodyneCalib::SetPlanePointsIndex(int index, const std::vector<int> &list)
{
	m_plane_points_list[index] = list;
}

bool CameraVelodyneCalib::Calibrate(double ransac_thre)
{
	if (m_plane_points_list.size()!=m_laser_list.size())
	{
		return false;
	}
	int count = m_plane_points_list.size();
	pcl::PointCloud<pcl::PointXYZI> cloud;
	
	for (int i = 0; i < count; i++)
	{
		int index = i;	
		vector<cv::Point3d> plane_pts(m_plane_points_list[i].size());
		pcl::io::loadPCDFile(m_laser_list[index], cloud);
		for (int j = 0;j < m_plane_points_list[i].size(); j++)
		{
			const pcl::PointXYZI &p = cloud.at(m_plane_points_list[i][j]);
			plane_pts[j] = cv::Point3d(p.x, p.y, p.z);
		}
		m_optimizer.SetDstData(plane_pts, i);
	}
	
	bool r= m_optimizer.Calibrate(0, ransac_thre);
	if (r)
	{
		m_calibrated = m_optimizer.GetCalibrated();
	}
	return r;
}

bool CameraVelodyneCalib::GetCalibrated() const
{
	return m_calibrated;
}


