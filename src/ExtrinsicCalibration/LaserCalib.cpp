#include "LaserCalib.h"
#include "ceres/ceres.h"
#include "glog/logging.h"
#include <qstring.h>
#include <fstream>
#include <qfiledialog.h>
#include <QDir>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <QBoxLayout>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/ransac.h>
#include <qfile.h>
using namespace std;
LaserCalib::LaserCalib()
{
}


LaserCalib::~LaserCalib()
{
}

bool LaserCalib::Init(const std::vector<std::string> &left, const std::vector<std::string> &right)
{
	m_laser1_list = left;
	m_laser2_list = right;
	int count = m_laser2_list.size();
	m_plane_points_list_1.resize(count);
	m_plane_points_list_2.resize(count);
	return true;
}

std::string LaserCalib::GetLaserPath(int index, bool is_laser1/*=true*/) const
{
	return is_laser1 ? m_laser1_list[index] : m_laser2_list[index];
}


std::vector<int> LaserCalib::GetPlanePoints(int index, bool is_laser1/*=true*/) const
{
	return is_laser1 ? m_plane_points_list_1[index] : m_plane_points_list_2[index];
}

void LaserCalib::SetPlanePointsIndex(int index, const std::vector<int> &list, bool is_laser1/*=true*/)
{
	if (is_laser1)
	{
		m_plane_points_list_1[index] = list;
	}
	else
		m_plane_points_list_2[index] = list;
}

bool LaserCalib::Calibrate(double ransac_thre)
{
	if (m_laser1_list.size() != m_laser2_list.size() && m_plane_points_list_1.size() != m_plane_points_list_2.size() && m_laser1_list.size() != m_plane_points_list_1.size())
	{
		return false;
	}
	int count_all = m_laser1_list.size();
	std::vector<int> index_list;
	for (int i = 0; i < count_all; i++)
	{
		if (m_plane_points_list_1[i].size()>0&&m_plane_points_list_2.size()>0)
		{
			index_list.push_back(i);
		}
	}
	int count = index_list.size();
	m_calib.Init(count);
	cout << "Prepare Data..." << endl;
#pragma omp parallel for
	for (int i = 0; i < count; i++)
	{
		int index = index_list[i];
		pcl::PointCloud<pcl::PointXYZI> cloud1,cloud2;
		pcl::io::loadPCDFile(m_laser1_list[index], cloud1);
		pcl::io::loadPCDFile(m_laser2_list[index], cloud2);

		vector<cv::Point3d> pts(m_plane_points_list_1[index].size());

		for (size_t j = 0; j < pts.size(); j++)
		{
			pts[j].x = cloud1[m_plane_points_list_1[index][j]].x;
			pts[j].y = cloud1[m_plane_points_list_1[index][j]].y;
			pts[j].z = cloud1[m_plane_points_list_1[index][j]].z;
		}
		m_calib.SetSrcData(pts, i);

		pts.resize(m_plane_points_list_2[index].size());
		for (size_t j = 0; j < pts.size(); j++)
		{
			pts[j].x = cloud2[m_plane_points_list_2[index][j]].x;
			pts[j].y = cloud2[m_plane_points_list_2[index][j]].y;
			pts[j].z = cloud2[m_plane_points_list_2[index][j]].z;
		}
		m_calib.SetDstData(pts, i);
	}
	cout << "Done." << endl;
	return m_calib.Calibrate(ransac_thre, ransac_thre);
}

bool LaserCalib::SaveCalibResult(const char *path)
{
	cv::FileStorage fs(path, cv::FileStorage::WRITE);
	fs << "TransMatFromLaser2ToLaser1" << m_calib.GetExtrinsicMat();
	for (int i = 0; i < m_plane_points_list_1.size(); i++)
	{
		std::string name = "PlanePoints1_" + string(QString::number(i + 1).toLocal8Bit());
		fs << name << "[:";

		//写点到txt
		// 		pcl::PointCloud<pcl::PointXYZI> pcd;
		// 		pcl::io::loadPCDFile(m_laser_list[i], pcd);
		// 		fstream fout;
		// 
		// 		fout.open(QString::number(index_list[i]+1).toLocal8Bit+".txt", ios_base::out);

		for (int j = 0; j < m_plane_points_list_1[i].size(); j++)
		{
			fs << m_plane_points_list_1[i].at(j);

			// 			const pcl::PointXYZI &temp = pcd.at(m_plane_points_list[i].at(j));
			// 			fout << temp.x << ' ' << temp.y << ' ' << temp.z << endl;

		}
		fs << "]";
	}

	for (int i = 0; i < m_plane_points_list_2.size(); i++)
	{
		std::string name = "PlanePoints2_" + string(QString::number(i + 1).toLocal8Bit());
		fs << name << "[:";

		//写点到txt
		// 		pcl::PointCloud<pcl::PointXYZI> pcd;
		// 		pcl::io::loadPCDFile(m_laser_list[i], pcd);
		// 		fstream fout;
		// 
		// 		fout.open(QString::number(index_list[i]+1).toLocal8Bit+".txt", ios_base::out);

		for (int j = 0; j < m_plane_points_list_2[i].size(); j++)
		{
			fs << m_plane_points_list_2[i].at(j);

			// 			const pcl::PointXYZI &temp = pcd.at(m_plane_points_list[i].at(j));
			// 			fout << temp.x << ' ' << temp.y << ' ' << temp.z << endl;

		}
		fs << "]";
	}

	return true;

}

bool LaserCalib::LoadCalibResult(const char *path)
{
	cv::FileStorage fs(path, cv::FileStorage::READ);
	if (fs.isOpened() == false)
	{
		return false;
	}
	m_plane_points_list_1.clear();
	m_plane_points_list_2.clear();
	std::vector<int> list;
	for (int i = 0; i < m_laser1_list.size(); i++)
	{
		std::string name = "PlanePoints1_" + string(QString::number(i + 1).toLocal8Bit());
		fs[name] >> list;
		m_plane_points_list_1.push_back(list);

		name = "PlanePoints2_" + string(QString::number(i + 1).toLocal8Bit());
		fs[name] >> list;
		m_plane_points_list_2.push_back(list);

	}
	cv::Mat mat;
	fs["TransMatFromLaser2ToLaser1"] >> mat;
	if (!mat.empty())
		m_calib.SetExtrinsicMat(mat);
	return true;

}

bool LaserCalib::MergePointCloud(int index, pcl::PointCloud<pcl::PointXYZI> &pc)
{
	if (m_laser1_list.size() != m_laser2_list.size() && m_plane_points_list_1.size() != m_plane_points_list_2.size() && m_laser1_list.size() != m_plane_points_list_1.size())
	{
		return false;
	}
	cv::Mat m_trans_mat = m_calib.GetExtrinsicMat();
	if (m_trans_mat.at<double>(3, 3) != 1 || index<0 || index>m_laser1_list.size()-1)
	{
		return false;
	}

	pcl::PointCloud<pcl::PointXYZI> cloud1,cloud2;
	pcl::io::loadPCDFile(m_laser1_list[index], cloud1);
	pcl::io::loadPCDFile(m_laser2_list[index], cloud2);
	pc.clear();
	pcl::PointXYZI temp_p;
	cv::Mat mat(4, 1, CV_64FC1, 1.0);
	for (int i = 0; i < cloud1.size();i++)
	{
		temp_p = cloud1.at(i);
		temp_p.intensity = 0;
		pc.push_back(temp_p);
	}
	for (int i = 0; i < cloud2.size(); i++)
	{
		temp_p = cloud2.at(i);

		mat.at<double>(0, 0) = temp_p.x;
		mat.at<double>(1, 0) = temp_p.y;
		mat.at<double>(2, 0) = temp_p.z;

		mat = m_trans_mat*mat;

		temp_p.x = mat.at<double>(0, 0);
		temp_p.y = mat.at<double>(1, 0);
		temp_p.z = mat.at<double>(2, 0);

		temp_p.intensity = 255;

		pc.push_back(temp_p);
	}
	return true;
}
