#pragma once
#include "ccGLWindow.h"
#include <ccPointCloud.h>
#include <QShowEvent>
#include <QHideEvent>
#include <QColor>
#ifndef Q_MOC_RUN
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#endif

#define POINTTYPE pcl::PointCloud<pcl::PointXYZI>

#include <QMetaType>
Q_DECLARE_METATYPE(POINTTYPE)
Q_DECLARE_METATYPE(pcl::PointCloud<pcl::PointXYZRGB>)

class ccPointCloud;
class PointCloudWindow :public QWidget
{
	Q_OBJECT
public:
	PointCloudWindow(QWidget *parent = 0);
	~PointCloudWindow();

	void SetCamearaFollow(bool value);
	void ZoomGlobal();
	void setNeighbourRadius(double value);
	void SetRotationCentre(const CCVector3d &pt);
public slots:
	void LoadPCLPointCloud(const POINTTYPE &pc1);
	void LoadPCLRGBPointCloud(const pcl::PointCloud<pcl::PointXYZRGB> &color_pc);

	void ClearAll();
signals:
	void SelectPoints(const std::vector<int> &index_list);
private:
	ccHObject *m_root;
	ccPointCloud m_point_clouds;

	pcl::KdTreeFLANN<pcl::PointXYZ> m_kd_tree;

	double m_neb_radius;
	ccGLWindow *m_gl_win;
	bool m_camera_follow;
private:
	void PCLPointCloud2ccPointCloud(const POINTTYPE &pc, ccPointCloud *cc);
	void PCLRGBPointCloud2ccPointCloud(const pcl::PointCloud<pcl::PointXYZRGB> &pc, ccPointCloud *cc);
	void XYZ2ccPointCloud(double x, double y, double z, ccPointCloud *cc);
	int extractPlane(const Eigen::Vector3d &seed, std::vector<int> &index_list, double neighborradius);

private slots:
	void itemPickedSlot(ccHObject* entity, unsigned subEntityID, int x, int y, const CCVector3& P);

};

