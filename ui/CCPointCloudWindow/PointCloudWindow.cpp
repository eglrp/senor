#include "PointCloudWindow.h"
#include "ccPointCloud.h"
#include <ScalarField.h>
#include <qdebug.h>
#include <pcl/common/time.h>
#include <QDateTime>
#include <QHBoxLayout>
#include <QTimer>
#include <QUdpSocket>
#include <opencv2/opencv.hpp>
PointCloudWindow::PointCloudWindow(QWidget *parent) :QWidget(parent), m_camera_follow(true),m_neb_radius(0.5)
{
	qRegisterMetaType<POINTTYPE>("pcl::PointCloud<pcl::PointXYZI>");
	qRegisterMetaType<POINTTYPE>("pcl::PointCloud<pcl::PointXYZRGB>");

	m_gl_win = new ccGLWindow(this);

	QHBoxLayout *layout = new QHBoxLayout(this);
	layout->setContentsMargins(0, 0, 0, 0);
	layout->addWidget(m_gl_win);

	m_root = new ccHObject;
	m_gl_win->setSceneDB(m_root);
	m_root->setDisplay_recursive(m_gl_win);
	m_root->addChild(&m_point_clouds);

	m_gl_win->setPickingMode(ccGLWindow::POINT_PICKING);
	m_gl_win->setRectangularPickingAllowed(false);
	ccGui::ParamStruct para = ccGui::Parameters();
	para.autoComputeOctree = ccGui::ParamStruct::NEVER;
	m_gl_win->setDisplayParameters(para);
	connect(m_gl_win, SIGNAL(itemPicked(ccHObject* , unsigned , int, int, const CCVector3& )),this,SLOT(itemPickedSlot(ccHObject*, unsigned, int, int, const CCVector3&)));
}


PointCloudWindow::~PointCloudWindow()
{
	m_gl_win->setSceneDB(NULL);
	delete m_gl_win;
	delete m_root;
	m_root = nullptr;
}

void PointCloudWindow::SetCamearaFollow(bool value)
{
	m_camera_follow = value;
}

void PointCloudWindow::ClearAll()
{
	m_point_clouds.clear();
	m_gl_win->redraw();
}

void PointCloudWindow::ZoomGlobal()
{
	m_gl_win->zoomGlobal();
}

void PointCloudWindow::setNeighbourRadius(double value)
{
	m_neb_radius = value;
}

void PointCloudWindow::SetRotationCentre(const CCVector3d &pt)
{
	m_gl_win->setPivotPoint(pt);
}

void PointCloudWindow::LoadPCLPointCloud(const POINTTYPE &pc1)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr pc_xyz(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::copyPointCloud(pc1, *pc_xyz);
	m_kd_tree.setInputCloud(pc_xyz);
	PCLPointCloud2ccPointCloud(pc1, &m_point_clouds);
	m_gl_win->redraw();
}


void PointCloudWindow::LoadPCLRGBPointCloud(const pcl::PointCloud<pcl::PointXYZRGB> &color_pc)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr pc_xyz(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::copyPointCloud(color_pc, *pc_xyz);
	m_kd_tree.setInputCloud(pc_xyz);
	PCLRGBPointCloud2ccPointCloud(color_pc, &m_point_clouds);
	m_gl_win->redraw();

}

void PointCloudWindow::PCLPointCloud2ccPointCloud(const POINTTYPE &pc, ccPointCloud *cc)
{
	unsigned int pc_size = pc.size();
	cc->clear();
	cc->deleteAllScalarFields();
	cc->reserve(pc_size);
	cc->showColors(false);
	cc->enableScalarField();
	size_t count = 0;
	for (auto temp : pc)
	{
		cc->addPoint(CCVector3(temp.x, temp.y, temp.z));
		ScalarType d = static_cast<ScalarType>(temp.intensity);
		cc->setPointScalarValue(count++, d);
	}
	CCLib::ScalarField* sf = cc->getCurrentInScalarField();
	sf->computeMinAndMax();
	cc->setCurrentDisplayedScalarField(cc->getCurrentInScalarFieldIndex());
	cc->showSF(true);
// 	cc->enableTempColor(true);
}

void PointCloudWindow::PCLRGBPointCloud2ccPointCloud(const pcl::PointCloud<pcl::PointXYZRGB> &pc, ccPointCloud *cc)
{
	unsigned int pc_size = pc.size();
	cc->clear();
	cc->deleteAllScalarFields();
	cc->reserve(pc_size);
	cc->reserveTheRGBTable();
	cc->showColors(true);
	for (auto temp : pc)
	{
		cc->addPoint(CCVector3(temp.x, temp.y, temp.z));
		uint32_t rgb = *reinterpret_cast<int*>(&temp.rgb);
		uint8_t r = (rgb >> 16) & 0x0000ff;
		uint8_t g = (rgb >> 8) & 0x0000ff;
		uint8_t b = (rgb) & 0x0000ff;
		cc->addRGBColor(r,g,b);
	}
}

void PointCloudWindow::XYZ2ccPointCloud(double x, double y, double z, ccPointCloud *cc)
{
	cc->reserve(cc->size() + 1);
	cc->addPoint(CCVector3(x, y, z));
// 	cc->enableTempColor(true);
}

int PointCloudWindow::extractPlane(const Eigen::Vector3d &seed, std::vector<int> &index_list, double neighborradius)
{
	index_list.clear();
	//æ‡¿Î
	std::vector<float> k_sqr_distances;
	pcl::PointXYZI tmpseed;
	tmpseed.x = float(seed(0)); tmpseed.y = float(seed(1)); tmpseed.z = float(seed(2));
	return m_kd_tree.radiusSearchT(tmpseed, neighborradius, index_list, k_sqr_distances);
}

void PointCloudWindow::itemPickedSlot(ccHObject* entity, unsigned subEntityID, int x, int y, const CCVector3& P)
{
	if (entity&&entity->isKindOf(CC_TYPES::POINT_CLOUD))
	{
		ccPointCloud* cloud = static_cast<ccPointCloud*>(entity);
		if (!cloud)
		{
			assert(false);
			ccLog::Warning("[Item picking] Picked point is not in pickable entities DB?!");
			return;
		}
		int count = m_point_clouds.size();
		if (count>0)
		{
			Eigen::Vector3d seed(P.x, P.y, P.z);
			std::vector<int> index_list;
			Eigen::Matrix3d mat;
			extractPlane(seed, index_list, m_neb_radius);
			emit SelectPoints(index_list);
			pcl::PointCloud<pcl::PointXYZI> pc;
			pcl::PointXYZI temp;
			const CCVector3 *data;
			for (int i = 0; i < count; i++)
			{
				data = m_point_clouds.getPoint(i);
				temp.x = data->x;
				temp.y = data->y;
				temp.z = data->z;
				temp.intensity = 0;
				pc.push_back(temp);
			}
			for (int i = 0; i < index_list.size(); i++)
			{
				pc.at(index_list[i]).intensity = 255;
			}
			LoadPCLPointCloud(pc);
		}
	}
	else
	{
		emit SelectPoints(std::vector<int>());
	}
}
