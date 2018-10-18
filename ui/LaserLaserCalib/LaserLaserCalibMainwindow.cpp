#include "LaserLaserCalibMainwindow.h"
#include <CameraCalibration/OmniCamera.h>
#include <CameraCalibration/PinholeCamera.h>
#include <CameraCalibration/FisheyeCamera.h>
#include <CornerFinder/CornerFinder.h>
#include <Common_GQ.h>
#include <QAction>
#include <QMenu>
#include <QHBoxLayout>
#include <QMenuBar>
#include <QFileDialog>
#include <QStatusBar>
#include <QKeyEvent>
using namespace std;
LaserLaserCalibMainwindow::LaserLaserCalibMainwindow(QWidget *parent/*=0*/) :QMainWindow(parent)
{
	InitUI();
}

LaserLaserCalibMainwindow::~LaserLaserCalibMainwindow()
{
}

void LaserLaserCalibMainwindow::InitUI()
{

	QWidget *widget = new QWidget(this);

	m_point_cloud_win_right = new PointCloudWindow(this);
	m_point_cloud_win_left = new PointCloudWindow(this);


	QHBoxLayout *h_layout = new QHBoxLayout;
	// 	h_layout->setContentsMargins(0, 0, 0, 0);
	h_layout->addWidget(m_point_cloud_win_left);
	h_layout->addWidget(m_point_cloud_win_right);
	h_layout->setStretch(0, 1);
	h_layout->setStretch(1, 1);
	widget->setLayout(h_layout);
	setCentralWidget(widget);

	QMenu *menu = menuBar()->addMenu("Project");
	connect(menu->addAction("Open"), SIGNAL(triggered()), SLOT(OpenProject()));
	connect(menu->addAction("Save"), SIGNAL(triggered()), SLOT(SaveProject()));

	menu = menuBar()->addMenu("View");

	QAction *act = menu->addAction("Last");
	act->setShortcut(Qt::Key_Up);
	connect(act, SIGNAL(triggered()), SLOT(LastFrame()));

	act = menu->addAction("Next");
	act->setShortcut(Qt::Key_Down);
	connect(act, SIGNAL(triggered()), SLOT(NextFrame()));

	menu = menuBar()->addMenu("Laser Calibration");
	connect(menu->addAction("LoadLaserBoardPoints"), SIGNAL(triggered()), SLOT(LoadLaserBoardPoints()));
	connect(menu->addAction("SaveLaserBoardPoints"), SIGNAL(triggered()), SLOT(SaveLaserBoardPoints()));
	connect(menu->addAction("Calibrate"), SIGNAL(triggered()), SLOT(CalibrateLaserWithLaser()));


	m_message_label = new QLabel(statusBar());
	m_message_label->setText("Current Index: N/A");
	m_message_label->setAlignment(Qt::AlignVCenter | Qt::AlignRight);
	statusBar()->addPermanentWidget(m_message_label);
	statusBar()->setStyleSheet(QString("QStatusBar::item{border: 0px}"));
	statusBar()->setSizeGripEnabled(true);
	statusBar()->showMessage("Please Open a project", 5);

	connect(this, SIGNAL(DisplayPointCloudRight(const POINTTYPE &)), m_point_cloud_win_right, SLOT(LoadPCLPointCloud(const POINTTYPE &)));
	connect(this, SIGNAL(DisplayColorPointCloudRight(const pcl::PointCloud<pcl::PointXYZRGB> &)), m_point_cloud_win_right, SLOT(LoadPCLRGBPointCloud(const pcl::PointCloud<pcl::PointXYZRGB> &)));
	connect(m_point_cloud_win_right, SIGNAL(SelectPoints(const std::vector<int> &)), this, SLOT(SelectPointsSlotRight(const std::vector<int> &)));

	connect(this, SIGNAL(DisplayPointCloudLeft(const POINTTYPE &)), m_point_cloud_win_left, SLOT(LoadPCLPointCloud(const POINTTYPE &)));
	connect(this, SIGNAL(DisplayColorPointCloudLeft(const pcl::PointCloud<pcl::PointXYZRGB> &)), m_point_cloud_win_left, SLOT(LoadPCLRGBPointCloud(const pcl::PointCloud<pcl::PointXYZRGB> &)));
	connect(m_point_cloud_win_left, SIGNAL(SelectPoints(const std::vector<int> &)), this, SLOT(SelectPointsSlotLeft(const std::vector<int> &)));

}

void LaserLaserCalibMainwindow::OpenProject()
{
	QString path = QFileDialog::getExistingDirectory(nullptr, "Project dir selection");
	if (!path.isEmpty())
	{
		if (CheckDirPath(path) > 0)
			InitProject();
	}
}

void LaserLaserCalibMainwindow::SaveProject()
{

}

void LaserLaserCalibMainwindow::NextFrame()
{
	if (m_calib_max_num == 0)
		return;
	DisplayCalibData(m_calib_index + 1);
}

void LaserLaserCalibMainwindow::LastFrame()
{
	if (m_calib_max_num == 0)
		return;
	DisplayCalibData(m_calib_index - 1);
}

void LaserLaserCalibMainwindow::LoadLaserBoardPoints()
{
	QString path = QFileDialog::getOpenFileName(nullptr, "Laser Board Points file select", QString(), "*.yaml");
	if (!path.isEmpty())
	{
		if(m_calib.LoadCalibResult(path.toLocal8Bit().constData()))
		{
			statusBar()->showMessage("LoadLaserBoardPoints Success!");
			DisplayCalibData(m_calib_index);
		}
		else
			statusBar()->showMessage("LoadLaserBoardPoints Fail!");
	}
}

void LaserLaserCalibMainwindow::SaveLaserBoardPoints()
{
	QString path = QFileDialog::getSaveFileName(nullptr, "Laser Board Points file select", QString(), "*.yaml");
	if (!path.isEmpty())
	{
		if (m_calib.SaveCalibResult(path.toLocal8Bit().constData()))
			statusBar()->showMessage("SaveLaserBoardPoints Success!");
		else
			statusBar()->showMessage("SaveLaserBoardPoints Fail!");
	}
}

void LaserLaserCalibMainwindow::CalibrateLaserWithLaser()
{
	if(m_calib.Calibrate())
		statusBar()->showMessage("CalibrateLaserWithCamera Success!");
	else
		statusBar()->showMessage("CalibrateLaserWithCamera Fail!");
	RefreshDisplay();
}


void LaserLaserCalibMainwindow::SelectPointsSlotRight(const std::vector<int> &indexs)
{
	m_calib.SetPlanePointsIndex(m_calib_index, indexs,false);
	if (indexs.size()==0)
	{
		RefreshDisplay();
	}
}

void LaserLaserCalibMainwindow::SelectPointsSlotLeft(const std::vector<int> &indexs)
{
	m_calib.SetPlanePointsIndex(m_calib_index, indexs,true);
	if (indexs.size() == 0)
	{
		RefreshDisplay();
	}

}

void LaserLaserCalibMainwindow::RefreshDisplay()
{
	DisplayCalibData(m_calib_index);
}

int LaserLaserCalibMainwindow::CheckDirPath(QString path)
{
	vector<string> list = Common_GQ::GetListFolders(path.toStdString()), temp_list;
	if (list.size() != 2)
	{
		return -1;
	}
	m_right_pcd_list = Common_GQ::GetListFiles(list[1], "pcd");
	m_left_pcd_list = Common_GQ::GetListFiles(list[0], "pcd");
	if (m_right_pcd_list.size() != m_left_pcd_list.size())
	{
		return -1;
	}
	return m_right_pcd_list.size();
}

void LaserLaserCalibMainwindow::InitProject()
{
	m_calib.Init(m_left_pcd_list, m_right_pcd_list);
	m_calib_max_num = m_right_pcd_list.size();
	m_corners.resize(m_calib_max_num);
	DisplayCalibData(0);
	m_point_cloud_win_right->ZoomGlobal();
	m_point_cloud_win_left->ZoomGlobal();
}

void LaserLaserCalibMainwindow::DisplayCalibData(int index)
{
	if (index >= m_calib_max_num)
		index = 0;
	if (index < 0)
		index = m_calib_max_num - 1;
	m_calib_index = index;
	QString disp;
	disp = QString(("Left: " + m_left_pcd_list[index] + "	 Right: " + m_right_pcd_list[index]).c_str());

	statusBar()->showMessage(disp);
	m_message_label->setText(" Current Index: " + QString::number(m_calib_index + 1) + '/' + QString::number(m_calib_max_num));
	
	POINTTYPE pc;
	pcl::io::loadPCDFile(m_right_pcd_list[m_calib_index], pc);

	vector<int> index_list = m_calib.GetPlanePoints(m_calib_index,false);
	if (index_list.size() > 0)
	{
		for (int i = 0; i < pc.size(); i++)
			pc.at(i).intensity = 0;
		for (int i = 0; i < index_list.size(); i++)
			pc.at(index_list[i]).intensity = 255;
	}
	emit DisplayPointCloudRight(pc);

	pcl::io::loadPCDFile(m_left_pcd_list[m_calib_index], pc);
	index_list = m_calib.GetPlanePoints(m_calib_index,true);
	if (index_list.size() > 0)
	{
		for (int i = 0; i < pc.size(); i++)
			pc.at(i).intensity = 0;
		for (int i = 0; i < index_list.size(); i++)
			pc.at(index_list[i]).intensity = 255;
	}
	emit DisplayPointCloudLeft(pc);

}
