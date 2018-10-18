#include "LaserCameraCalibMainwindow.h"
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
#include <QPushButton>
#include <QLineEdit>
#include <QKeyEvent>

using namespace std;
LasesLaserCalibMainwindow::LasesLaserCalibMainwindow(QWidget *parent/*=0*/) :QMainWindow(parent), m_camera(nullptr)
{
	InitUI();
}

LasesLaserCalibMainwindow::~LasesLaserCalibMainwindow()
{
	if (m_camera)
	{
		delete m_camera;
		m_camera = nullptr;
	}
}

void LasesLaserCalibMainwindow::InitUI()
{

	QWidget *widget = new QWidget(this);
	m_img_win = new ImgView(this);
	m_point_cloud_win = new PointCloudWindow(this);

	QHBoxLayout *h_layout = new QHBoxLayout;
	// 	h_layout->setContentsMargins(0, 0, 0, 0);
	h_layout->addWidget(m_img_win);
	h_layout->addWidget(m_point_cloud_win);
	h_layout->setStretch(0, 1);
	h_layout->setStretch(1, 1);
	widget->setLayout(h_layout);
	setCentralWidget(widget);

	QMenu *menu = menuBar()->addMenu("Project");
	connect(menu->addAction("Open"), SIGNAL(triggered()), SLOT(OpenProject()));
	connect(menu->addAction("Save"), SIGNAL(triggered()), SLOT(SaveProject()));

	menu = menuBar()->addMenu("View");

	m_jump_invalid = menu->addAction("Jump invalid");
	m_jump_invalid->setCheckable(true);
	m_jump_invalid->setChecked(true);

	QAction *act = menu->addAction("Last");
	act->setShortcut(Qt::Key_Up);
	connect(act, SIGNAL(triggered()), SLOT(LastFrame()));

	act = menu->addAction("Next");
	act->setShortcut(Qt::Key_Down);
	connect(act, SIGNAL(triggered()), SLOT(NextFrame()));

	m_reproject_laser2img = menu->addAction("ReprojectLaser2Image");
	m_reproject_laser2img->setCheckable(true);
	m_reproject_laser2img->setChecked(false);
	connect(m_reproject_laser2img, SIGNAL(triggered()), SLOT(RefreshDisplay()));

	m_reproject_board2laser = menu->addAction("ReprojectBoardToLaser");
	m_reproject_board2laser->setCheckable(true);
	m_reproject_board2laser->setChecked(false);
	connect(m_reproject_board2laser, SIGNAL(triggered()), SLOT(RefreshDisplay()));
	m_show_color_pc = menu->addAction("ColorizePointCloud");
	m_show_color_pc->setCheckable(true);
	m_show_color_pc->setChecked(false);
	connect(m_show_color_pc, SIGNAL(triggered()), SLOT(RefreshDisplay()));

	menu = menuBar()->addMenu("Camera Type");
	m_camera_type = new QActionGroup(this);
	act = menu->addAction("Pinhole");
	act->setCheckable(true);
	act->setChecked(true);
	m_camera_type->addAction(act);

	act = menu->addAction("Fisheye");
	act->setCheckable(true);
	act->setChecked(false);
	m_camera_type->addAction(act);

	act = menu->addAction("Omni");
	act->setCheckable(true);
	act->setChecked(false);
	m_camera_type->addAction(act);

	menu = menuBar()->addMenu("Camera Calibration");
	connect(menu->addAction("FindCorners"), SIGNAL(triggered()), SLOT(FindCorners()));
	connect(menu->addAction("LoadCornersFromMatlab"), SIGNAL(triggered()), SLOT(LoadCornersFromMatlab()));
	connect(menu->addAction("LoadIntrinsicPara"), SIGNAL(triggered()), SLOT(LoadIntrinsicPara()));
	connect(menu->addAction("SaveIntrinsicPara"), SIGNAL(triggered()), SLOT(SaveIntrinsicPara()));
	connect(menu->addAction("LoadExtrinsicPara"), SIGNAL(triggered()), SLOT(LoadExtrinsicPara()));
	connect(menu->addAction("SaveExtrinsicPara"), SIGNAL(triggered()), SLOT(SaveExtrinsicPara()));
	connect(menu->addAction("Calibrate"), SIGNAL(triggered()), SLOT(CalibrateCamera()));

	menu = menuBar()->addMenu("Laser Calibration");
	connect(menu->addAction("LoadLaserBoardPoints"), SIGNAL(triggered()), SLOT(LoadLaserBoardPoints()));
	connect(menu->addAction("SaveLaserBoardPoints"), SIGNAL(triggered()), SLOT(SaveLaserBoardPoints()));
	connect(menu->addAction("Calibrate"), SIGNAL(triggered()), SLOT(CalibrateLaserWithCamera()));


	m_message_label = new QLabel(statusBar());
	m_message_label->setText("Current Index: N/A");
	m_message_label->setAlignment(Qt::AlignVCenter | Qt::AlignRight);
	statusBar()->addPermanentWidget(m_message_label);
	statusBar()->setStyleSheet(QString("QStatusBar::item{border: 0px}"));
	statusBar()->setSizeGripEnabled(true);
	statusBar()->showMessage("Please Open a project", 5);

	connect(this, SIGNAL(DisplayPointCloud(const POINTTYPE &)), m_point_cloud_win, SLOT(LoadPCLPointCloud(const POINTTYPE &)));
	connect(this, SIGNAL(DisplayColorPointCloud(const pcl::PointCloud<pcl::PointXYZRGB> &)), m_point_cloud_win, SLOT(LoadPCLRGBPointCloud(const pcl::PointCloud<pcl::PointXYZRGB> &)));
	connect(m_point_cloud_win, SIGNAL(SelectPoints(const std::vector<int> &)), this, SLOT(SelectPointsSlot(const std::vector<int> &)));

// 	pcl::PointCloud<pcl::PointXYZRGB> pc;
// 	pcl::io::loadPCDFile("D:/Data/LadyBug_Velodyne/Velodyne-1.pcd", pc);
// 	emit DisplayColorPointCloud(pc);
}

void LasesLaserCalibMainwindow::OpenProject()
{
	QString path = QFileDialog::getExistingDirectory(nullptr, "Project dir selection");
	if (!path.isEmpty())
	{
		if (CheckDirPath(path) > 0)
			InitProject();
	}
}

void LasesLaserCalibMainwindow::SaveProject()
{
	 
}

void LasesLaserCalibMainwindow::NextFrame()
{
	if (m_calib_max_num == 0)
		return;
	if (!m_jump_invalid->isChecked())
		DisplayCalibData(m_calib_index + 1);
	else
	{
		int x = m_calib_index + 1;;
		while (true)
		{
			if (x>m_calib_max_num-1)
			{
				x = 0;
			}
			if (m_pattern_size.width == 0)
			{
				DisplayCalibData(x);
				return;
			}
			if (m_camera&&m_camera->GetCalibrated() && m_reproject_errors.size() > 0)
			{
				if (m_reproject_errors[x] > 0)
				{
					DisplayCalibData(x);
					return;
				}
			}
			else
			{
				if (m_corners[x].size() > 0)
				{
					DisplayCalibData(x);
					return;
				}
			}
			x++;
		}
	}
}

void LasesLaserCalibMainwindow::LastFrame()
{
	if (m_calib_max_num == 0)
		return;

	if (!m_jump_invalid->isChecked())
		DisplayCalibData(m_calib_index - 1);
	else
	{
		int x = m_calib_index - 1;;
		while (true)
		{
			if (x < 0)
			{
				x = m_calib_max_num - 1;
			}
			if (m_pattern_size.width == 0)
			{
				DisplayCalibData(x);
				return;
			}
			if (m_camera&&m_camera->GetCalibrated() && m_reproject_errors.size() > 0)
			{
				if (m_reproject_errors[x] > 0)
				{
					DisplayCalibData(x);
					return;
				}
			}
			else
			{
				if (m_corners[x].size() > 0)
				{
					DisplayCalibData(x);
					return;
				}
			}
			x--;
		}
	}
}

void LasesLaserCalibMainwindow::FindCornersSlot() {

	m_fov = edit_fov->text().toDouble();
	m_square_length = edit_length->text().toDouble();
	m_pattern_size = cv::Size(edit_size_W->text().toInt(), edit_size_H->text().toInt());
	m_remove_worst_group_count = edit_remove->text().toInt();
	
	//关闭对话框
	setting->close();

	if (m_png_list.size() > 0)
	{
		//检测角点
		CornerFinder finder;
		for (int i = 0; i < m_png_list.size(); i++) {
			cv::Mat img;
			img = cv::imread(m_png_list[i]);
			//存储角点
			vector<cv::Point2f> corner;
			finder.FindCornersOfAutoFixMissingCorners(img, corner);
			if (corner.size() == m_pattern_size.width*m_pattern_size.height)
			{
				m_corners[i] = corner;
			}

			cout << "第" << i + 1 << "张图片检测的角点：" << corner.size() << endl;
		}
		statusBar()->showMessage("FindCorners Success!");
	}
	else
	{
		statusBar()->showMessage("FindCorners Fail!");
		m_corners.clear();
	}

}

void LasesLaserCalibMainwindow::FindCorners()
{

	setting = new QDialog(this);
	setting->setWindowTitle("SettingPara");
	setting->setMinimumSize(QSize(300, 200));
	setting->setMaximumSize(QSize(300, 200));
	setting->setSizeGripEnabled(false);

	//相机视场角
	QLabel *fov = new QLabel(setting);
	fov->setText("Fov(Degree):");
	fov->setGeometry(30, 10, 100, 30);

	edit_fov = new QLineEdit(setting);
	edit_fov->setGeometry(120, 15, 100, 20);
	edit_fov->setText("75");

	//棋盘格尺寸
	QLabel *board_size = new QLabel(setting);
	board_size->setText("Height/Width:");
	board_size->setGeometry(30, 50, 100, 30);

	edit_size_H = new QLineEdit(setting);
	edit_size_H->setGeometry(120, 55, 45, 20);
	edit_size_H->setText("9");


	edit_size_W = new QLineEdit(setting);
	edit_size_W->setGeometry(175, 55, 45, 20);
	edit_size_W->setText("8");

	//棋盘格边长
	QLabel *board_length = new QLabel(setting);
	board_length->setText("Length(mm):");
	board_length->setGeometry(30, 90, 100, 30);

	edit_length = new QLineEdit(setting);
	edit_length->setGeometry(120, 95, 100, 20);
	edit_length->setText("100");

	//丢弃分组remove_worst_group_count
	QLabel *remove_group = new QLabel(setting);
	remove_group->setText("RemoveGroup:");
	remove_group->setGeometry(30, 130, 100, 30);

	 edit_remove = new QLineEdit(setting);
	edit_remove->setGeometry(120, 135, 100, 20);
	edit_remove->setText("2");


	QPushButton *m_ok = new QPushButton(setting);
	m_ok->setGeometry(200, 165, 50, 20);
	m_ok->setText("OK");

	QPushButton *m_cancel = new QPushButton(setting);
	m_cancel->setGeometry(40, 165, 50, 20);
	m_cancel->setText("Cancel");

	connect(m_ok, SIGNAL(clicked()), this, SLOT(FindCornersSlot()));
	connect(m_cancel, SIGNAL(clicked()), setting, SLOT(close()));
	
	setting->exec();
	
}

void LasesLaserCalibMainwindow::LoadCornersFromMatlab()
{
	QString path = QFileDialog::getOpenFileName(nullptr, "Corners file select", QString(), "*.yaml");
	if (!path.isEmpty())
	{
		if (CornerFinder::ReadCornersFromMatlab(path.toLocal8Bit().constData(), m_corners, m_pattern_size))
		{
			statusBar()->showMessage("LoadCorners Success!");
		}
		else
		{
			statusBar()->showMessage("LoadCorners Fail!");
			m_corners.clear();
		}
	}

}

void LasesLaserCalibMainwindow::LoadLaserBoardPoints()
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

void LasesLaserCalibMainwindow::SaveLaserBoardPoints()
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

void LasesLaserCalibMainwindow::CalibrateCamera()
{
	if (m_camera)
	{
		delete m_camera;
	}
	if (m_camera_type->checkedAction()->text() == "Fisheye")
	{
		m_camera = new FisheyeCamera;
	}
	else if (m_camera_type->checkedAction()->text() == "Pinhole")
	{
		m_camera = new PinholeCamera;
	}
	else
	{
		m_camera = new OmniCamera;
	}
	QSize size = m_img_win->ImgSize();
	m_camera->SetRawSize(cv::Size(size.width(), size.height()));

	//计算棋盘格角点的世界坐标系下的三维坐标
	CornerFinder::CalcChessboardCorners(m_pattern_size, m_square_length, m_corners.size(), m_objects_pts);
	
	double rms = m_camera->Calibrate(m_objects_pts, m_corners, m_pattern_size, m_square_length, m_fov, m_remove_worst_group_count, m_rotation_vectors, m_translation_vectors, m_reproject_errors);
	if (rms != 0)
	{
		statusBar()->showMessage("Calibrate Success with RMS " + QString::number(rms) + " !");
	}
	else
	{
		statusBar()->showMessage("Calibrate Fail!");
		delete m_camera;
		m_camera = nullptr;
	}
}

void LasesLaserCalibMainwindow::CalibrateLaserWithCamera()
{
	if (m_calib.ComputeBoardPlanes(m_objects_pts, m_rotation_vectors, m_translation_vectors)&&m_calib.Calibrate())
		statusBar()->showMessage("CalibrateLaserWithCamera Success!");
	else
		statusBar()->showMessage("CalibrateLaserWithCamera Fail!");
}

void LasesLaserCalibMainwindow::SelectPointsSlot(const std::vector<int> &indexs)
{
	m_calib.SetPlanePointsIndex(m_calib_index, indexs);
	if (indexs.size()==0)
	{
		RefreshDisplay();
	}
}

void LasesLaserCalibMainwindow::RefreshDisplay()
{
	DisplayCalibData(m_calib_index);
}

void LasesLaserCalibMainwindow::LoadIntrinsicPara()
{
	QString path = QFileDialog::getOpenFileName(nullptr, "Image Intrinsic Para file select", QString(), "*.yaml");
	if (!path.isEmpty())
	{
		if (m_camera)
			delete m_camera;
		if (m_camera_type->checkedAction()->text() == "Fisheye")
			m_camera = new FisheyeCamera;
		else if (m_camera_type->checkedAction()->text() == "Pinhole")
			m_camera = new PinholeCamera;
		else
			m_camera = new OmniCamera;
		if (m_camera->LoadIntrinsicPara(path.toLocal8Bit().constData()))
			statusBar()->showMessage("LoadIntrinsicPara Success!");
		else
			statusBar()->showMessage("LoadIntrinsicPara Fail!");
	}

}

void LasesLaserCalibMainwindow::SaveIntrinsicPara()
{
	if (!m_camera)
	{
		statusBar()->showMessage("SaveIntrinsicPara Fail!");
		return;
	}
	QString path = QFileDialog::getSaveFileName(nullptr, "Image Intrinsic Para file select", QString(), "*.yaml");
	if (!path.isEmpty())
	{
		if (m_camera->SaveIntrinsicPara(path.toLocal8Bit().constData()))
			statusBar()->showMessage("SaveIntrinsicPara Success!");
		else
			statusBar()->showMessage("SaveIntrinsicPara Fail!");
	}
}

void LasesLaserCalibMainwindow::SaveExtrinsicPara()
{
	QString path = QFileDialog::getSaveFileName(nullptr, "Image Extrinsic Para file select", QString(), "*.yaml");
	if (!path.isEmpty())
	{
		if (CameraBase::SaveExtrinsicPara(path.toLocal8Bit().constData(),m_rotation_vectors,m_translation_vectors))
			statusBar()->showMessage("SaveExtrinsicPara Success!");
		else
			statusBar()->showMessage("SaveExtrinsicPara Fail!");
	}
}

void LasesLaserCalibMainwindow::LoadExtrinsicPara()
{
	if (!m_camera)
		return;
	QString path = QFileDialog::getOpenFileName(nullptr, "Image Extrinsic Para file select", QString(), "*.yaml");
	if (!path.isEmpty())
	{
		if (CameraBase::LoadExtrinsicPara(path.toLocal8Bit().constData(),m_rotation_vectors,m_translation_vectors))
		{
			QSize size = m_img_win->ImgSize();
			m_camera->SetRawSize(cv::Size(size.width(), size.height()));
			float square_length = 100;
			CornerFinder::CalcChessboardCorners(m_pattern_size, square_length, m_corners.size(), m_objects_pts);
			m_reproject_errors.resize(m_corners.size());
#pragma omp parallel for
			for (int i = 0; i < m_corners.size(); i++)
			{
				vector<cv::Point2f> pts;
				if (m_translation_vectors[i]==cv::Vec3d(0,0,0))
				{
					m_reproject_errors[i] = 0;
				}
				else
				{
					m_camera->ProjectPoints(m_objects_pts[i], m_rotation_vectors[i], m_translation_vectors[i], pts);
					m_reproject_errors[i] = CameraBase::CalcAvgPixelDistance(pts, m_corners[i]);
				}
			}
			statusBar()->showMessage("LoadExtrinsicPara Success!");
		}
		else
			statusBar()->showMessage("LoadExtrinsicPara Fail!");
	}
}

int LasesLaserCalibMainwindow::CheckDirPath(QString path)
{
	vector<string> list = Common_GQ::GetListFolders(path.toStdString()), temp_list;
	if (list.size() != 2)
	{
		return -1;
	}
	temp_list = Common_GQ::GetListFiles(list[0], "pcd");
	if (temp_list.size() > 0)
	{
		m_pcd_list = temp_list;
		m_png_list = Common_GQ::GetListFiles(list[1], "png");
	}
	else
	{
		m_pcd_list = Common_GQ::GetListFiles(list[1], "pcd");;
		m_png_list = Common_GQ::GetListFiles(list[0], "png");
	}
	if (m_pcd_list.size() != m_png_list.size())
	{
		return -1;
	}
	return m_pcd_list.size();
}

void LasesLaserCalibMainwindow::InitProject()
{
	m_calib.Init(m_pcd_list);
	m_calib_max_num = m_pcd_list.size();
	m_corners.resize(m_calib_max_num);
	DisplayCalibData(0);
	m_point_cloud_win->ZoomGlobal();
}

void LasesLaserCalibMainwindow::DisplayCalibData(int index)
{
	if (index >= m_calib_max_num)
		index = 0;
	if (index < 0)
		index = m_calib_max_num - 1;
	m_calib_index = index;
	QString disp;
	disp = QString(("Left: " + m_png_list[index] + "	 Right: " + m_pcd_list[index]).c_str());

	pcl::PointCloud<pcl::PointXYZRGB> color_pc;
	if (m_camera)
	{
		cv::Mat img= cv::imread(m_png_list[m_calib_index]);
		m_calib.ReprojectLaserToImg(m_calib_index, img, m_camera, color_pc);
		if (m_reproject_laser2img->isChecked())
		{
			m_img_win->LoadImg(img);
		}
		else
			m_img_win->LoadImg(QString(m_png_list[index].c_str()));
	}
	else
		m_img_win->LoadImg(QString(m_png_list[index].c_str()));

	statusBar()->showMessage(disp);

	QString prefix;
	if (m_camera)
		prefix = "Reproject error: " + QString::number(m_reproject_errors[index]) + " /tValid: " + ((m_reproject_errors[index]>0) ? "True" : "False");
	else
		prefix = "Reproject error: N/A /tValid: False";

	if (m_corners.size() > 0)
	{
		m_img_win->ShowCorners(m_corners[index], m_pattern_size);
		prefix += ("/tCorners count: " + QString::number(m_corners[index].size()));
	}

	m_message_label->setText(prefix + " Current Index: " + QString::number(m_calib_index + 1) + '/' + QString::number(m_calib_max_num));
	
	if (m_camera&&m_show_color_pc->isChecked()&&m_calib.GetCalibrated())
	{
		emit DisplayColorPointCloud(color_pc);
		return;
	}
	if (m_camera&&m_reproject_board2laser->isChecked() && m_reproject_errors[index] > 0)
	{
		POINTTYPE pc;
		CCVector3d pt;
		cv::Vec3d v=m_calib.ReprojectImgToLaser(m_calib_index, m_objects_pts[m_calib_index], m_rotation_vectors[m_calib_index], m_translation_vectors[m_calib_index], pc);
		for (size_t i = 0; i < 3; i++)
			pt[i] = v[i];
		m_point_cloud_win->SetRotationCentre(pt);
		emit DisplayPointCloud(pc);
		return;
	}
	
	POINTTYPE pc;
	pcl::io::loadPCDFile(m_pcd_list[m_calib_index], pc);

	vector<int> index_list = m_calib.GetPlanePoints(m_calib_index);
	if (index_list.size() > 0)
	{
		CCVector3d pt(0, 0, 0);
		for (int i = 0; i < pc.size(); i++)
			pc.at(i).intensity = 0;
		for (int i = 0; i < index_list.size(); i++)
		{
			pcl::PointXYZI &p = pc.at(index_list[i]);
			p.intensity = 255;
			pt[0] += p.x;
			pt[1] += p.y;
			pt[2] += p.z;
		}
		pt /= index_list.size();
		m_point_cloud_win->SetRotationCentre(pt);
	}
	emit DisplayPointCloud(pc);
}
