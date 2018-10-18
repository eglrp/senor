#pragma once
#include <QMainWindow>
#include "../CCPointCloudWindow/PointCloudWindow.h"
#include <ExtrinsicCalibration/LaserCalib.h>
#include <QLabel>
#include <QActionGroup>
class LaserLaserCalibMainwindow:public QMainWindow
{
	Q_OBJECT
public:
	LaserLaserCalibMainwindow(QWidget *parent=0);
	~LaserLaserCalibMainwindow();
public:
	void InitUI();
signals:
	void DisplayPointCloudRight(const POINTTYPE &pc);
	void DisplayColorPointCloudRight(const pcl::PointCloud<pcl::PointXYZRGB> &pc);
	void DisplayPointCloudLeft(const POINTTYPE &pc);
	void DisplayColorPointCloudLeft(const pcl::PointCloud<pcl::PointXYZRGB> &pc);

private slots:
	void OpenProject();
	void SaveProject();
	void NextFrame();
	void LastFrame();
	void LoadLaserBoardPoints();
	void SaveLaserBoardPoints();
	void CalibrateLaserWithLaser();
	void SelectPointsSlotRight(const std::vector<int> &indexs);
	void SelectPointsSlotLeft(const std::vector<int> &indexs);

	void RefreshDisplay();

private:
	PointCloudWindow *m_point_cloud_win_right;
	PointCloudWindow *m_point_cloud_win_left;

	QLabel *m_message_label;

	std::vector<std::string> m_right_pcd_list;
	std::vector<std::string> m_left_pcd_list;

	int m_calib_index;
	int m_calib_max_num;

	LaserCalib m_calib;

	std::vector<std::vector<cv::Point2f>> m_corners;
	std::vector<std::vector<cv::Point3f>> m_objects_pts;

	std::vector<cv::Vec3d> m_rotation_vectors;
	std::vector<cv::Vec3d> m_translation_vectors;
	std::vector<double> m_reproject_errors;

private:
	int CheckDirPath(QString path);
	void InitProject();
	void DisplayCalibData(int index);
};

