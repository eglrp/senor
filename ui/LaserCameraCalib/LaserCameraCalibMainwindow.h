#pragma once
#include <QMainWindow>
#include "../CCPointCloudWindow/PointCloudWindow.h"
#include "../imgview.h"
#include <ExtrinsicCalibration/CameraVelodyneCalib.h>
#include <QLabel>
#include <QActionGroup>
#include <CameraCalibration/CameraBase.h>
class LasesLaserCalibMainwindow:public QMainWindow
{
	Q_OBJECT
public:
	LasesLaserCalibMainwindow(QWidget *parent=0);
	~LasesLaserCalibMainwindow();
public:
	void InitUI();
signals:
	void DisplayPointCloud(const POINTTYPE &pc);
	void DisplayColorPointCloud(const pcl::PointCloud<pcl::PointXYZRGB> &pc);

private slots:
	void OpenProject();
	void SaveProject();
	void NextFrame();
	void LastFrame();
	void FindCorners();
	void LoadCornersFromMatlab();
	void LoadLaserBoardPoints();
	void SaveLaserBoardPoints();
	void CalibrateCamera();
	void CalibrateLaserWithCamera();
	void SelectPointsSlot(const std::vector<int> &indexs);
	void RefreshDisplay();
	void LoadIntrinsicPara();
	void SaveIntrinsicPara();
	void SaveExtrinsicPara();
	void LoadExtrinsicPara();
	void FindCornersSlot();
private:
	//点云窗体
	PointCloudWindow *m_point_cloud_win;
	//图像窗体
	ImgView *m_img_win;
	QLabel *m_message_label;
	//选择相机类型
	QActionGroup *m_camera_type;
	QAction *m_reproject_board2laser;
	QAction *m_reproject_laser2img;
	QAction *m_show_color_pc;
	QAction *m_jump_invalid;

	QDialog *setting;
	QLineEdit* edit_fov;
	QLineEdit* edit_size_H;
	QLineEdit* edit_size_W;
	QLineEdit* edit_length;
	QLineEdit* edit_remove;


	//pcd文件路径集合
	std::vector<std::string> m_pcd_list;
	//png文件路径集合
	std::vector<std::string> m_png_list;
	//k维邻近树，用于查找选取点附近的临近点，用于拟合平面
	pcl::KdTreeFLANN<pcl::PointXYZI> m_kd_tree;

	int m_calib_index;
	int m_calib_max_num;

	CameraVelodyneCalib m_calib;

	//棋盘格角点的像素坐标
	std::vector<std::vector<cv::Point2f>> m_corners;
	//棋盘格角点的三维坐标（相机坐标系下）
	std::vector<std::vector<cv::Point3f>> m_objects_pts;

	//丢弃最坏组数,默认为1组
	int m_remove_worst_group_count = 1;
	//相机的视场角
	double m_fov=75;
	//棋盘格尺寸
	cv::Size m_pattern_size = cv::Size(8, 9);
	//棋盘格格子边长，单位毫米
	float m_square_length=100;
	//旋转向量
	std::vector<cv::Vec3d> m_rotation_vectors;
	//平移向量
	std::vector<cv::Vec3d> m_translation_vectors;
	//重投影误差
	std::vector<double> m_reproject_errors;
	//相机类型（基类）
	CameraBase *m_camera;
private:
	int CheckDirPath(QString path);
	void InitProject();
	void DisplayCalibData(int index);
};

