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
	//���ƴ���
	PointCloudWindow *m_point_cloud_win;
	//ͼ����
	ImgView *m_img_win;
	QLabel *m_message_label;
	//ѡ���������
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


	//pcd�ļ�·������
	std::vector<std::string> m_pcd_list;
	//png�ļ�·������
	std::vector<std::string> m_png_list;
	//kά�ڽ��������ڲ���ѡȡ�㸽�����ٽ��㣬�������ƽ��
	pcl::KdTreeFLANN<pcl::PointXYZI> m_kd_tree;

	int m_calib_index;
	int m_calib_max_num;

	CameraVelodyneCalib m_calib;

	//���̸�ǵ����������
	std::vector<std::vector<cv::Point2f>> m_corners;
	//���̸�ǵ����ά���꣨�������ϵ�£�
	std::vector<std::vector<cv::Point3f>> m_objects_pts;

	//���������,Ĭ��Ϊ1��
	int m_remove_worst_group_count = 1;
	//������ӳ���
	double m_fov=75;
	//���̸�ߴ�
	cv::Size m_pattern_size = cv::Size(8, 9);
	//���̸���ӱ߳�����λ����
	float m_square_length=100;
	//��ת����
	std::vector<cv::Vec3d> m_rotation_vectors;
	//ƽ������
	std::vector<cv::Vec3d> m_translation_vectors;
	//��ͶӰ���
	std::vector<double> m_reproject_errors;
	//������ͣ����ࣩ
	CameraBase *m_camera;
private:
	int CheckDirPath(QString path);
	void InitProject();
	void DisplayCalibData(int index);
};

