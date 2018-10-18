#pragma once
#include "CameraBase.h"
#define _USE_MATH_DEFINES
#include <math.h>

class OmniCamera :
	public CameraBase
{
public:
	OmniCamera();
	~OmniCamera();

	//************************************
	// Method:    GenerateOmniToPanoMap
	// FullName:  OmniCamera::GenerateOmniToPanoMap
	// Access:    public 
	// Returns:   bool
	// Qualifier: ����ȥ������ȫ��ͼ�����ھ���Ч������κ����Ǳ���Ҫ����ȫ��ͼ��
	// Parameter: cv::Mat & mapx
	// Parameter: cv::Mat & mapy
	// Parameter: int pano_width ȫ����
	// Parameter: int pano_height ȫ����,ӦΪ����һ��
	// Parameter: double fov ��ֱfov��180���ڣ�Ĭ��127���ֳ��豸�����ֵ
	// Parameter: double radian_offset ˮƽ��ʼ�ӽǵ�ƫ������Ĭ��Ϊ90�ȣ�������ȫ��������պ�0�ȴ����ֳ��豸��������
	//************************************
	bool GenerateOmniToPanoMap(cv::Mat &mapx, cv::Mat &mapy, int pano_width, int pano_height, double fov = 127.0, double radian_offset = M_PI / 2.0);
	
	//************************************
	// Method:    GeneratePanoToHexahedronMap
	// FullName:  OmniCamera::GeneratePanoToHexahedronMap
	// Access:    public 
	// Returns:   bool
	// Qualifier: ��ȫ��ͼ����������ͼ��ӳ���
	// Parameter: int pano_width ȫ����
	// Parameter: int pano_height ȫ����
	// Parameter: bool type ���������ͣ�trueΪ3:2��falseΪ6:1
	//************************************
	bool GeneratePanoToHexahedronMap(int pano_width, int pano_height, bool type=true);
	//************************************
	// Method:    RemapPanoToHexahedron
	// FullName:  OmniCamera::RemapPanoToHexahedron
	// Access:    public 
	// Returns:   bool
	// Qualifier: ӳ��ȫ��ͼ��������ͼ��
	// Parameter: const cv::Mat & pano
	// Parameter: cv::Mat & output
	//************************************
	bool RemapPanoToHexahedron(const cv::Mat &pano, cv::Mat &output);
	// ͶӰָ��3ά�㵽����ȫ��ͼ��
	//************************************
	// Method:    ProjectPointsToPano
	// FullName:  OmniCamera::ProjectPointsToPano
	// Access:    public 
	// Returns:   void
	// Qualifier: ͶӰָ���㼯��ȫ��ͼ��
	// Parameter: const std::vector<cv::Point3f> & pts_3d ��ά�㣬������ȫ��������ϵ�µ���ά����
	// Parameter: std::vector<cv::Point2f> & pts_2d �����ά��
	// Parameter: int pano_width ȫ����
	// Parameter: int pano_height ȫ����
	// Parameter: double radian_offset ˮƽ��ʼ�ӽǵ�ƫ������Ĭ��Ϊ90�ȣ�������ȫ��������պ�0�ȴ����ֳ��豸��������
	//************************************
	void ProjectPointsToPano(const std::vector<cv::Point3f> &pts_3d, std::vector<cv::Point2f> &pts_2d, int pano_width, int pano_height,double radian_offset = M_PI / 2.0);

	bool LoadIntrinsicPara(const std::string &path) override;
	bool SaveIntrinsicPara(const std::string &path) override;

	double GetXi() const;

protected:
	void InheritInitUndistortRectifyMap(const cv::Mat &R, const cv::Mat &K, const cv::Size &size, cv::Mat &mapx, cv::Mat &mapy) override;
	void InheritProjectPoints(const std::vector<cv::Point3f> &pts_3d, const cv::InputArray &r_vec, const cv::InputArray &t_vec, std::vector<cv::Point2f> &pts_2d) override;
	void InheritUndistortPoints(const std::vector<cv::Point2f> &pts_src, std::vector<cv::Point2f> &pts_dst) override;
	double InheritCalibrate(const std::vector<std::vector<cv::Point3f>> objects, const std::vector<std::vector<cv::Point2f>> &corners,
		std::vector<cv::Vec3d> &rotation_vectors, std::vector<cv::Vec3d> &translation_vectors,
		cv::TermCriteria critia, cv::Mat &idx) override;
	bool InheritSovlePnP(const std::vector<cv::Point3f> &object_pts, const std::vector<cv::Point2f> &corners, cv::Mat &R, cv::Mat &T) override;

private:
	cv::Mat m_xi;

	cv::Mat m_hex_mapx;
	cv::Mat m_hex_mapy;

};

