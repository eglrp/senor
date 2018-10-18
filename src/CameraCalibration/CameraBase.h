#pragma once
#include <opencv2/opencv.hpp>
class CameraBase
{
public:
	CameraBase();
	virtual ~CameraBase();
	static cv::Vec3b GetBGRImgPixel(const cv::Mat &img,const cv::Point2f&pt);
	static double GetGreyImgPixel(const cv::Mat &img, const cv::Point2f&pt);
	static bool SaveExtrinsicPara(const std::string &path,const std::vector<cv::Vec3d> &rotation_vectors, const 	std::vector<cv::Vec3d> &translation_vectors);
	static bool LoadExtrinsicPara(const std::string &path,std::vector<cv::Vec3d> &rotation_vectors,std::vector<cv::Vec3d> &translation_vectors);
	static double CalcAvgPixelDistance(const std::vector<cv::Point2f> &src, const std::vector<cv::Point2f> &dst);

	virtual double Calibrate(const std::vector<std::vector<cv::Point3f>> &object_pts, const std::vector<std::vector<cv::Point2f>> &corners, 
		const cv::Size &pattern_size, double square_length, double fov = -1, int remove_worst_group_count = 0,
		std::vector<cv::Vec3d> &rotation_vectors = std::vector<cv::Vec3d>(), 
		std::vector<cv::Vec3d> &translation_vectors = std::vector<cv::Vec3d>(),
		std::vector<double> &reproject_errors = std::vector<double>()) final;

	virtual bool SolvePNP(const std::vector<cv::Point3f> &object_pts, const std::vector<cv::Point2f> &corners, cv::Mat &RT, int index) final;
	virtual bool SolvePNP(const std::vector<cv::Point3f> &object_pts, const std::vector<cv::Point2f> &corners, cv::Mat &R,cv::Mat &T, int index) final;


	virtual bool ProjectPoints(const std::vector<cv::Point3f> &pts_3d, const cv::Mat &RT, std::vector<cv::Point2f> &pts_2d) final;
	virtual bool ProjectPoints(const std::vector<cv::Point3f> &pts_3d, const cv::InputArray &R, const cv::InputArray &T, std::vector<cv::Point2f> &pts_2d) final;

	virtual bool UndistortImage(const cv::Mat &img, cv::Mat &rec_img ,int interpolation=cv::INTER_CUBIC) final;
	virtual bool GenerateUndistortMap(const cv::Mat & rec_K=cv::Mat(),cv::Size rec_size=cv::Size()) final;
	virtual bool UndistortPoints(const std::vector<cv::Point2f> &pts_src, std::vector<cv::Point2f> &pts_dst) final;

	virtual bool LoadIntrinsicPara(const std::string &path);
	virtual bool SaveIntrinsicPara(const std::string &path);



	virtual cv::Size GetRawSize() const final;
	virtual void SetRawSize(cv::Size size) final;

	virtual bool GetCalibrated() const final;

	virtual cv::Mat GetK() const final;
	virtual cv::Mat GetD() const final;

	virtual cv::Mat GetRecK() const final;
	virtual cv::Size GetRecSize() const final;

	virtual void SetIntrinsicPara(const cv::Mat &K,const cv::Mat &D) final;

protected:
	bool m_calibrated;

	cv::Mat m_K;            //内参
	cv::Mat m_D;            //畸变参数

	cv::Size m_raw_size;            //原始图像大小

	cv::Mat m_rec_map_x;            //去畸变映射表
	cv::Mat m_rec_map_y;            //去畸变映射表

	cv::Mat m_rec_K;            //去畸变后的内参
	cv::Size m_rec_size;            //去畸变的图像大小



protected:
	virtual void InheritInitUndistortRectifyMap(const cv::Mat &R, const cv::Mat &K, const cv::Size &size,cv::Mat &mapx, cv::Mat &mapy) = 0;
	virtual void InheritProjectPoints(const std::vector<cv::Point3f> &pts_3d, const cv::InputArray &r_vec, const cv::InputArray &t_vec, std::vector<cv::Point2f> &pts_2d) = 0;
	virtual void InheritUndistortPoints(const std::vector<cv::Point2f> &pts_src, std::vector<cv::Point2f> &pts_dst) = 0;
	virtual double InheritCalibrate(const std::vector<std::vector<cv::Point3f>> objects, const std::vector<std::vector<cv::Point2f>> &corners,
		std::vector<cv::Vec3d> &rotation_vectors,	std::vector<cv::Vec3d> &translation_vectors,
		cv::TermCriteria critia, cv::Mat &idx	) = 0;
	virtual bool InheritSovlePnP(const std::vector<cv::Point3f> &object_pts, const std::vector<cv::Point2f> &corners, cv::Mat &R, cv::Mat &T) = 0;
};

