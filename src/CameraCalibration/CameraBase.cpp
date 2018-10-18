#include "CameraBase.h"
#include <Eigen/Geometry>
#include <Common_GQ.h>
#include "cost_functions.h"
#include "camera_models.h"

using namespace std;
using namespace cv;



CameraBase::CameraBase():m_calibrated(false)
{
}


CameraBase::~CameraBase()
{
}

cv::Vec3b CameraBase::GetBGRImgPixel(const cv::Mat &img, const cv::Point2f&pt)
{
	assert(img.channels() == 3);
	int x = pt.x, y = pt.y;
	float u = pt.x - x, v = pt.y - y;
	int step = img.cols * 3;
	uchar *data = img.data;
	cv::Vec3b r(0,0,0);

	if (x < 0 || x > img.cols - 1)
		return r;
	if (x >= img.cols - 2)
		x = img.cols - 2;
	if (y < 0 || y > img.rows - 1)
		return r;
	if (y >= img.rows - 2)
		y = img.rows - 2;

	//双线性插值
	Eigen::Vector4d weight((1 - u)*(1 - v), u*(1 - v), (1 - u)*v, u*v);
	for (int i = 0; i < 3; i++)
		r[i] = weight.dot(Eigen::Vector4d(data[y*step + x * 3 + i], data[(y + 1)*step + x * 3 + i], data[y*step + (x + 1) * 3 + i], data[(y + 1)*step + (x + 1) * 3 + i]));
	return r;
}

double CameraBase::GetGreyImgPixel(const cv::Mat &img, const cv::Point2f&pt)
{
	assert(img.channels() == 1);
	int x = pt.x, y = pt.y;
	float u = pt.x - x, v = pt.y - y;
	int step = img.cols * 3;
	uchar *data = img.data;
	double r = 0;

	if (x < 0 || x > img.cols - 1)
		return r;
	if (x >= img.cols - 2)
		x = img.cols - 2;
	if (y < 0 || y > img.rows - 1)
		return r;
	if (y >= img.rows - 2)
		y = img.rows - 2;

	//双线性插值
	Eigen::Vector4d weight((1 - u)*(1 - v), u*(1 - v), (1 - u)*v, u*v);
		r = weight.dot(Eigen::Vector4d(data[y*step + x * 3 ], data[(y + 1)*step + x * 3 ], data[y*step + (x + 1) * 3 ], data[(y + 1)*step + (x + 1) * 3 ]));
	return r;

}

bool CameraBase::SaveExtrinsicPara(const std::string &path, const std::vector<cv::Vec3d> &rotation_vectors, const std::vector<cv::Vec3d> &translation_vectors)
{
	cv::FileStorage fs(path, cv::FileStorage::WRITE);
	if (!fs.isOpened())
		return false;
	fs << "rotation_vectors" << rotation_vectors;
	fs << "translation_vectors" << translation_vectors;
	return true;
}

bool CameraBase::LoadExtrinsicPara(const std::string &path, std::vector<cv::Vec3d> &rotation_vectors, std::vector<cv::Vec3d> &translation_vectors)
{
	cv::FileStorage fs(path, cv::FileStorage::READ);
	if (!fs.isOpened())
		return false;
	rotation_vectors.clear();
	translation_vectors.clear();
	fs["rotation_vectors"] >> rotation_vectors;
	fs["translation_vectors"] >> translation_vectors;
	return true;
}

double CameraBase::Calibrate(const std::vector<std::vector<cv::Point3f>> &object_pts, const std::vector<std::vector<cv::Point2f>> &corners_pts, const cv::Size &pattern_size, double square_length, double fov /*= -1*/, int remove_worst_group_count /*= 0*/, std::vector<cv::Vec3d> &rotation_vectors /*= std::vector<cv::Vec3d>()*/, std::vector<cv::Vec3d> &translation_vectors /*= std::vector<cv::Vec3d>()*/, std::vector<double> &reproject_errors /*= std::vector<double>()*/)
{
	if (object_pts.size() != corners_pts.size())
		return 0;

	std::vector<std::vector<cv::Point2f>> corners, corners_new;
	std::vector<std::vector<cv::Point3f>> objects;
	std::vector<int> id_map;

	if (m_K.empty())
	{
		m_K = cv::Mat::eye(3, 3, CV_64FC1);
		m_K.at<double>(0, 0) = m_K.at<double>(1, 1) = m_raw_size.width / 2.0 / tan(fov / 360.0*M_PI);
		m_K.at<double>(0, 2) = m_raw_size.width / 2;
		m_K.at<double>(1, 2) = m_raw_size.height / 2;
	}

	double rms;
	int interation_time = 0;
	cv::Mat idx;

	while (1)
	{
		if (corners.size() == 0)
		{
			int count = object_pts.size();
			int pts_count = pattern_size.width*pattern_size.height;
			for (size_t i = 0; i < count; i++)
			{
				if (object_pts[i].size() == pts_count &&corners_pts[i].size() == pts_count)
				{
					objects.push_back(object_pts[i]);
					corners.push_back(corners_pts[i]);
					id_map.push_back(i);
				}
			}
		}
		int img_count = corners.size();

		rotation_vectors.clear();
		translation_vectors.clear();
		idx.release();
		reproject_errors.clear();

		cout << "Iter: " << interation_time << endl;
		cv::TermCriteria critia(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 200, 1e-6);
		rms = InheritCalibrate(objects, corners, rotation_vectors, translation_vectors, critia, idx);
		cout << "RMS: " << rms << endl;

		img_count = idx.cols;

		reproject_errors.resize(img_count);

		vector<pair<double, int>> rms_list(img_count);
#pragma omp parallel for
		for (int i = 0; i < img_count; i++)
		{
			int index = idx.at<int>(0, i);
			std::vector<cv::Point2f> reproject_corners;
			InheritProjectPoints(objects[index],rotation_vectors[i], translation_vectors[i], reproject_corners);
			reproject_errors[i] = CalcAvgPixelDistance(reproject_corners, corners[index]);
			rms_list[i] = make_pair(reproject_errors[i], index);
		}

		sort(rms_list.begin(), rms_list.end());

		cout << "Min Pixel Distance: " << rms_list[0].first << endl;
		cout << "Max Pixel Distance: " << rms_list[img_count - 1].first << endl;

		interation_time++;
		if (interation_time <= remove_worst_group_count)
		{
			img_count--;
			std::vector<std::vector<cv::Point2f>> corners_new;
			std::vector<std::vector<cv::Point3f>> objects_new;
			std::vector<int> id_map_new;
			for (int i = 0; i < img_count; i++)
			{
				int index = rms_list[i].second;
				corners_new.push_back(corners[index]);
				objects_new.push_back(objects[index]);
				id_map_new.push_back(id_map[index]);
			}
			corners.swap(corners_new);
			objects.swap(objects_new);
			id_map.swap(id_map_new);
		}
		else
			break;
	}
	std::vector<cv::Vec3d> rs = rotation_vectors, ts = translation_vectors;
	std::vector<double> es = reproject_errors;

	int all_count = object_pts.size();
	rotation_vectors.resize(all_count);
	translation_vectors.resize(all_count);
	reproject_errors.resize(all_count);

	for (size_t i = 0; i < all_count; i++)
	{
		rotation_vectors[i] = Vec3d(0, 0, 0);
		translation_vectors[i] = Vec3d(0, 0, 0);
		reproject_errors[i] = 0;
	}

	for (size_t i = 0; i < rs.size(); i++)
	{
		int index = id_map[idx.at<int>(0, i)];
		rotation_vectors[index] = rs[i];
		translation_vectors[index] = ts[i];
		reproject_errors[index] = es[i];
	}
	m_calibrated = true;
	return rms;
}

bool CameraBase::SolvePNP(const std::vector<cv::Point3f> &object_pts, const std::vector<cv::Point2f> &corners, cv::Mat &RT, int index)
{
	cv::Mat R, T;
	if (SolvePNP(object_pts, corners, R, T, index))
	{
		RT = Common_GQ::toMat44(R, T);
		return true;
	}
	return false;
}

bool CameraBase::SolvePNP(const std::vector<cv::Point3f> &object_pts, const std::vector<cv::Point2f> &corners, cv::Mat &R, cv::Mat &T, int index)
{
	InheritSovlePnP(object_pts, corners, R, T);
	std::vector<cv::Point2f> reprojectCorners;

	ProjectPoints(object_pts, R, T, reprojectCorners);
	double rms_b = CalcAvgPixelDistance(corners, reprojectCorners);

	std::vector<cv::Point2f> undistorted_corners;
	InheritUndistortPoints(corners, undistorted_corners);

	double para[4];
	para[0] = m_K.at<double>(0, 0);
	para[1] = m_K.at<double>(1, 1);
	para[2] = m_K.at<double>(0, 2);
	para[3] = m_K.at<double>(1, 2);
	
	cv::Mat R1;
	cv::Rodrigues(R, R1);
	
	double rt[6] = { R1.at<double>(0), R1.at<double>(1), R1.at<double>(2), T.at<double>(0), T.at<double>(1), T.at<double>(2) };
	ceres::Problem problem;
	for (int i = 0; i < object_pts.size(); i++)
	{
		ceres::CostFunction* cost_function = PinholeCameraModelCostFunction::Create(
			Eigen::Vector3d(object_pts[i].x, object_pts[i].y, object_pts[i].z), 
			Eigen::Vector2d(undistorted_corners[i].x, undistorted_corners[i].y),
			para);
		problem.AddResidualBlock(cost_function, new ceres::CauchyLoss(1), rt);
	}
	
	ceres::Solver::Options options;
	options.function_tolerance = 1e-9;
	options.linear_solver_type = ceres::DENSE_QR;
	options.use_nonmonotonic_steps = true;
	options.max_consecutive_nonmonotonic_steps = 1000;            //防止找到局部极值
	options.num_threads = omp_get_num_threads();
	options.minimizer_progress_to_stdout = false;
	options.max_num_iterations = 10000;            //迭代次数
	ceres::Solver::Summary summary;
	ceres::Solve(options, &problem, &summary);
	
	std::cout << summary.BriefReport() << "\n";
	
	cv::Mat Rtmp(3, 1, CV_64FC1);
	cv::Mat Ttmp(3, 1, CV_64FC1);
	for (int i = 0; i < 3; i++)
	{
		Rtmp.at<double>(i) = rt[i];
		Ttmp.at<double>(i) = rt[i+3];
	}
		
	cv::Rodrigues(Rtmp, Rtmp);
	
	cv::Mat R_t = Rtmp.clone();
	cv::Mat T_t = Ttmp.clone();

	ProjectPoints(object_pts, R_t, T_t, reprojectCorners);
	double rms_c = CalcAvgPixelDistance(corners, reprojectCorners);

	cout << "rms : " << rms_b << " -> " << rms_c << endl;

	R = R_t.clone();
	T = T_t.clone();
	return true;
}

bool CameraBase::ProjectPoints(const std::vector<cv::Point3f> &pts_3d, const cv::Mat &RT, std::vector<cv::Point2f> &pts_2d)
{
	cv::Mat R, T;
	if(!Common_GQ::toMat33AndMat31(RT, R, T))
		return false;
	cv::Rodrigues(R, R);
	return ProjectPoints(pts_3d, R, T, pts_2d);
}

bool CameraBase::ProjectPoints(const std::vector<cv::Point3f> &pts_3d, const cv::InputArray &R, const cv::InputArray &T, std::vector<cv::Point2f> &pts_2d)
{
	if (!m_calibrated)
		return false;
	cv::Mat R_mat = R.getMat();
	cv::Mat T_mat = T.getMat();

	if (R.size() == cv::Size(3, 3))
		cv::Rodrigues(R_mat, R_mat);
	if ((R_mat.rows*R_mat.cols != 3) || (T_mat.rows*T_mat.cols != 3))
		return false;
	InheritProjectPoints(pts_3d, R_mat, T_mat, pts_2d);
	return true;
}

bool CameraBase::UndistortImage(const cv::Mat &img, cv::Mat &rec_img, int interpolation)
{
	if (!m_calibrated||m_rec_map_x.empty()||m_rec_map_y.empty())
		return false;
	cv::remap(img, rec_img, m_rec_map_x, m_rec_map_y, interpolation);
	return true;
}

bool CameraBase::GenerateUndistortMap(const cv::Mat & rec_K/*=cv::Mat()*/, cv::Size rec_size/*=cv::Size()*/)
{
	if (!m_calibrated)
	{
		return false;
	}
	cv::Mat rec_K1 = rec_K.clone();
	if (rec_K1.empty())
	{
		m_K.copyTo(rec_K1);
	}
	if (rec_size == cv::Size())
	{
		rec_size = m_raw_size;
	}

	cv::Mat rec_map_x(rec_size.height, rec_size.width, CV_32FC1, -1.0);
	cv::Mat rec_map_y(rec_size.height, rec_size.width, CV_32FC1, -1.0);

	cv::Mat R = cv::Mat::eye(3, 3, CV_64FC1);

	InheritInitUndistortRectifyMap(R, rec_K1, rec_size, rec_map_x, rec_map_y);

	rec_map_x.copyTo(m_rec_map_x);
	rec_map_y.copyTo(m_rec_map_y);
	rec_K1.copyTo(m_rec_K);
	m_rec_size = rec_size;
	return true;

}

bool CameraBase::UndistortPoints(const std::vector<cv::Point2f> &pts_src, std::vector<cv::Point2f> &pts_dst)
{
	if (!m_calibrated)
	{
		return false;
	}
	InheritUndistortPoints(pts_src, pts_dst);
	return true;
}

bool CameraBase::LoadIntrinsicPara(const std::string &path)
{
	cv::FileStorage fs(path, cv::FileStorage::READ);
	if (!fs.isOpened())
		return false;
	fs["RawSize"] >> m_raw_size;
	fs["K"] >> m_K;
	fs["D"] >> m_D;
	fs["RecSize"] >> m_rec_size;
	fs["RecK"] >> m_rec_K;
	m_calibrated = true;
	return true;
}

bool CameraBase::SaveIntrinsicPara(const std::string &path)
{
	cv::FileStorage fs(path,cv::FileStorage::WRITE);
	if (!fs.isOpened())
		return false;
	fs << "RawSize" << m_raw_size;
	fs << "K" << m_K;
	fs << "D" << m_D;
	fs << "RecSize" << m_rec_size;
	fs << "RecK" << m_rec_K;
	return true;
}

cv::Size CameraBase::GetRawSize() const
{
	return m_raw_size;
}

void CameraBase::SetRawSize(cv::Size size)
{
	m_raw_size = size;
}

cv::Size CameraBase::GetRecSize() const
{
	return m_rec_size;
}

void CameraBase::SetIntrinsicPara(const cv::Mat &K, const cv::Mat &D)
{
	K.convertTo(m_K, CV_64FC1);
	D.convertTo(m_D, CV_64FC1);
	m_calibrated = true;
}

bool CameraBase::GetCalibrated() const
{
	return m_calibrated;
}

cv::Mat CameraBase::GetK() const
{
	return m_K;
}

cv::Mat CameraBase::GetD() const
{
	return m_D;
}

cv::Mat CameraBase::GetRecK() const
{
	return m_rec_K;
}

double CameraBase::CalcAvgPixelDistance(const std::vector<cv::Point2f> &src, const std::vector<cv::Point2f> &dst)
{
	double rms = 0;
	int size = src.size();
	for (int i = 0; i < size; i++)
	{
		cv::Point2f distance = src[i] - dst[i];
		rms += sqrt(distance.dot(distance));
	}
	return rms / size;
}