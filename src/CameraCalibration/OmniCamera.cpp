#include "OmniCamera.h"
#include <Common_GQ.h>
#include <opencv2/ccalib/omnidir.hpp>

using namespace std;
using namespace cv;
#define PI CV_PI

/***********************************************************************
* 球面转六面体坐标变换，用于生成map表
* 功能：求得每一个六面体上的像素在球面中的位置
* paras: boxX，boxY为六面体图像中像素的坐标
* boxwidth为盒子宽度（比如第四级是2048） imagewidth,imageheight为原始球面图的宽和高
************************************************************************/
void ConvertBoxToShpere(int boxX, int boxY, int boxwidth, int imagewidth, int imageheight, double& x, double& y)
{
	double rs = boxwidth / 2.0f;  //六面体半径
	double xs = boxX % boxwidth - rs;  //方格小图中对应的x坐标
	double ys = rs - boxY % boxwidth;  //方格小图中对应的y坐标

	double theta = atan(xs / rs);
	double phi = atan(ys / sqrt(pow(rs, 2.0) + pow(xs, 2.0)));

	if (boxX / boxwidth == 0 && boxY / boxwidth == 0)
	{   //1 左图
		theta += PI / 2;
		phi = PI / 2 - phi;
	}
	else if (boxX / boxwidth == 1 && boxY / boxwidth == 0)
	{   //2 中图
		theta += PI;
		phi = PI / 2 - phi;
	}
	else if (boxX / boxwidth == 2 && boxY / boxwidth == 0)
	{   //3 右图
		theta += PI * 3 / 2;
		phi = PI / 2 - phi;
	}
	else if (boxX / boxwidth == 0 && boxY / boxwidth == 1)
	{   //4 后图
		if (theta < 0)
			theta += 2 * PI;
		phi = PI / 2 - phi;
	}
	else if (boxX / boxwidth == 1 && boxY / boxwidth == 1)
	{   //5 上图
		double theta_shpere = 0;
		double phi_shpere = atan(sqrt(pow(xs, 2.0) + pow(ys, 2.0)) / rs);
		if (phi == 0 && theta == 0)
		{
			theta_shpere = PI;
		}
		else
		{
			theta_shpere = acos(ys / sqrt(pow(xs, 2.0) + pow(ys, 2.0)));
			if (theta >= 0)
				theta_shpere = PI - theta_shpere;
			else
				theta_shpere += PI;
		}

		phi = phi_shpere;
		theta = theta_shpere;

		theta += PI;
		if (theta >= 2 * PI)
			theta -= 2 * PI;
	}
	else if (boxX / boxwidth == 2 && boxY / boxwidth == 1)
	{   //6 下图
		double theta_shpere = 0;
		double phi_shpere = PI - atan(sqrt(pow(xs, 2.0) + pow(ys, 2.0)) / rs);
		if (phi == 0 && theta == 0)
		{
			theta_shpere = PI;
		}
		else
		{
			theta_shpere = acos(ys / sqrt(pow(xs, 2.0) + pow(ys, 2.0)));
			if (theta < 0)
				theta_shpere = 2 * PI - theta_shpere;
		}

		phi = phi_shpere;
		theta = theta_shpere;

		theta += PI;
		if (theta >= 2 * PI)
			theta -= 2 * PI;
	}
	x = (theta)*((double)imagewidth) / (2 * PI);
	y = phi * ((double)imageheight) / PI;
}

/***********************************************************************
* 计算当前球面图的内接立方体的边长
* 功能：计算当前球面图的内接立方体边长
* paras: srcHeight,srcWidth分别为球面图的宽和高
*　球面图的宽即为球面的圆周长
*   球面内接立方体的长度满足公式 r^2=(1/4+1/2)l^2
************************************************************************/
double cubicLength(int srcHeight, int srcWidth)
{
	double r = srcWidth / (2 * PI);
	double l = sqrt(2 * r*r);
	return l;
}

OmniCamera::OmniCamera()
{
}


OmniCamera::~OmniCamera()
{
}

bool OmniCamera::GenerateOmniToPanoMap(cv::Mat &mapx, cv::Mat &mapy, int pano_width, int pano_height,double fov,double radian_offset)
{
	if (!m_calibrated)
	{
		return false;
	}
	double r = pano_width / M_PI / 2;
// 	const double phi_each = 2 * M_PI / pano_width;
// 	const double theta_each = M_PI / pano_height;
	const double phi_each = 2 * M_PI / pano_width;
	const double theta_each = M_PI / pano_height;

	mapx = cv::Mat(pano_height, pano_width, CV_32FC1, -1.0);
	mapy = cv::Mat(pano_height, pano_width, CV_32FC1, -1.0);
	float *mapx_data = (float*)mapx.data, *mapy_data = (float*)mapy.data;

	Eigen::Matrix3d K_eigen = Common_GQ::toMatrix3d(m_K);

	Vec2d f(m_K.at<double>(0, 0), m_K.at<double>(1, 1));
	Vec2d c(m_K.at<double>(0, 2), m_K.at<double>(1, 2));
	Vec2d k(m_D.at<double>(0), m_D.at<double>(1));
	Vec2d p(m_D.at<double>(2), m_D.at<double>(3));
	double s = m_K.at<double>(0, 1);
	double xi = m_xi.at<double>(0);
	double p1 = p[0], p2 = p[1];
	double k1 = k[0], k2 = k[1];
	const double thre = M_PI / 180 * fov;
#pragma omp parallel for
	for (int i = 0; i < pano_height; i++)
	{
		for (int j = 0; j < pano_width; j++)
		{
			double phi = j * phi_each + radian_offset;
			double theta = i * theta_each;

			Vec3d Xs(sin(theta)*cos(phi),sin(theta)*sin(phi),cos(theta));



			// convert to normalized image plane
			Vec2d xu = Vec2d(Xs[0] / (Xs[2] + xi), Xs[1] / (Xs[2] + xi));

			// add distortion
			Vec2d xd;
			double r2 = xu[0] * xu[0] + xu[1] * xu[1];
			double r4 = r2 * r2;

			xd[0] = xu[0] * (1 + k1 * r2 + k2 * r4) + 2 * p1*xu[0] * xu[1] + p2 * (r2 + 2 * xu[0] * xu[0]);
			xd[1] = xu[1] * (1 + k1 * r2 + k2 * r4) + p1 * (r2 + 2 * xu[1] * xu[1]) + 2 * p2*xu[0] * xu[1];

			// convert to pixel coordinate
			Vec2d px;
			px[0] = f[0] * xd[0] + s * xd[1] + c[0];
			px[1] = f[1] * xd[1] + c[1];

			if (theta < thre)
			{
				mapx_data[j + i * pano_width] = px[0];
				mapy_data[j + i * pano_width] = px[1];
			}
		}
	}
	return true;
}

bool OmniCamera::GeneratePanoToHexahedronMap(int pano_width, int pano_height, bool type/*=true*/)
{
	if (!m_calibrated)
	{
		return false;
	}
	int width = cubicLength(pano_height, pano_width), ratio_width = type ? 6 : 3, ratio_height = type ? 1 : 2;
	cv::Size dst_size(width*ratio_width, width*ratio_height);
	double xs, ys;
	m_hex_mapx = Mat::zeros(dst_size, CV_32FC1);
	m_hex_mapy = Mat::zeros(dst_size, CV_32FC1);

	for (int i = 0; i < width*ratio_height; i++) //行
	{
		uchar *mapXptr = m_hex_mapx.ptr(i);
		uchar *mapYptr = m_hex_mapy.ptr(i);
		for (int j = 0; j < width*ratio_width; j++) //列
		{
			if (j >= width * 3)            //转换到公司的六面体格式(3:2)
			{
				ConvertBoxToShpere(j - width * 3, i + width, width, pano_width, pano_height, xs, ys);
			}
			else
			{
				ConvertBoxToShpere(j, i, width, pano_width, pano_height, xs, ys);
			}
			((float*)mapXptr)[j] = xs + 2;
			((float*)mapYptr)[j] = ys + 2;
		}
	}
	return true;
}

bool OmniCamera::RemapPanoToHexahedron(const cv::Mat &pano, cv::Mat &output)
{
	if (!m_calibrated || m_hex_mapx.empty() ||m_hex_mapy.empty())
	{
		return false;
	}
	Mat src_full = Mat::zeros(Size(pano.cols + 4, pano.rows + 2), CV_8UC3);            //左右和上三边各补两像素，否则后图中间会有黑边
	pano.copyTo(src_full(Rect(2,2,pano.cols,pano.rows)));
	remap(src_full, output, m_hex_mapx, m_hex_mapy, INTER_LANCZOS4);
}

void OmniCamera::ProjectPointsToPano(const std::vector<cv::Point3f> &pts_3d, std::vector<cv::Point2f> &pts_2d, int pano_width, int pano_height, double radian_offset /*= -M_PI / 2.0*/)
{
	const double phi_each = 2 * M_PI / pano_width;
	const double theta_each = M_PI / pano_height;
	pts_2d.resize(pts_3d.size());
#pragma omp parallel for
	for (int i = 0; i < pts_3d.size(); i++)
	{
			Eigen::Vector3d p(pts_3d[i].x, pts_3d[i].y, pts_3d[i].z);
			p.normalize();
			double theta = acosf(p[2]);
			double phi = atan2(p[1], p[0]);
			pts_2d[i] = Point2f((phi - radian_offset) / phi_each, theta / theta_each);
	}
}

void OmniCamera::InheritInitUndistortRectifyMap(const cv::Mat &R, const cv::Mat &K, const cv::Size &size, cv::Mat &mapx, cv::Mat &mapy)
{
	cv::omnidir::initUndistortRectifyMap(m_K, m_D, m_xi, R, K, size, CV_32FC1, mapx, mapy, cv::omnidir::RECTIFY_PERSPECTIVE);
}

void OmniCamera::InheritProjectPoints(const std::vector<cv::Point3f> &pts_3d, const cv::InputArray &r_vec, const cv::InputArray &t_vec, std::vector<cv::Point2f> &pts_2d)
{
	cv::omnidir::projectPoints(pts_3d, pts_2d, r_vec, t_vec, m_K, m_xi.at<double>(0), m_D);
}

void OmniCamera::InheritUndistortPoints(const std::vector<cv::Point2f> &pts_src, std::vector<cv::Point2f> &pts_dst)
{
	Vec2d f(m_K.at<double>(0, 0), m_K.at<double>(1, 1));
	Vec2d c(m_K.at<double>(0, 2), m_K.at<double>(1, 2));
	Vec2d k(m_D.at<double>(0), m_D.at<double>(1));
	Vec2d p(m_D.at<double>(2), m_D.at<double>(3));
	double s = m_K.at<double>(0, 1);
	double Xi = m_xi.at<double>(0);

	pts_dst.resize(pts_src.size());

	#pragma omp parallel for
	for (int i = 0; i < pts_src.size(); i++)
	{
		cv::Point2f ptd = pts_src[i];
		cv::Vec2d pp((ptd.x*f[1] - c[0] * f[1] - s*(ptd.y - c[1])) / (f[0] * f[1]), (ptd.y - c[1]) / f[1]);
		cv::Vec2d pu = pp;	// points without distortion

		// remove distortion iteratively
		for (int j = 0; j < 20; j++)
		{
			double r2 = pu[0] * pu[0] + pu[1] * pu[1];
			double r4 = r2*r2;
			pu[0] = (pp[0] - 2 * p[0] * pu[0] * pu[1] - p[1] * (r2 + 2 * pu[0] * pu[0])) / (1 + k[0] * r2 + k[1] * r4);
			pu[1] = (pp[1] - 2 * p[1] * pu[0] * pu[1] - p[0] * (r2 + 2 * pu[1] * pu[1])) / (1 + k[0] * r2 + k[1] * r4);
		}

		// project to unit sphere
		double r2 = pu[0] * pu[0] + pu[1] * pu[1];
		double a = (r2 + 1);
		double b = 2 * Xi*r2;
		double cc = r2*Xi*Xi - 1;
		double Zs = (-b + sqrt(b*b - 4 * a*cc)) / (2 * a);
		cv::Vec3d Xus = cv::Vec3d(pu[0] * (Zs + Xi), pu[1] * (Zs + Xi), Zs);

		// to norm plane
		Xus *= 1.0 / Zs;

		// to undistort points
		cv::Point2f pt(f[0] * Xus[0] + c[0], f[1] * Xus[1] + c[1]);
		pts_dst[i] = pt;
	}
}

double OmniCamera::InheritCalibrate(const std::vector<std::vector<cv::Point3f>> objects, const std::vector<std::vector<cv::Point2f>> &corners, std::vector<cv::Vec3d> &rotation_vectors, std::vector<cv::Vec3d> &translation_vectors, cv::TermCriteria critia, cv::Mat &idx)
{
	int flags = cv::omnidir::CALIB_FIX_SKEW;
	flags |= cv::omnidir::CALIB_USE_GUESS;
	double rms = cv::omnidir::calibrate(objects, corners, m_raw_size, m_K, m_xi, m_D, rotation_vectors, translation_vectors, flags, critia, idx);
	return rms;
}

bool OmniCamera::InheritSovlePnP(const std::vector<cv::Point3f> &object_pts, const std::vector<cv::Point2f> &corners, cv::Mat &R, cv::Mat &T)
{
	std::vector<cv::Point3d> pts_3d,pts_3d_3;
	std::vector<cv::Point2d> pts_2d, pts_2d_3;
	std::vector<cv::Point2f> undistort_corners,reproject_corners;
	UndistortPoints(corners, undistort_corners);
	for (size_t i = 0; i < undistort_corners.size(); i++)
	{
		//if (undistort_corners[i].x>=0&& undistort_corners[i].x<m_raw_size.width&&undistort_corners[i].y>=0&& undistort_corners[i].y<m_raw_size.height)
		{
			pts_3d.push_back(object_pts[i]);
			pts_2d.push_back(undistort_corners[i]);
		}

		if (i < 2)
		{
			pts_3d_3.push_back(object_pts[i]);
			pts_2d_3.push_back(undistort_corners[i]);
		}

	}
	vector<cv::Mat> Rs, Ts;
	pts_3d_3.push_back(object_pts[object_pts.size() - 1]);
	pts_2d_3.push_back(undistort_corners[undistort_corners.size() - 1]);
	cv::solveP3P(pts_3d_3, pts_2d_3, m_K, cv::Mat::zeros(4, 1, CV_64FC1), Rs, Ts,cv::SOLVEPNP_AP3P);
	double min_rms = 1e10;
	for (size_t i = 0; i < Rs.size(); i++)
	{
		ProjectPoints(object_pts, Rs[i], Ts[i], reproject_corners);
		double rms = CameraBase::CalcAvgPixelDistance(corners, reproject_corners);
		cout << rms << endl;
		if (rms<min_rms)
		{
			min_rms = rms;
			R = Rs[i];
			T = Ts[i];
		}
	}
	cv::Rodrigues(R, R);
	return true;
}

bool OmniCamera::LoadIntrinsicPara(const std::string &path)
{
	bool r=CameraBase::LoadIntrinsicPara(path);
	if (!r)
		return false;
	cv::FileStorage fs(path, cv::FileStorage::READ);
	if (!fs.isOpened())
		return false;
	fs["Xi"] >> m_xi;
	m_calibrated = true;
	return true;
}

bool OmniCamera::SaveIntrinsicPara(const std::string &path)
{
	bool r = CameraBase::SaveIntrinsicPara(path);
	if (!r)
		return false;
	cv::FileStorage fs(path, cv::FileStorage::APPEND);
	if (!fs.isOpened())
		return false;
	fs<<"Xi"<<m_xi;
	return true;
}

double OmniCamera::GetXi() const
{
	return m_xi.at<double>(0);
}
