#include "PlaneExtrinsicOptimization.h"
#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <QDebug>
#include <opencv2/opencv.hpp>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/io/pcd_io.h>
#include <iostream>
#include <Common_GQ.h>
using ceres::AutoDiffCostFunction;
using ceres::CostFunction;
using ceres::CauchyLoss;
using ceres::Problem;
using ceres::Solve;
using ceres::Solver;
using namespace cv;

//第一部分：构建代价函数(正向约束)
struct ExponentialResidual
{
	ExponentialResidual(double *x, unsigned int n)
		:x_(x), n_(n)
	{ 
	}

	//重载()操作符
	template <typename T> 
	bool operator()(const T* const x0, T* residual) const
	{
		T rot[9];
		T x[6] = { T(x0[0]), T(x0[1]), T(x0[2]), T(x0[3]), T(x0[4]), T(x0[5]) };
		//欧拉角转旋转矩阵
		ceres::AngleAxisToRotationMatrix(x, rot);
		Eigen::Matrix<T, 4, 4> tmat;
		tmat << rot[0], rot[1], rot[2], x[3],
			rot[3], rot[4], rot[5], x[4],
			rot[6], rot[7], rot[8], x[5],
			T(0), T(0), T(0), T(1);
		//平面+点集数据
		const double *d = x_;
		//获取坐标点的数量
		const int pointNum = (n_ - 4) / 3;
		//取平面信息
		T plane[4] = { T(d[0]), T(d[1]), T(d[2]), T(d[3]) };
		Eigen::Matrix<T, 4, 1> point;
		T sum = T(0.0);//初始值为0
		for (int i = 0, k = 4; i < pointNum; i++, k += 3)
		{
			point(0, 0) = T(d[k]);
			point(1, 0) = T(d[k + 1]);
			point(2, 0) = T(d[k + 2]);
			point(3, 0) = T(1.0);
			point = tmat*point;
			T x = point(0, 0);
			T y = point(1, 0);
			T z = point(2, 0);
			//约束条件---点到平面的距离
			T d = abs(plane[0] * x + plane[1] * y + plane[2] * z + plane[3]);
			d = d / sqrt(plane[0] * plane[0] + plane[1] * plane[1] + plane[2] * plane[2]);
			//sum += d;
			if (d > sum)
			{
				sum = d;
			}
		}
		//sum /= T(pointNum);
		//残差为点到平面的最大值
		residual[0] = sum;
		return true;
	}

private:
	//平面+点集数据
	const double *x_;
	//数据大小
	unsigned int n_;
	
};

//反向约束---与正向同理
struct ExponentialResidualINV
{
	ExponentialResidualINV(double *x, unsigned int n)
		:x_(x), n_(n)
	{
	}

	template <typename T> bool operator()(const T* const x0, T* residual) const
	{
		T rot[9];
		T x[6] = { T(x0[0]), T(x0[1]), T(x0[2]), T(x0[3]), T(x0[4]), T(x0[5]) };
		ceres::AngleAxisToRotationMatrix(x, rot);
		Eigen::Matrix<T, 4, 4> tmat;
		tmat << rot[0], rot[1], rot[2], x[3],
			rot[3], rot[4], rot[5], x[4],
			rot[6], rot[7], rot[8], x[5],
			T(0), T(0), T(0), T(1);

		tmat = tmat.inverse().eval();

		const double *d = x_;
		const int pointNum = (n_ - 4) / 3;
		T plane[4] = { T(d[0]), T(d[1]), T(d[2]), T(d[3]) };
		Eigen::Matrix<T, 4, 1> point;
		T sum = T(0.0);
		for (int i = 0, k = 4; i < pointNum; i++, k += 3)
		{
			point(0, 0) = T(d[k]);
			point(1, 0) = T(d[k + 1]);
			point(2, 0) = T(d[k + 2]);
			point(3, 0) = T(1.0);
			point = tmat*point;
			T x = point(0, 0);
			T y = point(1, 0);
			T z = point(2, 0);
			T d = abs(plane[0] * x + plane[1] * y + plane[2] * z + plane[3]);
			d = d / sqrt(plane[0] * plane[0] + plane[1] * plane[1] + plane[2] * plane[2]);
			//sum += d;
			if (d > sum)
			{
				sum = d;
			}
		}
		//sum /= T(pointNum);
		residual[0] = sum;
		return true;
	}

private:
	const double *x_;
	unsigned int n_;
};

PlaneExtrinsicOptimization::PlaneExtrinsicOptimization()
{
	for (size_t i = 0; i < 6; i++)
	{
		m_trans_vec[i] = 0;
	}
	Init();
}

PlaneExtrinsicOptimization::~PlaneExtrinsicOptimization()
{

}

bool PlaneExtrinsicOptimization::Calibrate(double src_ransac_thre /*= 0*/, double dst_ransac_thre /*= 0*/)
{
	int kNumObservations = m_data_src.size();
	if (kNumObservations !=m_data_dst.size())
	{
		std::cout << "Src Size != Dst Size" << std::endl;
		return false;
	}
	std::vector<std::vector<cv::Point3d>> data_src = m_data_src;
	std::vector<std::vector<cv::Point3d>> data_dst = m_data_dst;

	std::vector<int> valid_list(kNumObservations);
	kNumObservations = 0;
	for (size_t i = 0; i < data_src.size(); i++)
	{
		if (data_src[i].size() > 0 && data_dst[i].size() > 0)
		{
			valid_list[i] = kNumObservations;
			kNumObservations++;
		}
		else
			valid_list[i] = -1;
	}
	std::vector<std::vector<double>> dataSet0, dataSet1;
	//组织数据
	dataSet0.resize(kNumObservations);
	dataSet1.resize(kNumObservations);

 #pragma omp parallel for
	for (int i = 0; i < valid_list.size(); i++)
	{
		if (valid_list[i]!=-1)
		{
			double plane[4];
			int index = valid_list[i];
			//拟合参考平面（src平面）
			PlanePointsOptimize(data_src[i], src_ransac_thre, plane);
			int dataNum = 4 + data_dst[i].size() * 3;
			dataSet0[index].resize(dataNum);
			//src平面信息
			for (int j = 0; j < 4; j++)
			{
				//dataSet0前四位为src平面参数
				dataSet0[index][j] = plane[j];
			}
			//点集为dst
			for (int j = 0, k = 4; j < data_dst[i].size(); j++, k += 3)
			{
				//dataSet0后面为dst平面点集
				cv::Point3d &p = data_dst[i][j];
				dataSet0[index][k] = p.x;
				dataSet0[index][k + 1] = p.y;
				dataSet0[index][k + 2] = p.z;
			}
			//待转换平面(dst平面)
			PlanePointsOptimize(data_dst[i], dst_ransac_thre, plane);
			dataNum = 4 + data_src[i].size() * 3;
			dataSet1[index].resize(dataNum);
			//dst平面信息
			for (int j = 0; j < 4; j++)
			{
				//dataSet1前四位为dst平面参数
				dataSet1[index][j] = plane[j];
			}
			//点集为src
			for (int j = 0, k = 4; j < data_src[i].size(); j++, k += 3)
			{
				//dataSet1后面为src平面点集
				cv::Point3d &p = data_src[i][j];
				dataSet1[index][k] = p.x;
				dataSet1[index][k + 1] = p.y;
				dataSet1[index][k + 2] = p.z;
			}
		}
	}

	// 第二部分：构建寻优问题
	Problem problem;
	//注意dataSet不要释放,ExponentialResidual中使用了这块数据
	for (int i = 0; i < kNumObservations; ++i) {
		
		CostFunction* cost_function =
			//使用自动求导，将之前的代价函数结构体传入,第一个1是输出维度，即残差的维度（个数），第二个6是输入维度，即待寻优参数（未知数）x的维度（个数）
			new AutoDiffCostFunction<ExponentialResidual, 1, 6>(
				new ExponentialResidual(dataSet0[i].data(), (int)dataSet0[i].size()));
		//向问题中添加误差项
		problem.AddResidualBlock(cost_function,
			new CauchyLoss(0.5),//添加损失函数，损失比例为0.5
			m_trans_vec);
		//增加反向约束
		CostFunction* cost_functionINV =
			new AutoDiffCostFunction<ExponentialResidualINV, 1, 6>(
				new ExponentialResidualINV(dataSet1[i].data(), (int)dataSet1[i].size()));
		problem.AddResidualBlock(cost_functionINV,
			new CauchyLoss(0.5),
			m_trans_vec);
	}
	//第三部分： 配置并运行求解器
	Solver::Options options;
	options.linear_solver_type = ceres::DENSE_QR;         //配置增量方程的解法
	options.use_nonmonotonic_steps = true;
	options.max_consecutive_nonmonotonic_steps = 10;      //防止找到局部极值
	options.num_threads = omp_get_max_threads();
	options.minimizer_progress_to_stdout = true;		  //标准输出
	options.max_num_iterations = 10000;                   //迭代次数
	//优化信息
	Solver::Summary summary;
	//求解
	Solve(options, &problem, &summary);
	//输出优化的简要信息
	std::cout << summary.BriefReport() << std::endl;
	//输出外参
	std::cout << VecToMat44(m_trans_vec, m_trans_vec + 3) << std::endl;

// 	 FileStorage fs_("RT.yaml", FileStorage::WRITE);
// 	 fs_ << "RT" << VecToMat44(m_trans_vec, m_trans_vec + 3);

	m_calibrated = true;
 	return true;

}

void PlaneExtrinsicOptimization::Init(int count)
{
	m_calibrated = false;
	m_data_src.clear();
	m_data_src.resize(count);
	m_data_dst.clear();
	m_data_dst.resize(count);
}

bool PlaneExtrinsicOptimization::SaveCalibResult(const char *path)
{
	if (!m_calibrated)
	{
		return false;
	}
	int kNumObservations = m_data_src.size();
	if (kNumObservations != m_data_dst.size())
	{
		return false;
	}
	cv::FileStorage fs(path, cv::FileStorage::WRITE);
	fs << "TransMatFromDstToSrc" << VecToMat44(m_trans_vec,m_trans_vec+3);
	fs << "SrcPlanePoints" << m_data_src;
	fs << "DstPlanePoints" << m_data_dst;
	return true;
}

bool PlaneExtrinsicOptimization::LoadCalibResult(const char *path)
{
	cv::FileStorage fs(path, cv::FileStorage::READ);
	if (fs.isOpened() == false)
		return false;
	cv::Mat mat;
	fs["TransMatFromDstToSrc"] >> mat;
	fs ["SrcPlanePoints"] >> m_data_src;
	fs ["DstPlanePoints"] >> m_data_dst;
	if (!mat.empty())
	{
		Mat44ToVec(mat, m_trans_vec, m_trans_vec + 3);
		m_calibrated = true;
	}
	return true;
}

void PlaneExtrinsicOptimization::SetSrcData(const std::vector<cv::Point3d> &data, int index )
{
	if (index == -1)
		m_data_src.push_back(data);
	else
		m_data_src[index] = data;
}

void PlaneExtrinsicOptimization::SetDstData(const std::vector<cv::Point3d> &data, int index )
{
	if (index == -1)
		m_data_dst.push_back(data);
	else
		m_data_dst[index] = data;
}

bool PlaneExtrinsicOptimization::GetSrcData(std::vector<cv::Point3d> &data, int index)
{
	if (index < 0 || index >= m_data_src.size() - 1)
		return false;
	data = m_data_src[index];
	return true;
}

bool PlaneExtrinsicOptimization::GetDstData(std::vector<cv::Point3d> &data, int index)
{
	if (index < 0 || index >= m_data_dst.size() - 1)
		return false;
	data = m_data_dst[index];
	return true;
}

bool PlaneExtrinsicOptimization::GetCalibrated() const
{
	return m_calibrated;
}

cv::Mat PlaneExtrinsicOptimization::GetExtrinsicMat() const
{
	return VecToMat44(m_trans_vec,m_trans_vec+3);//加3的意义取后3个值
}

void PlaneExtrinsicOptimization::SetExtrinsicMat(const cv::Mat &RT)
{
	Mat44ToVec(RT, m_trans_vec, m_trans_vec + 3);//加3的意义取后3个值
}

//************************************
// Method:    PlanePointsOptimize
// FullName:  PlaneExtrinsicOptimization::PlanePointsOptimize
// Access:    protected 
// Returns:   void
// Qualifier:
// Parameter: std::vector<cv::Point3d> & pts---待拟合坐标点集合
// Parameter: double distance----阈值
// Parameter: double plane[4]---拟合平面
//************************************
void PlaneExtrinsicOptimization::PlanePointsOptimize(std::vector<cv::Point3d>& pts, double distance, double plane[4])
{
	/************************************************************************/
	/* Ransac拟合平面内点优化                                               */
	/************************************************************************/
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	std::vector<int> index_list;
	cloud->resize(pts.size());
	for (int i = 0; i < pts.size(); i++)
	{
		const cv::Point3d &p = pts.at(i);
		cloud->at(i).x = p.x;
		cloud->at(i).y = p.y;
		cloud->at(i).z = p.z;
	}
	pcl::SampleConsensusModelPlane<pcl::PointXYZ>::Ptr model_plane(new pcl::SampleConsensusModelPlane<pcl::PointXYZ>(cloud));
	pcl::RandomSampleConsensus<pcl::PointXYZ> ransac(model_plane);
	ransac.setDistanceThreshold(distance);
	ransac.computeModel();
	ransac.getInliers(index_list);
	Eigen::VectorXf coeff;
	ransac.getModelCoefficients(coeff);
	for (int i = 0; i < 4; i++)
		plane[i] = coeff(i);
	std::vector<cv::Point3d> result(index_list.size());
	for (size_t i = 0; i < index_list.size(); i++)
		result[i]=pts[index_list[i]];
	pts = result;
}

//************************************
// Method:    ComputePlaneCoeff------平面拟合
// FullName:  PlaneExtrinsicOptimization::ComputePlaneCoeff
// Access:    protected 
// Returns:   void
// Qualifier:
// Parameter: const std::vector<cv::Point3d> & points---待拟合的坐标集合
// Parameter: double plane[4]----拟合平面的系数（A,B,C,D）
//************************************
void PlaneExtrinsicOptimization::ComputePlaneCoeff(const std::vector<cv::Point3d> &points, double plane[4])
{
	int pointsNum = points.size();
	cv::Mat A(pointsNum, 3, CV_64FC1);
	cv::Mat B(pointsNum, 1, CV_64FC1, -1.0);
	cv::Mat X(3, 1, CV_64FC1);
	for (int i = 0; i < pointsNum; i++)
	{
		const cv::Point3d &p = points[i];
		A.at<double>(i, 0) = p.x;
		A.at<double>(i, 1) = p.y;
		A.at<double>(i, 2) = p.z;
	}
	//最小二乘法拟合平面
	cv::solve(A, B, X, cv::DECOMP_QR);
	plane[0] = X.at<double>(0, 0);
	plane[1] = X.at<double>(1, 0);
	plane[2] = X.at<double>(2, 0);
	plane[3] = 1;
}

void PlaneExtrinsicOptimization::Mat44ToVec(const cv::Mat &mat44, double R[3], double T[3])
{
	double R33[9];
	for (size_t i = 0; i < 3; i++)
	{
		for (size_t j = 0; j < 3; j++)
		{
			R33[i * 3 + j] = mat44.at<double>(i, j);
		}
	}
	ceres::RotationMatrixToAngleAxis<double>(R33, R);
	T[0] = mat44.at<double>(0, 3);
	T[1] = mat44.at<double>(1, 3);
	T[2] = mat44.at<double>(2, 3);
}

cv::Mat PlaneExtrinsicOptimization::VecToMat44(const double R[3], const double T[3]) const
{
	double R33[9];
	cv::Mat mat44=cv::Mat::eye(4,4,CV_64FC1);
	ceres::AngleAxisToRotationMatrix<double>(R,R33);
	for (size_t i = 0; i < 3; i++)
	{
		for (size_t j = 0; j < 3; j++)
		{
			mat44.at<double>(i, j)=R33[i * 3 + j];
		}
	}
	mat44.at<double>(0, 3) = T[0];
	mat44.at<double>(1, 3) = T[1];
	mat44.at<double>(2, 3) = T[2];
	return mat44.clone();
}
