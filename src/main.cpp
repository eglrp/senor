//#include "CornerFinder/CornerFinder.h"
#include <omp.h>
#include <opencv2/opencv.hpp>
#include <opencv2/ccalib/omnidir.hpp>
#include <iostream>
#include <Common_GQ.h>
#include <CameraCalibration/OmniCamera.h>
//#include <CameraCalibration/OmniCamera.h>

#include <ceres/ceres.h>
#include <ceres/rotation.h>

using namespace std;
using namespace cv;
using namespace Common_GQ;


class BundleAdjustmentCostFunction {
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	explicit BundleAdjustmentCostFunction(const Eigen::Vector3d &point3D, const Eigen::Vector2d &point2D, const double * para)
		: point3D_(point3D), point2D_(point2D), para_(para){}

	static ceres::CostFunction* Create(const Eigen::Vector3d & point3D, const Eigen::Vector2d &point2D, const double * para) {
		return (new ceres::AutoDiffCostFunction<
			BundleAdjustmentCostFunction, 1, 6>(	// “1”残差维度 | “6”待优化参数维度，对应于operator()函数参数
				new BundleAdjustmentCostFunction(point3D, point2D, para)));
	}

	template <typename T>
	bool operator()(const T* const rt, T* residuals) const {
		
		T x[6] = { T(rt[0]), T(rt[1]), T(rt[2]), T(rt[3]), T(rt[4]), T(rt[5]) };
		T R9[9];
		ceres::AngleAxisToRotationMatrix(rt, R9);
		Eigen::Matrix<T, 3, 3> R_(R9);

		Eigen::Matrix<T, 3, 1> T_;
		T_ << x[3], x[4], x[5];
		T fx = T(para_[0]);
		T fy = T(para_[1]);
		T cx = T(para_[2]);
		T cy = T(para_[3]);
		T D_[4] = { T(para_[4]), T(para_[5]), T(para_[6]), T(para_[7])};
		T Xi = T(para_[8]);

		T s = T(0); 
		Eigen::Matrix<T, 3, 1> objRT;
		objRT(0, 0) = T(point3D_[0]);
		objRT(1, 0) = T(point3D_[1]);
		objRT(2, 0) = T(point3D_[2]);

		objRT = R_*objRT + T_;
		T norm_ = ceres::sqrt(ceres::pow(objRT(0, 0), 2) + ceres::pow(objRT(1, 0), 2) + ceres::pow(objRT(2, 0), 2));
		objRT /= norm_;
		T z_ = objRT(2, 0) + Xi;
		objRT /= z_;

		T x0 = objRT(0, 0);
		T y0 = objRT(1, 0);
		T r2 = x0*x0 + y0*y0;

		T xd[2];
		xd[0] = x0*(T(1) + D_[0] * r2 + D_[1] * r2*r2) + T(2)*D_[2] * x0*y0 + D_[3] * (r2 + T(2)*x0*x0);
		xd[1] = y0*(T(1) + D_[0] * r2 + D_[1] * r2*r2) + D_[2] * (r2 + T(2)*y0*y0) + T(2)*D_[3] * x0*y0;

		T pix[2];
		pix[0] = fx*xd[0] + cx;
		pix[1] = fy*xd[1] + cy;

		residuals[0] = ceres::sqrt(ceres::pow(pix[0] - point2D_[0], 2) + ceres::pow(pix[1] - point2D_[1], 2));
		return true;
	}

private:
	Eigen::Vector3d point3D_;
	Eigen::Vector2d point2D_;
	const double * para_;
};

void main()
{
	cv::Mat K, D, Xi, R, T;

	vector<Vec3d> vobj;
	vector<Vec2d> vimg;
	FileStorage fs("246.yaml", FileStorage::READ);
	fs["K"] >> K;
	fs["D"] >> D;
	fs["Xi"] >> Xi;
	fs["R"] >> R;
	fs["T"] >> T;
	fs["objPoints"] >> vobj;
	fs["imgPoints"] >> vimg;
	fs.release();


// 	vector<Eigen::Vector3d> objPoints;
// 	vector<Eigen::Vector2d> imgPoints;
// 	for (int i = 0; i < vobj.size(); i++)
// 	{
// 		objPoints.push_back(Eigen::Vector3d(vobj[i][0], vobj[i][1], vobj[i][2]));
// 		imgPoints.push_back(Eigen::Vector2d(vimg[i][0], vimg[i][1]));
// 	}
// 
// 	double para[9];
// 	para[0] = K.at<double>(0, 0);
// 	para[1] = K.at<double>(1, 1);
// 	para[2] = K.at<double>(0, 2);
// 	para[3] = K.at<double>(1, 2);
// 	para[4] = D.at<double>(0);
// 	para[5] = D.at<double>(1);
// 	para[6] = D.at<double>(2);
// 	para[7] = D.at<double>(3);
// 	para[8] = Xi.at<double>(0);
// 
// 	srand((unsigned)time(NULL));
// 	double rt1 = rand()*1.0 / RAND_MAX;
// 	double rt2 = rand()*1.0 / RAND_MAX;
// 	double rt3 = rand()*1.0 / RAND_MAX;
// 	cout << rt1 << '\t' << rt2 << '\t' << rt3 << endl;
// 	double rt[6] = { rt1, rt2, rt3, 100, 100, 100};
// 	ceres::Problem problem;
// 	for (int i = 0; i < vobj.size(); i++)
// 	{
// 		ceres::CostFunction* cost_function = BundleAdjustmentCostFunction::Create(objPoints[i], imgPoints[i], para);
// 		problem.AddResidualBlock(cost_function, new ceres::CauchyLoss(0.5), rt);
// 	}
// 
// 	ceres::Solver::Options options;
// 	options.linear_solver_type = ceres::DENSE_QR;
// 	options.use_nonmonotonic_steps = true;
// 	options.max_consecutive_nonmonotonic_steps = 100;            //防止找到局部极值
// 	options.num_threads = omp_get_num_threads();
// 	options.minimizer_progress_to_stdout = true;
// 	options.max_num_iterations = 10000;            //迭代次数
// 	ceres::Solver::Summary summary;
// 	ceres::Solve(options, &problem, &summary);
// 
// 	std::cout << summary.BriefReport() << "\n";
// 	std::cout << rt[0] << " " << rt[1] << " " << rt[2] << std::endl;
// 	std::cout << rt[3] << " " << rt[4] << " " << rt[5] << std::endl;


	vector<Point3f> p3f(1);
	p3f[0] = Point3f(500, 500, 0);
	vector<Point2f> p2f;
// 	cv::Mat matR(3, 1, CV_64FC1), matT(3, 1, CV_64FC1);
// 	matR.at<double>(0) = rt[0];
// 	matR.at<double>(1) = rt[1];
// 	matR.at<double>(2) = rt[2];
// 	matT.at<double>(0) = rt[3];
// 	matT.at<double>(1) = rt[4];
// 	matT.at<double>(2) = rt[5];

	cv::omnidir::projectPoints(p3f, p2f, R, T, K, Xi.at<double>(0), D);
	cout << p2f << endl;

	
	
}