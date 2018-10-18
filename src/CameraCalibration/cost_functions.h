// COLMAP - Structure-from-Motion and Multi-View Stereo.
// Copyright (C) 2017  Johannes L. Schoenberger <jsch at inf.ethz.ch>
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.

#ifndef COST_FUNCTIONS_H_
#define COST_FUNCTIONS_H_

#include <Eigen/Core>
#include <iomanip>
#include <ceres/ceres.h>
#include <ceres/rotation.h>
// ceres using bundle adjustment to optimize the pose 

// 	{
// 		double camera_params_data[12] = { curr_->camera_->fx_,curr_->camera_->fy_,curr_->camera_->cx_,curr_->camera_->cy_,0 };
// 		ceres::Problem problem;
// 		ceres::LossFunction* loss_function =
// 			new ceres::CauchyLoss(1.0);
// 		Eigen::Quaterniond q(Common_GQ::toMatrix3d(R));
// 		Vector3d eigen_tvec = Vector3d(tvec.at<double>(0, 0), tvec.at<double>(1, 0), tvec.at<double>(2, 0));
// 		Eigen::Vector4d qvec(q.w(), q.x(), q.y(), q.z());            //x,y,z,w -> w,x,y,z
// 		std::vector<Eigen::Vector3d> points3D_copy(inliers.rows);
// 		for (int i = 0; i < inliers.rows; i++)
// 		{
// 			int index = inliers.at<int>(i, 0);
// 			points3D_copy[i] = Vector3d(pts3d[index].x, pts3d[index].y, pts3d[index].z);
// 			ceres::CostFunction* cost_function = colmap::BundleAdjustmentCostFunction<colmap::FullOpenCVCameraModel>::Create(Vector2d(pts2d[index].x, pts2d[index].y));
// 			problem.AddResidualBlock(cost_function, loss_function, qvec.data(), eigen_tvec.data(), points3D_copy[i].data(), camera_params_data);
// 			problem.SetParameterBlockConstant(points3D_copy[i].data());
// 		}
// 		problem.SetParameterBlockConstant(camera_params_data);
// 
// 		ceres::Solver::Options solver_options;
// 		solver_options.gradient_tolerance = 1.0;
// 		solver_options.max_num_iterations = 1000;
// 		solver_options.linear_solver_type = ceres::DENSE_QR;
// 		solver_options.minimizer_progress_to_stdout = false;
// 		solver_options.logging_type = ceres::LoggingType::SILENT;
// 		solver_options.num_threads = omp_get_max_threads();
// 		solver_options.num_linear_solver_threads = omp_get_max_threads();
// 		ceres::Solver::Summary summary;
// 		ceres::Solve(solver_options, &problem, &summary);
// 		PrintSolverSummary(summary);
// 		//  		cout << "Optimise: "<<summary.IsSolutionUsable() << endl;
// 		T_c_r_estimated_ = SE3(Eigen::Quaterniond(qvec(0), qvec(1), qvec(2), qvec(3)).toRotationMatrix(), eigen_tvec);
// 
// 	}

class PinholeCameraModelCostFunction {
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

		explicit PinholeCameraModelCostFunction(const Eigen::Vector3d &point3D, const Eigen::Vector2d &point2D, const double * para)
		: point3D_(point3D), point2D_(point2D), para_(para) {}

	static ceres::CostFunction* Create(const Eigen::Vector3d & point3D, const Eigen::Vector2d &point2D, const double * para) {
		return (new ceres::AutoDiffCostFunction<
			PinholeCameraModelCostFunction, 1, 6>(	// “1”残差维度 | “6”待优化参数维度，对应于operator()函数参数
				new PinholeCameraModelCostFunction(point3D, point2D, para)));
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

		Eigen::Matrix<T, 3, 1> objRT;
		objRT(0, 0) = T(point3D_[0]);
		objRT(1, 0) = T(point3D_[1]);
		objRT(2, 0) = T(point3D_[2]);

		objRT = R_*objRT + T_;
		objRT /= objRT(2, 0);

		T pix[2];
		pix[0] = fx*objRT(0, 0) + cx;
		pix[1] = fy*objRT(1, 0) + cy;

		residuals[0] = ceres::sqrt(ceres::pow(pix[0] - point2D_[0], 2) + ceres::pow(pix[1] - point2D_[1], 2));

		return true;
	}

private:
	Eigen::Vector3d point3D_;
	Eigen::Vector2d point2D_;
	const double * para_;
};

namespace colmap {


	inline void PrintSolverSummary(const ceres::Solver::Summary& summary)
	{
		std::cout << std::right << std::setw(16) << "Residuals : ";
		std::cout << std::left << summary.num_residuals_reduced << std::endl;

		std::cout << std::right << std::setw(16) << "Parameters : ";
		std::cout << std::left << summary.num_effective_parameters_reduced
			<< std::endl;

		std::cout << std::right << std::setw(16) << "Iterations : ";
		std::cout << std::left
			<< summary.num_successful_steps + summary.num_unsuccessful_steps
			<< std::endl;

		std::cout << std::right << std::setw(16) << "Time : ";
		std::cout << std::left << summary.total_time_in_seconds << " [s]"
			<< std::endl;

		std::cout << std::right << std::setw(16) << "Initial cost : ";
		std::cout << std::right << std::setprecision(6)
			<< std::sqrt(summary.initial_cost / summary.num_residuals_reduced)
			<< " [px]" << std::endl;

		std::cout << std::right << std::setw(16) << "Final cost : ";
		std::cout << std::right << std::setprecision(6)
			<< std::sqrt(summary.final_cost / summary.num_residuals_reduced)
			<< " [px]" << std::endl;

		std::cout << std::right << std::setw(16) << "Termination : ";

		std::string termination = "";

		switch (summary.termination_type) {
		case ceres::CONVERGENCE:
			termination = "Convergence";
			break;
		case ceres::NO_CONVERGENCE:
			termination = "No convergence";
			break;
		case ceres::FAILURE:
			termination = "Failure";
			break;
		case ceres::USER_SUCCESS:
			termination = "User success";
			break;
		case ceres::USER_FAILURE:
			termination = "User failure";
			break;
		default:
			termination = "Unknown";
			break;
		}

		std::cout << std::right << termination << std::endl;
		std::cout << std::endl;
	}

// Standard bundle adjustment cost function for variable
// camera pose and calibration and point parameters.
template <typename CameraModel>
class BundleAdjustmentCostFunction {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  explicit BundleAdjustmentCostFunction(const Eigen::Vector2d& point2D)
      : point2D_(point2D) {}

  static ceres::CostFunction* Create(const Eigen::Vector2d& point2D) {
    return (new ceres::AutoDiffCostFunction<
            BundleAdjustmentCostFunction<CameraModel>, 2, 4, 3, 3,            //第一个参数代表残差个数，后面开始跟operator()参数对应
            CameraModel::kNumParams>(
        new BundleAdjustmentCostFunction(point2D)));
  }

  template <typename T>
  bool operator()(const T* const qvec, const T* const tvec,
                  const T* const point3D, const T* const camera_params,
                  T* residuals) const {
    // Rotate and translate.
    T point3D_local[3];
    ceres::UnitQuaternionRotatePoint(qvec, point3D, point3D_local);
    point3D_local[0] += tvec[0];
    point3D_local[1] += tvec[1];
    point3D_local[2] += tvec[2];

    // Normalize to image plane.
    point3D_local[0] /= point3D_local[2];
    point3D_local[1] /= point3D_local[2];

    // Distort and transform to pixel space.
    T x, y;
    CameraModel::WorldToImage(camera_params, point3D_local[0], point3D_local[1],
                              &x, &y);

    // Re-projection error.
    residuals[0] = pow(x - T(point2D_(0)),2);
    residuals[1] = pow(y - T(point2D_(1)),2);

    return true;
  }

 private:
  const Eigen::Vector2d point2D_;
};

// Bundle adjustment cost function for variable
// camera calibration and point parameters, and fixed camera pose.
template <typename CameraModel>
class BundleAdjustmentConstantPoseCostFunction {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  BundleAdjustmentConstantPoseCostFunction(const Eigen::Vector4d& qvec,
                                           const Eigen::Vector3d& tvec,
                                           const Eigen::Vector2d& point2D)
      : qvec_(qvec), tvec_(tvec), point2D_(point2D) {}

  static ceres::CostFunction* Create(const Eigen::Vector4d& qvec,
                                     const Eigen::Vector3d& tvec,
                                     const Eigen::Vector2d& point2D) {
    return (new ceres::AutoDiffCostFunction<
            BundleAdjustmentConstantPoseCostFunction<CameraModel>, 2, 3,
            CameraModel::kNumParams>(
        new BundleAdjustmentConstantPoseCostFunction(qvec, tvec, point2D)));
  }

  template <typename T>
  bool operator()(const T* const point3D, const T* const camera_params,
                  T* residuals) const {
    T qvec[4] = {T(qvec_(0)), T(qvec_(1)), T(qvec_(2)), T(qvec_(3))};

    // Rotate and translate.
    T point3D_local[3];
    ceres::UnitQuaternionRotatePoint(qvec, point3D, point3D_local);
    point3D_local[0] += T(tvec_(0));
    point3D_local[1] += T(tvec_(1));
    point3D_local[2] += T(tvec_(2));

    // Normalize to image plane.
    point3D_local[0] /= point3D_local[2];
    point3D_local[1] /= point3D_local[2];

    // Distort and transform to pixel space.
    T x, y;
    CameraModel::WorldToImage(camera_params, point3D_local[0], point3D_local[1],
                              &x, &y);

    // Re-projection error.
    residuals[0] = x - T(point2D_(0));
    residuals[1] = y - T(point2D_(1));

    return true;
  }

 private:
  const Eigen::Vector4d qvec_;
  const Eigen::Vector3d tvec_;
  const Eigen::Vector2d point2D_;
};

// Rig bundle adjustment cost function for variable camera pose and calibration
// and point parameters. Different from the standard bundle adjustment function,
// this cost function is suitable for camera rigs with consistent relative poses
// of the cameras within the rig. The cost function first projects points into
// the local system of the camera rig and then into the local system of the
// camera within the rig.
template <typename CameraModel>
class RigBundleAdjustmentCostFunction {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  explicit RigBundleAdjustmentCostFunction(const Eigen::Vector2d& point2D)
      : point2D_(point2D) {}

  static ceres::CostFunction* Create(const Eigen::Vector2d& point2D) {
    return (new ceres::AutoDiffCostFunction<
            RigBundleAdjustmentCostFunction<CameraModel>, 2, 4, 3, 4, 3, 3,
            CameraModel::kNumParams>(
        new RigBundleAdjustmentCostFunction(point2D)));
  }

  template <typename T>
  bool operator()(const T* const rig_qvec, const T* const rig_tvec,
                  const T* const rel_qvec, const T* const rel_tvec,
                  const T* const point3D, const T* const camera_params,
                  T* residuals) const {
    // Concatenate rotations.
    T qvec[4];
    ceres::QuaternionProduct(rel_qvec, rig_qvec, qvec);

    // Concatenate translations.
    T tvec[3];
    ceres::UnitQuaternionRotatePoint(rel_qvec, rig_tvec, tvec);
    tvec[0] += rel_tvec[0];
    tvec[1] += rel_tvec[1];
    tvec[2] += rel_tvec[2];

    // Rotate and translate.
    T point3D_local[3];
    ceres::UnitQuaternionRotatePoint(qvec, point3D, point3D_local);
    point3D_local[0] += tvec[0];
    point3D_local[1] += tvec[1];
    point3D_local[2] += tvec[2];

    // Normalize to image plane.
    point3D_local[0] /= point3D_local[2];
    point3D_local[1] /= point3D_local[2];

    // Distort and transform to pixel space.
    T x, y;
    CameraModel::WorldToImage(camera_params, point3D_local[0], point3D_local[1],
                              &x, &y);

    // Re-projection error.
    residuals[0] = x - T(point2D_(0));
    residuals[1] = y - T(point2D_(1));

    return true;
  }

 private:
  const Eigen::Vector2d point2D_;
};

// Cost function for refining two-view geometry based on the Sampson-Error.
//
// First pose is assumed to be located at the origin with 0 rotation. Second
// pose is assumed to be on the unit sphere around the first pose, i.e. the
// pose of the second camera is parameterized by a 3D rotation and a
// 3D translation with unit norm. `tvec` is therefore over-parameterized as is
// and should be down-projected using `HomogeneousVectorParameterization`.
class RelativePoseCostFunction {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  RelativePoseCostFunction(const Eigen::Vector2d& x1, const Eigen::Vector2d& x2)
      : x1_(x1), x2_(x2) {}

  static ceres::CostFunction* Create(const Eigen::Vector2d& x1,
                                     const Eigen::Vector2d& x2) {
    return (new ceres::AutoDiffCostFunction<RelativePoseCostFunction, 1, 4, 3>(
        new RelativePoseCostFunction(x1, x2)));
  }

  template <typename T>
  bool operator()(const T* const qvec, const T* const tvec,
                  T* residuals) const {
    Eigen::Matrix<T, 3, 3, Eigen::RowMajor> R;
    ceres::QuaternionToRotation(qvec, R.data());

    // Matrix representation of the cross product t x R.
    Eigen::Matrix<T, 3, 3> t_x;
    t_x << T(0), -tvec[2], tvec[1], tvec[2], T(0), -tvec[0], -tvec[1], tvec[0],
        T(0);

    // Essential matrix.
    const Eigen::Matrix<T, 3, 3> E = t_x * R;

    // Homogeneous image coordinates.
    const Eigen::Matrix<T, 3, 1> x1_h(T(x1_(0)), T(x1_(1)), T(1));
    const Eigen::Matrix<T, 3, 1> x2_h(T(x2_(0)), T(x2_(1)), T(1));

    // Squared sampson error.
    const Eigen::Matrix<T, 3, 1> Ex1 = E * x1_h;
    const Eigen::Matrix<T, 3, 1> Etx2 = E.transpose() * x2_h;
    const T x2tEx1 = x2_h.transpose() * Ex1;
    residuals[0] = x2tEx1 * x2tEx1 /
                   (Ex1(0) * Ex1(0) + Ex1(1) * Ex1(1) + Etx2(0) * Etx2(0) +
                    Etx2(1) * Etx2(1));

    return true;
  }

 private:
  const Eigen::Vector2d x1_;
  const Eigen::Vector2d x2_;
};

}  // namespace colmap

#endif  // COST_FUNCTIONS_H_
