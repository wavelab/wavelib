#include <exception>
#include "wave/optimization/ceres/constant_velocity.hpp"

namespace wave {

/// the parameters are, in order, prev_transform, cur_transform, prev_vel, cur_velocity
bool ConstantVelocityPrior::Evaluate(double const *const *parameters, double *residuals, double **jacobians) const {
    auto tk_ptr = std::make_shared<Eigen::Map<const Eigen::Matrix<double, 3, 4>>>(parameters[0], 3, 4);
    auto tkp1_ptr = std::make_shared<Eigen::Map<const Eigen::Matrix<double, 3, 4>>>(parameters[1], 3, 4);

    Transformation<Eigen::Map<const Eigen::Matrix<double, 3, 4>>> Tk(tk_ptr);
    Transformation<Eigen::Map<const Eigen::Matrix<double, 3, 4>>> Tkp1(tkp1_ptr);

    Eigen::Map<const Vec6> prev_vel(parameters[2]);
    Eigen::Map<const Vec6>  cur_vel(parameters[3]);

    Eigen::Map<Eigen::Matrix<double, 12, 1>> res_map(residuals);

    Mat6 J_left, J_right;
    res_map.block<6,1>(0,0) = Tkp1.manifoldMinusAndJacobian(Tk, J_left, J_right) - this->delta_t * prev_vel;
    res_map.block<6,1>(6,0) = J_left * cur_vel - prev_vel;

    res_map = this->weight * res_map;

    if (jacobians) {
        Mat6 skew = Transformation<void>::skewSymmetric6(cur_vel);
        if (jacobians[0]) {
            Eigen::Map<Eigen::Matrix<double, 12, 12, Eigen::RowMajor>> jac_map(jacobians[0], 12, 12);
            jac_map.setZero();
            jac_map.block<6,6>(0,0) = J_right;
            jac_map.block<6,6>(6,0) = 0.5 * skew * J_right;
            jac_map.block<12, 6>(0,0) = this->weight * jac_map.block<12, 6>(0,0);
        }
        if (jacobians[1]) {
            Eigen::Map<Eigen::Matrix<double, 12, 12, Eigen::RowMajor>> jac_map(jacobians[1], 12, 12);
            jac_map.setZero();
            jac_map.block<6,6>(0,0) = J_left;
            jac_map.block<6,6>(6,0) = 0.5 * skew * J_left;
            jac_map.block<12, 6>(0,0) = this->weight * jac_map.block<12, 6>(0,0);
        }
        if (jacobians[2]) {
            Eigen::Map<Eigen::Matrix<double, 12, 6, Eigen::RowMajor>> jac_map(jacobians[2], 12, 6);
            jac_map.block<6, 6>(0,0) = - this->delta_t * Mat6::Identity();
            jac_map.block<6, 6>(6,0) = - Mat6::Identity();
            jac_map = this->weight * jac_map;
        }
        if (jacobians[3]) {
            Eigen::Map<Eigen::Matrix<double, 12, 6, Eigen::RowMajor>> jac_map(jacobians[3], 12, 6);
            jac_map.block<6,6>(6,0) = J_left;
            jac_map.block<6,6>(0,0).setZero();
            jac_map = this->weight * jac_map;
        }
    }
    return true;
}

}