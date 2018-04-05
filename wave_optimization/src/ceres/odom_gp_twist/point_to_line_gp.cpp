#include "wave/optimization/ceres/odom_gp_twist/point_to_line_gp.hpp"
#include <Eigen/QR>

namespace wave_optimization {

SE3PointToLineGP::SE3PointToLineGP(const double *const pA,
                                   const double *const pB,
                                   SE3PointToLineGPObjects &objects,
                                   const wave::Mat3 &CovZ,
                                   bool calculate_weight)
    : ptA(pA), ptB(pB), objects(objects) {

    this->objects.JP_T.setZero();
    this->objects.JP_T.block<3, 3>(0, 3).setIdentity();

    this->diff[0] = this->ptB[0] - this->ptA[0];
    this->diff[1] = this->ptB[1] - this->ptA[1];
    this->diff[2] = this->ptB[2] - this->ptA[2];
    this->bottom = diff[0] * diff[0] + diff[1] * diff[1] + diff[2] * diff[2];

    if (this->bottom < 1e-10) {
        throw std::out_of_range("Points defining line are too close!");
    }

    this->objects.Jres_P(0, 0) = 1 - (diff[0] * diff[0] / bottom);
    this->objects.Jres_P(0, 1) = -(diff[0] * diff[1] / bottom);
    this->objects.Jres_P(0, 2) = -(diff[0] * diff[2] / bottom);
    this->objects.Jres_P(1, 0) = -(diff[1] * diff[0] / bottom);
    this->objects.Jres_P(1, 1) = 1 - (diff[1] * diff[1] / bottom);
    this->objects.Jres_P(1, 2) = -(diff[1] * diff[2] / bottom);
    this->objects.Jres_P(2, 0) = -(diff[2] * diff[0] / bottom);
    this->objects.Jres_P(2, 1) = -(diff[2] * diff[1] / bottom);
    this->objects.Jres_P(2, 2) = 1 - (diff[2] * diff[2] / bottom);

    Eigen::Vector3d unitdiff;
    double invlength = 1.0 / sqrt(this->bottom);
    if (this->diff[2] > 0) {
        unitdiff[0] = this->diff[0] * invlength;
        unitdiff[1] = this->diff[1] * invlength;
        unitdiff[2] = this->diff[2] * invlength;
    } else {
        unitdiff[0] = -this->diff[0] * invlength;
        unitdiff[1] = -this->diff[1] * invlength;
        unitdiff[2] = -this->diff[2] * invlength;
    }

    Eigen::Vector3d unitz;
    unitz << 0, 0, 1;

    auto v = unitdiff.cross(unitz);
    auto s = v.norm();
    auto c = unitz.dot(unitdiff);
    auto skew = wave::Transformation<>::skewSymmetric3(v);
    this->objects.rotation = Eigen::Matrix3d::Identity() + skew + skew * skew * ((1 - c) / (s * s));

    this->objects.Jres_P = this->objects.rotation * this->objects.Jres_P;

    if (calculate_weight) {
        auto rotated = this->objects.Jres_P * CovZ * this->objects.Jres_P.transpose();
        this->weight_matrix = rotated.block<2, 2>(0, 0).inverse().sqrt();
    } else {
        this->weight_matrix.setIdentity();
    }
}

bool SE3PointToLineGP::Evaluate(double const *const *parameters, double *residuals, double **jacobians) const {
    Eigen::Map<const wave::Vec6> tk_map(parameters[0], 6, 1);
    Eigen::Map<const wave::Vec6> tkp1_map(parameters[1], 6, 1);
    Eigen::Map<const wave::Vec6> vel_k(parameters[2], 6, 1);
    Eigen::Map<const wave::Vec6> vel_kp1(parameters[3], 6, 1);

    this->objects.T_cur_twist = this->objects.hat.block<6, 6>(0, 0) * tk_map +
                                this->objects.hat.block<6, 6>(0, 6) * vel_k +
                                this->objects.candle.block<6, 6>(0, 0) * tkp1_map +
                                this->objects.candle.block<6, 6>(0, 6) * vel_kp1;

    this->objects.T_current.setFromExpMap(this->objects.T_cur_twist);

    wave::Vec3 point = this->objects.T_current.transform(this->objects.T0_pt);

    double p_A[3] = {point(0) - this->ptA[0], point(1) - this->ptA[1], point(2) - this->ptA[2]};

    double scaling = ceres::DotProduct(p_A, diff);
    // point on line closest to point
    double p_Tl[3] = {this->ptA[0] + (scaling / bottom) * diff[0],
                      this->ptA[1] + (scaling / bottom) * diff[1],
                      this->ptA[2] + (scaling / bottom) * diff[2]};

    Eigen::Map<const wave::Vec3> pt_Tl(p_Tl, 3, 1);
    Eigen::Map<Eigen::Vector2d> reduced(residuals, 2, 1);

    reduced = this->weight_matrix * (this->objects.rotation * (point - pt_Tl)).block<2, 1>(0, 0);

    if (jacobians != nullptr) {
        this->objects.JP_T(0, 1) = point(2);
        this->objects.JP_T(0, 2) = -point(1);
        this->objects.JP_T(1, 0) = -point(2);
        this->objects.JP_T(1, 2) = point(0);
        this->objects.JP_T(2, 0) = point(1);
        this->objects.JP_T(2, 1) = -point(0);

        // Jres_P already has rotation incorporated during construction
        this->objects.Jr_T = this->objects.Jres_P * this->objects.JP_T;

        wave::Mat6 Jexp = wave::Transformation<>::SE3ApproxLeftJacobian(this->objects.T_cur_twist);

        // Remains to be seen whether the jacobian of the exponential map matters
        if (jacobians[0]) {
            Eigen::Map<Eigen::Matrix<double, 2, 6, Eigen::RowMajor>> Jr_Tk(jacobians[0], 2, 6);
            Jr_Tk.block<2, 6>(0, 0) = this->weight_matrix * this->objects.Jr_T.block<2, 6>(0, 0) * Jexp * this->objects.hat.block<6, 6>(0,0);
        }
        if (jacobians[1]) {
            Eigen::Map<Eigen::Matrix<double, 2, 6, Eigen::RowMajor>> Jr_Tkp1(jacobians[1], 2, 6);
            Jr_Tkp1.block<2, 6>(0, 0) = this->weight_matrix * this->objects.Jr_T.block<2, 6>(0, 0) * Jexp * this->objects.candle.block<6, 6>(0,0);
        }
        if (jacobians[2]) {
            Eigen::Map<Eigen::Matrix<double, 2, 6, Eigen::RowMajor>> jac_map(jacobians[2], 2, 6);
            jac_map = this->weight_matrix * this->objects.Jr_T.block<2, 6>(0, 0) * Jexp * this->objects.hat.block<6, 6>(0,6);
        }
        if (jacobians[3]) {
            Eigen::Map<Eigen::Matrix<double, 2, 6, Eigen::RowMajor>> jac_map(jacobians[3], 2, 6);
            jac_map = this->weight_matrix * this->objects.Jr_T.block<2, 6>(0, 0) * Jexp * this->objects.candle.block<6, 6>(0,6);
        }
    }

    return true;
}

}  // namespace wave
