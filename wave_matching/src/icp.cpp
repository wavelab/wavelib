#include "wave/utils/config.hpp"
#include "wave/matching/icp.hpp"

namespace wave {

ICPMatcherParams::ICPMatcherParams(const std::string &config_path) {
    ConfigParser parser;
    int covar_est_temp;
    parser.addParam("max_corr", &(this->max_corr));
    parser.addParam("max_iter", &(this->max_iter));
    parser.addParam("t_eps", &(this->t_eps));
    parser.addParam("lidar_ang_covar", &(this->lidar_ang_covar));
    parser.addParam("lidar_lin_covar", &(this->lidar_lin_covar));
    parser.addParam("covar_estimator", &covar_est_temp);
    parser.addParam("res", &(this->res));
    parser.addParam("multiscale_steps", &(this->multiscale_steps));

    if (parser.load(config_path) != ConfigStatus::OK) {
        throw std::runtime_error{"Failed to Load Matcher Config"};
    }

    if ((covar_est_temp >= ICPMatcherParams::covar_method::LUM) &&
        (covar_est_temp <= ICPMatcherParams::covar_method::LUMold)) {
        this->covar_estimator =
          static_cast<ICPMatcherParams::covar_method>(covar_est_temp);
    } else {
        LOG_ERROR("Invalid covariance estimate method, using LUM");
        this->covar_estimator = ICPMatcherParams::covar_method::LUM;
    }
}

ICPMatcher::ICPMatcher(ICPMatcherParams params1) : params(params1) {
    this->ref = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    this->target = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    this->final = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    this->downsampled_ref =
      std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    this->downsampled_target =
      std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

    if (this->params.res > 0) {
        this->filter.setLeafSize(
          this->params.res, this->params.res, this->params.res);
    }
    this->resolution = this->params.res;

    this->icp.setMaxCorrespondenceDistance(this->params.max_corr);
    this->icp.setMaximumIterations(this->params.max_iter);
    this->icp.setTransformationEpsilon(this->params.t_eps);
    this->icp.setEuclideanFitnessEpsilon(this->params.fit_eps);
}

ICPMatcher::~ICPMatcher() {
    if (this->ref) {
        this->ref.reset();
        this->downsampled_ref.reset();
    }
    if (this->target) {
        this->target.reset();
        this->downsampled_target.reset();
    }
    if (this->final) {
        this->final.reset();
    }
}

void ICPMatcher::setRef(const PCLPointCloudPtr &ref) {
    this->ref = ref;
}

void ICPMatcher::setTarget(const PCLPointCloudPtr &target) {
    this->target = target;
}

bool ICPMatcher::match() {
    if (this->params.res > 0) {
        if (this->params.multiscale_steps > 0) {
            Affine3 running_transform = Affine3::Identity();
            for (int i = this->params.multiscale_steps; i >= 0; i--) {
                float leaf_size = pow(2, i) * this->params.res;
                this->filter.setLeafSize(leaf_size, leaf_size, leaf_size);
                this->filter.setInputCloud(this->ref);
                this->filter.filter(*(this->downsampled_ref));
                pcl::transformPointCloud(*(this->downsampled_ref),
                                         *(this->downsampled_ref),
                                         running_transform);
                this->icp.setInputSource(this->downsampled_ref);

                this->filter.setInputCloud(this->target);
                this->filter.filter(*(this->downsampled_target));
                this->icp.setInputTarget(this->downsampled_target);

                this->icp.setMaxCorrespondenceDistance(pow(2, i) *
                                                       this->params.max_corr);
                this->icp.align(*(this->final));
                if (!icp.hasConverged()) {
                    return false;
                }
                running_transform.matrix() =
                  icp.getFinalTransformation().cast<double>() *
                  running_transform.matrix();
            }
            this->result = running_transform;
            return true;
        } else {
            this->filter.setLeafSize(
              this->params.res, this->params.res, this->params.res);
            this->filter.setInputCloud(this->ref);
            this->filter.filter(*(this->downsampled_ref));
            this->icp.setInputSource(this->downsampled_ref);

            this->filter.setInputCloud(this->target);
            this->filter.filter(*(this->downsampled_target));
            this->icp.setInputTarget(this->downsampled_target);

            this->icp.align(*(this->final));
            if (icp.hasConverged()) {
                this->result =
                  this->icp.getFinalTransformation().cast<double>();
                return true;
            }
        }
    } else {
        this->icp.setInputTarget(this->target);
        this->icp.setInputSource(this->ref);
        this->icp.align(*(this->final));
        if (this->icp.hasConverged()) {
            this->result.matrix() = icp.getFinalTransformation().cast<double>();
            return true;
        }
    }
    return false;
}

void ICPMatcher::estimateInfo() {
    switch (this->params.covar_estimator) {
        case ICPMatcherParams::covar_method::LUM: this->estimateLUM();
        case ICPMatcherParams::covar_method::CENSI: this->estimateCensi();
        case ICPMatcherParams::covar_method::LUMold: this->estimateLUMold();
        default: return;
    }
}

// This is an implementation of the Haralick or Censi covariance approximation
// for ICP
// The core idea behind this is that the covariance of the cost f'n J wrt
// optimization variable x is
// cov(x) ~= (d2J/dx2)^-1*(d2J/dzdx)*cov(z)*(d2J/dzdx)'*(d2J/dx2)^-1

// Idea was taken from
// https://censi.science/pub/research/2007-icra-icpcov.pdf

// This is an implementation for euler angles, what is below is a cleaned up
// version of
// http://ieeexplore.ieee.org/stamp/stamp.jsp?arnumber=7153246

//@INPROCEEDINGS{3d_icp_cov,
// author={Prakhya, S.M. and Liu Bingbing and Yan Rui and Weisi Lin},
//        booktitle={Machine Vision Applications (MVA), 2015 14th IAPR
//        International Conference on},
//        title={A closed-form estimate of 3D ICP covariance},
//        year={2015},
//        pages={526-529},
//        doi={10.1109/MVA.2015.7153246},
//        month={May},}

void ICPMatcher::estimateCensi() {
    auto &ref = this->ref;
    auto &target = this->target;
    if (this->params.res > 0) {
        ref = this->downsampled_ref;
        target = this->downsampled_target;
    }
    if (this->icp.hasConverged()) {
        const auto eulers = this->result.rotation().eulerAngles(0, 1, 2);
        const auto translation = this->result.translation();
        // set up aliases to shrink following lines
        const double &X1 = translation.x(), X2 = translation.y(),
                     X3 = translation.z();
        // precompute trig quantities
        double cr, sr, cp, sp, cy, sy;
        // r = roll, p = pitch, y = yaw, c = cos, s = sine
        cr = cos(eulers[0]);
        sr = sin(eulers[0]);
        cp = cos(eulers[1]);
        sp = sin(eulers[1]);
        cy = cos(eulers[2]);
        sy = sin(eulers[2]);

        Eigen::DiagonalMatrix<double, 6> sphere_cov;
        sphere_cov.diagonal() << this->params.lidar_lin_covar,
          this->params.lidar_ang_covar, this->params.lidar_ang_covar,
          this->params.lidar_lin_covar, this->params.lidar_ang_covar,
          this->params.lidar_ang_covar;
        Eigen::MatrixXd cov_Z(Eigen::MatrixXd::Zero(6, 6));
        Eigen::MatrixXd j(Eigen::MatrixXd::Zero(6, 6));
        double az, br, rg;  // azimuth, bearing and range

        // The ordering for partials is x, y, z, rotx, roty, rotz
        // This is a symmetric matrix, so only need to fill out the upper
        // triangular portion
        Eigen::MatrixXd d2J_dX2(Eigen::MatrixXd::Zero(6, 6));

        // To hold running total of d2J_dZdX*cov(z)*d2J_dZdX'
        Eigen::MatrixXd middle(Eigen::MatrixXd::Zero(6, 6));

        // Gets overwritten each loop
        Eigen::MatrixXd d2J_dZdX(Eigen::MatrixXd::Zero(6, 6));
        d2J_dZdX(3, 0) = -2;
        d2J_dZdX(4, 1) = -2;
        d2J_dZdX(5, 2) = -2;

        auto list = this->icp.correspondences_.get();
        for (auto it = list->begin(); it != list->end(); ++it) {
            if (it->index_match > -1) {
                // it is -1 if there is no match in the target cloud
                // set up some aliases to make following lines more compact
                const float &Z1 = (target->points[it->index_match].x),
                            Z2 = (target->points[it->index_match].y),
                            Z3 = (target->points[it->index_match].z),
                            Z4 = (ref->points[it->index_query].x),
                            Z5 = (ref->points[it->index_query].y),
                            Z6 = (ref->points[it->index_query].z);

                rg = std::sqrt(Z1 * Z1 + Z2 * Z2 + Z3 * Z3);
                br = std::atan2(Z2, Z1);
                az = std::atan(Z3 / std::sqrt(Z1 * Z1 + Z2 * Z2));
                j(0, 0) = cos(br) * sin(az);
                j(1, 0) = sin(br) * sin(az);
                j(2, 0) = cos(az);
                j(0, 1) = -rg * sin(br) * sin(az);
                j(1, 1) = rg * cos(br) * sin(az);
                j(0, 2) = rg * cos(br) * cos(az);
                j(1, 2) = rg * cos(az) * sin(br);
                j(2, 2) = -rg * sin(az);
                rg = std::sqrt(Z4 * Z4 + Z5 * Z5 + Z6 * Z6);
                br = std::atan2(Z5, Z4);
                az = std::atan(Z6 / std::sqrt(Z4 * Z4 + Z5 * Z5));
                j(3, 3) = cos(br) * sin(az);
                j(4, 3) = sin(br) * sin(az);
                j(5, 3) = cos(az);
                j(3, 4) = -rg * sin(br) * sin(az);
                j(4, 4) = rg * cos(br) * sin(az);
                j(3, 5) = rg * cos(br) * cos(az);
                j(4, 5) = rg * cos(az) * sin(br);
                j(5, 5) = -rg * sin(az);
                cov_Z = j * sphere_cov.derived() * j.transpose();

                // clang-format off

                // coordinate transform jacobian. Order is range, bearing,
                // azimuth for first and 2nd point
                // [ cos(S2)*sin(S3), -S1*sin(S2)*sin(S3),   S1*cos(S2)*cos(S3),               0,                0,                  0]
                // [ sin(S2)*sin(S3),  S1*cos(S2)*sin(S3),   S1*cos(S3)*sin(S2),               0,                0,                  0]
                // [         cos(S3),                   0,          -S1*sin(S3),               0,                0,                  0]
                // [               0,                   0,                    0, cos(S5)*sin(S6), -S4*sin(S5)*sin(S6), S4*cos(S5)*cos(S6)]
                // [               0,                   0,                   0, sin(S5)*sin(S6),  S4*cos(S5)*sin(S6),  S4*cos(S6)*sin(S5)]
                // [               0,                   0,                  0,         cos(S6),                   0,       -S4*sin(S6)]

                // d2J_dx2

                d2J_dX2(0, 0) += 2;
                d2J_dX2(1, 1) += 2;
                d2J_dX2(2, 2) += 2;

                d2J_dX2(0, 3) += 2 * Z2 * (sr * sy + cr * cy * sp) + 2 * Z3 * (cr * sy - cy * sr * sp);
                d2J_dX2(1, 3) += -2 * Z2 * (cy * sr - cr * sp * sy) - 2 * Z3 * (cr * cy + sr * sp * sy);
                d2J_dX2(2, 3) += 2 * cp * (Z2 * cr - Z3 * sr);
                d2J_dX2(3, 3) += (2 * Z2 * (cr * sy - cy * sr * sp) - 2 * Z3 * (sr * sy + cr * cy * sp)) * (X1 - Z4 - Z2 * (cr * sy - cy * sr * sp) +
                     Z3 * (sr * sy + cr * cy * sp) + Z1 * cp * cy) - (2 * Z2 * (cr * cy + sr * sp * sy) - 2 * Z3 * (cy * sr - cr * sp * sy)) *
                    (X2 - Z5 + Z2 * (cr * cy + sr * sp * sy) - Z3 * (cy * sr - cr * sp * sy) + Z1 * cp * sy) - (2 * Z3 * cr * cp + 2 * Z2 * cp * sr) *
                    (X3 - Z6 - Z1 * sp + Z3 * cr * cp + Z2 * cp * sr) + (Z2 * (sr * sy + cr * cy * sp) + Z3 * (cr * sy - cy * sr * sp)) *
                    (2 * Z2 * (sr * sy + cr * cy * sp) + 2 * Z3 * (cr * sy - cy * sr * sp)) + (Z2 * (cy * sr - cr * sp * sy) +
                    Z3 * (cr * cy + sr * sp * sy)) * (2 * Z2 * (cy * sr - cr * sp * sy) + 2 * Z3 * (cr * cy + sr * sp * sy)) +
                    (Z2 * cr * cp - Z3 * cp * sr) * (2 * Z2 * cr * cp - 2 * Z3 * cp * sr);

                d2J_dX2(0, 4) += 2 * cy * (Z3 * cr * cp - Z1 * sp + Z2 * cp * sr);
                d2J_dX2(1, 4) += 2 * sy * (Z3 * cr * cp - Z1 * sp + Z2 * cp * sr);
                d2J_dX2(2, 4) += -2 * Z1 * cp - 2 * Z3 * cr * sp - 2 * Z2 * sr * sp;
                d2J_dX2(3, 4) += -2 * (Z2 * cr - Z3 * sr) * (X3 * sp - Z6 * sp - X1 * cp * cy + Z4 * cp * cy - X2 * cp * sy + Z5 * cp * sy);
                d2J_dX2(4, 4) += (Z1 * cp + Z3 * cr * sp + Z2 * sr * sp) * (2 * Z1 * cp + 2 * Z3 * cr * sp + 2 * Z2 * sr * sp) -
                  (2 * Z3 * cr * cp - 2 * Z1 * sp + 2 * Z2 * cp * sr) *(X3 - Z6 - Z1 * sp + Z3 * cr * cp + Z2 * cp * sr) +
                  2 * cy * cy * pow((Z3 * cr * cp - Z1 * sp + Z2 * cp * sr), 2) +2 * sy * sy *
                    pow((Z3 * cr * cp - Z1 * sp + Z2 * cp * sr), 2) - 2 * cy * (Z1 * cp + Z3 * cr * sp + Z2 * sr * sp) *
                    (X1 - Z4 + Z1 * cp * cy - Z2 * cr * sy + Z3 * sr * sy + Z2 * cy * sr * sp + Z3 * cr * cy * sp) -
                  2 * sy * (Z1 * cp + Z3 * cr * sp + Z2 * sr * sp) * (X2 - Z5 + Z2 * cr * cy + Z1 * cp * sy - Z3 * cy * sr +
                     Z3 * cr * sp * sy + Z2 * sr * sp * sy);

                d2J_dX2(0, 5) += 2 * Z3 * (cy * sr - cr * sp * sy) - 2 * Z2 * (cr * cy + sr * sp * sy) - 2 * Z1 * cp * sy;
                d2J_dX2(1, 5) += 2 * Z3 * (sr * sy + cr * cy * sp) - 2 * Z2 * (cr * sy - cy * sr * sp) + 2 * Z1 * cp * cy;
                // d2J_dX2(2,5) += 0;  This quantity is zero
                d2J_dX2(3, 5) +=
                  2 * X1 * Z3 * cr * cy - 2 * Z3 * Z4 * cr * cy +
                  2 * X1 * Z2 * cy * sr + 2 * X2 * Z3 * cr * sy -
                  2 * Z2 * Z4 * cy * sr - 2 * Z3 * Z5 * cr * sy +
                  2 * X2 * Z2 * sr * sy - 2 * Z2 * Z5 * sr * sy +
                  2 * X2 * Z2 * cr * cy * sp - 2 * Z2 * Z5 * cr * cy * sp -
                  2 * X1 * Z2 * cr * sp * sy - 2 * X2 * Z3 * cy * sr * sp +
                  2 * Z2 * Z4 * cr * sp * sy + 2 * Z3 * Z5 * cy * sr * sp +
                  2 * X1 * Z3 * sr * sp * sy - 2 * Z3 * Z4 * sr * sp * sy;
                d2J_dX2(4, 5) += 2 * (Z3 * cr * cp - Z1 * sp + Z2 * cp * sr) * (X2 * cy - Z5 * cy - X1 * sy + Z4 * sy);
                d2J_dX2(5, 5) +=
                  2 * Z1 * Z4 * cp * cy - 2 * X2 * Z2 * cr * cy -
                  2 * X1 * Z1 * cp * cy + 2 * Z2 * Z5 * cr * cy +
                  2 * X1 * Z2 * cr * sy - 2 * X2 * Z1 * cp * sy +
                  2 * X2 * Z3 * cy * sr - 2 * Z2 * Z4 * cr * sy +
                  2 * Z1 * Z5 * cp * sy - 2 * Z3 * Z5 * cy * sr -
                  2 * X1 * Z3 * sr * sy + 2 * Z3 * Z4 * sr * sy -
                  2 * X1 * Z3 * cr * cy * sp + 2 * Z3 * Z4 * cr * cy * sp -
                  2 * X1 * Z2 * cy * sr * sp - 2 * X2 * Z3 * cr * sp * sy +
                  2 * Z2 * Z4 * cy * sr * sp + 2 * Z3 * Z5 * cr * sp * sy -
                  2 * X2 * Z2 * sr * sp * sy + 2 * Z2 * Z5 * sr * sp * sy;

                // d2J_dZdX
                // Instead of Filling out this quantity directly,
                // d2J_dZdX*cov(z)*d2J_dZdX' will be calculated for the
                // current correspondence. This is then added elementwise to a
                // running total matrix. This is because
                // the number of columns d2J_dZdX grows linearly with the number
                // of correspondences. This approach can be
                // done because each measurement is assumed independent
                d2J_dZdX(0, 0) = 2 * cp * cy;
                d2J_dZdX(1, 0) = 2 * cy * sr * sp - 2 * cr * sy;
                d2J_dZdX(2, 0) = 2 * sr * sy + 2 * cr * cy * sp;
                // d2J_dZdX(3,0) = -2;
                // d2J_dZdX(4,0) = 0;
                // d2J_dZdX(5,0) = 0;

                d2J_dZdX(0, 1) = 2 * cp * sy;
                d2J_dZdX(1, 1) = 2 * cr * cy + 2 * sr * sp * sy;
                d2J_dZdX(2, 1) = 2 * cr * sp * sy - 2 * cy * sr;
                // d2J_dZdX(3,1) = 0;
                // d2J_dZdX(4,1) = -2;
                // d2J_dZdX(5,1) = 0;

                d2J_dZdX(0, 2) = -2 * sp;
                d2J_dZdX(1, 2) = 2 * cp * sr;
                d2J_dZdX(2, 2) = 2 * cr * cp;
                // d2J_dZdX(3,2) = 0;
                // d2J_dZdX(4,2) = 0;
                // d2J_dZdX(5,2) = -2;

                // d2J_dZdX(0,3) = 0;
                d2J_dZdX(1, 3) = 2 * X3 * cr * cp - 2 * Z6 * cr * cp -
                                 2 * X2 * cy * sr + 2 * Z5 * cy * sr +
                                 2 * X1 * sr * sy - 2 * Z4 * sr * sy +
                                 2 * X2 * cr * sp * sy - 2 * Z5 * cr * sp * sy +
                                 2 * X1 * cr * cy * sp - 2 * Z4 * cr * cy * sp;
                d2J_dZdX(2, 3) = 2 * Z5 * cr * cy - 2 * X2 * cr * cy +
                                 2 * X1 * cr * sy - 2 * X3 * cp * sr -
                                 2 * Z4 * cr * sy + 2 * Z6 * cp * sr -
                                 2 * X1 * cy * sr * sp + 2 * Z4 * cy * sr * sp -
                                 2 * X2 * sr * sp * sy + 2 * Z5 * sr * sp * sy;
                d2J_dZdX(3, 3) = -2 * Z2 * (sr * sy + cr * cy * sp) -
                                 2 * Z3 * (cr * sy - cy * sr * sp);
                d2J_dZdX(4, 3) = 2 * Z2 * (cy * sr - cr * sp * sy) +
                                 2 * Z3 * (cr * cy + sr * sp * sy);
                d2J_dZdX(5, 3) = -2 * cp * (Z2 * cr - Z3 * sr);

                d2J_dZdX(0, 4) = 2 * Z6 * cp - 2 * X3 * cp - 2 * X1 * cy * sp +
                                 2 * Z4 * cy * sp - 2 * X2 * sp * sy +
                                 2 * Z5 * sp * sy;
                d2J_dZdX(1, 4) = -2 * sr * (X3 * sp - Z6 * sp - X1 * cp * cy + Z4 * cp * cy -
                             X2 * cp * sy + Z5 * cp * sy);
                d2J_dZdX(2, 4) = -2 * cr * (X3 * sp - Z6 * sp - X1 * cp * cy + Z4 * cp * cy -
                             X2 * cp * sy + Z5 * cp * sy);
                d2J_dZdX(3, 4) = -2 * cy * (Z3 * cr * cp - Z1 * sp + Z2 * cp * sr);
                d2J_dZdX(4, 4) = -2 * sy * (Z3 * cr * cp - Z1 * sp + Z2 * cp * sr);
                d2J_dZdX(5, 4) = 2 * Z1 * cp + 2 * Z3 * cr * sp + 2 * Z2 * sr * sp;

                d2J_dZdX(0, 5) = 2 * cp * (X2 * cy - Z5 * cy - X1 * sy + Z4 * sy);
                d2J_dZdX(1, 5) = 2 * Z4 * cr * cy - 2 * X1 * cr * cy -
                                 2 * X2 * cr * sy + 2 * Z5 * cr * sy +
                                 2 * X2 * cy * sr * sp - 2 * Z5 * cy * sr * sp -
                                 2 * X1 * sr * sp * sy + 2 * Z4 * sr * sp * sy;
                d2J_dZdX(2, 5) = 2 * X1 * cy * sr - 2 * Z4 * cy * sr +
                                 2 * X2 * sr * sy - 2 * Z5 * sr * sy -
                                 2 * X1 * cr * sp * sy + 2 * Z4 * cr * sp * sy +
                                 2 * X2 * cr * cy * sp - 2 * Z5 * cr * cy * sp;
                d2J_dZdX(3, 5) = 2 * Z2 * (cr * cy + sr * sp * sy) -
                                 2 * Z3 * (cy * sr - cr * sp * sy) +
                                 2 * Z1 * cp * sy;
                d2J_dZdX(4, 5) = 2 * Z2 * (cr * sy - cy * sr * sp) -
                                 2 * Z3 * (sr * sy + cr * cy * sp) -
                                 2 * Z1 * cp * cy;
                // d2J_dZdX(5,5) = 0;

                middle.noalias() += d2J_dZdX * cov_Z * (d2J_dZdX.transpose());

                // clang-format on
            }
        }
        MatX inverse =
          (d2J_dX2.selfadjointView<Eigen::Upper>()).toDenseMatrix().inverse();
        this->information = (inverse * middle * inverse).inverse();
    }
}

}  // namespace wave
