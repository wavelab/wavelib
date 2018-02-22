#include <ceres/ceres.h>
#include <Eigen/Core>
#include <unsupported/Eigen/MatrixFunctions>

#include "wave/optimization/ceres/odom_gp/point_to_plane_gp.hpp"
#include "wave/optimization/ceres/odom_gp/point_to_line_gp.hpp"
#include "wave/optimization/ceres/odom_gp_coupled_states/point_to_line_gp.hpp"
#include "wave/optimization/ceres/odom_gp_coupled_states/point_to_plane_gp.hpp"
#include "wave/wave_test.hpp"
#include "wave/utils/math.hpp"
#include "wave/geometry/transformation.hpp"
#include "wave/kinematics/constant_velocity_gp_prior.hpp"

// This is a numerical check for some residuals where ceres gradient checker is not helpful

namespace wave {

TEST(point_to_line, jacobian) {

    double ptA[3] = {1, 1, 0};
    double ptB[3] = {1, 3, -4};
    double pt[3] = {1, 2, -4};

    const double delta_T = 0.5;
    const double **params;
    params = new const double *[4];

    Transformation<Eigen::Matrix<double, 3, 4>, false> T_k, T_kp1;
    Vec6 vel_k, vel_kp1;

    params[0] = T_k.getInternalMatrix().derived().data();
    params[1] = T_kp1.getInternalMatrix().derived().data();
    params[2] = vel_k.data();
    params[3] = vel_kp1.data();

    T_k.setIdentity();
    vel_k << 0.1, -0.1, 0.2, 5, 1, -1;
    vel_kp1 = vel_k;
    T_kp1.deepCopy(T_k);
    T_kp1.manifoldPlus(delta_T * vel_k);

    double zero = 0;
    double tau = 0.34;
    Mat6 Qc = Mat6::Identity();
    Mat6 inv_Qc = Qc.inverse();

    wave_kinematics::ConstantVelocityPrior motion_prior(zero, delta_T, &tau, Qc, inv_Qc);

    Eigen::Matrix<double, 12, 12> hat, candle;

    motion_prior.calculateStuff(hat, candle);

    ceres::CostFunction *cost_function = new SE3PointToLineGP(pt,
                                                              ptA,
                                                              ptB,
                                                              hat.block<6, 12>(0, 0),
                                                              candle.block<6, 12>(0, 0),
                                                              Mat3::Identity(),
                                                              true);
    double **jacobian;
    jacobian = new double *[4];
    jacobian[0] = new double[24];
    jacobian[1] = new double[24];
    jacobian[2] = new double[12];
    jacobian[3] = new double[12];

    Vec2 op_result;

    cost_function->Evaluate(params, op_result.data(), jacobian);

    double const step_size = 1e-9;
    Transformation<Eigen::Matrix<double, 3, 4>, false> Tk_perturbed, Tkp1_perturbed;
    Vec6 vel_k_perturbed, vel_kp1_perturbed;

    Tk_perturbed.deepCopy(T_k);
    Tkp1_perturbed.deepCopy(T_kp1);
    vel_k_perturbed = vel_k;
    vel_kp1_perturbed = vel_kp1;

    params[0] = Tk_perturbed.getInternalMatrix().derived().data();
    params[1] = Tkp1_perturbed.getInternalMatrix().derived().data();
    params[2] = vel_k_perturbed.data();
    params[3] = vel_kp1_perturbed.data();

    std::vector<Eigen::Matrix<double, 2, 6>> an_jacs, num_jacs;
    an_jacs.resize(4);
    num_jacs.resize(4);

    Vec6 delta;
    delta.setZero();

    Vec2 result;
    Vec2 diff;

    double inv_step = 1.0 / step_size;

    for (uint32_t i = 0; i < 6; i++) {
        delta(i) = step_size;
        // First parameter
        Tk_perturbed.manifoldPlus(delta);
        cost_function->Evaluate(params, result.data(), nullptr);
        diff = result - op_result;
        num_jacs.at(0).block<2,1>(0,i) = inv_step * diff;
        Tk_perturbed.deepCopy(T_k);
        // Second parameter
        Tkp1_perturbed.manifoldPlus(delta);
        cost_function->Evaluate(params, result.data(), nullptr);
        diff = result - op_result;
        num_jacs.at(1).block<2,1>(0,i) = inv_step * diff;
        Tkp1_perturbed.deepCopy(T_kp1);
        // Third parameter
        vel_k_perturbed = vel_k_perturbed + delta;
        cost_function->Evaluate(params, result.data(), nullptr);
        diff = result - op_result;
        num_jacs.at(2).block<2,1>(0,i) = inv_step * diff;
        vel_k_perturbed = vel_k;
        // Fourth parameter
        vel_kp1_perturbed = vel_kp1_perturbed + delta;
        cost_function->Evaluate(params, result.data(), nullptr);
        diff = result - op_result;
        num_jacs.at(3).block<2,1>(0,i) = inv_step * diff;
        vel_kp1_perturbed = vel_kp1;

        delta.setZero();
    }

    // now get the analytical jacobians
    Eigen::Map<Eigen::Matrix<double, 2, 12, Eigen::RowMajor>> an_jac_1(jacobian[0], 2, 12);
    an_jacs.at(0) = an_jac_1.block<2,6>(0,0);

    Eigen::Map<Eigen::Matrix<double, 2, 12, Eigen::RowMajor>> an_jac_2(jacobian[1], 2, 12);
    an_jacs.at(1) = an_jac_2.block<2,6>(0,0);

    Eigen::Map<Eigen::Matrix<double, 2, 6, Eigen::RowMajor>> an_jac_3(jacobian[2], 2, 6);
    an_jacs.at(2) = an_jac_3;

    Eigen::Map<Eigen::Matrix<double, 2, 6, Eigen::RowMajor>> an_jac_4(jacobian[3], 2, 6);
    an_jacs.at(3) = an_jac_4;

    for (uint32_t i = 0; i < 4; i++) {
        double err = (num_jacs.at(i) - an_jacs.at(i)).norm();
        if (err > 1e-10) {
            std::cout << "Index " << i << " with error = " << err << std::endl
                     << "Numerical: " << std::endl << num_jacs.at(i) << std::endl
                     << "Analytical:" << std::endl << an_jacs.at(i) << std::endl << std::endl;
        }
        EXPECT_NEAR(err, 0.0, 1e-6);
    }
}

TEST(point_to_line, trajectory_param) {
    double memblock[30];
    const double **params;
    params = new const double *;
    params[0] = memblock;

    Eigen::Map<Vec6> vel(memblock);
    auto tk_ptr = std::make_shared<Eigen::Map<Eigen::Matrix<double, 3, 4>>>(memblock + 6, 3, 4);
    auto tkp1_ptr = std::make_shared<Eigen::Map<Eigen::Matrix<double, 3, 4>>>(memblock + 18, 3, 4);
    Transformation<Eigen::Map<Eigen::Matrix<double, 3, 4>>> T_k(tk_ptr);
    Transformation<Eigen::Map<Eigen::Matrix<double, 3, 4>>> T_kp1(tkp1_ptr);

    T_k.setIdentity();
    vel << 0.1, -0.1, 0.2, 5, 1, -1;
    T_kp1.deepCopy(T_k);
    const double delta_T = 0.5;
    T_kp1.manifoldPlus(delta_T * vel);

    double zero = 0;
    double tau = 0.34;
    Mat6 Qc = Mat6::Identity();
    Mat6 inv_Qc = Qc.inverse();

    wave_kinematics::ConstantVelocityPrior motion_prior(zero, delta_T, &tau, Qc, inv_Qc);

    Eigen::Matrix<double, 12, 12> hat, candle;

    motion_prior.calculateStuff(hat, candle);

    double ptA[3] = {1, 1, 0};
    double ptB[3] = {1, 3, -4};
    double pt[3] = {1, 2, -4};

    ceres::CostFunction *cost_function = new SE3PointToLineGPCoupled<30>(pt,
                                                              ptA,
                                                              ptB,
                                                              hat.block<6, 12>(0, 0),
                                                              candle.block<6, 12>(0, 0),
                                                                     0,
                                                              Mat3::Identity(),
                                                              true);
    double **jacobian;
    jacobian = new double *[4];
    jacobian[0] = new double[24];
    jacobian[1] = new double[24];
    jacobian[2] = new double[12];
    jacobian[3] = new double[12];

    Vec2 op_result;

    cost_function->Evaluate(params, op_result.data(), jacobian);

    double const step_size = 1e-9;
    double memblock_perturb[30];

    Eigen::Map<Vec6> vel_perturbed(memblock_perturb);
    auto tk_ptr_perturb = std::make_shared<Eigen::Map<Eigen::Matrix<double, 3, 4>>>(memblock_perturb + 6, 3, 4);
    auto tkp1_ptr_perturb = std::make_shared<Eigen::Map<Eigen::Matrix<double, 3, 4>>>(memblock_perturb + 18, 3, 4);
    Transformation<Eigen::Map<Eigen::Matrix<double, 3, 4>>> Tk_perturbed(tk_ptr_perturb);
    Transformation<Eigen::Map<Eigen::Matrix<double, 3, 4>>> Tkp1_perturbed(tkp1_ptr_perturb);

    Tk_perturbed.deepCopy(T_k);
    Tkp1_perturbed.deepCopy(T_kp1);
    vel_perturbed = vel;

    std::vector<Eigen::Matrix<double, 2, 6>> an_jacs, num_jacs;
    an_jacs.resize(3);
    num_jacs.resize(3);

    Vec6 delta;
    delta.setZero();

    Vec2 result;
    Vec2 diff;

    double inv_step = 1.0 / step_size;

    params[0] = memblock_perturb;

    for (uint32_t i = 0; i < 6; i++) {
        delta(i) = step_size;
        // First parameter
        vel_perturbed = vel_perturbed + delta;
        cost_function->Evaluate(params, result.data(), nullptr);
        diff = result - op_result;
        num_jacs.at(0).block<2,1>(0,i) = inv_step * diff;
        vel_perturbed = vel;

        // Second parameter
        Tk_perturbed.manifoldPlus(delta);
        cost_function->Evaluate(params, result.data(), nullptr);
        diff = result - op_result;
        num_jacs.at(1).block<2,1>(0,i) = inv_step * diff;
        Tk_perturbed.deepCopy(T_k);

        // Third parameter
        Tkp1_perturbed.manifoldPlus(delta);
        cost_function->Evaluate(params, result.data(), nullptr);
        diff = result - op_result;
        num_jacs.at(2).block<2,1>(0,i) = inv_step * diff;
        Tkp1_perturbed.deepCopy(T_kp1);

        delta.setZero();
    }

    // now get the analytical jacobians
    Eigen::Map<Eigen::Matrix<double, 2, 30, Eigen::RowMajor>> jac_map(jacobian[0], 2, 30);
    an_jacs.at(0) = jac_map.block<2,6>(0,0);
    an_jacs.at(1) = jac_map.block<2,6>(0,6);
    an_jacs.at(2) = jac_map.block<2,6>(0,18);

    for (uint32_t i = 0; i < 3; i++) {
        double err = (num_jacs.at(i) - an_jacs.at(i)).norm();
        if (err > 1e-10) {
            std::cout << "Index " << i << " with error = " << err << std::endl
                      << "Numerical: " << std::endl << num_jacs.at(i) << std::endl
                      << "Analytical:" << std::endl << an_jacs.at(i) << std::endl << std::endl;
        }
        EXPECT_NEAR(err, 0.0, 1e-6);
    }
}

TEST(point_to_plane, jacobian) {

    double ptA[3] = {1, 1, 0};
    double ptB[3] = {1, 3, -4};
    double ptC[3] = {4, -1, 0};
    double pt[3] = {1, 2, -4};

    const double delta_T = 0.5;
    const double **params;
    params = new const double *[4];

    Transformation<Eigen::Matrix<double, 3, 4>, false> T_k, T_kp1;
    Vec6 vel_k, vel_kp1;

    params[0] = T_k.getInternalMatrix().derived().data();
    params[1] = T_kp1.getInternalMatrix().derived().data();
    params[2] = vel_k.data();
    params[3] = vel_kp1.data();

    T_k.setIdentity();
    vel_k << 0.1, -0.1, 0.2, 5, 1, -1;
    vel_kp1 = vel_k;
    T_kp1.deepCopy(T_k);
    T_kp1.manifoldPlus(delta_T * vel_k);

    double zero = 0;
    double tau = 0.34;
    Mat6 Qc = Mat6::Identity();
    Mat6 inv_Qc = Qc.inverse();

    wave_kinematics::ConstantVelocityPrior motion_prior(zero, delta_T, &tau, Qc, inv_Qc);

    Eigen::Matrix<double, 12, 12> hat, candle;

    motion_prior.calculateStuff(hat, candle);

    ceres::CostFunction *cost_function = new SE3PointToPlaneGP(pt,
                                                              ptA,
                                                              ptB,
                                                              ptC,
                                                              hat.block<6, 12>(0, 0),
                                                              candle.block<6, 12>(0, 0),
                                                              Mat3::Identity(),
                                                              false);
    double **jacobian;
    jacobian = new double *[4];
    jacobian[0] = new double[12];
    jacobian[1] = new double[12];
    jacobian[2] = new double[6];
    jacobian[3] = new double[6];

    Eigen::Matrix<double, 1, 1> op_result;

    cost_function->Evaluate(params, op_result.data(), jacobian);

    double const step_size = 1e-9;
    Transformation<Eigen::Matrix<double, 3, 4>> Tk_perturbed, Tkp1_perturbed;
    Vec6 vel_k_perturbed, vel_kp1_perturbed;

    Tk_perturbed.deepCopy(T_k);
    Tkp1_perturbed.deepCopy(T_kp1);
    vel_k_perturbed = vel_k;
    vel_kp1_perturbed = vel_kp1;

    params[0] = Tk_perturbed.getInternalMatrix().derived().data();
    params[1] = Tkp1_perturbed.getInternalMatrix().derived().data();
    params[2] = vel_k_perturbed.data();
    params[3] = vel_kp1_perturbed.data();

    std::vector<Eigen::Matrix<double, 1, 6>> an_jacs, num_jacs;
    an_jacs.resize(4);
    num_jacs.resize(4);

    Vec6 delta;
    delta.setZero();

    Eigen::Matrix<double, 1, 1> diff, result;

    double inv_step = 1.0 / step_size;

    for (uint32_t i = 0; i < 6; i++) {
        delta(i) = step_size;
        // First parameter
        Tk_perturbed.manifoldPlus(delta);
        cost_function->Evaluate(params, result.data(), nullptr);
        diff = result - op_result;
        num_jacs.at(0).block<1,1>(0,i) = inv_step * diff;
        Tk_perturbed.deepCopy(T_k);
        // Second parameter
        Tkp1_perturbed.manifoldPlus(delta);
        cost_function->Evaluate(params, result.data(), nullptr);
        diff = result - op_result;
        num_jacs.at(1).block<1,1>(0,i) = inv_step * diff;
        Tkp1_perturbed.deepCopy(T_kp1);
        // Third parameter
        vel_k_perturbed = vel_k_perturbed + delta;
        cost_function->Evaluate(params, result.data(), nullptr);
        diff = result - op_result;
        num_jacs.at(2).block<1,1>(0,i) = inv_step * diff;
        vel_k_perturbed = vel_k;
        // Fourth parameter
        vel_kp1_perturbed = vel_kp1_perturbed + delta;
        cost_function->Evaluate(params, result.data(), nullptr);
        diff = result - op_result;
        num_jacs.at(3).block<1,1>(0,i) = inv_step * diff;
        vel_kp1_perturbed = vel_kp1;

        delta.setZero();
    }

    // now get the analytical jacobians
    Eigen::Map<Eigen::Matrix<double, 1, 12, Eigen::RowMajor>> an_jac_1(jacobian[0], 1, 12);
    an_jacs.at(0) = an_jac_1.block<1,6>(0,0);

    Eigen::Map<Eigen::Matrix<double, 1, 12, Eigen::RowMajor>> an_jac_2(jacobian[1], 1, 12);
    an_jacs.at(1) = an_jac_2.block<1,6>(0,0);

    Eigen::Map<Eigen::Matrix<double, 1, 6, Eigen::RowMajor>> an_jac_3(jacobian[2], 1, 6);
    an_jacs.at(2) = an_jac_3;

    Eigen::Map<Eigen::Matrix<double, 1, 6, Eigen::RowMajor>> an_jac_4(jacobian[3], 1, 6);
    an_jacs.at(3) = an_jac_4;

    for (uint32_t i = 0; i < 4; i++) {
        double err = (num_jacs.at(i) - an_jacs.at(i)).norm();
        std::cout << "Index " << i << " has error: " << err << std::endl
                  << "Numerical: " << std::endl << num_jacs.at(i) << std::endl
                  << "Analytical:" << std::endl << an_jacs.at(i) << std::endl << std::endl;
        EXPECT_NEAR(err, 0.0, 1e-6);
    }
}

TEST(point_to_plane, trajectory_param) {
    double memblock[30];
    const double **params;
    params = new const double *;
    params[0] = memblock;

    Eigen::Map<Vec6> vel(memblock);
    auto tk_ptr = std::make_shared<Eigen::Map<Eigen::Matrix<double, 3, 4>>>(memblock + 6, 3, 4);
    auto tkp1_ptr = std::make_shared<Eigen::Map<Eigen::Matrix<double, 3, 4>>>(memblock + 18, 3, 4);
    Transformation<Eigen::Map<Eigen::Matrix<double, 3, 4>>> T_k(tk_ptr);
    Transformation<Eigen::Map<Eigen::Matrix<double, 3, 4>>> T_kp1(tkp1_ptr);

    T_k.setIdentity();
    vel << 0.1, -0.1, 0.2, 5, 1, -1;
    T_kp1.deepCopy(T_k);
    const double delta_T = 0.5;
    T_kp1.manifoldPlus(delta_T * vel);

    double zero = 0;
    double tau = 0.34;
    Mat6 Qc = Mat6::Identity();
    Mat6 inv_Qc = Qc.inverse();

    wave_kinematics::ConstantVelocityPrior motion_prior(zero, delta_T, &tau, Qc, inv_Qc);

    Eigen::Matrix<double, 12, 12> hat, candle;

    motion_prior.calculateStuff(hat, candle);

    double ptA[3] = {1, 1, 0};
    double ptB[3] = {1, 3, -4};
    double ptC[3] = {4, -1, 0};
    double pt[3] = {1, 2, -4};

    ceres::CostFunction *cost_function = new SE3PointToPlaneGPCoupled<30>(pt,
                                                                         ptA,
                                                                         ptB,
                                                                          ptC,
                                                                         hat.block<6, 12>(0, 0),
                                                                         candle.block<6, 12>(0, 0),
                                                                         0,
                                                                         Mat3::Identity(),
                                                                         true);
    double **jacobian;
    jacobian = new double *[1];
    jacobian[0] = new double[30];

    double op_result;

    cost_function->Evaluate(params, &op_result, jacobian);

    double const step_size = 1e-9;
    double memblock_perturb[30];

    Eigen::Map<Vec6> vel_perturbed(memblock_perturb);
    auto tk_ptr_perturb = std::make_shared<Eigen::Map<Eigen::Matrix<double, 3, 4>>>(memblock_perturb + 6, 3, 4);
    auto tkp1_ptr_perturb = std::make_shared<Eigen::Map<Eigen::Matrix<double, 3, 4>>>(memblock_perturb + 18, 3, 4);
    Transformation<Eigen::Map<Eigen::Matrix<double, 3, 4>>> Tk_perturbed(tk_ptr_perturb);
    Transformation<Eigen::Map<Eigen::Matrix<double, 3, 4>>> Tkp1_perturbed(tkp1_ptr_perturb);

    Tk_perturbed.deepCopy(T_k);
    Tkp1_perturbed.deepCopy(T_kp1);
    vel_perturbed = vel;

    std::vector<Eigen::Matrix<double, 1, 6>> an_jacs, num_jacs;
    an_jacs.resize(3);
    num_jacs.resize(3);

    Vec6 delta;
    delta.setZero();

    double result;
    double diff;

    double inv_step = 1.0 / step_size;

    params[0] = memblock_perturb;

    for (uint32_t i = 0; i < 6; i++) {
        delta(i) = step_size;
        // First parameter
        vel_perturbed = vel_perturbed + delta;
        cost_function->Evaluate(params, &result, nullptr);
        diff = result - op_result;
        num_jacs.at(0).coeffRef(0,i) = inv_step * diff;
        vel_perturbed = vel;

        // Second parameter
        Tk_perturbed.manifoldPlus(delta);
        cost_function->Evaluate(params, &result, nullptr);
        diff = result - op_result;
        num_jacs.at(1).coeffRef(0,i) = inv_step * diff;
        Tk_perturbed.deepCopy(T_k);

        // Third parameter
        Tkp1_perturbed.manifoldPlus(delta);
        cost_function->Evaluate(params, &result, nullptr);
        diff = result - op_result;
        num_jacs.at(2).coeffRef(0,i) = inv_step * diff;
        Tkp1_perturbed.deepCopy(T_kp1);

        delta.setZero();
    }

    // now get the analytical jacobians
    Eigen::Map<Eigen::Matrix<double, 1, 30, Eigen::RowMajor>> jac_map(jacobian[0], 1, 30);
    an_jacs.at(0) = jac_map.block<1,6>(0,0);
    an_jacs.at(1) = jac_map.block<1,6>(0,6);
    an_jacs.at(2) = jac_map.block<1,6>(0,18);

    for (uint32_t i = 0; i < 3; i++) {
        double err = (num_jacs.at(i) - an_jacs.at(i)).norm();
        if (err > 1e-10) {
            std::cout << "Index " << i << " with error = " << err << std::endl
                      << "Numerical: " << std::endl << num_jacs.at(i) << std::endl
                      << "Analytical:" << std::endl << an_jacs.at(i) << std::endl << std::endl;
        }
        EXPECT_NEAR(err, 0.0, 1e-6);
    }
}

}