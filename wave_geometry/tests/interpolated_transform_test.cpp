#include <iostream>
#include <random>

#include <gtest/gtest.h>

#include "wave/kinematics/constant_velocity_gp_prior.hpp"
#include "wave/geometry/transformation.hpp"
#include "wave/geometry/numerical_test_functors.hpp"
#include "wave/utils/math.hpp"
#include "wave/utils/utils.hpp"

namespace wave {

namespace {
using t_Type = Transformation<Eigen::Matrix<double, 3, 4>, true>;
}

class TransformationTestFixture : public ::testing::Test {
 public:
    t_Type transform_expected;
    Vec6 transformation_twist_parameters;
    double comparison_threshold = 1e-5;

    void SetUp() {
        Mat4 transform_matrix;
        transform_matrix << 0.936293363584199, -0.275095847318244, 0.218350663146334, 1, 0.289629477625516,
          0.956425085849232, -0.036957013524625, 2, -0.198669330795061, 0.097843395007256, 0.975170327201816, 3, 0, 0,
          0, 1;
        transformation_twist_parameters << 0.068924613882066, 0.213225926957886, 0.288748939228676, 0.965590777183138,
          1.960945901104432, 3.037052911306709;
        this->transform_expected.setFromMatrix(transform_matrix);
    }
};


TEST_F(TransformationTestFixture, testInterpolation) {
    t_Type T_k, T_kp1;

    Vec6 vel_k, vel_kp1;
    // Jacobian approximations rely on small rotation
    vel_k << 0.0, -0.0, 0.3, 5, 1, -1;
    vel_kp1 = vel_k;

    double tk = 0;
    double tkp1 = 0.1;
    double tau = 0.034;

    T_k.setIdentity();
    T_kp1.setIdentity();

    T_kp1.manifoldPlus(tkp1 * vel_k);

    Mat6 Qc = Mat6::Identity();
    Mat6 inv_Qc = Qc.inverse();

    Eigen::Matrix<double, 12, 12> hat, candle;

    wave_kinematics::ConstantVelocityPrior prior(tk, tkp1, &tau, Qc, inv_Qc);

    prior.calculateStuff(hat, candle);

    Mat6 J_T_k_num, J_T_kp1_num, J_vel_k_num, J_vel_kp1_num;
    Mat6 J_T_k_an, J_T_kp1_an, J_vel_k_an, J_vel_kp1_an;
    Vec6 perturbation_vec = Vec6::Zero();

    Vec6 increment;

    auto Tint = t_Type::interpolateAndJacobians(
      T_k, T_kp1, vel_k, vel_kp1, hat.block<6,12>(0,0), candle.block<6,12>(0,0), J_T_k_an, J_T_kp1_an, J_vel_k_an, J_vel_kp1_an);

    auto Tint_nojac = t_Type::interpolate(T_k, T_kp1, vel_k, vel_kp1, hat.block<6,12>(0,0), candle.block<6,12>(0,0));

    auto diff1 = Tint.manifoldMinus(Tint_nojac);

    Transformation<Eigen::Matrix<double, 3, 4>, false> T_k_full, T_kp1_full;
    T_k_full.deepCopy(T_k);
    T_kp1_full.deepCopy(T_kp1);

    auto Tint_full = Transformation<Eigen::Matrix<double, 3, 4>, false>::interpolateAndJacobians(
            T_k_full, T_kp1_full, vel_k, vel_kp1, hat.block<6,12>(0,0), candle.block<6,12>(0,0), J_T_k_an, J_T_kp1_an, J_vel_k_an, J_vel_kp1_an);

    auto Tint_full_nojac = Transformation<Eigen::Matrix<double, 3, 4>, false>::interpolate(T_k_full, T_kp1_full, vel_k, vel_kp1, hat.block<6,12>(0,0), candle.block<6,12>(0,0));

    Vec6 diff2 = Tint_full.manifoldMinus(Tint_full_nojac);

    Vec6 diff = Tint_full.manifoldMinus(Tint);

    TInterpolatedJTLeftFunctor<t_Type> J_Tleft_functor(T_k, T_kp1, vel_k, vel_kp1, hat.block<6,12>(0,0), candle.block<6,12>(0,0));
    numerical_jacobian(J_Tleft_functor, perturbation_vec, J_T_k_num);
    perturbation_vec = Vec6::Zero();

    TInterpolatedJTRightFunctor<t_Type> J_Tright_functor(T_k, T_kp1, vel_k, vel_kp1, hat.block<6,12>(0,0), candle.block<6,12>(0,0));
    numerical_jacobian(J_Tright_functor, perturbation_vec, J_T_kp1_num);
    perturbation_vec = Vec6::Zero();

    TInterpolatedJVLeftFunctor<t_Type> J_Vleft_functor(T_k, T_kp1, vel_k, vel_kp1, hat.block<6,12>(0,0), candle.block<6,12>(0,0));
    numerical_jacobian(J_Vleft_functor, perturbation_vec, J_vel_k_num);
    perturbation_vec = Vec6::Zero();

    TInterpolatedJVRightFunctor<t_Type> J_Vright_functor(T_k, T_kp1, vel_k, vel_kp1, hat.block<6,12>(0,0), candle.block<6,12>(0,0));
    numerical_jacobian(J_Vright_functor, perturbation_vec, J_vel_kp1_num);

    wave::MatX errmat = J_T_k_an - J_T_k_num;
    double err = errmat.norm();
    std::cout << "For Tk, error is " << err << "\n";
    std::cout << "\n" << J_T_k_an << std::endl << std::endl;
    std::cout << J_T_k_num << std::endl;
    if(err > this->comparison_threshold) {
        EXPECT_TRUE(false);
    }

    errmat = J_T_kp1_an - J_T_kp1_num;
    err = errmat.norm();
    std::cout << "For Tkp1, error is " << err << "\n";
    std::cout << J_T_kp1_an << std::endl << "\n";
    std::cout << J_T_kp1_num << std::endl;
    if(err > this->comparison_threshold) {
        EXPECT_TRUE(false);
    }

    errmat = J_vel_k_an - J_vel_k_num;
    err = errmat.norm();
    std::cout << "For velk, error is " << err << "\n";
    if(err > this->comparison_threshold) {
        std::cout << J_vel_k_an << std::endl;
        std::cout << J_vel_k_num << std::endl;
        EXPECT_TRUE(false);
    }

    errmat = J_vel_kp1_an - J_vel_kp1_num;
    err = errmat.norm();
    std::cout << "For velkp1, error is " << err << "\n";
    if(err > this->comparison_threshold) {
        std::cout << J_vel_kp1_an << std::endl;
        std::cout << J_vel_kp1_num << std::endl;
        EXPECT_TRUE(false);
    }
}
}