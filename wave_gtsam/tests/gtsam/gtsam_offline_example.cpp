#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/ProjectionFactor.h>
#include <gtsam/slam/BetweenFactor.h>

#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/Sampler.h>
#include <gtsam/geometry/SimpleCamera.h>

#include "wave/wave_test.hpp"
#include "wave/vision/dataset/VoDataset.hpp"
#include "gtsam_helpers.hpp"

namespace wave {

// Set true to add odometry factors between consecutive poses, false for pure VO
const auto use_odometry_factors = false;

class GtsamExample : public ::testing::Test {
 protected:
    VoDatasetGenerator generator;
    VoDataset dataset;
    gtsam::Cal3_S2::shared_ptr kParams =
      boost::make_shared<gtsam::Cal3_S2>(200, 200, 0, 320, 240);

    GtsamExample() {
        generator.camera.image_width = 640;
        generator.camera.image_height = 480;
        generator.camera.K = kParams->matrix();
        generator.camera.hz = 10.0;
        generator.nb_landmarks = 100;
        generator.landmark_x_bounds << -10, 10;
        generator.landmark_y_bounds << -10, 10;
        generator.landmark_z_bounds << -1, 1;
        dataset = generator.generate();
    }
};

TEST_F(GtsamExample, run) {
    gtsam::NonlinearFactorGraph graph;
    gtsam::Values initial_estimate;

    std::vector<gtsam::Pose3> true_poses;

    // Since the synthetic dataset has zero noise, we will generate some
    // Set up a gaussian noise source of 1.1 pixel in u and v
    auto measurement_noise_model = gtsam::noiseModel::Isotropic::Sigma(2, 1.1);
    auto measurement_noise_sampler = gtsam::Sampler{measurement_noise_model};

    // Loop over the different poses, adding the observations to iSAM
    // incrementally
    for (auto i = 0u; i < this->dataset.states.size(); ++i) {
        const auto &state = this->dataset.states[i];
        const auto &observations = state.features_observed;

        // We have to convert things to gtsam types
        // @todo better integration

        const auto pose = gtsamPoseFromState(state);
        true_poses.push_back(pose);

        if (use_odometry_factors && i > 0) {
            // Add odometry factor
            auto between = poseBetweenStates(dataset.states[i - 1], state);

            Vec6 noise_vec;
            noise_vec << Vec3::Constant(0.001), Vec3::Constant(0.0001);

            auto noise = gtsam::noiseModel::Diagonal::Sigmas(noise_vec);
            auto odometry_factor = gtsam::BetweenFactor<gtsam::Pose3>{
              gtsam::Symbol{'x', i - 1}, gtsam::Symbol{'x', i}, between, noise};
            graph.push_back(odometry_factor);
        }

        // Add a purposely offset initial estimate
        const auto offset =
          gtsam::Pose3{gtsam::Rot3::Rodrigues(-0.02, 0.02, 0.22),
                       gtsam::Point3{0.05, -0.10, 0.20}};

        auto pose_estimate = pose;
        pose_estimate = pose_estimate.compose(offset);
        initial_estimate.insert(gtsam::Symbol{'x', i}, pose_estimate);


        // Add factors for each landmark observation
        for (size_t j = 0; j < observations.size(); ++j) {
            // Convert observation
            const auto measurement = gtsam::Point2{observations[j].second};
            const auto landmark_id = observations[j].first;

            // Double check that our measurement matches gtsam's projection fn
            const auto &G_p_GF = this->dataset.landmarks[landmark_id];
            gtsam::SimpleCamera camera(pose, *this->kParams);
            const gtsam::Point2 gt_measurement =
              camera.project(gtsam::Point3{G_p_GF});
            ASSERT_PRED2(VectorsNear, gt_measurement, measurement);

            // Add some artificial noise to the synthetic measurement
            auto noisy_measurement =
              measurement + measurement_noise_sampler.sample();

            auto projection_factor =
              gtsam::GenericProjectionFactor<gtsam::Pose3,
                                             gtsam::Point3,
                                             gtsam::Cal3_S2>{
                noisy_measurement,
                measurement_noise_model,
                gtsam::Symbol{'x', i},
                gtsam::Symbol{'l', landmark_id},
                this->kParams};

            graph.push_back(projection_factor);

            // Add a purposely offset initial estimate the first time we see it
            const auto key = gtsam::Symbol{'l', landmark_id};
            if (!initial_estimate.exists(key)) {
                auto offset = gtsam::Point3{-0.25, 0.20, 0.15};
                initial_estimate.insert<gtsam::Point3>(key, G_p_GF + offset);
            }
        }

        // Add priors on the first two poses to fix the origin and scale
        if (i < 2) {
            VecX noise_vec{6};
            noise_vec << Vec3::Constant(1e-5), Vec3::Constant(1e-6);
            auto poseNoise = gtsam::noiseModel::Diagonal::Sigmas(noise_vec);
            graph.push_back(gtsam::PriorFactor<gtsam::Pose3>{
              gtsam::Symbol{'x', i}, pose, poseNoise});
        }
    }

    gtsam::LevenbergMarquardtOptimizer optimizer{graph, initial_estimate};
    auto result = optimizer.optimize();


    // Uncomment to print estimated values
    // result.print("Final result:\n");

    auto avg_rot_error = 0.0, avg_pos_error = 0.0, avg_landmark_error = 0.0;

    // Check camera poses
    for (auto i = 0u; i < true_poses.size(); ++i) {
        const auto key = gtsam::Symbol{'x', i};
        // We have to cast the generic Value back to the known type
        const auto estimated_pose = result.at(key).cast<gtsam::Pose3>();
        const auto &true_pose = true_poses[i];

        double pos_error =
          (true_pose.translation() - estimated_pose.translation()).norm();
        EXPECT_LT(pos_error, 0.1) << "x" << i;

        const auto true_q = true_poses[i].rotation().toQuaternion();
        const auto estimated_q = estimated_pose.rotation().toQuaternion();
        const auto rotation_error = true_q.angularDistance(estimated_q);
        EXPECT_LT(rotation_error, 0.05) << "x" << i;

        avg_rot_error += rotation_error / true_poses.size();
        avg_pos_error += pos_error / true_poses.size();
    }

    // Check landmarks
    for (const auto &l : this->dataset.landmarks) {
        const auto &landmark_id = l.first;
        const auto &true_pos = l.second;

        const auto key = gtsam::Symbol{'l', landmark_id};
        const auto estimated_pos = result.at(key).cast<gtsam::Point3>();
        const auto landmark_error = (true_pos - estimated_pos).norm();

        EXPECT_LT(landmark_error, 2.0);

        avg_landmark_error += landmark_error / this->dataset.landmarks.size();
    }

    std::cout << "gtsam_offline_example results:" << std::endl;
    std::cout << "Initial cost = " << graph.error(initial_estimate)
              << std::endl;
    std::cout << "Final cost = " << graph.error(result) << std::endl;
    std::cout << "Mean position error = " << avg_pos_error << std::endl;
    std::cout << "Mean rotation error = " << avg_rot_error << std::endl;
    std::cout << "Mean landmark error = " << avg_landmark_error << std::endl;
    std::cout << "with synthetic measurement noise of "
              << measurement_noise_model->sigma() << " px" << std::endl;
}

}  // namespace wave
