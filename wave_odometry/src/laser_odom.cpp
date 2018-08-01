#include "wave/odometry/laser_odom.hpp"

namespace wave {

LaserOdom::LaserOdom(const LaserOdomParams params,
                     const FeatureExtractorParams feat_params,
                     const TransformerParams transformer_params)
    : param(params), feature_extractor(feat_params, params.n_ring), transformer(transformer_params) {
    this->continue_output = true;

    this->cv_model =
      std::make_shared<wave_kinematics::ConstantVelocityPrior>(0, 0, nullptr, this->param.Qc, this->param.inv_Qc);

    auto n_ring = static_cast<size_t>(param.n_ring);
    this->feature_extractor.setParams(feat_params, n_ring);
    this->counters.resize(n_ring);
    std::fill(this->counters.begin(), this->counters.end(), 0);

    // todo don't do this
    this->cur_feature_candidates.resize(this->N_FEATURES);
    this->cur_feature_candidatesT.resize(this->N_FEATURES);
    this->prev_feature_candidates.resize(this->N_FEATURES);
    this->prev_feature_candidatesT.resize(this->N_FEATURES);
    this->cur_feat_map.resize(this->N_FEATURES);
    this->prev_feat_map.resize(this->N_FEATURES);
    this->ave_pts.resize(this->N_FEATURES);
    this->feature_tracks.resize(this->N_FEATURES);
    this->cur_feat_idx.resize(this->N_FEATURES);
    this->prev_feat_idx.resize(this->N_FEATURES);
    this->ptT_jacobians.resize(this->N_FEATURES);
    this->geometry_landmarks.resize(this->N_FEATURES);
    this->undis_features.resize(this->N_FEATURES);
    this->undis_candidates_cur.resize(this->N_FEATURES);
    this->undis_candidates_prev.resize(this->N_FEATURES);
    this->volatile_feature_tracks.resize(this->N_FEATURES);
    this->cm1_feat_pts_size.resize(this->N_FEATURES);

    this->cur_scan.resize(n_ring);
    this->signals.resize(n_ring);

    for (uint32_t i = 0; i < n_ring; i++) {
        this->cur_scan.at(i) = Eigen::Tensor<float, 2>(4, this->MAX_POINTS);
        this->signals.at(i) = Eigen::Tensor<float, 2>(this->N_SIGNALS, this->MAX_POINTS);
    }

    this->indices.resize(this->N_FEATURES);
    for (uint32_t i = 0; i < this->N_FEATURES; i++) {
        this->indices.at(i).resize(this->param.n_ring);
    }

    this->range_sensor = std::make_shared<RangeSensor>(param.sensor_params);

    this->undis_features.resize(this->N_FEATURES);

    if (this->param.num_trajectory_states < 2) {
        throw std::out_of_range("Number of parameter states must be at least 2");
    }
    if (this->param.n_window < 2) {
        throw std::out_of_range("Window size must be at least 2");
    }

}

void LaserOdom::updateParams(const LaserOdomParams new_params) {
    this->param = new_params;
}

LaserOdomParams LaserOdom::getParams() {
    return this->param;
}

LaserOdom::~LaserOdom() {
    if (this->output_thread) {
        this->continue_output = false;
        this->output_condition.notify_one();
        this->output_thread->join();
    }
}

void LaserOdom::registerOutputFunction(std::function<void()> output_function) {
    this->f_output = std::move(output_function);
    this->output_thread = std::make_unique<std::thread>(&LaserOdom::spinOutput, this);
}

void LaserOdom::spinOutput() {
    std::unique_lock<std::mutex> lk(this->output_mutex);
    while (this->continue_output) {
        while (!this->fresh_output) {  // wait can have spurious wakeups
            this->output_condition.wait(lk);
            if (!this->continue_output) {
                break;
            }
        }
        this->f_output();
        this->fresh_output = false;
    }
}

/**
 * This corrects for motion distortion of the current scan. Transforming it to the end.
 */
void LaserOdom::undistort() {
    Eigen::Tensor<float, 2> output;
    this->transformer.update(this->cur_trajectory, this->trajectory_stamps);
    this->undistorted_cld.clear();
    for (uint32_t ring_id = 0; ring_id < this->counters.size(); ++ring_id) {
        Eigen::Tensor<float, 2> real_points = this->cur_scan.at(ring_id).slice(ar2{0,0}, ar2{4, this->counters.at(ring_id)});
        this->transformer.transformToStart(real_points, output, this->scan_stampsf.size() - 1);
        for (int i = 0; i < this->counters.at(ring_id); ++i) {
            pcl::PointXYZI pt;
            pt.x = output(0,i);
            pt.y = output(1,i);
            pt.z = output(2,i);
            pt.intensity = 2;
            this->undistorted_cld.push_back(pt);
        }
    }
    for (uint32_t feat_id = 0; feat_id < this->N_FEATURES; ++feat_id) {
        this->geometry_landmarks.at(feat_id).clear();
        for (const auto &track : this->feature_tracks.at(feat_id)) {
            Eigen::Matrix<float, 7, 1> geometry;
            geometry.block<6, 1>(0,0) = track.geometry.cast<float>();
            geometry(6) = track.mapping.size();
            this->geometry_landmarks.at(feat_id).emplace_back(geometry);
        }
        for (const auto &track : this->volatile_feature_tracks.at(feat_id)) {
            Eigen::Matrix<float, 7, 1> geometry;
            geometry.block<6, 1>(0,0) = track.geometry.cast<float>();
            geometry(6) = track.mapping.size();
            this->geometry_landmarks.at(feat_id).emplace_back(geometry);
        }
        this->undis_features.at(feat_id).clear();
        auto n_scans = this->feat_pts_T.size();
        for (uint32_t i = 0; i < n_scans; ++i) {
            for (uint32_t j = 0; j < this->feat_pts_T.at(i).at(feat_id).dimension(1); ++j) {
                pcl::PointXYZ pt;
                pt.x = this->feat_pts_T.at(i).at(feat_id)(0,j);
                pt.y = this->feat_pts_T.at(i).at(feat_id)(1,j);
                pt.z = this->feat_pts_T.at(i).at(feat_id)(2,j);
                this->undis_features.at(feat_id).push_back(pt);
            }
        }
        this->undis_candidates_prev.at(feat_id).clear();
        this->undis_candidates_cur.at(feat_id).clear();
        for (uint32_t i = 0; i < this->cur_feature_candidatesT.at(feat_id).dimension(1); ++i){
            pcl::PointXYZ pt;
            pt.x = this->cur_feature_candidatesT.at(feat_id)(0,i);
            pt.y = this->cur_feature_candidatesT.at(feat_id)(1,i);
            pt.z = this->cur_feature_candidatesT.at(feat_id)(2,i);
            this->undis_candidates_cur.at(feat_id).push_back(pt);
        }
        for (uint32_t i = 0; i < this->prev_feature_candidatesT.at(feat_id).dimension(1); ++i) {
            pcl::PointXYZ pt;
            pt.x = this->prev_feature_candidatesT.at(feat_id)(0,i);
            pt.y = this->prev_feature_candidatesT.at(feat_id)(1,i);
            pt.z = this->prev_feature_candidatesT.at(feat_id)(2,i);
            this->undis_candidates_prev.at(feat_id).push_back(pt);
        }
    }
}

template <class S_TYPE, class D_TYPE>
void LaserOdom::copyTrajectory(const VecE<S_TYPE> &src, VecE<D_TYPE> &dst) {
    dst.resize(src.size());
    for (uint32_t i = 0; i < src.size(); i++) {
        dst.at(i).pose = src.at(i).pose;
        dst.at(i).vel = src.at(i).vel;
    }
}

void LaserOdom::updateFeatureCandidates() {
    for (uint32_t i = 0; i < this->N_FEATURES; i++) {
        std::swap(this->cur_feature_candidates.at(i), this->prev_feature_candidates.at(i));
        std::swap(this->cur_feat_idx.at(i), this->prev_feat_idx.at(i));
        auto &feat = this->cur_feature_candidates.at(i);
        long featcnt = 0;
        for (const auto &elem : this->indices.at(i)) {
            featcnt += elem.dimension(0);
        }
        feat.resize(4, featcnt);
        this->cur_feat_idx.at(i).resize(featcnt);
        std::fill(this->cur_feat_idx.at(i).begin(), this->cur_feat_idx.at(i).end(), -1);
        long offset = 0;
        for (uint32_t j = 0; j < this->indices.at(i).size(); ++j) {
            // can't use omp parallel for because of offset. Need to manually split up task to parallelize.
            //#pragma omp parallel for
            for (uint32_t k = 0; k < this->indices.at(i).at(j).dimension(0); ++k) {
                const int &idx = this->indices.at(i).at(j)(k);
                feat.slice(ar2({0, offset}), ar2({4, 1})) = this->cur_scan.at(j).slice(ar2({0, idx}), ar2({4, 1}));
                ++offset;
            }
        }
    }
}

void LaserOdom::addPoints(const std::vector<PointXYZIR> &pts, const int tick, TimeType stamp) {
    if (tick - this->prv_tick < -200) {  // current scan has ended
        this->feature_extractor.getFeatures(this->cur_scan, this->signals, this->counters, this->indices);
        this->updateFeatureCandidates();
        if (this->initialized) {
            this->match(stamp);

            if (this->output_thread) {
                {
                    std::unique_lock<std::mutex> lk(this->output_mutex);
                    if (this->fresh_output) {
                        // data from last time hasn't been consumed yet
                        LOG_ERROR("Overwriting previous output");
                    }
                    this->copyTrajectory(this->cur_trajectory, this->undistort_trajectory);
                    for (uint32_t i = 0; i < this->undistort_trajectory.size(); ++i) {
                        std::chrono::duration<float> fsec(this->trajectory_stamps.at(i));
                        this->undistort_trajectory.at(i).stamp =
                          this->scan_stamps_chrono.front() + std::chrono::duration_cast<std::chrono::microseconds>(fsec);
                    }

                    this->undistort();
                    this->fresh_output = true;
                }
                this->output_condition.notify_one();
            }
        }
        this->rollover(stamp);
        std::fill(this->counters.begin(), this->counters.end(), 0);
    }

    for (PointXYZIR pt : pts) {
        if (counters.at(pt.ring) >= static_cast<int>(this->MAX_POINTS)) {
            throw std::out_of_range("Rebuild with higher max points");
        }
        this->cur_scan.at(pt.ring)(0, counters.at(pt.ring)) = pt.x;
        this->cur_scan.at(pt.ring)(1, counters.at(pt.ring)) = pt.y;
        this->cur_scan.at(pt.ring)(2, counters.at(pt.ring)) = pt.z;

        auto diff = stamp - this->scan_stamps_chrono.back();
        float secon = std::chrono::duration<float, std::ratio<1>>(diff).count();

        if (secon < 0.0f) {
            secon = 0;
        }

        this->cur_scan.at(pt.ring)(3, counters.at(pt.ring)) = secon;

        this->signals.at(pt.ring)(0, counters.at(pt.ring)) = sqrtf(pt.x * pt.x + pt.y * pt.y + pt.z * pt.z);
        this->signals.at(pt.ring)(1, counters.at(pt.ring)) = pt.intensity;

        this->counters.at(pt.ring)++;
    }

    this->prv_tick = tick;
}

void LaserOdom::updateTracks() {
    // transform all landmark states to start of current scan and decrement the scan id of each associated feature point
    // if the new scan_id < 0, remove those feature points and freeze the landmark state
    const auto &transform = this->cur_trajectory.at(1).pose;
    for (auto &tracks : this->feature_tracks) {
        VecE<FeatureTrack> updated_tracks;
        for (auto &track : tracks) {
            bool reduce_length = false;
            Vec<FeatureTrack::Mapping> updated_mapping;
            for (auto &map : track.mapping) {
                if (map.scan_idx != 0) {
                    --(map.scan_idx);
                    updated_mapping.emplace_back(map);
                } else {
                    reduce_length = true;
                }
            }
            std::swap(track.mapping, updated_mapping);
            if(reduce_length && track.length == 1) {
                continue;
            }
            if (reduce_length) {
                --(track.length);
                track.optimize = false;
            }
            track.geometry.block<3,1>(0,0) = transform.storage.block<3,3>(0,0).transpose() * track.geometry.block<3,1>(0,0);
            auto ref = track.geometry.block<3,1>(3,0);
            transform.inverseTransform(ref, ref);
            updated_tracks.emplace_back(std::move(track));
        }
        std::swap(tracks, updated_tracks);
    }
}

void LaserOdom::rollover(TimeType stamp) {
    // If there are n_scans worth of data, rotate
    if (this->scan_stamps_chrono.size() == this->param.n_window) {
        // Perform a left rotation
        this->updateTracks();
        std::rotate(this->feat_pts.begin(), this->feat_pts.begin() + 1, this->feat_pts.end());
        std::rotate(this->feat_pts_T.begin(), this->feat_pts_T.begin() + 1, this->feat_pts_T.end());
        std::rotate(
          this->scan_stamps_chrono.begin(), this->scan_stamps_chrono.begin() + 1, this->scan_stamps_chrono.end());

        std::rotate(this->cur_trajectory.begin(),
                    this->cur_trajectory.begin() + this->param.num_trajectory_states - 1,
                    this->cur_trajectory.end());

        // adjust timestamps and trajectory states to start of new window
        for (uint32_t i = 0; i < this->scan_stampsf.size() - 1; ++i) {
            for (uint32_t j = 0; j < this->param.num_trajectory_states - 1; ++j) {
                this->cur_trajectory.at(i * this->param.num_trajectory_states + j).pose =
                  this->cur_trajectory.front().pose.transformInverse() *
                  this->cur_trajectory.at(i * this->param.num_trajectory_states + j).pose;
            }
        }
        this->scan_stamps_chrono.back() = stamp;
        auto index = (this->param.num_trajectory_states - 1) * (this->scan_stampsf.size() - 1);
        this->cur_trajectory.at(index).pose =
          this->cur_trajectory.front().pose.transformInverse() * this->cur_trajectory.at(index).pose;
    } else {
        // grow storage
        this->scan_stamps_chrono.emplace_back(stamp);
        this->scan_stampsf.resize(this->scan_stamps_chrono.size());
        VecE<Eigen::Tensor<float, 2>> vec(this->N_FEATURES);
        VecE<Eigen::Tensor<float, 2>> vec2(this->N_FEATURES);

        for (auto &elem : vec) {
            elem.resize(4, 0);
        }

        this->feat_pts.emplace_back(std::move(vec));
        this->feat_pts_T.emplace_back(std::move(vec2));

        this->feat_T_map.resize(this->feat_T_map.size() + 1);
        this->feat_T_map.back().resize(this->N_FEATURES);

        for (auto &elem : this->ptT_jacobians) {
            elem.emplace_back(VecE<Eigen::Tensor<double, 3>>(4));
        }

        if (this->cur_trajectory.empty()) {
            this->cur_trajectory.resize(this->param.num_trajectory_states);
            this->trajectory_stamps.resize(this->param.num_trajectory_states);
        } else {
            this->cur_trajectory.resize(this->cur_trajectory.size() + this->param.num_trajectory_states - 1);
            this->trajectory_stamps.resize(this->trajectory_stamps.size() + this->param.num_trajectory_states - 1);
        }
        this->prev_trajectory.resize(this->cur_trajectory.size());
    }

    uint32_t mult = this->param.num_trajectory_states - 1;
    for (uint32_t i = 0; i < this->scan_stamps_chrono.size(); i++) {
        auto diff = this->scan_stamps_chrono.at(i) - this->scan_stamps_chrono.front();
        this->scan_stampsf.at(i) = std::chrono::duration<float, std::ratio<1>>(diff).count();
        this->trajectory_stamps.at(i * mult) = this->scan_stampsf.at(i);
        if (i != 0) {
            float step = (this->scan_stampsf.at(i) - this->scan_stampsf.at(i - 1)) / mult;
            for (uint32_t j = 1; j < mult; ++j) {
                this->trajectory_stamps.at((i - 1) * mult + j) = this->trajectory_stamps.at((i - 1) * mult) + j * step;
            }
        }
    }

    std::fill(this->counters.begin(), this->counters.end(), 0);

    if (!this->initialized) {
        size_t feature_count = 0;
        for (const auto &cand : this->prev_feature_candidates) {
            feature_count += cand.dimension(1);
        }

        if (feature_count >= (size_t)(this->param.min_features)) {
            this->initialized = true;
        }
    }
}

bool LaserOdom::runOptimization(ceres::Problem &problem) {
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
    options.max_num_iterations = this->param.max_inner_iters;
    options.max_num_consecutive_invalid_steps = 30;
//    options.function_tolerance = 1e-8;
//    options.parameter_tolerance = 1e-7;
    options.logging_type = ceres::LoggingType::SILENT;
    options.use_nonmonotonic_steps = true;

    ceres::EvaluationCallback *callback = new OdometryCallback(&(this->feat_pts),
                                         &(this->feat_pts_T),
                                         &(this->cur_trajectory),
                                         &(this->ptT_jacobians),
                                         &(this->trajectory_stamps),
                                         &(this->scan_stampsf),
                                         &(this->transformer));
    options.evaluation_callback = callback;

    ceres::Covariance::Options covar_options;
    covar_options.sparse_linear_algebra_library_type = ceres::SparseLinearAlgebraLibraryType::SUITE_SPARSE;
    covar_options.algorithm_type = ceres::CovarianceAlgorithmType::SPARSE_QR;

    if (this->param.solver_threads < 1) {
        options.num_threads = std::thread::hardware_concurrency();
        options.num_linear_solver_threads = std::thread::hardware_concurrency();
        covar_options.num_threads = std::thread::hardware_concurrency();
    } else {
        options.num_threads = this->param.solver_threads;
        options.num_linear_solver_threads = this->param.solver_threads;
        covar_options.num_threads = this->param.solver_threads;
    }

    if (problem.NumResidualBlocks() < this->param.min_residuals) {
        LOG_ERROR("Less than expected residuals, resetting");
        LOG_ERROR("%d residuals, threshold is %d", problem.NumResidualBlocks(), this->param.min_residuals);
        this->resetTrajectory();
        this->initialized = false;
        return false;
    } else if (!this->param.only_extract_features) {
        ceres::Solver::Summary summary;
        ceres::Solve(options, &problem, &summary);
        if (this->param.print_opt_sum) {
            LOG_INFO("%s", summary.BriefReport().c_str());
        }
        //                ceres::Covariance covariance(covar_options);
        //                if (!covariance.Compute(this->param_blocks, &problem)) {
        //                    LOG_ERROR("covariance did not compute");
        //                }
        //                covariance.GetCovarianceMatrixInTangentSpace(this->param_blocks, this->covar.data());
    }
    return true;
}

void LaserOdom::extendFeatureTracks(const Eigen::MatrixXi &idx, const Eigen::MatrixXf &dist, uint32_t feat_id) {
    uint32_t knn = 2;
    auto residualType = PointToLine;
    // todo don't do this
    if (feat_id == 2) {
        knn = 3;
        residualType = PointToPlane;
    }
    /// Each column represents a query (feature track)
    Eigen::Tensor<float, 2> new_feat_points(4, 2000);
    auto offset = this->feat_pts.back().at(feat_id).dimension(1);
    long new_feat_cnt = 0;
    for (uint32_t j = 0; j < idx.cols(); ++j) {
        std::vector<uint32_t> matches;
        double min_elev = 0.0;
        double max_elev = 0.0;
        double new_min = 0.0, new_max = 0.0;
        bool wide_spread = false;
        bool new_spread = false;
        /// Each row is a nearest neighbour
        for (uint32_t i = 0; i < idx.rows(); ++i) {
            if (std::isinf(dist(i, j))) {
                continue;
            }
            Vec3f pt = this->cur_feat_map.at(feat_id)->block<3, 1>(0, idx(i, j));
            if (matches.empty()) {
                matches.emplace_back(idx(i, j));
                min_elev = std::atan2(pt(2), std::sqrt(pt(0) * pt(0) + pt(1) * pt(1)));
                max_elev = min_elev;
                continue;
            }
            if (wide_spread) {
                matches.emplace_back(idx(i, j));
            } else {
                double new_elev = std::atan2(pt(2), std::sqrt(pt(0) * pt(0) + pt(1) * pt(1)));
                if (new_elev > max_elev) {
                    new_max = new_elev;
                    new_min = min_elev;
                } else if (new_elev < min_elev) {
                    new_min = new_elev;
                    new_max = max_elev;
                }
                if (new_max - new_min > this->param.azimuth_tol)
                    new_spread = true;
                if(new_spread) {
                    wide_spread = true;
                    min_elev = new_min;
                    max_elev = new_max;
                }
                if (wide_spread || matches.size() + 1 < knn) {
                    matches.emplace_back(idx(i, j));
                }
            }
            if (matches.size() == knn) {
                break;
            }
        }
        /// add to track if error against current landmark is low
        if (matches.size() == knn) {
            for (const auto &elem : matches) {
                if (this->cur_feat_idx.at(feat_id).at(elem) != -1) {
                    continue;
                }
                const auto &geometry = this->feature_tracks.at(feat_id).at(j).geometry;
                const auto &pt = this->cur_feat_map.at(feat_id)->block<3, 1>(0, elem);
                Vec3 diff = (pt.cast<double>() - geometry.block<3, 1>(3, 0));
                Vec1 error;
                if (residualType == PointToLine) {
                    Vec3 err = (diff - geometry.block<3, 1>(0, 0) * (diff.transpose() * geometry.block<3, 1>(0, 0)));
                    error = err.transpose() * err;
                } else {
                    error = (diff.transpose() * geometry.block<3, 1>(0, 0));
                    error = error.transpose() * error;
                }
                // copy candidate point into the feature point set (both original and transformed) if it is not already
                // used in another residual
                if (error(0) < sqrt(this->param.max_residual_val)) {
                    this->cur_feat_idx.at(feat_id).at(elem) = j;

                    Eigen::array<int, 2> offsets_new = {0, static_cast<int>(new_feat_cnt)};
                    Eigen::array<int, 2> offsets_candidate = {0, static_cast<int>(elem)};
                    Eigen::array<int, 2> extents = {4, 1};

                    new_feat_points.slice(offsets_new, extents) =
                      this->cur_feature_candidates.at(feat_id).slice(offsets_candidate, extents);

                    auto new_scan_idx = this->scan_stampsf.size() - 1;
                    if (this->feature_tracks.at(feat_id).at(j).mapping.back().scan_idx != new_scan_idx) {
                        this->feature_tracks.at(feat_id).at(j).length += 1;
                    }
                    this->feature_tracks.at(feat_id).at(j).mapping.emplace_back(offset + new_feat_cnt,
                                                                                new_scan_idx);
                    auto bnd = std::upper_bound(
                      this->trajectory_stamps.begin(), this->trajectory_stamps.end(), new_feat_points(3, new_feat_cnt) + this->scan_stampsf.back());
                    auto traj_idx = static_cast<uint32_t>(bnd - this->trajectory_stamps.begin() - 1);

                    this->feature_tracks.at(feat_id).at(j).mapping.back().state_id = traj_idx;

                    ++new_feat_cnt;
                }
            }
        }
    }

    auto &cur_feat_points = this->feat_pts.back().at(feat_id);
    Eigen::TensorMap<Eigen::Tensor<float, 2>> reduced_new_feat_points(new_feat_points.data(), 4, new_feat_cnt);
    if (cur_feat_points.dimension(1) == 0) {
        cur_feat_points = reduced_new_feat_points;
    } else if (reduced_new_feat_points.dimension(1) != 0) {
        Eigen::Tensor<float, 2> stupid_eigen = cur_feat_points.concatenate(reduced_new_feat_points, 1);
        cur_feat_points = stupid_eigen;
    }
}

// queries are newest scan, searching next newer scan
// todo reduce duplicated code
void LaserOdom::createNewFeatureTracks(const Eigen::MatrixXi &idx, const Eigen::MatrixXf &dist, uint32_t feat_id) {
    uint32_t knn = 2;
    auto residualType = PointToLine;
    // todo don't do this
    if (feat_id == 2) {
        knn = 3;
        residualType = PointToPlane;
    }
    /// Each column represents a query (feature track)
    Eigen::Tensor<float, 2> new_feat_points(4, 2000), prev_new_feat_points(4, 2000);
    long new_feat_cnt = 0, prev_new_feat_cnt = 0;
    auto &prev_feat_points = this->feat_pts.at(this->feat_pts.size() - 2).at(feat_id);
    auto &cur_feat_points = this->feat_pts.at(this->feat_pts.size() - 1).at(feat_id);
    auto prev_offset = prev_feat_points.dimension(1);
    auto cur_offset = cur_feat_points.dimension(1);
    for (uint32_t j = 0; j < idx.cols(); ++j) {
        if (this->cur_feat_idx.at(feat_id).at(j) != -1) {
            continue;
        }

        std::vector<uint32_t> matches;
        double min_elev = 0.0;
        double max_elev = 0.0;
        double new_min = 0.0, new_max = 0.0;
        bool wide_spread = false;
        bool new_spread = false;
        /// Each row is a nearest neighbour
        for (uint32_t i = 0; i < idx.rows(); ++i) {
            if (this->prev_feat_idx.at(feat_id).at(idx(i, j)) != -1) {
                continue;
            }
            if (std::isinf(dist(i, j))) {
                continue;
            }
            Vec3f pt = this->prev_feat_map.at(feat_id)->block<3, 1>(0, idx(i, j));
            if (matches.empty()) {
                matches.emplace_back(idx(i, j));
                min_elev = std::atan2(pt(2), std::sqrt(pt(0) * pt(0) + pt(1) * pt(1)));
                max_elev = min_elev;
                continue;
            }
            if (wide_spread) {
                matches.emplace_back(idx(i, j));
            } else {
                double new_elev = std::atan2(pt(2), std::sqrt(pt(0) * pt(0) + pt(1) * pt(1)));
                if (new_elev > max_elev) {
                    new_max = new_elev;
                    new_min = min_elev;
                } else if (new_elev < min_elev) {
                    new_min = new_elev;
                    new_max = max_elev;
                }
                if (new_max - new_min > this->param.azimuth_tol)
                    new_spread = true;
                if(new_spread) {
                    wide_spread = true;
                    min_elev = new_min;
                    max_elev = new_max;
                }
                if (wide_spread || matches.size() + 1 < knn) {
                    matches.emplace_back(idx(i, j));
                }
            }
            if (matches.size() == knn) {
                break;
            }
        }
        /// create feature track if error is low, and points are not part of another track
        if (matches.size() == knn) {
            // Depending on the type of geometry, need to initialize feature track with reasonable estimate
            Vec6 geometry = Vec6::Zero();

            if (residualType == PointToLine) {
                geometry.block<3, 1>(3, 0) =
                  (this->prev_feat_map.at(feat_id)->block<3, 1>(0, matches.at(0)).cast<double>() +
                   this->prev_feat_map.at(feat_id)->block<3, 1>(0, matches.at(1)).cast<double>()) /
                  2.0;

                geometry.block<3, 1>(0, 0) =
                  this->prev_feat_map.at(feat_id)->block<3, 1>(0, matches.at(0)).cast<double>() -
                  this->prev_feat_map.at(feat_id)->block<3, 1>(0, matches.at(1)).cast<double>();
            } else {
                geometry.block<3, 1>(3, 0) =
                  (this->prev_feat_map.at(feat_id)->block<3, 1>(0, matches.at(0)).cast<double>() +
                   this->prev_feat_map.at(feat_id)->block<3, 1>(0, matches.at(1)).cast<double>() +
                   this->prev_feat_map.at(feat_id)->block<3, 1>(0, matches.at(2)).cast<double>()) /
                  3.0;

                Vec3f a = this->prev_feat_map.at(feat_id)->block<3, 1>(0, matches.at(0)) -
                          this->prev_feat_map.at(feat_id)->block<3, 1>(0, matches.at(1));
                Vec3f b = this->prev_feat_map.at(feat_id)->block<3, 1>(0, matches.at(0)) -
                          this->prev_feat_map.at(feat_id)->block<3, 1>(0, matches.at(2));
                geometry.block<3, 1>(0, 0) = a.cross(b).cast<double>();
            }
            geometry.block<3, 1>(0, 0).normalize();
            if (geometry(2) < 0) {
                geometry.block<3, 1>(0, 0) = -geometry.block<3, 1>(0, 0);
            }

            Vec3 diff = (this->cur_feat_map.at(feat_id)->block<3, 1>(0, j).cast<double>() - geometry.block<3, 1>(3, 0));
            Vec1 error;
            if (residualType == PointToLine) {
                Vec3 err = (diff - geometry.block<3, 1>(0, 0) * (diff.transpose() * geometry.block<3, 1>(0, 0)));
                error = err.transpose() * err;
            } else {
                error = (diff.transpose() * geometry.block<3, 1>(0, 0));
                error = error.transpose() * error;
            }

            // create new residual, may need to increase tolerance a bit to find new features when initializing
            if (error(0) < this->param.max_residual_val * this->param.max_residual_val) {
                // create feature track and add queried point
                this->volatile_feature_tracks.at(feat_id).emplace_back(FeatureTrack());
                auto &track = this->volatile_feature_tracks.at(feat_id).back();
                track.geometry = geometry;
                track.length = 1;
                track.jacs = &(this->ptT_jacobians.at(feat_id));
                track.mapping.emplace_back(cur_offset + new_feat_cnt, this->scan_stampsf.size() - 1);

                float pt_time = this->cur_feature_candidates.at(feat_id)(3, j) + this->scan_stampsf.back();
                auto bnd = std::upper_bound(this->trajectory_stamps.begin(), this->trajectory_stamps.end(), pt_time);
                auto traj_idx = static_cast<uint32_t>(bnd - this->trajectory_stamps.begin() - 1);

                track.mapping.back().state_id = traj_idx;

                Eigen::array<int, 2> offsets_new = {0, static_cast<int>(new_feat_cnt)};
                Eigen::array<int, 2> offsets_candidate = {0, static_cast<int>(j)};
                Eigen::array<int, 2> extents = {4, 1};

                new_feat_points.slice(offsets_new, extents) =
                  this->cur_feature_candidates.at(feat_id).slice(offsets_candidate, extents);
                this->cur_feat_idx.at(feat_id).at(j) = static_cast<int>(this->volatile_feature_tracks.at(feat_id).size() - 1);

                ++new_feat_cnt;

                // now add points from previous scan
                for (auto elem : matches) {
                    track.mapping.emplace_back(prev_offset + prev_new_feat_cnt, this->scan_stampsf.size() - 2);

                    pt_time = this->prev_feature_candidates.at(feat_id)(3, elem) +
                              this->scan_stampsf.at(this->scan_stampsf.size() - 2);
                    bnd = std::upper_bound(this->trajectory_stamps.begin(), this->trajectory_stamps.end(), pt_time);
                    traj_idx = static_cast<uint32_t>(bnd - this->trajectory_stamps.begin() - 1);

                    track.mapping.back().state_id = traj_idx;

                    offsets_new.at(1) = static_cast<int>(prev_new_feat_cnt);
                    offsets_candidate.at(1) = static_cast<int>(elem);

                    if (prev_new_feat_cnt >= prev_new_feat_points.dimension(1)) {
                        LOG_ERROR("Inefficient resizing taking place");
                        Eigen::Tensor<float, 2> copy(4, prev_new_feat_points.dimension(1) * 2);
                        copy.slice(ar2{0,0}, ar2{4, prev_new_feat_points.dimension(1)}) = prev_new_feat_points;
                        std::swap(prev_new_feat_points, copy);
                    }

                    prev_new_feat_points.slice(offsets_new, extents) =
                      this->prev_feature_candidates.at(feat_id).slice(offsets_candidate, extents);
                    this->prev_feat_idx.at(feat_id).at(elem) = static_cast<int>(this->volatile_feature_tracks.at(feat_id).size() - 1);

                    ++prev_new_feat_cnt;
                }
            }
        }
    }

    Eigen::TensorMap<Eigen::Tensor<float, 2>> reduced_new_feat_points(new_feat_points.data(), 4, new_feat_cnt);
    if (cur_feat_points.dimension(1) == 0) {
        cur_feat_points = reduced_new_feat_points;
    } else if (reduced_new_feat_points.dimension(1) != 0) {
        Eigen::Tensor<float, 2> stupid_eigen = cur_feat_points.concatenate(reduced_new_feat_points, 1);
        cur_feat_points = stupid_eigen;
    }

    Eigen::TensorMap<Eigen::Tensor<float, 2>> reduced_prev_new_feat_points(
      prev_new_feat_points.data(), 4, prev_new_feat_cnt);
    if (prev_feat_points.dimension(1) == 0) {
        prev_feat_points = reduced_prev_new_feat_points;
    } else if (reduced_prev_new_feat_points.dimension(1) != 0) {
        Eigen::Tensor<float, 2> stupid_eigen = prev_feat_points.concatenate(reduced_prev_new_feat_points, 1);
        prev_feat_points = stupid_eigen;
    }
}

void LaserOdom::clearVolatileTracks() {
    for(auto &tracks : this->volatile_feature_tracks) {
        tracks.clear();
    }
    auto last_scan = this->feat_pts.size() - 1;
    for(uint32_t feat_id = 0; feat_id < this->feature_tracks.size(); ++feat_id) {
        for (uint32_t i = 0; i < this->feature_tracks.at(feat_id).size(); ++i) {
            auto &track = this->feature_tracks.at(feat_id).at(i);
            auto iter = track.mapping.rbegin();
            if (iter->scan_idx < last_scan) {
                continue;
            }
            while (iter->scan_idx == last_scan) {
                ++iter;
            }
            long dist = iter - track.mapping.rbegin();
            unsigned long new_size = track.mapping.size() - dist;
            track.mapping.resize(new_size);
            track.length -= 1;
        }
    }
    for (uint32_t feat_id = 0; feat_id < this->feat_pts.back().size(); ++feat_id) {
        auto &pts_cur = this->feat_pts.back().at(feat_id);
        pts_cur.resize(pts_cur.dimension(0), 0);
        auto &pts_prev = (this->feat_pts.rbegin() + 1)->at(feat_id);
        Eigen::Tensor<float, 2> temp = pts_prev.slice(ar2{0,0}, ar2{4, this->cm1_feat_pts_size.at(feat_id)});
        std::swap(temp, pts_prev);
    }
}

void LaserOdom::mergeFeatureTracks(uint32_t feat_id) {
    this->mergeFeatureTracksInternal(feat_id, this->feature_tracks.at(feat_id));
    this->mergeFeatureTracksInternal(feat_id, this->volatile_feature_tracks.at(feat_id));
}

//todo This is a greedy approach. Could try using reciprocal correspondences
void LaserOdom::mergeFeatureTracksInternal(uint32_t feat_id, VecE<FeatureTrack> &track_list) {
    auto &tracks = track_list;
    if (tracks.size() <= 1) {
        return;
    }
    auto &trk_pts = this->ave_pts.at(feat_id);
    trk_pts.resize(3, tracks.size());

    const int knn = 10 < (tracks.size() - 1) ? 10 : (tracks.size() - 1);

    Vec<std::list<uint32_t>> merge_history(tracks.size());
    Vec<uint32_t> addr(tracks.size());
    std::iota(addr.begin(), addr.end(), 0);

#pragma omp parallel for
    for (uint32_t t_idx = 0; t_idx < tracks.size(); t_idx++) {
        merge_history.at(t_idx).emplace_back(t_idx);
        const auto &track = tracks.at(t_idx);
        trk_pts.block<3, 1>(0, t_idx) = track.geometry.block<3, 1>(3, 0).cast<float>();
    }

    Nabo::NNSearchF* kd_idx = Nabo::NNSearchF::createKDTreeLinearHeap(trk_pts);

    MatXf dists(knn, trk_pts.cols());
    Eigen::MatrixXi nn_idx(knn, trk_pts.cols());

    kd_idx->knn(trk_pts, nn_idx, dists, knn, 0.1, 0, this->param.max_correspondence_dist);

    std::vector<float> score(knn);
    for (uint32_t c_idx = 0; c_idx < nn_idx.cols(); ++c_idx) {
        uint32_t query_idx = addr.at(c_idx);
        auto &query_track = tracks.at(query_idx);
        std::fill(score.begin(), score.end(), std::numeric_limits<float>::infinity());
        for (uint32_t r_idx = 0; r_idx < nn_idx.rows(); ++r_idx) {
            //calculate similarity score to each correspondence

            if (std::isinf(dists(r_idx, c_idx))) {
                continue;
            }
            float dist_cost, dir_cost;

            uint32_t knn_index = addr.at(nn_idx(r_idx, c_idx));

            auto &knn_track = tracks.at(knn_index);
            Vec3f diff = query_track.geometry.block<3,1>(3,0).cast<float>() - knn_track.geometry.block<3,1>(3,0).cast<float>();
            dir_cost = 1.0f - std::abs((query_track.geometry.block<3,1>(0,0).transpose() * knn_track.geometry.block<3,1>(0,0))(0));
            // use planar similarity score
            if (feat_id == 2) {
                dist_cost = std::abs((diff.transpose() * query_track.geometry.block<3,1>(0,0).cast<float>())) +
                        std::abs((diff.transpose() * knn_track.geometry.block<3,1>(0,0).cast<float>()));
                if (dist_cost > this->param.max_planar_dist_threshold || dir_cost > this->param.max_planar_ang_threshold) {
                    continue;
                }
            } else {
                dist_cost = (diff.transpose() * knn_track.geometry.block<3,1>(0,0).cross(query_track.geometry.block<3,1>(0,0)).cast<float>()).norm() /
                        knn_track.geometry.block<3,1>(0,0).cross(query_track.geometry.block<3,1>(0,0)).norm();
                if (dist_cost > this->param.max_linear_dist_threshold || dir_cost > this->param.max_linear_ang_threshold) {
                    continue;
                }
            }
            score.at(r_idx) = dist_cost + this->param.ang_scaling_param * dir_cost;
        }
        auto merge = std::min_element(score.begin(), score.end()) - score.begin();
        if (merge == knn || std::isinf(score.at(merge))) {
            continue;
        } else {
            //merge the two feature tracks
            auto first_index = addr.at(c_idx);
            auto second_index = addr.at(nn_idx(merge, c_idx));

            if (first_index == second_index) {
                continue;
            }

            if (first_index > second_index) {
                std::swap(first_index, second_index);
            }

            auto &merged_track = tracks.at(second_index);
            auto total = query_track.mapping.size() + merged_track.mapping.size();
            float w1 = (float)(query_track.mapping.size()) / (float) (total);
            float w2 = (float)(merged_track.mapping.size()) / (float) (total);

            if ((query_track.geometry.block<3,1>(0,0).transpose() * merged_track.geometry.block<3, 1>(0,0))(0) < 0) {
                query_track.geometry.block<3, 1>(3,0) = w1 * query_track.geometry.block<3,1>(3,0) + w2 * merged_track.geometry.block<3,1>(3,0);
                query_track.geometry.block<3, 1>(0,0) = w1 * query_track.geometry.block<3,1>(0,0) - w2 * merged_track.geometry.block<3,1>(0,0);
            } else {
                query_track.geometry = w1 * query_track.geometry + w2 * merged_track.geometry;
            }

            query_track.geometry = w1 * query_track.geometry + w2 * merged_track.geometry;
            query_track.geometry.block<3,1>(0,0).normalize();
            for (const auto &map : merged_track.mapping) {
                query_track.mapping.emplace_back(map);
            }
            std::sort(query_track.mapping.begin(), query_track.mapping.end(), [](const FeatureTrack::Mapping& lhs, const FeatureTrack::Mapping& rhs) {
                return lhs.scan_idx < rhs.scan_idx;
            });
            query_track.length = query_track.mapping.back().scan_idx - query_track.mapping.front().scan_idx;

            //update address map
            merge_history.at(first_index).splice(merge_history.at(first_index).end(), merge_history.at(second_index));

            for (const auto &elem : merge_history.at(first_index)) {
                addr.at(elem) = first_index;
            }

            if (second_index != merge_history.size() - 1) {
                std::swap(merge_history.at(second_index), merge_history.back());
                std::swap(tracks.at(second_index), tracks.back());
                for (const auto &elem : merge_history.at(second_index)) {
                    addr.at(elem) = second_index;
                }
            }

            merge_history.resize(tracks.size() - 1);
            tracks.resize(tracks.size() - 1);
        }
    }
    delete kd_idx;
}

void LaserOdom::prepTrajectory(const TimeType &stamp) {
    // scan stamps f holds the timestamps at the start of each scan
    // need to update the latter half of trajectory stamps based on the difference between
    // stamp and the last timestamp in stamps chrono.
    auto diff = stamp - this->scan_stamps_chrono.back();
    float scan_duration = std::chrono::duration<float, std::ratio<1>>(diff).count();

    // set up timestamps and trajectory for current scan
    float step = scan_duration / (this->param.num_trajectory_states - 1);

    unlong idx = this->trajectory_stamps.size() - this->param.num_trajectory_states + 1;
    for (; idx < this->trajectory_stamps.size(); ++idx) {
        this->trajectory_stamps.at(idx) = this->trajectory_stamps.at(idx - 1) + step;

        this->cur_trajectory.at(idx).pose =
          this->cur_trajectory.at(idx - 1).pose.manifoldPlus(step * this->cur_trajectory.at(idx - 1).vel);
        this->cur_trajectory.at(idx).vel = this->cur_trajectory.at(idx - 1).vel;
    }
    // as to limit floating point error
    this->trajectory_stamps.back() =
      std::chrono::duration<float, std::ratio<1>>(stamp - this->scan_stamps_chrono.front()).count();

    this->prior_twist = this->cur_trajectory.back().vel;

    // Now previous trajectory will hold the "motion generated" trajectory
    this->copyTrajectory(this->cur_trajectory, this->prev_trajectory);
}

bool LaserOdom::match(const TimeType &stamp) {
    const int knn = 5;
    this->prepTrajectory(stamp);

    T_TYPE last_transform;
    auto &ref = this->cur_trajectory.back().pose;

    for (uint32_t feat_id = 0; feat_id < this->feat_pts.back().size(); ++feat_id) {
        this->cm1_feat_pts_size.at(feat_id) = (this->feat_pts.rbegin() + 1)->at(feat_id).dimension(1);
    }

    for (int op = 0; op < this->param.opt_iters; op++) {
        this->clearVolatileTracks();
        if (op > 0) {
            last_transform = ref;
        }

        this->transformer.update(this->cur_trajectory, this->trajectory_stamps);

        for (uint32_t j = 0; j < this->N_FEATURES; j++) {
            /// 1 Transform all candidate points to the start of the window
            this->transformer.transformToStart(this->cur_feature_candidates.at(j),
                                               this->cur_feature_candidatesT.at(j),
                                               this->scan_stamps_chrono.size() - 1);
            this->transformer.transformToStart(this->prev_feature_candidates.at(j),
                                               this->prev_feature_candidatesT.at(j),
                                               this->scan_stamps_chrono.size() - 2);
            auto &cfeat = this->cur_feature_candidatesT.at(j);
            this->cur_feat_map.at(j) =
              std::make_shared<Eigen::Map<MatXf>>(cfeat.data(), cfeat.dimension(0), cfeat.dimension(1));

            auto &pfeat = this->prev_feature_candidatesT.at(j);
            this->prev_feat_map.at(j) =
              std::make_shared<Eigen::Map<MatXf>>(pfeat.data(), pfeat.dimension(0), pfeat.dimension(1));

            /// 2. Update average point position from each feature tracks
            this->ave_pts.at(j).resize(3, this->feature_tracks.at(j).size());
#pragma omp parallel for
            for (uint32_t t_idx = 0; t_idx < this->feature_tracks.at(j).size(); t_idx++) {
                const auto &track = this->feature_tracks.at(j).at(t_idx);
                this->ave_pts.at(j).block<3, 1>(0, t_idx) = track.geometry.block<3, 1>(3, 0).cast<float>();
            }
            /// 3. Build kd trees on previous two scans, and on average track locations
            if (this->cur_feat_map.at(j)->cols() > 10) {
                MatXf tempcur = *(this->cur_feat_map.at(j));
                auto cur_kd_idx = Nabo::NNSearchF::createKDTreeLinearHeap(tempcur);
                Eigen::MatrixXi nn_idx;
                Eigen::MatrixXf nn_dist;
                this->cur_feat_idx.at(j).resize(static_cast<unsigned long>(this->cur_feat_map.at(j)->size()));
                std::fill(this->cur_feat_idx.at(j).begin(), this->cur_feat_idx.at(j).end(), -1);
                if (!this->feature_tracks.at(j).empty()) {
                    /// 4. Find correspondences for existing feature tracks in current scan
                    nn_idx.resize(knn, this->ave_pts.at(j).cols());
                    nn_dist.resize(knn, this->ave_pts.at(j).cols());
                    cur_kd_idx->knn(
                      this->ave_pts.at(j), nn_idx, nn_dist, 5, 0, Nabo::NNSearchF::SORT_RESULTS, this->param.max_correspondence_dist);

                    this->extendFeatureTracks(nn_idx, nn_dist, j);
                }
                this->prev_feat_idx.at(j).resize(static_cast<unsigned long>(this->prev_feat_map.at(j)->size()));
                std::fill(this->prev_feat_idx.at(j).begin(), this->prev_feat_idx.at(j).end(), -1);
                if (this->prev_feat_map.at(j)->cols() > 10) {
                    MatXf tempprev = *(this->prev_feat_map.at(j));
                    auto curm1_kd_idx = Nabo::NNSearchF::createKDTreeLinearHeap(tempprev);
                    /// 5. Create new feature tracks between new and old scan
                    nn_idx.resize(knn, this->cur_feat_map.at(j)->cols());
                    nn_dist.resize(knn, this->cur_feat_map.at(j)->cols());
                    curm1_kd_idx->knn(
                      *(this->cur_feat_map.at(j)), nn_idx, nn_dist, 5, 0, Nabo::NNSearchF::SORT_RESULTS, this->param.max_correspondence_dist);
                    this->createNewFeatureTracks(nn_idx, nn_dist, j);
                    delete curm1_kd_idx;
                }
                delete cur_kd_idx;
            }

            /// 6. Merge feature tracks
            this->mergeFeatureTracks(j);

            /// 6.5 Transform all features to the start of the window
            for (uint32_t i = 0; i < this->feat_pts.size(); i++) {
                auto &feat = this->feat_pts.at(i).at(j);
                auto &featT = this->feat_pts_T.at(i).at(j);
                this->transformer.transformToStart(feat, featT, i);
            }
        }
        if (!(this->param.only_extract_features)) {
            ceres::Problem::Options options;
            options.cost_function_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
            options.loss_function_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
            options.local_parameterization_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
            options.disable_all_safety_checks = true;
            ceres::Problem problem(options);
            /// 7. Build Feature Residuals. todo, reuse problem
            this->buildResiduals(problem);

            if (!this->runOptimization(problem))
                return false;
        }
    }
    for(uint32_t feat_id = 0; feat_id < this->N_FEATURES; ++feat_id) {
        auto &dest = this->feature_tracks.at(feat_id);
        auto &src = this->volatile_feature_tracks.at(feat_id);
        dest.insert(dest.end(), std::make_move_iterator(src.begin()), std::make_move_iterator(src.end()));
    }
    return true;
}

void LaserOdom::trackResiduals(ceres::Problem &problem, uint32_t f_idx, VecE <FeatureTrack> &track_list) {
    for (auto &track : track_list) {
        // plane_cost
        if (f_idx == 2) {
            this->local_params.emplace_back(std::make_shared<PlaneParameterization>());
        } else {
            this->local_params.emplace_back(std::make_shared<LineParameterization>());
        }
        problem.AddParameterBlock(track.geometry.data(), 6, this->local_params.back().get());
        if (!(track.optimize)) {
            problem.SetParameterBlockConstant(track.geometry.data());
        }
        for (uint32_t p_idx = 0; p_idx < track.mapping.size(); ++p_idx) {
            this->loss_functions.emplace_back(new BisquareLoss(this->param.robust_param));
            if (f_idx == 2) {
                this->costs.emplace_back(
                        new ImplicitPlaneResidual<12, 6, 12, 6>(p_idx, f_idx, &track, &(this->feat_T_map)));
            } else {
                this->costs.emplace_back(
                        new ImplicitLineResidual<12, 6, 12, 6>(p_idx, f_idx, &track, &(this->feat_T_map)));
            }
            uint32_t start_offset = track.mapping.at(p_idx).state_id;
            problem.AddResidualBlock(this->costs.back().get(),
                                     this->loss_functions.back().get(),
                                     track.geometry.data(),
                                     this->cur_trajectory.at(start_offset).pose.storage.data(),
                                     this->cur_trajectory.at(start_offset).vel.data(),
                                     this->cur_trajectory.at(start_offset + 1).pose.storage.data(),
                                     this->cur_trajectory.at(start_offset + 1).vel.data());
        }
    }
}

void LaserOdom::buildResiduals(ceres::Problem &problem) {
    this->costs.clear();
    this->local_params.clear();
    this->loss_functions.clear();
    for (uint32_t f_idx = 0; f_idx < this->N_FEATURES; ++f_idx) {
        for (uint32_t s_idx = 0; s_idx < this->feat_pts_T.size(); ++s_idx) {
            auto &cfeat = this->feat_pts_T.at(s_idx).at(f_idx);
            this->feat_T_map.at(s_idx).at(f_idx) =
              std::make_shared<Eigen::Map<MatXf>>(cfeat.data(), cfeat.dimension(0), cfeat.dimension(1));
        }
        this->trackResiduals(problem, f_idx, this->feature_tracks.at(f_idx));
        this->trackResiduals(problem, f_idx, this->volatile_feature_tracks.at(f_idx));
    }
    for (uint32_t state_id = 0; state_id < this->cur_trajectory.size(); ++state_id) {
        auto &state = this->cur_trajectory[state_id];
        this->local_params.emplace_back(std::make_shared<NullSE3Parameterization>());
        problem.AddParameterBlock(state.pose.storage.data(), 12, this->local_params.back().get());
        problem.AddParameterBlock(state.vel.data(), 6);
        if (state_id > 0) {
            // create constant velocity residuals between each state
            auto &pstate = this->cur_trajectory[state_id - 1];
            auto dT = this->trajectory_stamps.at(state_id) - this->trajectory_stamps.at(state_id - 1);
            Mat12 weight;
            this->cv_model->calculateLinInvCovariance(
              weight, this->trajectory_stamps.at(state_id - 1), this->trajectory_stamps.at(state_id));
            weight = weight.sqrt();
            this->costs.emplace_back(new ConstantVelocityPrior(weight, dT));
            problem.AddResidualBlock(this->costs.back().get(),
                                     nullptr,
                                     pstate.pose.storage.data(),
                                     state.pose.storage.data(),
                                     pstate.vel.data(),
                                     state.vel.data());
        }
    }
    // finally, just fix the first pose
    problem.SetParameterBlockConstant(this->cur_trajectory.front().pose.storage.data());
}

void LaserOdom::resetTrajectory() {
    for (auto tra : this->cur_trajectory) {
        tra.pose.setIdentity();
        tra.vel.setZero();
    }
    this->prior_twist.setZero();
}

}  // namespace wave