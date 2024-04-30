#ifndef WAVE_GROUNDSEGMENTATION_IMPL_HPP
#define WAVE_GROUNDSEGMENTATION_IMPL_HPP

namespace wave {

bool compareSignalPoints(const SignalPoint &a, const SignalPoint &b) {
    return a.height < b.height;
}

template <typename PointT>
GroundSegmentation<PointT>::GroundSegmentation(
  const GroundSegmentationParams &config)
    : params{config} {
    this->initializePolarBinGrid();
}

template <typename PointT>
void GroundSegmentation<PointT>::initializePolarBinGrid() {
    this->polar_bin_grid.ang_cells.clear();
    this->polar_bin_grid.ang_cells.resize(this->params.num_bins_a);
    for (int i = 0; i < this->params.num_bins_a; i++) {
        auto &ang_cell = this->polar_bin_grid.ang_cells[i];
        ang_cell.lin_cell.resize(this->params.num_bins_l);
        ang_cell.sig_points.resize(this->params.num_bins_l);
        ang_cell.range_height_signal.resize(this->params.num_bins_l);
        for (int j = 0; j < this->params.num_bins_l; j++) {
            ang_cell.lin_cell[j].prototype_index = -1;
            ang_cell.range_height_signal[j].x = NAN;
            ang_cell.range_height_signal[j].y = NAN;
            ang_cell.lin_cell[j].cluster_assigned = -1;
        }
    }
}

template <typename PointT>
void GroundSegmentation<PointT>::genPolarBinGrid() {
    this->initializePolarBinGrid();

    double bsize_rad = (double) ((360.0) / this->params.num_bins_a);
    double bsize_lin = (double) this->params.rmax / this->params.num_bins_l;
    const auto num_points = this->input_->size();
    for (auto i = 0u; i < num_points; ++i) {
        const auto &cur_point = (*this->input_)[i];
        const auto &px = cur_point.x;
        const auto &py = cur_point.y;
        const auto &pz = cur_point.z;

        if (sqrt(px * px + py * py + pz * pz) < this->params.rmax) {
            double ph = (atan2(py, px)) * (180 / M_PI);  // in degrees
            ph = wrapTo360(ph);

            // bin into sector

            unsigned int bind_rad =
              static_cast<unsigned int>(ph / bsize_rad);  // got the radial bin

            // get the linear bin
            float xy_dist = std::sqrt(px * px + py * py);

            unsigned int bind_lin = static_cast<unsigned int>(
              xy_dist / bsize_lin);  // got the radial bin

            this->polar_bin_grid.ang_cells[bind_rad]
              .lin_cell[bind_lin]
              .bin_indices.push_back(i);
            // add the point to the bin
            // check the prototype point
            auto &prototype_index = polar_bin_grid.ang_cells[bind_rad]
                                      .lin_cell[bind_lin]
                                      .prototype_index;
            if (prototype_index < 0 ||
                pz < (*this->input_)[prototype_index].z)  // smallest by z
            {
                prototype_index = i;
                this->polar_bin_grid.ang_cells[bind_rad]
                  .range_height_signal[bind_lin]
                  .x = xy_dist;
                this->polar_bin_grid.ang_cells[bind_rad]
                  .range_height_signal[bind_lin]
                  .y = pz;
            }
        }
    }
}

template <typename PointT>
MatX GroundSegmentation<PointT>::genGPModel(std::vector<SignalPoint> &ps1,
                                            std::vector<SignalPoint> &ps2,
                                            float sig_f,
                                            float p_l) {
    size_t nP1 = ps1.size();
    size_t nP2 = ps2.size();

    MatX cmat;
    cmat.resize(nP1, nP2);
    float coeff = (-1 / (2 * p_l * p_l));

    for (size_t i = 0; i < nP1; i++) {
        for (size_t j = 0; j < nP2; j++) {
            double diff = (ps1[i].range - ps2[j].range);
            cmat(i, j) = sig_f * exp(coeff * (diff * diff));
        }
    }
    return cmat;
}

template <typename PointT>
void GroundSegmentation<PointT>::sectorINSAC(int sector_index) {
    if (sector_index >= this->params.num_bins_a) {
        return;
    }
    int num_filled = 0;

    // pull out the valid points from the sector
    auto &sig_points = polar_bin_grid.ang_cells[sector_index].sig_points;
    sig_points.clear();
    for (int i = 0; i < this->params.num_bins_l; i++) {
        if (!std::isnan(polar_bin_grid.ang_cells[sector_index]
                          .range_height_signal[i]
                          .x) &&
            this->polar_bin_grid.ang_cells[sector_index]
                .lin_cell[i]
                .bin_indices.size() > 5) {
            // bin has a valid point, and enough points to make a good
            // guess for a protopoint
            SignalPoint new_point;
            new_point.range =
              polar_bin_grid.ang_cells[sector_index].range_height_signal[i].x;
            new_point.height =
              polar_bin_grid.ang_cells[sector_index].range_height_signal[i].y;
            new_point.index = i;
            sig_points.push_back(new_point);
            num_filled++;
        }
    }
    // get the seed points.  Select the 3 lowest points.  Sort based on height
    // values
    sort(sig_points.begin(), sig_points.end(), compareSignalPoints);

    // now that the z points are sorted by height, take the
    // this->params.num_seed_points worth
    // as the seed
    size_t num_points =
      sig_points.size() < static_cast<size_t>(this->params.num_seed_points)
        ? sig_points.size()
        : static_cast<size_t>(this->params.num_seed_points);
    std::vector<SignalPoint> current_model;
    int point_count = 0;
    int curr_idx = 0;
    bool keep_going = true;
    bool sufficient_model = true;

    while (true) {
        if (static_cast<size_t>(curr_idx) >= sig_points.size())  // overflow
        {
            break;
        }

        if (sig_points[curr_idx].range < this->params.max_seed_range &&
            fabs(sig_points[curr_idx].height) < this->params.max_seed_height) {
            // close enough to
            // robot and height
            // makese sense in
            // robot locality

            sig_points[curr_idx].is_ground = true;
            current_model.push_back(sig_points[curr_idx]);
            sig_points.erase(sig_points.begin() + curr_idx);
            point_count++;

        } else {
            curr_idx++;
        }

        if (static_cast<size_t>(point_count) >= num_points)  // done
        {
            break;
        }
    }

    // check size
    if (current_model.size() < 2)  // not enough for model, all obs pts
    {
        keep_going = false;
        sufficient_model = false;
    }

    // got the seedpoints, start theINSAC process
    // cov matrices
    MatX C_XsX;
    MatX C_XX;
    MatX C_XsXs;
    MatX C_XXs;

    if (sig_points.size() == 0)
        // no points to insac, put the seed points in as ground
        keep_going = false;

    MatX temp;
    MatX f_s;
    MatX Vf_s;
    while (keep_going) {
        // generate the covariance matrices

        C_XsX = genGPModel(
          sig_points, current_model, this->params.p_sf, this->params.p_l);
        C_XX = genGPModel(
          current_model, current_model, this->params.p_sf, this->params.p_l);
        C_XsXs = genGPModel(
          sig_points, sig_points, this->params.p_sf, this->params.p_l);
        C_XXs = C_XsX.transpose();

        // temporary calc
        MatX temp_calc1 =
          C_XX + (this->params.p_sn * MatX::Identity(C_XX.rows(), C_XX.cols()));
        MatX temp_calc2 = C_XsX * temp_calc1.inverse();

        // test the points against the current model

        MatX model_z(current_model.size(), 1);
        for (unsigned int i = 0; i < current_model.size(); i++) {
            model_z(i, 0) = current_model[i].height;
        }

        f_s = temp_calc2 * model_z;
        Vf_s = C_XsXs - temp_calc2 * C_XXs;

        if (Vf_s.rows() == 0) {
            keep_going = false;
            LOG_INFO("Ground segmentation: BREAKING LOOP: VF_s does not exist");
            continue;
        }

        bool search_candidate_points = true;
        unsigned int k = 0;
        // test for inliers using INSAC algorithm
        const auto start_size =
          current_model.size();  // beginning size of the model set
        while (search_candidate_points) {
            double vf = Vf_s(k, k);
            double met = (sig_points[k].height - f_s(k)) /
                         (sqrt(this->params.p_sn + vf * vf));

            if (vf < this->params.p_tmodel &&
                std::abs(met) < this->params.p_tdata) {  // we have an inlier!
                // add to model set
                current_model.push_back(sig_points[k]);
                // remove from sample set
                sig_points.erase(sig_points.begin() + k);

                // delete row from f_s
                temp = f_s;
                f_s.resize(f_s.rows() - 1, f_s.cols());
                f_s.topRows(k) = temp.topRows(k);
                f_s.bottomRows(temp.rows() - k - 1) =
                  temp.bottomRows(temp.rows() - k - 1);

                // delete row from Vf_s
                temp = Vf_s;
                Vf_s.resize(Vf_s.rows() - 1, Vf_s.cols());
                Vf_s.topRows(k) = temp.topRows(k);
                Vf_s.bottomRows(temp.rows() - k - 1) =
                  temp.bottomRows(temp.rows() - k - 1);

                // delete col from Vf_s
                temp = Vf_s;
                Vf_s.resize(Vf_s.rows(), Vf_s.cols() - 1);
                Vf_s.leftCols(k) = temp.leftCols(k);
                Vf_s.rightCols(temp.cols() - k - 1) =
                  temp.rightCols(temp.cols() - k - 1);

            } else {
                k++;
            }

            if (sig_points.size() == k) {
                search_candidate_points = false;
            }
        }

        const auto end_size =
          current_model.size();  // end size of the model set
        if (start_size == end_size || sig_points.size() == 0) {
            keep_going = false;
        }
    }  // end INSAC

    // fill in the ground and obs pointclouds

    double num_obs = 0;
    Vec3 obs_sum(0, 0, 0);

    for (int i = 0; i < (int) current_model.size(); i++) {
        int currIdx = current_model[i].index;
        auto &cur_cell =
          this->polar_bin_grid.ang_cells[sector_index].lin_cell[currIdx];

        // go through all the points in this cell and assign to ground/not
        // ground
        for (const auto j : cur_cell.bin_indices) {
            const auto &cur_point = (*this->input_)[j];
            float h = std::abs(current_model[i].height - cur_point.z);
            if (h < this->params.p_tg)  // z heights are close
            {
                this->ground_indices.push_back(j);
                cur_cell.ground_indices.push_back(j);
            } else {
                // check drivability
                if (h > this->params.robot_height) {
                    // @todo repetitive code
                    this->drv_indices.push_back(j);
                    cur_cell.drv_indices.push_back(j);
                } else {
                    this->obs_indices.push_back(j);
                    cur_cell.obs_indices.push_back(j);
                    obs_sum += Vec3(cur_point.x, cur_point.y, cur_point.z);
                    num_obs++;
                }
            }
        }
        // mean of obs points
        // @todo is this unused?
        cur_cell.obs_mean = obs_sum / num_obs;
    }

    // FIXME: WHY IS F_S < SIGPTR SOMETIMES?
    int i;
    if (sufficient_model) {
        // add all the obs points from the non ground classified pts
        for (i = 0; i < (int) sig_points.size(); i++) {
            auto &cur_cell = polar_bin_grid.ang_cells[sector_index]
                               .lin_cell[sig_points[i].index];

            for (const auto j : cur_cell.bin_indices) {
                const auto &cur_point = (*this->input_)[j];
                float h = std::abs(cur_point.z - f_s(i));
                // check drivability
                if (h > this->params.robot_height) {
                    // @todo repetitive code
                    this->drv_indices.push_back(j);
                    cur_cell.drv_indices.push_back(j);
                } else {
                    this->obs_indices.push_back(j);
                    cur_cell.obs_indices.push_back(j);
                    cur_cell.obs_indices.push_back(j);
                    obs_sum += Vec3{cur_point.x, cur_point.y, cur_point.z};
                    num_obs++;
                }
            }
            // mean of obs points
            // @todo is this unused?
            cur_cell.obs_mean = obs_sum / num_obs;
        }
    }
}

template <typename PointT>
void GroundSegmentation<PointT>::applyFilter(PointCloud &output) {
    // Do the work and fill the indices vectors
    this->genPolarBinGrid();
    for (int i = 0; i < this->params.num_bins_a; i++) {
        this->sectorINSAC(i);
    }

    // Copy the points the user wants
    std::vector<int> out_indices;
    if (this->keep_ground) {
        out_indices = this->ground_indices;
    }
    if (this->keep_obs) {
        out_indices.insert(out_indices.end(),
                           this->obs_indices.begin(),
                           this->obs_indices.end());
    }
    if (this->keep_drv) {
        out_indices.insert(out_indices.end(),
                           this->drv_indices.begin(),
                           this->drv_indices.end());
    }
    pcl::copyPointCloud(*this->input_, out_indices, output);
}

}  // namespace wave

// Helper for explicitly instantiating this template
// See http://pointclouds.org/documentation/tutorials/writing_new_classes.php
#ifdef PCL_NO_PRECOMPILE
#include "impl/ground_segmentation.hpp"
#else
#define PCL_INSTANTIATE_GroundSegmentation(T) \
    template class PCL_EXPORTS wave::GroundSegmentation<T>;
#endif  // PCL_NO_PRECOMPILE

#endif  // WAVE_GROUNDSEGMENTATION_IMPL_HPP
