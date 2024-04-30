// Software License Agreement (BSD License)
//
// Point Cloud Library (PCL) - www.pointclouds.org
// Copyright (c) 2009-2012, Willow Garage, Inc.
// Copyright (c) 2012-, Open Perception, Inc.
// Copyright (c) XXX, respective authors.
//
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
//        modification, are permitted provided that the following conditions
// are met:
//
//* Redistributions of source code must retain the above copyright
//        notice, this list of conditions and the following disclaimer.
//* Redistributions in binary form must reproduce the above
// copyright notice, this list of conditions and the following
//        disclaimer in the documentation and/or other materials provided
//        with the distribution.
//* Neither the name of the copyright holder(s) nor the names of its
//        contributors may be used to endorse or promote products derived
//        from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
//"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
//        LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
//        FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
//        COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
//        INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
//                                                                  BUT NOT
//                                                                  LIMITED TO,
//                                                                  PROCUREMENT
//                                                                  OF
//                                                                  SUBSTITUTE
//                                                                  GOODS OR
//                                                                  SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
//        CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
// LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
// ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
//        POSSIBILITY OF SUCH DAMAGE.

#include "wave/matching/icp.hpp"

/// 3D formulation of the approach by Lu & Milios
/// http://www-robotics.usc.edu/~gaurav/CS547/milios_map.pdf
/// Implementation from PCL

namespace wave {

void ICPMatcher::estimateLUMold() {
    auto &source_trans = this->final;
    PCLPointCloudPtr targetc;
    if (this->params.res > 0) {
        targetc = this->downsampled_target;
    } else {
        targetc = this->target;
    }
    uint64_t numSourcePts = source_trans->size();
    std::vector<Eigen::Vector3f> corrs_aver{numSourcePts};
    std::vector<Eigen::Vector3f> corrs_diff{numSourcePts};
    int numCorr = 0;

    Mat6 edgeCov = Mat6::Identity();

    // build kd tree for source points
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud(targetc);

    // iterate through the source cloud and compute match covariance

    for (uint64_t i = 0; i < numSourcePts; i++) {
        pcl::PointXYZ qpt = source_trans->points[i];
        std::vector<int> nn_idx;
        std::vector<float> nn_sqr_dist;
        kdtree.nearestKSearch(
          qpt,
          1,
          nn_idx,
          nn_sqr_dist);  // returns the index of the nn point in the targetc

        if (nn_sqr_dist[0] < this->params.max_corr *
                               this->params.max_corr)  // if the distance to
        // point is less than max
        // correspondence
        // distance, use it to
        // calculate
        {
            Eigen::Vector3f source_pt = qpt.getVector3fMap();
            Eigen::Vector3f target_pt =
              targetc->points[nn_idx[0]].getVector3fMap();

            // Compute the point pair average and difference and store for later
            // use
            corrs_aver[numCorr] = 0.5 * (source_pt + target_pt);
            corrs_diff[numCorr] = source_pt - target_pt;
            numCorr++;
        } else {
            continue;
        }
    }
    corrs_aver.resize(numCorr);
    corrs_diff.resize(numCorr);

    // now compute the M matrix
    wave::Mat6 MM = wave::Mat6::Zero();
    wave::Vec6 MZ = wave::Vec6::Zero();
    for (int ci = 0; ci != numCorr; ++ci)  // ci = correspondence iterator
    {
        // Fast computation of summation elements of M'M
        MM(0, 4) -= corrs_aver[ci](1);
        MM(0, 5) += corrs_aver[ci](2);
        MM(1, 3) -= corrs_aver[ci](2);
        MM(1, 4) += corrs_aver[ci](0);
        MM(2, 3) += corrs_aver[ci](1);
        MM(2, 5) -= corrs_aver[ci](0);
        MM(3, 4) -= corrs_aver[ci](0) * corrs_aver[ci](2);
        MM(3, 5) -= corrs_aver[ci](0) * corrs_aver[ci](1);
        MM(4, 5) -= corrs_aver[ci](1) * corrs_aver[ci](2);
        MM(3, 3) += corrs_aver[ci](1) * corrs_aver[ci](1) +
                    corrs_aver[ci](2) * corrs_aver[ci](2);
        MM(4, 4) += corrs_aver[ci](0) * corrs_aver[ci](0) +
                    corrs_aver[ci](1) * corrs_aver[ci](1);
        MM(5, 5) += corrs_aver[ci](0) * corrs_aver[ci](0) +
                    corrs_aver[ci](2) * corrs_aver[ci](2);

        // Fast computation of M'Z
        MZ(0) += corrs_diff[ci](0);
        MZ(1) += corrs_diff[ci](1);
        MZ(2) += corrs_diff[ci](2);
        MZ(3) += corrs_aver[ci](1) * corrs_diff[ci](2) -
                 corrs_aver[ci](2) * corrs_diff[ci](1);
        MZ(4) += corrs_aver[ci](0) * corrs_diff[ci](1) -
                 corrs_aver[ci](1) * corrs_diff[ci](0);
        MZ(5) += corrs_aver[ci](2) * corrs_diff[ci](0) -
                 corrs_aver[ci](0) * corrs_diff[ci](2);
    }
    // Remaining elements of M'M
    MM(0, 0) = MM(1, 1) = MM(2, 2) = static_cast<float>(numCorr);
    MM(4, 0) = MM(0, 4);
    MM(5, 0) = MM(0, 5);
    MM(3, 1) = MM(1, 3);
    MM(4, 1) = MM(1, 4);
    MM(3, 2) = MM(2, 3);
    MM(5, 2) = MM(2, 5);
    MM(4, 3) = MM(3, 4);
    MM(5, 3) = MM(3, 5);
    MM(5, 4) = MM(4, 5);

    // Compute pose difference estimation
    wave::Vec6 D = static_cast<wave::Vec6>(MM.inverse() * MZ);

    // Compute s^2
    float ss = 0.0f;
    for (int ci = 0; ci != numCorr; ++ci)  // ci = correspondence iterator
    {
        ss += static_cast<float>(
          pow(corrs_diff[ci](0) -
                (D(0) + corrs_aver[ci](2) * D(5) - corrs_aver[ci](1) * D(4)),
              2.0f) +
          pow(corrs_diff[ci](1) -
                (D(1) + corrs_aver[ci](0) * D(4) - corrs_aver[ci](2) * D(3)),
              2.0f) +
          pow(corrs_diff[ci](2) -
                (D(2) + corrs_aver[ci](1) * D(3) - corrs_aver[ci](0) * D(5)),
              2.0f));
    }

    // When reaching the limitations of computation due to linearization
    if (ss < 0.0000000000001 || !std::isfinite(ss)) {
        LOG_ERROR("Covariance matrix calculation was unsuccessful");
        this->information = wave::Mat6::Identity();
    }

    // Store the results in the slam graph
    edgeCov = MM * (1.0f / ss);

    this->information = edgeCov;
}

// Taken from the Lu and Milios matcher in PCL
void ICPMatcher::estimateLUM() {
    auto &ref = this->final;
    PCLPointCloudPtr targetc;
    if (this->params.res > 0) {
        targetc = this->downsampled_target;
    } else {
        targetc = this->target;
    }
    if (this->icp.hasConverged()) {
        auto list = this->icp.correspondences_.get();
        Mat6 MM = Mat6::Zero();
        Vec6 MZ = Vec6::Zero();
        std::vector<Eigen::Vector3f> corrs_aver;
        std::vector<Eigen::Vector3f> corrs_diff;


        int numCorr = 0;
        for (auto it = list->begin(); it != list->end(); ++it) {
            if (it->index_match > -1) {
                corrs_aver.push_back(
                  Eigen::Vector3f(0.5f * (ref->points[it->index_query].x +
                                          targetc->points[it->index_match].x),
                                  0.5f * (ref->points[it->index_query].y +
                                          targetc->points[it->index_match].y),
                                  0.5f * (ref->points[it->index_query].z +
                                          targetc->points[it->index_match].z)));
                corrs_diff.push_back(
                  Eigen::Vector3f(ref->points[it->index_query].x -
                                    targetc->points[it->index_match].x,
                                  ref->points[it->index_query].y -
                                    targetc->points[it->index_match].y,
                                  ref->points[it->index_query].z -
                                    targetc->points[it->index_match].z));
                numCorr++;
            }
        }

        for (int ci = 0; ci != numCorr; ++ci)  // ci = correspondence iterator
        {
            // Fast computation of summation elements of M'M
            MM(0, 4) -= corrs_aver[ci](1);
            MM(0, 5) += corrs_aver[ci](2);
            MM(1, 3) -= corrs_aver[ci](2);
            MM(1, 4) += corrs_aver[ci](0);
            MM(2, 3) += corrs_aver[ci](1);
            MM(2, 5) -= corrs_aver[ci](0);
            MM(3, 4) -= corrs_aver[ci](0) * corrs_aver[ci](2);
            MM(3, 5) -= corrs_aver[ci](0) * corrs_aver[ci](1);
            MM(4, 5) -= corrs_aver[ci](1) * corrs_aver[ci](2);
            MM(3, 3) += corrs_aver[ci](1) * corrs_aver[ci](1) +
                        corrs_aver[ci](2) * corrs_aver[ci](2);
            MM(4, 4) += corrs_aver[ci](0) * corrs_aver[ci](0) +
                        corrs_aver[ci](1) * corrs_aver[ci](1);
            MM(5, 5) += corrs_aver[ci](0) * corrs_aver[ci](0) +
                        corrs_aver[ci](2) * corrs_aver[ci](2);

            // Fast computation of M'Z
            MZ(0) += corrs_diff[ci](0);
            MZ(1) += corrs_diff[ci](1);
            MZ(2) += corrs_diff[ci](2);
            MZ(3) += corrs_aver[ci](1) * corrs_diff[ci](2) -
                     corrs_aver[ci](2) * corrs_diff[ci](1);
            MZ(4) += corrs_aver[ci](0) * corrs_diff[ci](1) -
                     corrs_aver[ci](1) * corrs_diff[ci](0);
            MZ(5) += corrs_aver[ci](2) * corrs_diff[ci](0) -
                     corrs_aver[ci](0) * corrs_diff[ci](2);
        }
        // Remaining elements of M'M
        MM(0, 0) = MM(1, 1) = MM(2, 2) = static_cast<float>(numCorr);
        MM(4, 0) = MM(0, 4);
        MM(5, 0) = MM(0, 5);
        MM(3, 1) = MM(1, 3);
        MM(4, 1) = MM(1, 4);
        MM(3, 2) = MM(2, 3);
        MM(5, 2) = MM(2, 5);
        MM(4, 3) = MM(3, 4);
        MM(5, 3) = MM(3, 5);
        MM(5, 4) = MM(4, 5);

        // Compute pose difference estimation
        Vec6 D = static_cast<Vec6>(MM.inverse() * MZ);

        // Compute s^2
        float ss = 0.0f;
        for (int ci = 0; ci != numCorr; ++ci)  // ci = correspondence iterator
        {
            ss += static_cast<float>(
              pow(corrs_diff[ci](0) - (D(0) + corrs_aver[ci](2) * D(5) -
                                       corrs_aver[ci](1) * D(4)),
                  2.0f) +
              pow(corrs_diff[ci](1) - (D(1) + corrs_aver[ci](0) * D(4) -
                                       corrs_aver[ci](2) * D(3)),
                  2.0f) +
              pow(corrs_diff[ci](2) - (D(2) + corrs_aver[ci](1) * D(3) -
                                       corrs_aver[ci](0) * D(5)),
                  2.0f));
        }

        // When reaching the limitations of computation due to linearization
        if (ss < 0.0000000000001 || !std::isfinite(ss)) {
            LOG_ERROR("Covariance matrix calculation was unsuccessful");
            this->information = Mat6::Identity();
            return;
        }

        this->information = MM * (1.0f / ss);
    }
}

}  // namespace wave