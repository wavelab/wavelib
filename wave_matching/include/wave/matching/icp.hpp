/** @file
 * @ingroup matching
 *
 * Wrapper of ICP in PCL
 *
 * There are a few parameters that may be changed specific to this algorithm.
 * They can be set in the yaml config file.
 *
 * - max_corr: correspondences behind this many distance-units are
 * discarded
 * - max_iter: Limits number of ICP iterations
 * - t_eps: Criteria to stop iterating. If the difference between consecutive
 * transformations is less than this, stop.
 * - fit_eps: Criteria to stop iterating. If the cost function does not improve
 * by more than this quantity, stop.
 */

#ifndef WAVE_MATCHING_ICP_HPP
#define WAVE_MATCHING_ICP_HPP

#include <pcl/registration/icp.h>

#include "wave/matching/pcl_common.hpp"
#include "wave/matching/matcher.hpp"

namespace wave {
/** @addtogroup matching
 *  @{ */

struct ICPMatcherParams {
    ICPMatcherParams(const std::string &config_path);
    ICPMatcherParams() = default;

    /// Maximum distance to correspond points for icp
    double max_corr = 3;
    /// Maximum iterations of ICP
    int max_iter = 100;
    /// Transformation epsilon. Stopping criteria. If the transform changes by
    /// less
    /// than this amount, stop
    double t_eps = 1e-8;
    /// Stopping criteria, if cost function decreases by less than this, stop
    double fit_eps = 1e-2;
    /// Angular variance for lidar sensor model. Used if Censi covariance
    /// estimation is set
    double lidar_ang_covar = 7.78e-9;
    /// Linear variance for lidar sensor model. Used if Censi covariance
    /// estimation is set
    double lidar_lin_covar = 2.5e-4;

    /// When set to more than 0, each match is performed multiple times from
    /// a coarse to fine scale (in terms of voxel downsampling).
    /// Each step doubles the resolution
    int multiscale_steps = 3;

    /// Voxel side length for downsampling. If set to 0, downsampling is
    /// not performed. If multiscale matching is set, this is the resolution
    /// of the final, fine-scale match
    float res = 0.1;
    enum covar_method : int {
        LUM,
        CENSI,
        LUMold
    } covar_estimator = covar_method::LUM;
};

class ICPMatcher : public Matcher<PCLPointCloudPtr> {
 public:
    /** This constructor takes an argument in order to adjust how much
     * downsampling is done before matching is attempted. Pointclouds are
     * downsampled using a voxel filter, the argument is the edge length of
     * each voxel. If resolution is non-positive, no downsampling is used.
     */
    explicit ICPMatcher(ICPMatcherParams params1);
    ~ICPMatcher();
    explicit ICPMatcher(const ICPMatcher &) = delete;
    explicit ICPMatcher(ICPMatcher &&) = delete;
    ICPMatcher&
    operator=(const ICPMatcher&) = delete;
    ICPMatcher&
    operator=(ICPMatcher&&) = delete;
    /** sets the reference pointcloud for the matcher
     * @param ref - Pointcloud
     */
    void setRef(const PCLPointCloudPtr &ref);

    /** sets the target (or scene) pointcloud for the matcher
     * @param targer - Pointcloud
     */
    void setTarget(const PCLPointCloudPtr &target);

    /** runs the matcher, blocks until finished.
     * Returns true if successful
     */
    bool match();

    /** runs covariance estimator, blocks until finished.
     */
    void estimateInfo();

    ICPMatcherParams params;

 private:
    /** An instance of the ICP class from PCL */
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    /** An instance of a PCL voxel filter. It is used to downsample input. */
    pcl::VoxelGrid<pcl::PointXYZ> filter;

    /** Pointers to the reference and target pointclouds. The "final" pointcloud
     * is not exposed. PCL's ICP class creates an aligned verison of the target
     * pointcloud after matching, so the "final" member is used as a sink for
     * it. */
    PCLPointCloudPtr ref, target, final, downsampled_ref, downsampled_target;

    /**
     * Calculates a covariance estimate based on Lu and Milios Scan Matching
     */
    void estimateLUM();
    void estimateLUMold();

    /**
     * Calculates a covariance estimate based on Censi
     */
    void estimateCensi();
};

/** @} group matching */
}  // namespace wave

#endif  // WAVE_MATCHING_ICP_HPP
