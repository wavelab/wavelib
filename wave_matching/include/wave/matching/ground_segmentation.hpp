/** @file
 * @ingroup matching
 * @author Arun Das (adas@uwaterloo.ca)
 * @author James Servos (jdservos@uwaterloo.ca)
 * @author Leo Koppel (lkoppel@uwaterloo.ca)
 *
 * Gaussian Process Ground Segmentation
 *
 * Based on approach in
 * https://link.springer.com/article/10.1007/s10846-013-9889-4
 *
 */

#ifndef WAVE_GROUNDSEGMENTATION_HPP
#define WAVE_GROUNDSEGMENTATION_HPP

#include <vector>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/transforms.h>
#include <wave/matching/ground_segmentation_params.hpp>
#include <wave/utils/math.hpp>

namespace wave {
/** @addtogroup matching
*  @{ */

/**
 *  Structure to hold point data for ground model
 */
struct SignalPoint {
    double range;
    double height;
    int index;
    bool is_ground;
};

/**
 * Structure to hold collection of points in each linear bin
 */
struct LinCell {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    // Points in the input cloud are referred to by indices
    std::vector<int> bin_indices;  // all the points
    std::vector<int> obs_indices;  // just the obs points
    std::vector<int> drv_indices;
    std::vector<int> ground_indices;  // just the ground points
    int prototype_index;              // index of prototype point
    int cluster_assigned;             // what cluster is it assigned to
    Vec3 obs_mean;                    // mean of obstacle points
};

/**
 *  Structure to hold all linear bins in that angular bin
 */
struct AngCell {
    std::vector<SignalPoint> sig_points;  // range height signal for that sector
    std::vector<LinCell, Eigen::aligned_allocator<LinCell>>
      lin_cell;  // the linear cells within that angle sector
    std::vector<pcl::PointXY> range_height_signal;
};

/**
 *  Structure to hold all angular bins
 *
 *  For now there is not much point to having this as a separate struct.
 *  @todo refactor methods genPolarBinGrid() and initializePolarBinGrid() here.
 */
struct PolarBinGrid {
    std::vector<AngCell, Eigen::aligned_allocator<AngCell>> ang_cells;
};

/**
 * Gaussian process-based ground segmentation filter for point cloud data.
 *
 * Based on Chen, Tongtong, et al. "Gaussian-process-based real-time ground
 * segmentation for autonomous land vehicles." Journal of Intelligent & Robotic
 * Systems 76.3-4 (2014)
 *
 * (https://link.springer.com/article/10.1007/s10846-013-9889-4)
 *
 * Can be a useful pre-processing step for ground vehicle pointclouds.
 *
 * This filter segments the input into three pointclouds:
 *  - ground
 *  - obstacles
 *  - overhanging obstacles - non-ground points which are "drivable" under,
 *  based on the value of GroundSegmentationParams.robot_height.
 *
 *  Use this filter as follows:
 *  pcl::PointCloud<pcl::PointXYZ>::ConstPtr input = getInputSomehow(...);
 *  pcl::PointCloud<pcl::PointXYZ> output{};
 *
 *  ```
 *  GroundSegmentationParams params{};
 *  // Set some params, or keep the default
 *  GroundSegmentation<pcl::PointXYZ> gs{params};
 *  gs.setInputCloud(input);
 *  gs.filter(output);
 *  ```
 *
 *  By default, the output contains obstacles and overhanging obstacles. To
 *  change this, use the setKeepGround, setKeepObstacle, and setKeepOverhanging
 *  methods before calling filter().
 *
 *  @note this filter is templated on the point type. It is pre-compiled in
 *  libwave_matching for PCL's standard 3D point types (see
 *  PCL_XYZ_POINT_TYPES).
 *  To use it with a custom point type, add the following to your source file:
 *  ```
 *  #include <wave/matching/impl/ground_segmentation.hpp>
 *  ```
 */
template <typename PointT>
class GroundSegmentation : public pcl::Filter<PointT> {
 public:
    using PointCloud = typename pcl::Filter<PointT>::PointCloud;

    /** If true, include ground points in the output of filter() */
    void setKeepGround(bool v) {
        this->keep_ground = v;
    }

    /** If true, include undrivable obstacle points in the output of filter() */
    void setKeepObstacle(bool v) {
        this->keep_obs = v;
    }

    /** If true, include "drivable" overhanging obstacle points in the output of
     * filter() */
    void setKeepOverhanging(bool v) {
        this->keep_drv = v;
    }

    /**
     * Constructor
     * @param config: GroundSegmentationParams settings
     */
    explicit GroundSegmentation(const GroundSegmentationParams &config);

    /** Filters the input and sets the output */
    void applyFilter(PointCloud &output) override;


 private:
    /**
     * Collection of all configurable parameters
     */
    GroundSegmentationParams params;

    // Data storage
    // The input pointcloud is in the input_ member inherited from pcl::Filter.
    // Instead of copying points, we keep track of indices to input_.
    PolarBinGrid polar_bin_grid;
    std::vector<int> ground_indices;  // indices of ground points in input cloud
    std::vector<int> obs_indices;  // indices of obstacle points in input cloud
    std::vector<int> drv_indices;  // indices of drivable points in input cloud

    // Options on which outputs to keep when fliter() is called
    bool keep_ground = false;
    bool keep_obs = true;
    bool keep_drv = true;


    /**
     * Bins all points from the input cloud
     *
     * @param[in] input_cloud
     */
    void genPolarBinGrid();

    /**
     * Resets bin data structure
     */
    void initializePolarBinGrid();

    /**
     * Calculates Gram Matrix for given data points. Squared exponential kernel
     *
     * @param[in] ps1 First set of signal points
     * @param[in] ps2 Second set of signal points
     * @param[in] p_l Kernel hyperparameter
     * @param[in] sig_f Kernel hyperparameter
     * @return  Gram Matrix
     */
    MatX genGPModel(std::vector<SignalPoint> &ps1,
                    std::vector<SignalPoint> &ps2,
                    float sig_f,
                    float p_l);

    /**
     * Iteratively builds up ground model
     */
    void sectorINSAC(int);
};

}  // namespace wave

#endif  // WAVE_GROUNDSEGMENTATION_HPP
