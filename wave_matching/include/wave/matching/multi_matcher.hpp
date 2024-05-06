/** @file
 * @ingroup matching
 *
 * @defgroup multi_matcher
 * Class to hold multiple instances of a matcher and multithread matches
 */

#ifndef WAVE_MULTI_MATCHER_HPP
#define WAVE_MULTI_MATCHER_HPP

#include <thread>
#include <mutex>
#include <condition_variable>
#include <queue>
#include <tuple>
#include "wave/utils/math.hpp"
#include "wave/matching/pcl_common.hpp"
#include "wave/matching/matcher.hpp"

namespace wave {
/** @addtogroup matching
 *  @{ */

/**
 * Class is templated for different matcher types
 * Runs eight matchers in parallel
 * @tparam T matcher type
 * @tparam R matcher params type
 */
template <typename T, typename R>
class MultiMatcher {
 public:
    explicit MultiMatcher(int queue_s = 10, R params = R())
             : n_thread(8), queue_size(queue_s), config(params), 
               matcher0(params), matcher1(params), matcher2(params), matcher3(params),
               matcher4(params), matcher5(params), matcher6(params), matcher7(params) {
        for (int i = 0; i < this->n_thread; i++) {
            this->pool.emplace_back(std::thread(&MultiMatcher<T, R>::spin, this, i));
        }
    }
   explicit MultiMatcher(const MultiMatcher &) = delete;
   explicit MultiMatcher(MultiMatcher &&) = delete;
   MultiMatcher&
   operator=(const MultiMatcher&) = delete;
   MultiMatcher&
   operator=(MultiMatcher&&) = delete;
   
    ~MultiMatcher();

    /** inserts a pair of scans into the queue to be matched. The resulting
     * transform is the transform used to map
     * the source pointcloud to the target pointcloud
     *
     * @param id for result
     * @param source pointcloud
     * @param target pointcloud
     */
    void insert(const int &id,
                const PCLPointCloudPtr &src,
                const PCLPointCloudPtr &target);

    /**
     * Checks to see if there are any remaining matches in the queue.
     * @return
     */
    bool done();

    /**
     * Gets a result at the start of the queue. Will block until a result is
     * ready if the output buffer
     * is empty but there are matches pending.
     *
     * @param id id of result
     * @param transform result
     * @param info information matrix of match
     * @return true if result has been output, false if the output queue is
     * empty and there are no
     * matches pending
     */
    bool getResult(int *id, Eigen::Affine3d *transform, Mat6 *info);

 private:
    const int n_thread;
    const int queue_size;
    size_t remaining_matches{0};
    const R config;
    std::queue<std::tuple<int, PCLPointCloudPtr, PCLPointCloudPtr>> input;
    std::queue<std::tuple<int, Eigen::Affine3d, Mat6>> output;
    std::vector<std::thread> pool;
    T matcher0, matcher1, matcher2, matcher3, matcher4, matcher5, matcher6, matcher7;
    // Synchronization
    std::mutex ip_mutex, op_mutex, cnt_mutex;
    std::condition_variable ip_condition;
    std::condition_variable op_condition;
    bool stop{false};
    T& getMatcher(size_t threadid) {
      switch (threadid) {
         case 0:
            return this->matcher0;
         case 1:
            return this->matcher1;
         case 2:
            return this->matcher2;
         case 3:
            return this->matcher3;
         case 4:
            return this->matcher4;
         case 5:
            return this->matcher5;
         case 6:
            return this->matcher6;
         case 7:
            return this->matcher7;
         default:
            throw std::runtime_error("Invalid thread id");
      }
    }
    /** Function run by each worker thread
     * @param threadid pair of clouds to match
     */
    void spin(int threadid);
};

}  // namespace wave

#endif  // WAVE_MULTI_MATCHER_HPP

#include "wave/matching/impl/multi_matcher_impl.hpp"
