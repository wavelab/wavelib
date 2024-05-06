#ifndef WAVE_MULTI_MATCHER_IMPL_HPP
#define WAVE_MULTI_MATCHER_IMPL_HPP

#include <iostream>

namespace wave {

template <class T, class R>
MultiMatcher<T, R>::~MultiMatcher() {
    // Destroy all the workers
    this->stop = true;
    this->ip_condition.notify_all();
    for (int id = 0; id < this->n_thread; ++id) {
        this->pool.at(id).join();
    }
}

template <class T, class R>
void MultiMatcher<T, R>::spin(int threadid) {
    std::tuple<int, PCLPointCloudPtr, PCLPointCloudPtr> val;
    while (true) {
        {
            std::unique_lock<std::mutex> lock(this->ip_mutex);
            while (!this->stop && this->input.empty()) {
                this->ip_condition.wait(lock);
            }
            if (this->stop) {
                return;
            }
            val = this->input.front();
            this->input.pop();
            lock.unlock();
            this->ip_condition.notify_one();
        }
        this->getMatcher(threadid).setRef(std::get<1>(val));
        this->getMatcher(threadid).setTarget(std::get<2>(val));
        this->getMatcher(threadid).match();
        this->getMatcher(threadid).estimateInfo();
        {
            std::unique_lock<std::mutex> lockop(this->op_mutex);
            this->output.emplace(std::get<0>(val),
                                 this->getMatcher(threadid).getResult(),
                                 this->getMatcher(threadid).getInfo());
            {
                std::unique_lock<std::mutex> lockcnt(this->cnt_mutex);
                --(this->remaining_matches);
            }
            lockop.unlock();
            this->op_condition.notify_one();
        }
    }
}

template <class T, class R>
void MultiMatcher<T, R>::insert(const int &id,
                                const PCLPointCloudPtr &src,
                                const PCLPointCloudPtr &target) {
    {
        std::unique_lock<std::mutex> lock(this->ip_mutex);
        while (this->input.size() >= static_cast<size_t>(this->queue_size)) {
            this->ip_condition.wait(lock);
        }
        this->input.emplace(id, src, target);
        {
            std::unique_lock<std::mutex> lockcnt(this->cnt_mutex);
            ++(this->remaining_matches);
        }
        lock.unlock();
        this->ip_condition.notify_one();
    }
}

template <class T, class R>
bool MultiMatcher<T, R>::done() {
    {
        std::unique_lock<std::mutex> lockcnt(this->cnt_mutex);
        if (this->remaining_matches == 0) {
            return true;
        } else {
            return false;
        }
    }
}

}  // namespace wave

#endif  // WAVE_MULTI_MATCHER_IMPL_HPP
