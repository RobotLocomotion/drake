/******************************************************************************
 * include/ips2ra/synchronization.hpp
 *
 * In-place Parallel Super Scalar Radix Sort (IPS²Ra)
 *
 ******************************************************************************
 * BSD 2-Clause License
 *
 * Copyright © 2017, Michael Axtmann <michael.axtmann@gmail.com>
 * Copyright © 2017, Daniel Ferizovic <daniel.ferizovic@student.kit.edu>
 * Copyright © 2017, Sascha Witt <sascha.witt@kit.edu>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *****************************************************************************/

#pragma once
#if defined(_REENTRANT)

#include <condition_variable>
#include <mutex>

namespace ips2ra {
namespace detail {

/**
 * Thread barrier, also supports single() execution.
 */
class Barrier {
 public:
    explicit Barrier(int num_threads)
        : init_count_(num_threads), hit_count_(num_threads), flag_(false) {}

    inline void barrier();

    template <class F>
    inline void single(F&& func);

    /**
     * Reset the number of threads.
     * No thread must currently be waiting at this barrier.
     */
    void setNumThreads(int num_threads) {
        hit_count_ = init_count_ = num_threads;
    }

 private:
    std::mutex mutex_;
    std::condition_variable cv_;
    int init_count_;
    int hit_count_;
    bool flag_;

    inline void notify_all(std::unique_lock<std::mutex>& lk);
};

/**
 * General synchronization support.
 */
class Sync : public Barrier {
 public:
    explicit Sync(int num_threads) : Barrier(num_threads) {}

    template <class F>
    inline void critical(F&& func);

 private:
    std::mutex mutex_critical_;
};

/**
 * Barrier: Execution resumes only after all threads reached this point.
 */
void Barrier::barrier() {
    std::unique_lock<std::mutex> lk(mutex_);
    if (--hit_count_ == 0) {
        notify_all(lk);
    } else
        cv_.wait(lk, [this, f = flag_] { return f != flag_; });
}

/**
 * Single: Only the first thread executes F, other threads wait.
 */
template <class F>
void Barrier::single(F&& func) {
    std::unique_lock<std::mutex> lk(mutex_);
    if (hit_count_-- == init_count_) {
        lk.unlock();
        func();
        lk.lock();
        --hit_count_;
    }
    if (hit_count_ < 0)
        notify_all(lk);
    else
        cv_.wait(lk, [this, f = flag_] { return f != flag_; });
}

/**
 * Wakes up waiting threads.
 */
void Barrier::notify_all(std::unique_lock<std::mutex>& lk) {
    hit_count_ = init_count_;
    flag_ = !flag_;
    lk.unlock();
    cv_.notify_all();
}

/**
 * Critical: Each thread executes F, one after another.
 */
template <class F>
void Sync::critical(F&& func) {
    std::unique_lock<std::mutex> lk(mutex_critical_);
    func();
}

}  // namespace detail
}  // namespace ips2ra
#endif  // defined(_REENTRANT)
