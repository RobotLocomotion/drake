/******************************************************************************
 * include/ips2ra/thread_pool.hpp
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
#ifdef _OPENMP
#include <omp.h>

#include "synchronization.hpp"
#endif  // _OPENMP

#ifdef _REENTRANT
#include <algorithm>
#include <cassert>
#include <functional>
#include <memory>
#include <thread>
#include <vector>

#include "synchronization.hpp"
#endif  // _REENTRANT

namespace ips2ra {

#ifdef _OPENMP

/**
 * A thread pool using OpenMP.
 */

class OpenMPThreadPool {
 public:
    class Sync {
     public:
        void barrier() const {
#pragma omp barrier
        }

#ifdef __INTEL_COMPILER
        // Workaround: icc with OpenMP fails here when using lambdas as
        template parameter void single(std::function<void()> func) const {
#else
        template <class F>
        void single(F&& func) const {
#endif
#pragma omp single
            func();
        }

        template <class F>
        void critical(F&& func) const {
#pragma omp critical
            func();
        }
    };

    explicit OpenMPThreadPool(int num_threads = OpenMPThreadPool::maxNumThreads())
        : num_threads_(num_threads) {}

    /**
     * Entry point for parallel execution.
     */
    template <class F>
    void operator()(F&& func, int num_threads = std::numeric_limits<int>::max()) {
        num_threads = std::min(num_threads, num_threads_);
        if (num_threads > 1)
#pragma omp parallel num_threads(num_threads)
        {
            int my_id = omp_get_thread_num();
            int threads = omp_get_num_threads();

            func(my_id, threads);
        } else
            func(0, 1);
    }

    Sync sync() const { return {}; }

    int numThreads() const { return num_threads_; }

    static int maxNumThreads() { return omp_get_max_threads(); }

 private:
    int num_threads_;
};

#endif  // _OPENMP

#ifdef _REENTRANT

/**
 * A thread pool using std::thread.
 */
class StdThreadPool {
 public:
    using Sync = detail::Sync;

    explicit StdThreadPool(int num_threads = StdThreadPool::maxNumThreads())
        : impl_(new Impl(num_threads)) {}

    template <class F>
    void operator()(F&& func, int num_threads = std::numeric_limits<int>::max()) {
        num_threads = std::min(num_threads, numThreads());
        if (num_threads > 1)
            impl_.get()->run(std::forward<F>(func), num_threads);
        else
            func(0, 1);
    }

    Sync& sync() { return impl_.get()->sync_; }

    int numThreads() const { return impl_.get()->threads_.size() + 1; }

    static int maxNumThreads() { return std::thread::hardware_concurrency(); }

 private:
    struct Impl {
        Sync sync_;
        detail::Barrier pool_barrier_;
        std::vector<std::thread> threads_;
        std::function<void(int, int)> func_;
        int num_threads_;
        bool done_ = false;

        Impl(int num_threads);
        ~Impl();

        template <class F>
        inline void run(F&& func, const int num_threads);

        inline void main(const int my_id);
    };

    std::unique_ptr<Impl> impl_;
};

/**
 * Constructor for the std::thread pool.
 */
inline StdThreadPool::Impl::Impl(int num_threads)
    : sync_(std::max(1, num_threads))
    , pool_barrier_(std::max(1, num_threads))
    , num_threads_(num_threads)
{
    num_threads = std::max(1, num_threads);
    threads_.reserve(num_threads - 1);
    for (int i = 1; i < num_threads; ++i)
        threads_.emplace_back(&Impl::main, this, i);
}

/**
 * Destructor for the std::thread pool.
 */
inline StdThreadPool::Impl::~Impl() {
    done_ = true;
    pool_barrier_.barrier();
    for (auto& t : threads_)
        t.join();
}

/**
 * Entry point for parallel execution for the std::thread pool.
 */
template <class F>
void StdThreadPool::Impl::run(F&& func, int num_threads) {
    func_ = func;
    num_threads_ = num_threads;
    sync_.setNumThreads(num_threads);

    pool_barrier_.barrier();
    func_(0, num_threads);
    pool_barrier_.barrier();
}

/**
 * Main loop for threads created by the std::thread pool.
 */
inline void StdThreadPool::Impl::main(const int my_id) {
    for (;;) {
        pool_barrier_.barrier();
        if (done_) break;
        if (my_id < num_threads_)
            func_(my_id, num_threads_);
        pool_barrier_.barrier();
    }
}

#endif  // _REENTRANT

#if defined(_REENTRANT)

/**
 * A thread pool to which external threads can join.
 */
class ThreadJoiningThreadPool {
 public:
    using Sync = detail::Sync;

    explicit ThreadJoiningThreadPool(int num_threads) : impl_(new Impl(num_threads)) {
        assert(num_threads >= 2);
    }

    void join(int my_id) { impl_->join(my_id); }

    void release_threads() { impl_->release_threads(); }

    template <class F>
    void operator()(F&& func, int num_threads = std::numeric_limits<int>::max()) {
        num_threads = std::min(num_threads, numThreads());
        if (num_threads > 1)
            impl_->run(std::forward<F>(func), num_threads);
        else
            func(0, 1);
    }

    Sync& sync() { return impl_.get()->sync_; }

    int numThreads() const { return impl_.get()->num_threads_; }

 private:
    struct Impl {
        Sync sync_;
        detail::Barrier pool_barrier_;
        std::function<void(int, int)> func_;
        int num_threads_;
        bool done_ = false;

        Impl(int num_threads);
        ~Impl();

        template <class F>
        inline void run(F&& func, const int num_threads);

        inline void join(int my_id);
        inline void release_threads();

        inline void main(const int my_id);
    };

    std::unique_ptr<Impl> impl_;
};

/**
 * Constructor for the std::thread pool.
 */
inline ThreadJoiningThreadPool::Impl::Impl(int num_threads)
    : sync_(num_threads), pool_barrier_(num_threads), num_threads_(num_threads) {}

/**
 * Destructor for the std::thread pool.
 */
inline ThreadJoiningThreadPool::Impl::~Impl() { assert(done_ == true); }

/**
 * Entry point for parallel execution for the std::thread pool.
 */
template <class F>
void ThreadJoiningThreadPool::Impl::run(F&& func, int num_threads) {
    func_ = func;
    num_threads_ = num_threads;
    sync_.setNumThreads(num_threads);

    pool_barrier_.barrier();
    func_(0, num_threads);
    pool_barrier_.barrier();
}

/**
 * Main loop for threads which have joined.
 */
inline void ThreadJoiningThreadPool::Impl::main(const int my_id) {
    for (;;) {
        pool_barrier_.barrier();
        if (done_) break;
        if (my_id < num_threads_) func_(my_id, num_threads_);
        pool_barrier_.barrier();
    }
}

inline void ThreadJoiningThreadPool::Impl::join(int my_id) { main(my_id); }

inline void ThreadJoiningThreadPool::Impl::release_threads() {
    done_ = true;
    pool_barrier_.barrier();
}

#endif  // defined(_REENTRANT)

#if defined(_REENTRANT) && defined(_OPENMP)
using DefaultThreadPool = OpenMPThreadPool;
#elif defined(_REENTRANT)
using DefaultThreadPool = StdThreadPool;
#else
#error No Default Thread Pool Defined
#endif

}  // namespace ips2ra
