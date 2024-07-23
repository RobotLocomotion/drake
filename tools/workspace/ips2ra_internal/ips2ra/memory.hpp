/******************************************************************************
 * include/ips2ra/memory.hpp
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

#include <algorithm>
#include <atomic>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <mutex>
#include <random>
#include <utility>
#include <vector>

#include <tbb/concurrent_queue.h>

#include "ips2ra_fwd.hpp"
#include "bucket_pointers.hpp"
#include "buffers.hpp"
#include "classifier.hpp"
#include "config.hpp"
#include "scheduler.hpp"

namespace ips2ra {
namespace detail {

/**
 * Aligns a pointer.
 */
template <class T>
static T* alignPointer(T* ptr, std::size_t alignment) {
    uintptr_t v = reinterpret_cast<std::uintptr_t>(ptr);
    v = (v - 1 + alignment) & ~(alignment - 1);
    return reinterpret_cast<T*>(v);
}

/**
 * Constructs an object at the specified alignment.
 */
template <class T>
class AlignedPtr {
 public:
    AlignedPtr() {}

    template <class... Args>
    explicit AlignedPtr(std::size_t alignment, Args&&... args)
        : alloc_(new char[sizeof(T) + alignment])
        , value_(new (alignPointer(alloc_, alignment)) T(std::forward<Args>(args)...)) {}

    AlignedPtr(const AlignedPtr&) = delete;
    AlignedPtr& operator=(const AlignedPtr&) = delete;

    AlignedPtr(AlignedPtr&& rhs) : alloc_(rhs.alloc_), value_(rhs.value_) {
        rhs.alloc_ = nullptr;
    }
    AlignedPtr& operator=(AlignedPtr&& rhs) {
        std::swap(alloc_, rhs.alloc_);
        std::swap(value_, rhs.value_);
        return *this;
    }

    ~AlignedPtr() {
        if (alloc_) {
            value_->~T();
            delete[] alloc_;
        }
    }

    T& get() { return *value_; }

 private:
    char* alloc_ = nullptr;
    T* value_;
};

/**
 * Provides aligned storage without constructing an object.
 */
template <>
class AlignedPtr<void> {
 public:
    AlignedPtr() {}

    template <class... Args>
    explicit AlignedPtr(std::size_t alignment, std::size_t size)
        : alloc_(new char[size + alignment]), value_(alignPointer(alloc_, alignment)) {}

    AlignedPtr(const AlignedPtr&) = delete;
    AlignedPtr& operator=(const AlignedPtr&) = delete;

    AlignedPtr(AlignedPtr&& rhs) : alloc_(rhs.alloc_), value_(rhs.value_) {
        rhs.alloc_ = nullptr;
    }
    AlignedPtr& operator=(AlignedPtr&& rhs) {
        std::swap(alloc_, rhs.alloc_);
        std::swap(value_, rhs.value_);
        return *this;
    }

    ~AlignedPtr() {
        if (alloc_) { delete[] alloc_; }
    }

    char* get() { return value_; }

 private:
    char* alloc_ = nullptr;
    char* value_;
};

/**
 * Aligned storage for use in buffers.
 */
template <class Cfg>
class Sorter<Cfg>::BufferStorage : public AlignedPtr<void> {
 public:
    static constexpr const auto kPerThread = Cfg::kBlockSizeInBytes * Cfg::kMaxBuckets;

    BufferStorage() {}

    explicit BufferStorage(int num_threads)
        : AlignedPtr<void>(Cfg::kDataAlignment, num_threads * kPerThread) {}

    char* forThread(int id) { return this->get() + id * kPerThread; }
};

/**
 * Data local to each thread.
 */
template <class Cfg>
struct Sorter<Cfg>::LocalData {
    using diff_t = typename Cfg::difference_type;

    // Buffers
    diff_t bucket_size[Cfg::kMaxBuckets];
    Buffers buffers;
    Block swap[2];
    Block overflow;

    // Bucket information
    BucketPointers bucket_pointers[Cfg::kMaxBuckets];
    PrivateQueue<Task> seq_task_queue;

    // Classifier
    Classifier classifier;

    // Information used during empty block movement
    diff_t first_block;
    diff_t first_empty_block;

    // Random bit generator for sampling
    // LCG using constants by Knuth (for 64 bit) or Numerical Recipes (for 32 bit)
    std::linear_congruential_engine<
            std::uintptr_t, Cfg::kIs64Bit ? 6364136223846793005u : 1664525u,
            Cfg::kIs64Bit ? 1442695040888963407u : 1013904223u, 0u>
            random_generator;

    // Offsets for the base case algorithm
    std::ptrdiff_t offs[2 * (Cfg::kBaseCaseSize + 1)];

    LocalData(typename Cfg::Extractor extractor, char* buffer_storage)
        : buffers(buffer_storage), classifier(std::move(extractor)) {
        std::random_device rdev;
        std::ptrdiff_t seed = rdev();
        if (Cfg::kIs64Bit)
            seed = (seed << (Cfg::kIs64Bit * 32)) | rdev();
        random_generator.seed(seed);
        reset();
    }

    /**
     * Resets local data after partitioning is done.
     */
    void reset() { std::fill_n(bucket_size, Cfg::kMaxBuckets, 0); }
};

/**
 * Data describing a parallel task and the corresponding threads.
 */
struct BigTask {
    BigTask() : has_task{false} {}
    // TODO or Cfg::iterator???
    std::ptrdiff_t begin;
    std::ptrdiff_t end;
    // Index of the byte used for classification.
    int level;
    // My thread id of this task.
    int task_thread_id;
    // Index of the thread owning the thread pool used by this task.
    int root_thread;
    // Indicates whether this is a task or not
    bool has_task;
};

/**
 * Data shared between all threads.
 */
template <class Cfg>
struct Sorter<Cfg>::SharedData {
    // Bucket information
    typename Cfg::difference_type bucket_start[Cfg::kMaxBuckets + 1];
    BucketPointers bucket_pointers[Cfg::kMaxBuckets];
    Block* overflow;

    // Classifier for parallel partitioning
    Classifier classifier;

    // Synchronisation support
    typename Cfg::Sync sync;

    // Local thread data
    std::vector<LocalData*> local;

    // Thread pools for bigtasks. One entry for each thread.
    std::vector<std::shared_ptr<SubThreadPool>> thread_pools;

    // Bigtasks. One entry per thread.
    std::vector<BigTask> big_tasks;

    // Scheduler of small tasks.
    Scheduler<Task> scheduler;

    SharedData(typename Cfg::Extractor extractor, typename Cfg::Sync sync,
               int num_threads)
        : classifier(std::move(extractor))
        , sync(std::forward<typename Cfg::Sync>(sync))
        , local(num_threads)
        , thread_pools(num_threads)
        , big_tasks(num_threads)
        , scheduler(num_threads) {
        reset();
    }

    /**
     * Resets shared data after partitioning is done.
     */
    void reset() {
        std::fill_n(bucket_start, Cfg::kMaxBuckets + 1, 0);
        overflow = nullptr;
        scheduler.reset();
    }
};

}  // namespace detail
}  // namespace ips2ra
