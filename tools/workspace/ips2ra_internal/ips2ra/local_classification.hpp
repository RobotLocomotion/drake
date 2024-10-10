/******************************************************************************
 * include/ips2ra/local_classification.hpp
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

#include "ips2ra_fwd.hpp"
#include "classifier.hpp"
#include "empty_block_movement.hpp"
#include "memory.hpp"

namespace ips2ra {
namespace detail {

/**
 * Local classification phase.
 */
template <class Cfg>
typename Cfg::difference_type Sorter<Cfg>::classifyLocally(const iterator my_begin,
                                                           const iterator my_end) {
    auto write = my_begin;
    auto& buffers = local_.buffers;

    // Do the classification
    classifier_->template classify(my_begin, my_end, level_,
                                   [&](typename Cfg::bucket_type bucket, iterator it) {
                                       // Only flush buffers on overflow
                                       if (buffers.isFull(bucket)) {
                                           buffers.writeTo(bucket, write);
                                           write += Cfg::kBlockSize;
                                           local_.bucket_size[bucket] += Cfg::kBlockSize;
                                       }
                                       buffers.push(bucket, std::move(*it));
                                   });

    // Update bucket sizes to account for partially filled buckets
    for (int i = 0, end = Cfg::kMaxBuckets; i < end; ++i)
        local_.bucket_size[i] += local_.buffers.size(i);

    return write - begin_;
}

/**
 * Local classification in the sequential case.
 */
template <class Cfg>
void Sorter<Cfg>::sequentialClassification() {
    const auto my_first_empty_block = classifyLocally(begin_, end_);
    // Find bucket boundaries
    diff_t sum = 0;
    bucket_start_[0] = 0;
    for (int i = 0, end = Cfg::kMaxBuckets; i < end; ++i) {
        sum += local_.bucket_size[i];
        bucket_start_[i + 1] = sum;
    }
    IPS2RA_ASSUME_NOT(bucket_start_[Cfg::kMaxBuckets] != end_ - begin_);

    // Set write/read pointers for all buckets
    for (int bucket = 0, end = Cfg::kMaxBuckets; bucket < end; ++bucket) {
        const auto start = Cfg::alignToNextBlock(bucket_start_[bucket]);
        const auto stop = Cfg::alignToNextBlock(bucket_start_[bucket + 1]);
        bucket_pointers_[bucket].set(
                start,
                (start >= my_first_empty_block
                         ? start
                         : (stop <= my_first_empty_block ? stop : my_first_empty_block))
                        - Cfg::kBlockSize);
    }
}

/**
 * Local classification in the parallel case.
 */
template <class Cfg>
void Sorter<Cfg>::parallelClassification() {
    // Compute stripe for each thread
    const auto elements_per_thread = static_cast<double>(end_ - begin_) / num_threads_;
    const auto my_begin =
            begin_ + Cfg::alignToNextBlock(my_id_ * elements_per_thread + 0.5);
    const auto my_end = [&] {
        auto e = begin_ + Cfg::alignToNextBlock((my_id_ + 1) * elements_per_thread + 0.5);
        e = end_ < e ? end_ : e;
        return e;
    }();

    local_.first_block = my_begin - begin_;

    // Do classification
    if (my_begin >= my_end) {
        // Small input (less than two blocks per thread), wait for other threads to finish
        local_.first_empty_block = my_begin - begin_;
        shared_->sync.barrier();
        shared_->sync.barrier();
    } else {
        const auto my_first_empty_block = classifyLocally(my_begin, my_end);

        // Find bucket boundaries
        diff_t sum = 0;
        for (int i = 0, end = Cfg::kMaxBuckets; i < end; ++i) {
            sum += local_.bucket_size[i];
            __atomic_fetch_add(&bucket_start_[i + 1], sum, __ATOMIC_RELAXED);
        }

        local_.first_empty_block = my_first_empty_block;

        shared_->sync.barrier();

#ifdef IPS4O_TIMER
	g_classification.stop(end_ - begin_, "class");
	g_empty_block.start();
#endif

        // Move empty blocks and set bucket write/read pointers
        moveEmptyBlocks(my_begin - begin_, my_end - begin_, my_first_empty_block);

        shared_->sync.barrier();

#ifdef IPS4O_TIMER
        g_empty_block.stop();
        g_classification.start();
#endif
    }
}

}  // namespace detail
}  // namespace ips2ra
