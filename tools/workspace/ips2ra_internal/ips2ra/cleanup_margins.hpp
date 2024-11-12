/******************************************************************************
 * include/ips2ra/cleanup_margins.hpp
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

#include <limits>
#include <utility>

#include "ips2ra_fwd.hpp"
#include "base_case.hpp"
#include "memory.hpp"
#include "utils.hpp"

namespace ips2ra {
namespace detail {

/**
 * Saves margins at thread boundaries.
 */
template <class Cfg>
std::pair<int, typename Cfg::difference_type> Sorter<Cfg>::saveMargins(int last_bucket) {
    // Find last bucket boundary in this thread's area
    diff_t tail = bucket_start_[last_bucket];
    const diff_t end = Cfg::alignToNextBlock(tail);

    // Don't need to do anything if there is no overlap, or we are in the overflow case
    if (tail == end || end >= (end_ - begin_)) return {-1, 0};

    // Find bucket this last block belongs to
    {
        const auto start_of_last_block = end - Cfg::kBlockSize;
        diff_t last_start;
        do {
            --last_bucket;
            last_start = bucket_start_[last_bucket];
        } while (last_start > start_of_last_block);
    }

    // Check if the last block has been written
    const auto write = shared_->bucket_pointers[last_bucket].getWrite();
    if (write < end) return {-1, 0};

    // Read excess elements, if necessary
    tail = bucket_start_[last_bucket + 1];
    local_.swap[0].readFrom(begin_ + tail, end - tail);

    return {last_bucket, end - tail};
}

/**
 * Fills margins from buffers.
 */
template <class Cfg>
template <bool kIsParallel>
void Sorter<Cfg>::writeMargins(const int first_bucket, const int last_bucket,
                               const int overflow_bucket, const int swap_bucket,
                               const diff_t in_swap_buffer, const int level) {
    const bool is_last_level = isLastLevel(level_);
    const auto extractor = classifier_->getExtractor();

    for (int i = first_bucket; i < last_bucket; ++i) {
        // Get bucket information
        const auto bstart = bucket_start_[i];
        const auto bend = bucket_start_[i + 1];
        const auto bwrite = bucket_pointers_[i].getWrite();
        // Destination where elements can be written
        auto dst = begin_ + bstart;
        auto remaining = Cfg::alignToNextBlock(bstart) - bstart;

        if (i == overflow_bucket && overflow_) {
            // Is there overflow?

            // Overflow buffer has been written => write pointer must be at end of bucket
            IPS2RA_ASSUME_NOT(Cfg::alignToNextBlock(bend) != bwrite);

            auto src = overflow_->data();
            // There must be space for at least BlockSize elements
            IPS2RA_ASSUME_NOT((bend - (bwrite - Cfg::kBlockSize)) + remaining
                              < Cfg::kBlockSize);
            auto tail_size = Cfg::kBlockSize - remaining;

            // Fill head
            std::move(src, src + remaining, dst);
            src += remaining;
            remaining = std::numeric_limits<diff_t>::max();

            // Write remaining elements into tail
            dst = begin_ + bwrite - Cfg::kBlockSize;
            dst = std::move(src, src + tail_size, dst);

            overflow_->reset(Cfg::kBlockSize);
        } else if (i == swap_bucket && in_swap_buffer) {
            // Did we save this in saveMargins?

            // Bucket of last block in this thread's area => write swap buffer
            auto src = local_.swap[0].data();
            // All elements from the buffer must fit
            IPS2RA_ASSUME_NOT(in_swap_buffer > remaining);

            // Write to head
            dst = std::move(src, src + in_swap_buffer, dst);
            remaining -= in_swap_buffer;

            local_.swap[0].reset(in_swap_buffer);
        } else if (bwrite > bend && bend - bstart > Cfg::kBlockSize) {
            // Final block has been written => move excess elements to head
            IPS2RA_ASSUME_NOT(Cfg::alignToNextBlock(bend) != bwrite);

            auto src = begin_ + bend;
            auto head_size = bwrite - bend;
            // Must fit, no other empty space left
            IPS2RA_ASSUME_NOT(head_size > remaining);

            // Write to head
            dst = std::move(src, src + head_size, dst);
            remaining -= head_size;
        }

        // Write elements from buffers
        for (int t = 0; t < num_threads_; ++t) {
            auto& buffers = kIsParallel ? shared_->local[t]->buffers : local_.buffers;
            auto src = buffers.data(i);
            auto count = buffers.size(i);

            if (count <= remaining) {
                dst = std::move(src, src + count, dst);
                remaining -= count;
            } else {
                std::move(src, src + remaining, dst);
                src += remaining;
                count -= remaining;
                remaining = std::numeric_limits<diff_t>::max();

                dst = begin_ + bwrite;
                dst = std::move(src, src + count, dst);
            }

            buffers.reset(i);
        }

        // Perform final base case sort here, while the data is still cached
        if (!is_last_level && !kIsParallel) {
#ifdef IPS4O_TIMER
            g_cleanup.stop();
            g_base_case.start();
#endif

            // ::detail::StdSortFallback(begin_ + bstart, begin_ + bend, extractor);
            auto comp = [&extractor](auto&& l, auto&& r) {
                return extractor(l) < extractor(r);
            };
            detail::smallestSortIfAtMostThreshold<Cfg::kSmallestSortSize>(
                    begin_ + bstart, begin_ + bend, comp);

#ifdef IPS4O_TIMER
            g_base_case.stop();
            g_cleanup.start();
#endif
        }
    }
}

}  // namespace detail
}  // namespace ips2ra
