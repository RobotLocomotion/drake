/******************************************************************************
 * include/ips2ra/empty_block_movement.hpp
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

#include "ips2ra_fwd.hpp"
#include "memory.hpp"

namespace ips2ra {
namespace detail {

/**
 * Moves empty blocks to establish invariant:
 * All buckets must consist of full blocks followed by empty blocks.
 */
template <class Cfg>
void Sorter<Cfg>::moveEmptyBlocks(const diff_t my_begin, const diff_t my_end,
                                  const diff_t my_first_empty_block) {
    // Find range of buckets that start in this stripe
    const int bucket_range_start = [&](int i) {
        while (Cfg::alignToNextBlock(bucket_start_[i]) < my_begin) ++i;
        return i;
    }(0);
    const int bucket_range_end = [&](int i) {
        if (my_id_ == num_threads_ - 1) return Cfg::kMaxBuckets;
        while (i < Cfg::kMaxBuckets && Cfg::alignToNextBlock(bucket_start_[i]) < my_end)
            ++i;
        return i;
    }(bucket_range_start);

    /*
     * After classification, a stripe consists of full blocks followed by empty blocks.
     * This means that the invariant above already holds for all buckets except those that
     * cross stripe boundaries.
     *
     * The following cases exist:
     * 1)  The bucket is fully contained within one stripe.
     *     In this case, nothing needs to be done, just set the bucket pointers.
     *
     * 2)  The bucket starts in stripe i, and ends in stripe i+1.
     *     In this case, thread i moves full blocks from the end of the bucket (from the
     *     stripe of thread i+1) to fill the holes at the end of its stripe.
     *
     * 3)  The bucket starts in stripe i, crosses more than one stripe boundary, and ends
     *     in stripe i+k. This is an extension of case 2. In this case, multiple threads
     *     work on the same bucket. Each thread is responsible for filling the empty
     *     blocks in its stripe. The left-most thread will take the right-most blocks.
     *     Therefore, we count how many blocks are fetched by threads to our left before
     *     moving our own blocks.
     */

    // Check if last bucket overlaps the end of the stripe
    const auto bucket_end = Cfg::alignToNextBlock(bucket_start_[bucket_range_end]);
    const bool last_bucket_is_overlapping = bucket_end > my_end;

    // Case 1)
    for (int b = bucket_range_start; b < bucket_range_end - last_bucket_is_overlapping;
         ++b) {
        const auto start = Cfg::alignToNextBlock(bucket_start_[b]);
        const auto stop = Cfg::alignToNextBlock(bucket_start_[b + 1]);
        auto read = stop;
        if (my_first_empty_block <= start) {
            // Bucket is completely empty
            read = start;
        } else if (my_first_empty_block < stop) {
            // Bucket is partially empty
            read = my_first_empty_block;
        }
        bucket_pointers_[b].set(start, read - Cfg::kBlockSize);
    }

    // Cases 2) and 3)
    if (last_bucket_is_overlapping) {
        const int overlapping_bucket = bucket_range_end - 1;
        const auto bucket_start =
                Cfg::alignToNextBlock(bucket_start_[overlapping_bucket]);

        // If it is a very large bucket, other threads will also move blocks around in it
        // (case 3) Count how many filled blocks are in this bucket
        diff_t flushed_elements_in_bucket = 0;
        if (bucket_start < my_begin) {
            int prev_id = my_id_ - 1;
            // Iterate over stripes which are completely contained in this bucket
            while (bucket_start < shared_->local[prev_id]->first_block) {
                const auto eb = shared_->local[prev_id]->first_empty_block;
                flushed_elements_in_bucket += eb - shared_->local[prev_id]->first_block;
                --prev_id;
            }
            // Count blocks in stripe where bucket starts
            const auto eb = shared_->local[prev_id]->first_empty_block;
            // Check if there are any filled blocks in this bucket
            if (eb > bucket_start)
                flushed_elements_in_bucket += eb - bucket_start;
        }

        // Threads to our left will move this many blocks (0 if we are the left-most
        // thread)
        diff_t elements_reserved = 0;
        if (my_begin > bucket_start) {
            // Thread to the left of us get priority
            elements_reserved = my_begin - bucket_start - flushed_elements_in_bucket;

            // Count how many elements we flushed into this bucket
            flushed_elements_in_bucket += my_first_empty_block - my_begin;
        } else if (my_first_empty_block > bucket_start) {
            // We are the left-most thread
            // Count how many elements we flushed into this bucket
            flushed_elements_in_bucket += my_first_empty_block - bucket_start;
        }

        // Find stripe which contains last block of this bucket (off by one)
        // Also continue counting how many filled blocks are in this bucket
        int read_from_thread = my_id_ + 1;
        while (read_from_thread < num_threads_
               && bucket_end > shared_->local[read_from_thread]->first_block) {
            const auto eb = std::min<diff_t>(
                    shared_->local[read_from_thread]->first_empty_block, bucket_end);
            flushed_elements_in_bucket +=
                    eb - shared_->local[read_from_thread]->first_block;
            ++read_from_thread;
        }

        // After moving blocks, this will be the first empty block in this bucket
        const auto first_empty_block_in_bucket =
                bucket_start + flushed_elements_in_bucket;

        // This is the range of blocks we want to fill
        auto write_ptr = begin_ + std::max(my_first_empty_block, bucket_start);
        const auto write_ptr_end = begin_ + std::min(first_empty_block_in_bucket, my_end);

        // Read from other stripes until we filled our blocks
        while (write_ptr < write_ptr_end) {
            --read_from_thread;
            // This is the range of blocks we can read from stripe 'read_from_thread'
            auto read_ptr = std::min(shared_->local[read_from_thread]->first_empty_block,
                                     bucket_end);
            auto read_range_size =
                    read_ptr - shared_->local[read_from_thread]->first_block;

            // Skip reserved blocks
            if (elements_reserved >= read_range_size) {
                elements_reserved -= read_range_size;
                continue;
            }
            read_ptr -= elements_reserved;
            read_range_size -= elements_reserved;
            elements_reserved = 0;

            // Move blocks
            const auto size = std::min(read_range_size, write_ptr_end - write_ptr);
            write_ptr = std::move(begin_ + read_ptr - size, begin_ + read_ptr, write_ptr);
        }

        // Set bucket pointers if the bucket starts in this stripe
        if (my_begin <= bucket_start) {
            bucket_pointers_[overlapping_bucket].set(
                    bucket_start, first_empty_block_in_bucket - Cfg::kBlockSize);
        }
    }
}

}  // namespace detail
}  // namespace ips2ra
