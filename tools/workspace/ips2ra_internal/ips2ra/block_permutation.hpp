/******************************************************************************
 * include/ips2ra/block_permutation.hpp
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

#include <tuple>

#include "ips2ra_fwd.hpp"
#include "classifier.hpp"
#include "memory.hpp"

namespace ips2ra {
namespace detail {

/**
 * Computes which bucket can cause an overflow,
 * i.e., the last bucket that has more than one full block.
 */
template <class Cfg>
int Sorter<Cfg>::computeOverflowBucket() {
    int bucket = Cfg::kMaxBuckets - 1;
    while (bucket >= 0
           && (bucket_start_[bucket + 1] - bucket_start_[bucket]) <= Cfg::kBlockSize)
        --bucket;
    return bucket;
}

/**
 * Tries to read a block from read_bucket.
 */
template <class Cfg>
template <bool kIsParallel>
int Sorter<Cfg>::classifyAndReadBlock(const int read_bucket) {
    auto& bp = bucket_pointers_[read_bucket];

    diff_t write, read;
    std::tie(write, read) = bp.template decRead<kIsParallel>();

    if (read < write) {
        // No more blocks in this bucket
        if (kIsParallel) bp.stopRead();
        return -1;
    }

    // Read block
    local_.swap[0].readFrom(begin_ + read);
    if (kIsParallel) bp.stopRead();

    return classifier_->classify(local_.swap[0].head(), level_);
}

/**
 * Finds a slot for the block in the swap buffer. May or may not read another block.
 */
template <class Cfg>
template <bool kIsParallel>
int Sorter<Cfg>::swapBlock(const diff_t max_off, const int dest_bucket,
                           const bool current_swap) {
    diff_t write, read;
    int new_dest_bucket;
    auto& bp = bucket_pointers_[dest_bucket];
    do {
        std::tie(write, read) = bp.template incWrite<kIsParallel>();
        if (write > read) {
            // Destination block is empty
            if (write >= max_off) {
                // Out-of-bounds; write to overflow buffer instead
                local_.swap[current_swap].writeTo(local_.overflow);
                overflow_ = &local_.overflow;
                return -1;
            }
            // Make sure no one is currently reading this block
            while (kIsParallel && bp.isReading()) {}
            // Write block
            local_.swap[current_swap].writeTo(begin_ + write);
            return -1;
        }
        // Check if block needs to be moved
        new_dest_bucket = classifier_->template classify(begin_[write], level_);
    } while (new_dest_bucket == dest_bucket);

    // Swap blocks
    local_.swap[!current_swap].readFrom(begin_ + write);
    local_.swap[current_swap].writeTo(begin_ + write);

    return new_dest_bucket;
}

/**
 * Block permutation phase.
 */
template <class Cfg>
template <bool kIsParallel>
void Sorter<Cfg>::permuteBlocks() {
    // Distribute starting points of threads
    int read_bucket = (my_id_ * Cfg::kMaxBuckets / num_threads_) % Cfg::kMaxBuckets;
    // Not allowed to write to this offset, to avoid overflow
    const diff_t max_off = Cfg::alignToNextBlock(end_ - begin_ + 1) - Cfg::kBlockSize;

    // Go through all buckets
    for (int count = Cfg::kMaxBuckets; count; --count) {
        int dest_bucket;
        // Try to read a block ...
        while ((dest_bucket = classifyAndReadBlock<kIsParallel>(read_bucket)) != -1) {
            bool current_swap = 0;
            // ... then write it to the correct bucket
            while ((dest_bucket =
                            swapBlock<kIsParallel>(max_off, dest_bucket, current_swap))
                   != -1) {
                // Read another block, keep going
                current_swap = !current_swap;
            }
        }
        read_bucket = (read_bucket + 1) % Cfg::kMaxBuckets;
    }
}

}  // namespace detail
}  // namespace ips2ra
