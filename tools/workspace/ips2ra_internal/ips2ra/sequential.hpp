/******************************************************************************
 * include/ips2ra/sequential.hpp
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

#include <utility>

#include "ips2ra_fwd.hpp"
#include "base_case.hpp"
#include "memory.hpp"
#include "partitioning.hpp"
#include "sampling.hpp"
#include "ska_sort.hpp"
#include "utils.hpp"

namespace ips2ra {
namespace detail {

#if defined(_REENTRANT)

/**
 * Recursive entry point of the sequential algorithm for the parallel algorithm
 */
template <class Cfg>
void Sorter<Cfg>::sequential(const iterator begin, const Task& task,
                             PrivateQueue<Task>& queue) {
    IPS2RA_IS_NOT(!levelExists(task.level));
    IPS2RA_IS_NOT(task.end - task.begin <= Cfg::kSmallestSortSize);
    IPS2RA_IS_NOT(level_end_ == -1);

    // Check for medium sized base case.
    const auto n = task.end - task.begin;
    if (n < Cfg::kSkasort) {
#ifdef IPS4O_TIMER
        g_overhead.stop();
        g_base_case.start();
#endif

        ips2ra::detail::SimpleSkaSort<Cfg::kSmallestSortSize, Cfg::kBaseCaseSize,
                                      Cfg::kAmericanFlagSort,
                                      key_type>::Sort(begin + task.begin,
                                                      begin + task.end, local_.offs,
                                                      local_.classifier.getExtractor(),
                                                      task.level, level_end_);

#ifdef IPS4O_TIMER
        g_base_case.stop();
        g_overhead.start();
#endif

        return;
    }

    diff_t bucket_start[Cfg::kMaxBuckets + 1];

    // Do the partitioning
    partition<false>(begin + task.begin, begin + task.end, task.level, bucket_start, 0,
                     1);

    // Final base case is executed in cleanup step, so we're done here
    if (isLastLevel(task.level)) { return; }

    // Recurse
    for (int i = Cfg::kMaxBuckets - 1; i >= 0; --i) {
        const auto start = bucket_start[i];
        const auto stop = bucket_start[i + 1];
        if (stop - start > Cfg::kSmallestSortSize)
            queue.emplace(task.begin + start, task.begin + stop, task.level + 1);
    }
}

#endif  // _REENTRANT

/**
 * Entry point of the sequential algorithm.
 */
template <class Cfg>
void Sorter<Cfg>::sequential(const iterator begin, const iterator end) {
    // Check for base case
    static constexpr std::ptrdiff_t threshold = Cfg::kSmallestSortSize;
    auto& extract_key = local_.classifier.getExtractor();
    auto comp = [&extract_key](auto&& l, auto&& r) {
        return extract_key(l) < extract_key(r);
    };
    if (!detail::smallestSortIfAtMostThreshold<threshold>(begin, end, comp)) {
        const auto [level_begin, level_end] = sequentialGetLevels(begin, end);
        level_end_ = level_end;

        if (levelExists(level_begin)) { sequentialRec(begin, end, level_begin); }
    }
}

/**
 * Recursive entry point for sequential algorithm.
 */
template <class Cfg>
void Sorter<Cfg>::sequentialRec(const iterator begin, const iterator end,
                                const int level) {
    IPS2RA_IS_NOT(!levelExists(level));
    IPS2RA_IS_NOT(end - begin <= Cfg::kSmallestSortSize);
    IPS2RA_IS_NOT(level_end_ == -1);

    // Check for medium sized base case.
    const auto n = end - begin;
    if (n < Cfg::kSkasort) {
#ifdef IPS4O_TIMER
        g_overhead.stop();
        g_base_case.start();
#endif

        ips2ra::detail::SimpleSkaSort<Cfg::kSmallestSortSize, Cfg::kBaseCaseSize,
                                      Cfg::kAmericanFlagSort,
                                      key_type>::Sort(begin, end, local_.offs,
                                                      local_.classifier.getExtractor(),
                                                      level, level_end_);

#ifdef IPS4O_TIMER
        g_base_case.stop();
        g_overhead.start();
#endif

        return;
    }

    diff_t bucket_start[Cfg::kMaxBuckets + 1];

    // Do the partitioning
    partition<false>(begin, end, level, bucket_start, 0, 1);

    // Final base case is executed in cleanup step, so we're done here
    if (isLastLevel(level)) return;

#ifdef IPS4O_TIMER
    g_ips4o_level++;
#endif
    // Recurse
    for (int i = 0; i < Cfg::kMaxBuckets; ++i) {
        const auto start = bucket_start[i];
        const auto stop = bucket_start[i + 1];
        if (stop - start > Cfg::kSmallestSortSize)
            sequentialRec(begin + start, begin + stop, level + 1);
    }
#ifdef IPS4O_TIMER
    g_ips4o_level--;
#endif
}

}  // namespace detail

/**
 * Reusable sequential sorter.
 */
template <class Cfg>
class SequentialSorter {
    using Sorter = detail::Sorter<Cfg>;
    using iterator = typename Cfg::iterator;

 public:
    explicit SequentialSorter(typename Cfg::Extractor extractor)
        : buffer_storage_(1)
        , local_ptr_(Cfg::kDataAlignment, std::move(extractor), buffer_storage_.get()) {}

    explicit SequentialSorter(typename Cfg::Extractor extractor, char* buffer_storage)
        : local_ptr_(Cfg::kDataAlignment, std::move(extractor), buffer_storage) {}

    void operator()(iterator begin, iterator end) {
        Sorter(local_ptr_.get()).sequential(std::move(begin), std::move(end));
    }

 private:
    typename Sorter::BufferStorage buffer_storage_;
    detail::AlignedPtr<typename Sorter::LocalData> local_ptr_;
};

}  // namespace ips2ra
