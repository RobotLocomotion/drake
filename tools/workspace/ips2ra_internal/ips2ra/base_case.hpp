/******************************************************************************
 * include/ips2ra/base_case.hpp
 *
 * In-place Parallel Super Scalar Radix Sort (IPS²Ra)
 *
 ******************************************************************************
 * BSD 2-Clause License
 *
 * Copyright © 2017-2020, Michael Axtmann <michael.axtmann@gmail.com>
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
#include <cstddef>
#include <utility>

#include "utils.hpp"

namespace ips2ra {
namespace detail {

/**
 * Insertion sort.
 */
template <class It, class Comp>
void insertionSort(const It begin, const It end, Comp comp) {
    IPS2RA_ASSUME_NOT(begin >= end);

    for (It it = begin + 1; it < end; ++it) {
        auto val = std::move(*it);
        if (comp(val, *begin)) {
            std::move_backward(begin, it, it + 1);
            *begin = std::move(val);
        } else {
            auto cur = it;
            for (auto next = it - 1; comp(val, *next); --next) {
                *cur = std::move(*next);
                cur = next;
            }
            *cur = std::move(val);
        }
    }
}

template <std::ptrdiff_t threshold, class It, class Comp>
bool insertionsortIfAtMostThreshold(It begin, It end, Comp&& comp) {
    const bool is_small = end - begin <= threshold;
    if (is_small) {
        insertionSort(std::move(begin), std::move(end), std::forward<Comp>(comp));
    }
    return is_small;
}

template <std::ptrdiff_t kMaxSize, class It, class Comp>
inline bool smallestSortIfAtMostThreshold(It begin, It end, Comp&& comp) {
    return insertionsortIfAtMostThreshold<kMaxSize>(begin, end, std::forward<Comp>(comp));
}

// Size of offs must be at least 2 (kMaxSize + 1).
template <std::ptrdiff_t kMaxSize, class iterator, class T, class Comp>
iterator binPartition(iterator begin, iterator end, std::ptrdiff_t* offs, T splitter,
                      Comp comp) {
    const std::ptrdiff_t size = end - begin;
    assert(size <= kMaxSize);

    std::ptrdiff_t sizes[2] = {0, (kMaxSize + 1) + 1};
    offs[(kMaxSize + 1)] = -1;
    for (std::ptrdiff_t i = 0; i != end - begin; ++i) {
        const auto res = comp(begin[i], splitter);
        offs[sizes[res]] = i;
        ++sizes[res];
    }
    offs[sizes[0]] = size;
    ++sizes[0];

    std::ptrdiff_t l = 0;
    std::ptrdiff_t r = sizes[1] - 1;

    while (offs[r] > offs[l]) {
        std::swap(begin[offs[r]], begin[offs[l]]);
        ++l;
        --r;
    }

    return begin + sizes[1] - 1 - (kMaxSize + 1);
}

// Size of offs must be at least 2 (kMaxSize + 1).
template <std::ptrdiff_t kMaxSize, class iterator, class T, class Comp>
iterator binPartitionSmallerEqual(iterator begin, iterator end, std::ptrdiff_t* offs,
                                  T splitter, Comp comp) {
    const std::ptrdiff_t size = end - begin;
    assert(size <= kMaxSize);

    // std::ptrdiff_t offs[2 * (kMaxSize + 1)];
    std::ptrdiff_t sizes[2] = {1, (kMaxSize + 1)};
    offs[0] = -1;
    for (std::ptrdiff_t i = 0; i != end - begin; ++i) {
        const auto res = comp(splitter, begin[i]);
        offs[sizes[res]] = i;
        ++sizes[res];
    }
    offs[sizes[1]] = size;
    ++sizes[1];

    std::ptrdiff_t l = (kMaxSize + 1);
    std::ptrdiff_t r = sizes[0] - 1;

    while (offs[r] > offs[l]) {
        std::swap(begin[offs[r]], begin[offs[l]]);
        ++l;
        --r;
    }

    return begin + sizes[0] - 1;
}

// Size of offs must be at least 2 (kMaxSize + 1)
template <std::ptrdiff_t kMaxSize, std::ptrdiff_t kSmallestSortMaxSize, class It, class T,
          class Comp>
void quickSortBranchless(const It begin, const It end, std::ptrdiff_t* offs, Comp comp,
                         T& prev_splitter) {
    const auto splitter = begin[(end - begin) / 2];

    // If splitter is not equal to the previous splitter, we just
    // partition. Otherwise we create a bucket with equal elements.
    if (comp(splitter, prev_splitter) || comp(prev_splitter, splitter)) {
        // partitions with <=
        const auto m =
                binPartitionSmallerEqual<kMaxSize>(begin, end, offs, splitter, comp);

        if (!smallestSortIfAtMostThreshold<kSmallestSortMaxSize>(begin, m, comp)) {
            quickSortBranchless<kMaxSize, kSmallestSortMaxSize>(
                    begin, m, offs, std::forward<Comp>(comp), splitter);
        }
        if (!smallestSortIfAtMostThreshold<kSmallestSortMaxSize>(m, end, comp)) {
            quickSortBranchless<kMaxSize, kSmallestSortMaxSize>(
                    m, end, offs, std::forward<Comp>(comp), splitter);
        }

    } else {
        // partitions with <
        const auto m = binPartition<kMaxSize>(begin, end, offs, splitter, comp);

        // No recursion on right partition. This partition only contains
        // elements equal to the splitter.
        if (!smallestSortIfAtMostThreshold<kSmallestSortMaxSize>(begin, m, comp)) {
            quickSortBranchless<kMaxSize, kSmallestSortMaxSize>(
                    begin, m, offs, std::forward<Comp>(comp), splitter);
        }
    }
}

// Size of offs must be at least 2 (kMaxSize + 1)
template <std::ptrdiff_t kMaxSize, std::ptrdiff_t kSmallestSortMaxSize, class It,
          class Comp>
void quickSortBranchless(const It begin, const It end, std::ptrdiff_t* offs, Comp comp) {
    const auto splitter = begin[(end - begin) / 2];

    const auto m = binPartitionSmallerEqual<kMaxSize>(begin, end, offs, splitter, comp);

    if (!smallestSortIfAtMostThreshold<kSmallestSortMaxSize>(begin, m, comp)) {
        quickSortBranchless<kMaxSize, kSmallestSortMaxSize>(
                begin, m, offs, std::forward<Comp>(comp), splitter);
    }
    if (!smallestSortIfAtMostThreshold<kSmallestSortMaxSize>(m, end, comp)) {
        quickSortBranchless<kMaxSize, kSmallestSortMaxSize>(
                m, end, offs, std::forward<Comp>(comp), splitter);
    }
}

/**
 * Wrapper for base case sorter, for easier swapping.
 *
 *  Size of offs must be at least 2 (kMaxSize + 1)
 */
template <std::ptrdiff_t kSmallestSortMaxSize, std::ptrdiff_t kMaxSize, class It,
          class ExtractKey>
inline void baseCaseSort(It begin, It end, std::ptrdiff_t* offs,
                         ExtractKey&& extract_key) {
    auto comp = [&](auto&& l, auto&& r) { return extract_key(l) < extract_key(r); };

    if (!smallestSortIfAtMostThreshold<kSmallestSortMaxSize>(begin, end,
                                                             std::move(comp))) {
        quickSortBranchless<kMaxSize, kSmallestSortMaxSize>(begin, end, offs,
                                                            std::move(comp));
    }
}

}  // namespace detail
}  // namespace ips2ra
