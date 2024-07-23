/******************************************************************************
 * include/ips2ra/sampling.hpp // todo 
 *
 * In-place Parallel Super Scalar Radix Sort (IPS²Ra) // todo
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

// todo rename file

#pragma once

#include <iterator>
#include <random>
#include <utility>

#include "ips2ra_fwd.hpp"
#include "classifier.hpp"
#include "config.hpp"
#include "memory.hpp"
#include "utils.hpp"

namespace ips2ra {
namespace detail {

/**
 * Selects a random sample in-place.
 */
template <class It, class RandomGen>
void selectSample(It begin, const It end,
                  typename std::iterator_traits<It>::difference_type num_samples,
                  RandomGen&& gen) {
    assert(end - begin >= num_samples);
    using std::swap;

    auto n = end - begin;
    while (num_samples--) {
        const auto i = std::uniform_int_distribution<
                typename std::iterator_traits<It>::difference_type>(0, --n)(gen);
        swap(*begin, begin[i]);
        ++begin;
    }
}

template <class Cfg>
std::pair<int, int> Sorter<Cfg>::sampleLevels(const iterator begin, const iterator end) {
    if (begin == end) return {0, 0};

    const auto n = end - begin;
    const auto oversampling = std::max<diff_t>(1, 0.25 * detail::log2(n));
    const auto buckets = std::min<diff_t>(std::max<diff_t>(1, detail::log2(n)), 256);
    const auto num_samples = oversampling * buckets;
    assert(num_samples <= n);

    // Select the sample
    detail::selectSample(begin, end, num_samples, local_.random_generator);

    const typename Cfg::Extractor& extractor = local_.classifier.getExtractor();
    key_type ref = extractor(*begin);
    key_type differing_bits{0};
    for (auto it = begin + 1; it != begin + num_samples; ++it) {
        differing_bits |= ref ^ extractor(*it);
    }

    const int lz = detail::clz(differing_bits);
    const int tz = detail::ctz(differing_bits);

    return {lz / 8, sizeof(key_type) - tz / 8};
}

template <class Cfg>
std::pair<int, int> Sorter<Cfg>::parallelGetLevels(const iterator begin,
                                                   const iterator end,
                                                   std::vector<key_type>& tmp,
                                                   std::vector<bool>& sorted_tmp, int id,
                                                   int num_threads) {
    assert(tmp.size() == num_threads);
    assert(sorted_tmp.size() == num_threads);

    if (begin == end) return {0, 0};

    const auto n = end - begin;
    const auto stripe = (n + num_threads - 1) / num_threads;
    const auto stripe_begin = begin + stripe * id;
    const auto stripe_end = begin + std::min(n, stripe * (id + 1));

    const typename Cfg::Extractor& extractor = local_.classifier.getExtractor();
    key_type differing_bits{0};
    key_type ref = extractor(*begin);

    if (stripe_begin == stripe_end) {
        sorted_tmp[id] = true;

    } else if (extractor(*stripe_begin) <= extractor(*(stripe_end - 1))) {
        // Calculate differing bits.
        // As last element is not smaller than first element,
        // test for sorted input (input is not reverse sorted).

        bool my_sorted = true;
        iterator it = stripe_begin;
        for (; (it + 1) != stripe_end; ++it) {
            differing_bits |= ref ^ extractor(*it);
            my_sorted &= extractor(*it) <= extractor(*(it + 1));
        }
        differing_bits |= ref ^ extractor(*it);

        sorted_tmp[id] = my_sorted;

    } else {
        // Calculate differing bits.
        // Check whether the input is reverse sorted.

        iterator it = stripe_begin;
        for (; it != stripe_end; ++it) { differing_bits |= ref ^ extractor(*it); }

        sorted_tmp[id] = false;
    }

    // for (auto it = stripe_begin; it != stripe_end; ++it) {
    //   differing_bits |= ref ^ extractor(*it);
    // }

    tmp[id] = differing_bits;

    shared_->sync.barrier();

    if (id == 0) {
        const bool all_sorted = std::all_of(sorted_tmp.begin(), sorted_tmp.end(),
                                            [](const bool& a) { return a; });

        if (all_sorted) { return {0, 0}; }

        differing_bits = std::accumulate(
                tmp.begin(), tmp.end(), key_type{0},
                [](const key_type& ka, const key_type& kb) { return ka | kb; });

        const int lz = detail::clz(differing_bits);
        const int tz = detail::ctz(differing_bits);

        return {lz / 8, sizeof(key_type) - tz / 8};

    } else {
        return {0, 0};
    }
}

template <class Cfg>
std::pair<int, int> Sorter<Cfg>::sequentialGetLevels(iterator begin, iterator end) {
    if (begin == end) { return {0, 0}; }

    auto [level_begin, level_end] = sampleLevels(begin, end);

    if (level_begin != 0 || level_end != sizeof(key_type)) {
        const typename Cfg::Extractor& extractor = local_.classifier.getExtractor();

        key_type ref = extractor(*begin);
        key_type differing_bits{0};

        // Calculate differing bits.
        // If last element is not smaller than first element,
        // test for sorted input (input is not reverse sorted).
        if (extractor(*begin) <= extractor(*(end - 1))) {
            bool sorted = true;
            iterator it = begin;
            for (; (it + 1) != end; ++it) {
                differing_bits |= ref ^ extractor(*it);
                sorted &= extractor(*it) <= extractor(*(it + 1));
            }

            if (sorted) { return {0, 0}; }

            differing_bits |= ref ^ extractor(*it);

        } else {
            // Check whether the input is reverse sorted.

            bool reverse_sorted = true;
            iterator it = begin;
            for (; (it + 1) != end; ++it) {
                differing_bits |= ref ^ extractor(*it);
                reverse_sorted &= extractor(*(it + 1)) < extractor(*it);
            }

            if (reverse_sorted) {
                std::reverse(begin, end);
                return {0, 0};
            }

            differing_bits |= ref ^ extractor(*it);
        }

        const int lz = detail::clz(differing_bits);
        const int tz = detail::ctz(differing_bits);

        return {lz / 8, sizeof(key_type) - tz / 8};
    } else {
        return {level_begin, level_end};
    }
}

template <class Cfg>
bool Sorter<Cfg>::nextLevelExists(int level) {
    return level + 1 < level_end_;
}

template <class Cfg>
bool Sorter<Cfg>::levelExists(int level) {
    return level < level_end_;
}

template <class Cfg>
bool Sorter<Cfg>::isLastLevel(int level) {
    assert(levelExists(level));
    return level + 1 == level_end_;
}

}  // namespace detail
}  // namespace ips2ra
