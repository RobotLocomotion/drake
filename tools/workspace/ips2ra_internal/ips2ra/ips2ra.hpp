/******************************************************************************
 * include/ips2ra/ips2ra.hpp
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

#include <functional>
#include <iterator>
#include <type_traits>

#include "ips2ra_fwd.hpp"
#include "base_case.hpp"
#include "config.hpp"
#include "memory.hpp"
#include "parallel.hpp"
#include "sequential.hpp"

namespace ips2ra {

/**
 * Helper function for creating a reusable sequential sorter.
 */
template <class It, class Extractor = Config<>::identity, class Cfg = Config<>>
SequentialSorter<ExtendedConfig<It, Extractor, Cfg>> make_sorter(
        Extractor extractor = Extractor{}) {
    return SequentialSorter<ExtendedConfig<It, Extractor, Cfg>>{std::move(extractor)};
}

/**
 * Configurable interface.
 */
template <class Cfg, class It, class Extractor = Config<>::identity>
void sort(It begin, It end, Extractor extractor = Extractor{}) {
#ifdef IPS4O_TIMER
    g_active_counters = -1;
    g_total.start();
    g_overhead.start();
#endif

    static constexpr std::ptrdiff_t threshold =
            Cfg::kSmallestSortMultiplier * Cfg::kSmallestSortSize;
    if (end - begin <= threshold) {
#ifdef IPS4O_TIMER
        g_overhead.stop();
        g_base_case.start();
#endif

        std::ptrdiff_t offs[2 * (threshold + 1)];
        detail::baseCaseSort<Cfg::kSmallestSortSize, threshold>(begin, end, offs,
                                                                extractor);

#ifdef IPS4O_TIMER
        g_base_case.stop();
        g_overhead.start();
#endif
    } else {
        ips2ra::make_sorter<It, Extractor, Cfg>(std::move(extractor))(std::move(begin),
                                                                      std::move(end));
    }

#ifdef IPS4O_TIMER
    g_overhead.stop();
    g_total.stop();
#endif
}

/**
 * Standard interface.
 */
template <class It, class Extractor>
void sort(It begin, It end, Extractor extractor) {
    ips2ra::sort<Config<>>(std::move(begin), std::move(end), std::move(extractor));
}

template <class It>
void sort(It begin, It end) {
    ips2ra::sort<Config<>>(std::move(begin), std::move(end), Config<>::identity{});
}

#if defined(_REENTRANT) || defined(_OPENMP)
namespace parallel {

/**
 * Helper functions for creating a reusable parallel sorter.
 */
template <class It, class Cfg = Config<>, class ThreadPool,
          class Extractor = Config<>::identity>
std::enable_if_t<std::is_class<std::remove_reference_t<ThreadPool>>::value,
                 ParallelSorter<ExtendedConfig<It, Extractor, Cfg, ThreadPool>>>
make_sorter(ThreadPool&& thread_pool, Extractor extractor = Extractor{}) {
    return ParallelSorter<ExtendedConfig<It, Extractor, Cfg, ThreadPool>>(
            std::move(extractor), std::forward<ThreadPool>(thread_pool));
}

template <class It, class Cfg = Config<>, class Extractor = Config<>::identity>
ParallelSorter<ExtendedConfig<It, Extractor, Cfg>> make_sorter(
        int num_threads = DefaultThreadPool::maxNumThreads(),
        Extractor extractor = Extractor{}) {
    return ParallelSorter<ExtendedConfig<It, Extractor, Cfg>>(
            std::move(extractor), DefaultThreadPool(num_threads));
}

/**
 * Configurable interface.
 */
template <class Cfg = Config<>, class It, class Extractor, class ThreadPool>
std::enable_if_t<std::is_class<std::remove_reference_t<ThreadPool>>::value> sort(
        It begin, It end, Extractor extractor, ThreadPool&& thread_pool) {
#ifdef IPS4O_TIMER
    g_active_counters = -1;
    g_total.start();
    g_overhead.start();
#endif

    if (Cfg::numThreadsFor(begin, end, thread_pool.numThreads()) < 2) {
        ips2ra::sort<Cfg>(std::move(begin), std::move(end), std::move(extractor));
    } else {
        ips2ra::parallel::make_sorter<It, Cfg>(std::forward<ThreadPool>(thread_pool),
                                               std::move(extractor))(std::move(begin),
                                                                     std::move(end));
    }

#ifdef IPS4O_TIMER
    g_overhead.stop();
    g_total.stop();
#endif
}

template <class Cfg = Config<>, class It, class Extractor>
void sort(It begin, It end, Extractor extractor, int num_threads) {
#ifdef IPS4O_TIMER
    g_active_counters = -1;
    g_total.start();
    g_overhead.start();
#endif

    num_threads = Cfg::numThreadsFor(begin, end, num_threads);
    if (num_threads < 2) {
        ips2ra::sort<Cfg>(std::move(begin), std::move(end), std::move(extractor));
    } else {
        ips2ra::parallel::make_sorter<It, Cfg>(num_threads, std::move(extractor))(
                std::move(begin), std::move(end));
    }

#ifdef IPS4O_TIMER
    g_overhead.stop();
    g_total.stop();
#endif
}

/**
 * Standard interface.
 */
template <class It, class Extractor>
void sort(It begin, It end, Extractor extractor) {
    ips2ra::parallel::sort<Config<>>(std::move(begin), std::move(end),
                                     std::move(extractor),
                                     DefaultThreadPool::maxNumThreads());
}
template <class It>
void sort(It begin, It end) {
    ips2ra::parallel::sort<Config<>>(std::move(begin), std::move(end),
                                     Config<>::identity{},
                                     DefaultThreadPool::maxNumThreads());
}

}  // namespace parallel
#endif  // _REENTRANT || _OPENMP
}  // namespace ips2ra
