/******************************************************************************
 * include/ips2ra/config.hpp
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
#include <cstddef>
#include <cstdint>
#include <iterator>
#include <type_traits>
#include <utility>

#if defined(_REENTRANT)
#include "thread_pool.hpp"
#endif
#include "utils.hpp"

#ifndef IPS2RA_SMALLEST_CASE_SIZE
#define IPS2RA_SMALLEST_CASE_SIZE 32
#endif

#ifndef IPS2RA_BASE_CASE_SIZE
#define IPS2RA_BASE_CASE_SIZE 128
#endif

#ifndef IPS2RA_AMERICAN_FLAG_SORT_SIZE
#define IPS2RA_AMERICAN_FLAG_SORT_SIZE 1024
#endif

#ifndef IPS2RA_SKA_SORT_SIZE
#define IPS2RA_SKA_SORT_SIZE 2048
#endif

#ifndef IPS2RA_SMALLEST_CASE_MULTIPLIER
#define IPS2RA_SMALLEST_CASE_MULTIPLIER 8
#endif

#ifndef IPS2RA_BLOCK_SIZE
#define IPS2RA_BLOCK_SIZE (2 << 10)
#endif

#ifndef IPS2RA_BUCKET_TYPE
#define IPS2RA_BUCKET_TYPE std::ptrdiff_t
#endif

#ifndef IPS2RA_DATA_ALIGNMENT
#define IPS2RA_DATA_ALIGNMENT (4 << 10)
#endif

#ifndef IPS2RA_LOG_BUCKETS
#define IPS2RA_LOG_BUCKETS 8
#endif

#ifndef IPS2RA_MIN_PARALLEL_BLOCKS_PER_THREAD
#define IPS2RA_MIN_PARALLEL_BLOCKS_PER_THREAD 4
#endif

#ifndef IPS2RA_OVERSAMPLING_FACTOR_PERCENT
#define IPS2RA_OVERSAMPLING_FACTOR_PERCENT 20
#endif

#ifndef IPS2RA_UNROLL_CLASSIFIER
#define IPS2RA_UNROLL_CLASSIFIER 7
#endif

namespace ips2ra {

template <std::ptrdiff_t SmallestSort_     = IPS2RA_SMALLEST_CASE_SIZE
        , std::ptrdiff_t BaseCaseSize_     = IPS2RA_BASE_CASE_SIZE
        , std::ptrdiff_t Skasort_          = IPS2RA_SKA_SORT_SIZE
        , std::ptrdiff_t AmericanFlagSort_ = IPS2RA_AMERICAN_FLAG_SORT_SIZE
        , std::ptrdiff_t SmallestSortM_    = IPS2RA_SMALLEST_CASE_MULTIPLIER
        , std::ptrdiff_t BlockSize_        = IPS2RA_BLOCK_SIZE
        , class BucketT_                   = IPS2RA_BUCKET_TYPE
        , std::size_t DataAlign_           = IPS2RA_DATA_ALIGNMENT
        , int LogBuckets_                  = IPS2RA_LOG_BUCKETS
        , std::ptrdiff_t MinParBlks_       = IPS2RA_MIN_PARALLEL_BLOCKS_PER_THREAD
        , int OversampleF_                 = IPS2RA_OVERSAMPLING_FACTOR_PERCENT
        , int UnrollClass_                 = IPS2RA_UNROLL_CLASSIFIER
        >
struct Config {
    /**
     * The type used for bucket indices in the classifier.
     */
    using bucket_type = BucketT_;

    /**
     * Whether we are on 64 bit or 32 bit.
     */
    static constexpr const bool kIs64Bit = sizeof(std::uintptr_t) == 8;
    static_assert(kIs64Bit || sizeof(std::uintptr_t) == 4,
                  "Architecture must be 32 or 64 bit");

    /**
     * Smallest case size (Bitonic Sort or Insertionsort).
     */
    static constexpr const std::ptrdiff_t kSmallestSortSize = SmallestSort_;
    /**
     * Base case size (Quicksort).
     */
    static constexpr const std::ptrdiff_t kBaseCaseSize = BaseCaseSize_;
    /**
     * Desired Skasort size.
     */
    static constexpr const std::ptrdiff_t kSkasort = Skasort_;
    /**
     * Desired AmericanFlagSort size.
     */
    static constexpr const std::ptrdiff_t kAmericanFlagSort = AmericanFlagSort_;
    /**
     * Multiplier for base case threshold.
     */
    static constexpr const int kSmallestSortMultiplier = SmallestSortM_;
    /**
     * Number of bytes in one block.
     */
    static constexpr const std::ptrdiff_t kBlockSizeInBytes = BlockSize_;
    /**
     * Alignment for shared and thread-local data.
     */
    static constexpr const std::size_t kDataAlignment = DataAlign_;
    /**
     * Logarithm of the maximum number of buckets (excluding equality buckets).
     */
    static constexpr const int kLogBuckets = LogBuckets_;
    /**
     * Minimum number of blocks per thread for which parallelism is used.
     */
    static constexpr const std::ptrdiff_t kMinParallelBlocksPerThread = MinParBlks_;
    static_assert(kMinParallelBlocksPerThread > 0,
                  "Min. blocks per thread must be at least 1.");
    /**
     * How many times the classification loop is unrolled.
     */
    static constexpr const int kUnrollClassifier = UnrollClass_;

    /**
     * The oversampling factor to be used for input of size n.
     */
    static constexpr double oversamplingFactor(std::ptrdiff_t n) {
        const double f = OversampleF_ / 100.0 * detail::log2(n);
        return f < 1.0 ? 1.0 : f;
    }

    struct identity {
        template <class T>
        constexpr T&& operator()(T&& t) const {
            return std::forward<T>(t);
        }
    };

    template <class Extractor>
    struct LessExtractor {
        LessExtractor(Extractor extractor) : extractor_(extractor) {}

        template <class T1, class T2>
        bool operator()(T1&& x, T2&& y) const {
            return extractor_(std::forward<T1>(x)) < extractor_(std::forward<T2>(y));
        }

     private:
        Extractor extractor_;
    };

    /**
     * Returns the number of threads that should be used for the given input range.
     */
    template <class It>
#if defined(_REENTRANT) || defined(_OPENMP)
    static constexpr int numThreadsFor(const It& begin, const It& end, int max_threads) {
        const std::ptrdiff_t blocks =
                (end - begin) * sizeof(decltype(*begin)) / kBlockSizeInBytes;
        return (blocks < (kMinParallelBlocksPerThread * max_threads)) ? 1 : max_threads;
    }
#else
    static constexpr int numThreadsFor(const It&, const It&, int) {
        return 1;
    }
#endif
};

template <class It_, class Extractor_, class Cfg = Config<>
#if defined(_REENTRANT) || defined(_OPENMP)
          , class ThreadPool_ = DefaultThreadPool
#endif
         >
struct ExtendedConfig : public Cfg {
    /**
     * Base config containing user-specified parameters.
     */
    using BaseConfig = Cfg;
    /**
     * The iterator type for the input data.
     */
    using iterator = It_;

    /**
     * Extracts the key from the element.
     */
    using Extractor = Extractor_;

    /**
     * The difference type for the iterator.
     */
    using difference_type = typename std::iterator_traits<iterator>::difference_type;
    /**
     * The value type of the input data.
     */
    using value_type = typename std::iterator_traits<iterator>::value_type;
    /**
     * The key
     */
    using key_type = std::remove_reference_t<std::invoke_result_t<Extractor, value_type>>;

    static_assert(std::is_unsigned<key_type>::value);

#if defined(_REENTRANT) || defined(_OPENMP)
    /**
     * Thread pool for parallel algorithm.
     */
    using ThreadPool = ThreadPool_;

    using SubThreadPool = ThreadJoiningThreadPool;

    /**
     * Synchronization support for parallel algorithm.
     */
    using Sync = decltype(std::declval<ThreadPool&>().sync());

#else

    struct Sync {
        constexpr void barrier() const {}
        template <class F>
        constexpr void single(F&&) const {}
    };

    /**
     * Dummy thread pool.
     */
    class SubThreadPool {
     public:
        explicit SubThreadPool(int) {}

        void join(int) {}

        void release_threads() {}

        template <class F>
        void operator()(F&&, int) {}

        Sync& sync() { return sync_; }

        int numThreads() const { return 1; }

     private:
        Sync sync_;
    };
#endif

    template <int level, class T>
    static inline T getBucket(const T val) {
        static_assert(sizeof(T) > level, "Too many levels");
        return (val >> (8 * (sizeof(T) - level - 1))) & 0xFF;
    }

    template <class T>
    static inline T getBucket(const T val, int level) {
        assert(sizeof(T) > level);
        return (val >> (8 * (sizeof(T) - level - 1))) & 0xFF;
    }

    /**
     * Maximum number of buckets (including equality buckets).
     */
  static constexpr const int kMaxBuckets = 1ul << Cfg::kLogBuckets;

    /**
     * Number of elements in one block.
     */
    static constexpr const difference_type kBlockSize =
            1ul << (detail::log2(
                    Cfg::kBlockSizeInBytes < sizeof(value_type)
                            ? 1
                            : (Cfg::kBlockSizeInBytes / sizeof(value_type))));

    // Redefine applicable constants as difference_type.
    static constexpr const difference_type kSmallestSortSize = Cfg::kSmallestSortSize;

    // Cannot sort without random access.
    static_assert(std::is_same<typename std::iterator_traits<iterator>::iterator_category,
                  std::random_access_iterator_tag>::value, "Iterator must be a random access iterator.");
    // The implementation of the block alignment limits the possible block sizes.
    static_assert((kBlockSize & (kBlockSize - 1)) == 0, "Block size must be a power of two.");
    // The main classifier function assumes that the loop can be unrolled at least once.
    static_assert(Cfg::kUnrollClassifier <= kSmallestSortSize, "Base case size must be larger than unroll factor.");

    /**
     * Aligns an offset to the next block boundary, upwards.
     */
    static constexpr difference_type alignToNextBlock(difference_type p) {
        return (p + kBlockSize - 1) & ~(kBlockSize - 1);
    }

};

#undef IPS2RA_SMALLEST_CASE_SIZE
#undef IPS2RA_SKA_SORT_SIZE
#undef IPS2RA_AMERICAN_FLAG_SORT_SIZE
#undef IPS2RA_SMALLEST_CASE_MULTIPLIER
#undef IPS2RA_BLOCK_SIZE
#undef IPS2RA_BUCKET_TYPE
#undef IPS2RA_DATA_ALIGNMENT
#undef IPS2RA_LOG_BUCKETS
#undef IPS2RA_MIN_PARALLEL_BLOCKS_PER_THREAD
#undef IPS2RA_OVERSAMPLING_FACTOR_PERCENT
#undef IPS2RA_UNROLL_CLASSIFIER

}  // namespace ips2ra
