/******************************************************************************
 * include/ips2ra/ips2ra_fwd.hpp
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

#include <iterator>
#include <memory>
#include <utility>
#include <vector>

#include "config.hpp"
#include "scheduler.hpp"
#include "task.hpp"

namespace ips2ra {

template <class Cfg>
class SequentialSorter;

namespace detail {

template <class It, class Extractor>
inline void baseCaseSort(It begin, It end, Extractor&& extractor);

inline constexpr unsigned long log2(unsigned long n);

template <class It, class RandomGen>
inline void selectSample(It begin, It end,
                         typename std::iterator_traits<It>::difference_type num_samples,
                         RandomGen&& gen);

template <class Cfg>
class Sorter {
 public:
    using iterator = typename Cfg::iterator;
    using diff_t = typename Cfg::difference_type;
    using value_type = typename Cfg::value_type;
    using SubThreadPool = typename Cfg::SubThreadPool;
    using key_type = typename Cfg::key_type;
    using LessExtractor =
            typename Cfg::BaseConfig::template LessExtractor<typename Cfg::Extractor>;

    class BufferStorage;
    class Block;
    class Buffers;
    class BucketPointers;
    class Classifier;
    struct LocalData;
    struct SharedData;
    explicit Sorter(LocalData& local) : local_(local), level_end_(-1) {}

    void sequential(const iterator begin, const Task& task, PrivateQueue<Task>& queue);
    void sequentialRec(iterator begin, iterator end, const int level);
    void sequential(iterator begin, iterator end);

    std::pair<int, int> sampleLevels(iterator begin, iterator end);
    std::pair<int, int> parallelGetLevels(iterator begin, iterator end,
                                          std::vector<key_type>& tmp,
                                          std::vector<bool>& sorted_tmp, int id,
                                          int num_threads);
    std::pair<int, int> sequentialGetLevels(iterator begin, iterator end);

    bool nextLevelExists(int level);
    bool levelExists(int level);
    bool isLastLevel(int level);

    // todo add sort parallel flag?
#if defined(_REENTRANT) || defined(_OPENMP)
    void parallelSortPrimary(iterator begin, iterator end, int num_threads,
                             BufferStorage& buffer_storage,
                             std::vector<std::shared_ptr<SubThreadPool>>& tp_trash,
                             int level_begin, int level_end);

    void parallelSortSecondary(iterator begin, iterator end, int id, int num_threads,
                               BufferStorage& buffer_storage,
                               std::vector<std::shared_ptr<SubThreadPool>>& tp_trash,
                               int level_begin, int level_end);

    // todo change return type.
    std::vector<diff_t> parallelPartitionPrimary(iterator begin, iterator end, int level,
                                                 int level_end, int num_threads);

    void parallelPartitionSecondary(iterator begin, iterator end, int level,
                                    int level_end, int id, int num_threads);

    void setShared(SharedData* shared_);
#endif

 private:
    LocalData& local_;
    SharedData* shared_;
    Classifier* classifier_;

    diff_t* bucket_start_;
    BucketPointers* bucket_pointers_;
    Block* overflow_;

    iterator begin_;
    iterator end_;
    int my_id_;
    int num_threads_;
    int level_;
    int level_end_;

    static inline int computeLogBuckets(diff_t n);

    int buildClassifier(iterator begin, iterator end, Classifier& classifier,
                        double sample_multiplicator);

    __attribute__((flatten))
    diff_t classifyLocally(iterator my_begin, iterator my_end);

    inline void parallelClassification();

    inline void sequentialClassification();

    void moveEmptyBlocks(diff_t my_begin, diff_t my_end, diff_t my_first_empty_block);

    inline int computeOverflowBucket();

    template <bool kIsParallel>
    inline int classifyAndReadBlock(int read_bucket);

    template <bool kIsParallel>
    inline int swapBlock(diff_t max_off, int dest_bucket, bool current_swap);

    template <bool kIsParallel>
    void permuteBlocks();

    inline std::pair<int, diff_t> saveMargins(int last_bucket);

    template <bool kIsParallel>
    void writeMargins(int first_bucket, int last_bucket, int overflow_bucket,
                      int swap_bucket, diff_t in_swap_buffer, int level);

    template <bool kIsParallel>
    void partition(iterator begin, iterator end, int level, diff_t* bucket_start,
                   int my_id, int num_threads);

    void processSmallTasks(iterator begin, int num_threads);

    void processBigTasks(const iterator begin, const diff_t stripe, const int my_id,
                         BufferStorage& buffer_storage,
                         std::vector<std::shared_ptr<SubThreadPool>>& tp_trash);

    void processBigTaskPrimary(const iterator end, const diff_t stripe, const int my_id,
                               BufferStorage& buffer_storage,
                               std::vector<std::shared_ptr<SubThreadPool>>& tp_trash);

    void processBigTasksSecondary(const int my_id, BufferStorage& buffer_storage);

    void queueTasks(const int level, const diff_t stripe, const int id,
                    const int num_threads, const diff_t parent_task_size,
                    const diff_t offset, const diff_t* bucket_start);
};

}  // namespace detail

template <class Cfg>
class ParallelSorter;

template <class It, class Extractor>
inline void sort(It begin, It end, Extractor extractor);

template <class It>
inline void sort(It begin, It end);

#if defined(_REENTRANT)
namespace parallel {

template <class It, class Extractor>
inline void sort(It begin, It end, Extractor extractor);

template <class It>
inline void sort(It begin, It end);

}  // namespace parallel
#endif  // _REENTRANT
}  // namespace ips2ra
