/******************************************************************************
 * include/ips2ra/utils.hpp
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
#include <atomic>
#include <cassert>
#include <cstddef>
#include <numeric>
#include <vector>

#include <tbb/concurrent_queue.h>

namespace ips2ra {
namespace detail {

template <class T>
class PrivateQueue {
 public:
    PrivateQueue(size_t init_size = ((1ul << 12) + sizeof(T) - 1) / sizeof(T))
        : m_v(), m_off(0) {
        // Preallocate memory. By default, the vector covers at least one page.
        m_v.reserve(init_size);
    }

    template <class Iterator>
    void push(Iterator begin, Iterator end) {
        m_v.insert(m_v.end(), begin, end);
    }

    template <class T1>
    void push(const T1&& e) {
        m_v.emplace_back(std::forward(e));
    }

    template <typename... Args>
    void emplace(Args... args) {
        m_v.emplace_back(args...);
    }

    size_t size() const { return m_v.size() - m_off; }

    bool empty() const { return size() == 0; }

    T popBack() {
        assert(m_v.size() > m_off);

        const T e = m_v.back();
        m_v.pop_back();

        if (m_v.size() == m_off) { clear(); }

        return e;
    }

    T popFront() {
        assert(m_v.size() > m_off);

        const T e = m_v[m_off];
        ++m_off;

        if (m_v.size() == m_off) { clear(); }

        return e;
    }

    void clear() {
        m_off = 0;
        m_v.clear();
    }

 protected:
    std::vector<T> m_v;
    size_t m_off;
};

template <class Job>
class Scheduler {
 public:
    Scheduler(size_t num_threads) : m_num_idle_threads(0), m_num_threads(num_threads) {}

    bool getJob(PrivateQueue<Job>& my_queue, Job& j) {
        // Try to get local job.
        if (!my_queue.empty()) {
            j = my_queue.popBack();
            return true;
        }

        // Try to get global job.
        const bool succ = m_glob_queue.try_pop(j);
        if (succ) return succ;

        // Signal idle.
        m_num_idle_threads.fetch_add(1, std::memory_order_relaxed);

        while (m_num_idle_threads.load(std::memory_order_relaxed) != m_num_threads) {
            if (!m_glob_queue.empty()) {
                m_num_idle_threads.fetch_sub(1, std::memory_order_relaxed);

                const bool succ = m_glob_queue.try_pop(j);
                if (succ) { return succ; }

                m_num_idle_threads.fetch_add(1, std::memory_order_relaxed);
            }
        }

        return false;
    }

    void offerJob(PrivateQueue<Job>& my_queue) {
        if (my_queue.size() > 1 && m_num_idle_threads.load(std::memory_order_relaxed) > 0
            && m_glob_queue.empty()) {
            addJob(my_queue.popFront());
        }
    }

    void addJob(const Job& j) { m_glob_queue.push(j); }

    void addJob(const Job&& j) { m_glob_queue.push(j); }

    void reset() { m_num_idle_threads.store(0, std::memory_order_relaxed); }

 protected:
    tbb::concurrent_queue<Job> m_glob_queue;
    std::atomic_uint64_t m_num_idle_threads;
    const size_t m_num_threads;
};

}  // namespace detail
}  // namespace ips2ra
