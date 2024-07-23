/******************************************************************************
 * include/ips2ra/bucket_pointers.hpp
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

#include <atomic>
#include <climits>
#include <cstdint>
#include <mutex>
#include <new>
#include <tuple>
#include <utility>

#include "ips2ra_fwd.hpp"

namespace ips2ra {
namespace detail {

template <class Cfg>
class Sorter<Cfg>::BucketPointers {
    using diff_t = typename Cfg::difference_type;

#if UINTPTR_MAX == UINT32_MAX || defined(__SIZEOF_INT128__)

#if UINTPTR_MAX == UINT32_MAX

    using atomic_type = std::uint64_t;

#elif defined(__SIZEOF_INT128__)

    using atomic_type = unsigned __int128;

#endif  // defined(__SIZEOF_INT128__)

    // alignas(std::hardware_destructive_interference_size)
    class Uint128 {
     public:
        inline void set(diff_t l, diff_t m) {
            single_.m_ = m;
            single_.l_ = l;
        }

        inline diff_t getLeastSignificant() const { return single_.l_; }

        template <bool kAtomic>
        inline std::pair<diff_t, diff_t> fetchSubMostSignificant(diff_t m) {
            if (kAtomic) {
                const atomic_type atom_m = static_cast<atomic_type>(m) << kShift;
                const auto p = __atomic_fetch_sub(&all_, atom_m, __ATOMIC_RELAXED);
                return {p & kMask, p >> kShift};
            } else {
                const auto tmp = single_.m_;
                single_.m_ -= m;
                return {single_.l_, tmp};
            }
        }

        template <bool kAtomic>
        inline std::pair<diff_t, diff_t> fetchAddLeastSignificant(diff_t l) {
            if (kAtomic) {
                const auto p = __atomic_fetch_add(&all_, l, __ATOMIC_RELAXED);
                return {p & kMask, p >> kShift};
            } else {
                const auto tmp = single_.l_;
                single_.l_ += l;
                return {tmp, single_.m_};
            }
        }

     private:
        static constexpr const int kShift = sizeof(atomic_type) * CHAR_BIT / 2;
        static constexpr const atomic_type kMask =
                (static_cast<atomic_type>(1) << kShift) - 1;

        struct Pointers {
            diff_t l_, m_;
        };
        union {
            atomic_type all_;
            Pointers single_;
        };
    };

#else

    class Uint128 {
     public:
        inline void set(diff_t l, diff_t m) {
            m_ = m;
            l_ = l;
        }

        inline diff_t getLeastSignificant() const { return l_; }

        template <bool kAtomic>
        inline std::pair<diff_t, diff_t> fetchSubMostSignificant(diff_t m) {
            if (kAtomic) {
                std::lock_guard<std::mutex> lock(mtx_);
                std::pair<diff_t, diff_t> p{l_, m_};
                m_ -= m;
                return p;
            } else {
                const auto tmp = m_;
                m_ -= m;
                return {l_, tmp};
            }
        }

        template <bool kAtomic>
        inline std::pair<diff_t, diff_t> fetchAddLeastSignificant(diff_t l) {
            if (kAtomic) {
                std::lock_guard<std::mutex> lock(mtx_);
                std::pair<diff_t, diff_t> p{l_, m_};
                l_ += l;
                return p;
            } else {
                const auto tmp = l_;
                l_ += l;
                return {tmp, m_};
            }
        }

     private:
        diff_t m_, l_;
        std::mutex mtx_;
    };

#endif

 public:
    /**
     * Sets write/read pointers.
     */
    void set(diff_t w, diff_t r) {
        ptr_.set(w, r);
        num_reading_.store(0, std::memory_order_relaxed);
    }

    /**
     * Gets the write pointer.
     */
    diff_t getWrite() const {
        return ptr_.getLeastSignificant();
    }

    /**
     * Gets write/read pointers and increases the write pointer.
     */
    template <bool kAtomic>
    std::pair<diff_t, diff_t> incWrite() {
        return ptr_.template fetchAddLeastSignificant<kAtomic>(Cfg::kBlockSize);
    }

    /**
     * Gets write/read pointers, decreases the read pointer, and increases the read
     * counter.
     */
    template <bool kAtomic>
    std::pair<diff_t, diff_t> decRead() {
        if (kAtomic) {
            // Must not be moved after the following fetch_sub, as that could lead to
            // another thread writing to our block, because isReading() returns false.
            num_reading_.fetch_add(1, std::memory_order_acquire);
            const auto p =
                    ptr_.template fetchSubMostSignificant<kAtomic>(Cfg::kBlockSize);
            return {p.first, p.second & ~(Cfg::kBlockSize - 1)};
        } else {
            return ptr_.template fetchSubMostSignificant<kAtomic>(Cfg::kBlockSize);
        }
    }

    /**
     * Decreases the read counter.
     */
    void stopRead() {
        // Synchronizes with threads wanting to write to this bucket
        num_reading_.fetch_sub(1, std::memory_order_release);
    }

    /**
     * Returns true if any thread is currently reading from here.
     */
    bool isReading() {
        // Synchronizes with threads currently reading from this bucket
        return num_reading_.load(std::memory_order_acquire) != 0;
    }

 private:
    Uint128 ptr_;
    std::atomic_int num_reading_;
};

}  // namespace detail
}  // namespace ips2ra
