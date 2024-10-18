/******************************************************************************
 * include/ips2ra/buffers.hpp
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
#include <type_traits>
#include <utility>

#include "ips2ra_fwd.hpp"

namespace ips2ra {
namespace detail {

/**
 * A single buffer block.
 */
template <class Cfg>
class Sorter<Cfg>::Block {
    using iterator = typename Cfg::iterator;
    using diff_t = typename Cfg::difference_type;
    using value_type = typename Cfg::value_type;

 public:
    static constexpr const bool kInitializedStorage =
            std::is_trivially_default_constructible<value_type>::value;
    static constexpr const bool kDestruct =
            !kInitializedStorage && !std::is_trivially_destructible<value_type>::value;

    /**
     * Pointer to data.
     */
    value_type* data() { return static_cast<value_type*>(static_cast<void*>(storage_)); }

    /**
     * First element.
     */
    const value_type& head() { return *data(); }

    /**
     * Reads a full block from input.
     */
    void readFrom(iterator src) {
        if (kInitializedStorage) {
            std::move(src, src + Cfg::kBlockSize, data());
        } else {
            for (auto p = data(), end = p + Cfg::kBlockSize; p < end; ++p) {
                IPS2RA_ASSUME_NOT(p == nullptr);
                new (p) value_type(std::move(*src++));
            }
        }
    }

    /**
     * Reads a partial block from input.
     */
    void readFrom(iterator src, const diff_t n) {
        if (kInitializedStorage) {
            std::move(src, src + n, data());
        } else {
            for (auto p = data(), end = p + n; p < end; ++p) {
                IPS2RA_ASSUME_NOT(p == nullptr);
                new (p) value_type(std::move(*src++));
            }
        }
    }

    /**
     * Resets a partial block.
     */
    void reset(const diff_t n) {
        if (kDestruct)
            for (auto p = data(), end = p + n; p < end; ++p)
                p->~value_type();
    }

    /**
     * Writes a full block to other block.
     */
    void writeTo(Block& block) {
        if (kInitializedStorage) {
            std::move(data(), data() + Cfg::kBlockSize, block.data());
        } else {
            for (auto src = data(), dst = block.data(), end = src + Cfg::kBlockSize;
                 src < end; ++src, ++dst) {
                IPS2RA_ASSUME_NOT(dst == nullptr);
                new (dst) value_type(std::move(*src));
            }
        }
        if (kDestruct)
            for (auto p = data(), end = p + Cfg::kBlockSize; p < end; ++p)
                p->~value_type();
    }

    /**
     * Writes a full block to input.
     */
    void writeTo(iterator dest) {
        std::move(data(), data() + Cfg::kBlockSize, std::move(dest));
        if (kDestruct)
            for (auto p = data(), end = p + Cfg::kBlockSize; p < end; ++p)
                p->~value_type();
    }

 private:
    using storage_type = std::conditional_t<
            kInitializedStorage, value_type,
            std::aligned_storage_t<sizeof(value_type), alignof(value_type)>>;
    storage_type storage_[Cfg::kBlockSize];
};

/**
 * Per-thread buffers for each bucket.
 */
template <class Cfg>
class Sorter<Cfg>::Buffers {
    using diff_t = typename Cfg::difference_type;
    using value_type = typename Cfg::value_type;

 public:
    Buffers(char* storage) : storage_(static_cast<Block*>(static_cast<void*>(storage))) {
        for (diff_t i = 0; i < Cfg::kMaxBuckets; ++i) {
            resetBuffer(i);
            buffer_[i].end = buffer_[i].ptr + Cfg::kBlockSize;
        }
    }

    /**
     * Checks if buffer is full.
     */
    bool isFull(const int i) const { return buffer_[i].ptr == buffer_[i].end; }

    /**
     * Pointer to buffer data.
     */
    value_type* data(const int i) {
        return static_cast<value_type*>(static_cast<void*>(storage_))
               + i * Cfg::kBlockSize;
    }

    /**
     * Number of elements in buffer.
     */
    diff_t size(const int i) const {
        return Cfg::kBlockSize - (buffer_[i].end - buffer_[i].ptr);
    }

    /**
     * Resets buffer.
     */
    void reset(const int i) {
        if (Block::kDestruct)
            for (auto p = data(i), end = p + size(i); p < end; ++p)
                p->~value_type();
        resetBuffer(i);
    }

    /**
     * Pushes new element to buffer.
     */
    void push(const int i, value_type&& value) {
        if (Block::kInitializedStorage) {
            *buffer_[i].ptr++ = std::move(value);
        } else {
            IPS2RA_ASSUME_NOT(buffer_[i].ptr == nullptr);
            new (buffer_[i].ptr++) value_type(std::move(value));
        }
    }

    /**
     * Flushes buffer to input.
     */
    void writeTo(const int i, typename Cfg::iterator dest) {
        resetBuffer(i);
        auto ptr = buffer_[i].ptr;
        std::move(ptr, ptr + Cfg::kBlockSize, std::move(dest));

        if (Block::kDestruct)
            for (const auto end = buffer_[i].end; ptr < end; ++ptr)
                ptr->~value_type();
    }

 private:
    struct Info {
        value_type* ptr;
        const value_type* end;
    };

    void resetBuffer(const int i) {
        buffer_[i].ptr = static_cast<value_type*>(static_cast<void*>(storage_))
                         + i * Cfg::kBlockSize;
    }

    Info buffer_[Cfg::kMaxBuckets];
    Block* storage_;
    // Blocks should have no extra elements or padding
    static_assert(sizeof(Block) == sizeof(typename Cfg::value_type) * Cfg::kBlockSize,
                  "Block size mismatch.");
    static_assert(std::is_trivially_default_constructible<Block>::value,
                  "Block must be trivially default constructible.");
    static_assert(std::is_trivially_destructible<Block>::value,
                  "Block must be trivially destructible.");
};

}  // namespace detail
}  // namespace ips2ra
