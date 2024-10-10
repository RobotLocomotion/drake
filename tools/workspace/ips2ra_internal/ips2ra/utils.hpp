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

#include <cassert>
#include <climits>
#include <limits>

#define IPS2RA_ASSUME_NOT(c) if (c) __builtin_unreachable()
#define IPS2RA_IS_NOT(c) assert(!(c))

namespace ips2ra {
namespace detail {

inline constexpr unsigned clz(unsigned int n) {
    if (n == 0) return CHAR_BIT * sizeof(n);
    return __builtin_clz(n);
}

inline constexpr unsigned clz(unsigned long n) {
    if (n == 0) return CHAR_BIT * sizeof(n);
    return __builtin_clzl(n);
}

inline constexpr unsigned clz(unsigned long long n) {
    if (n == 0) return CHAR_BIT * sizeof(n);
    return __builtin_clzll(n);
}

inline constexpr unsigned ctz(unsigned int n) {
    if (n == 0) return CHAR_BIT * sizeof(n);
    return __builtin_ctz(n);
}

inline constexpr unsigned ctz(unsigned long n) {
    if (n == 0) return CHAR_BIT * sizeof(n);
    return __builtin_ctzl(n);
}

inline constexpr unsigned ctz(unsigned long long n) {
    if (n == 0) return CHAR_BIT * sizeof(n);
    return __builtin_ctzll(n);
}

/**
 * Compute the logarithm to base 2, rounded down.
 */
inline constexpr unsigned long log2(unsigned long n) {
    return (std::numeric_limits<unsigned long>::digits - 1 - clz(n));
}

template <int tmpl_idx, int last, class E, typename... Ts>
void switchUnroll(E& e, int idx, Ts... args) {
    assert(idx <= last);
    if constexpr (tmpl_idx > last)
        return;
    else if constexpr (tmpl_idx < 0)
        return;
    else if (idx == tmpl_idx) {
        e.template operator()<tmpl_idx>(args...);
    } else {
        switchUnroll<tmpl_idx + 1, last>(e, idx, args...);
    }
}

}  // namespace detail
}  // namespace ips2ra
