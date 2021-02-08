#pragma once

// This file is a modification of Eigen-3.3.3's AutoDiffScalar.h file which is
// available at
// https://gitlab.com/libeigen/eigen/-/blob/3.3.3/unsupported/Eigen/src/AutoDiff/AutoDiffScalar.h
//
// Copyright (C) 2009 Gael Guennebaud <gael.guennebaud@inria.fr>
// Copyright (C) 2017 Drake Authors
//
// This Source Code Form is subject to the terms of the Mozilla
// Public License v. 2.0. If a copy of the MPL was not distributed
// with this file, You can obtain one at http://mozilla.org/MPL/2.0/.

#ifndef DRAKE_COMMON_AUTODIFF_HEADER
// TODO(soonho-tri): Change to #error.
#warning Do not directly include this file. Include "drake/common/autodiff.h".
#endif

#include <algorithm>
#include <cmath>
#include <ostream>
#include <utility>
#include <vector>

#include <Eigen/Dense>
#include <immintrin.h>

#include "drake/common/never_destroyed.h"

// Comment this out to use the heap instead of the pool.
#define DRAKE_AUTODIFF_USE_POOL

namespace drake {
namespace internal {

/* Manages a pool of fixed-size memory block of doubles obtained from the heap.
We burn the first few entries of each block for debugging and future fancier
stuff. A magic number is stamped into the burned area so we can verify that
we're getting back memory that we are managing. */
class Pool {
 public:
  Pool(int dim, int preload) : dim_(dim), max_size_(preload) {
    for (int i=0; i < preload; ++i) {
      pool_.push_back(new_aligned(dim_));
      pool_.back()[0] = kMagic;
    }
  }

  ~Pool() { FreePoolMemory(); }

  void FreePoolMemory() {
    while (!pool_.empty()) {
      double* p = pool_.back();
      DRAKE_ASSERT(p[0] == kMagic);
      delete_aligned(p);
      pool_.pop_back();
    }
  }

  // Note that the caller does not get a pointer to the beginning of the
  // block, but rather some offset into the block.
  double* alloc(int64_t size) {
    DRAKE_ASSERT(size <= dim() - kOffset);
    if (size == 0) return nullptr;
    double* p;
    if (pool_.empty()) {
      p = new_aligned(dim_);
      p[0] = kMagic;
    } else {
      p = pool_.back();
      pool_.pop_back();
    }
    return p + kOffset;
  }

  // The caller is expected to return the offset pointer which we must
  // move back to the start of the block for saving in the pool.
  void free(double* p_offset) {
    if (p_offset == nullptr) return;
    double* p = p_offset - kOffset;
    DRAKE_ASSERT(p[0] == kMagic);
    DRAKE_ASSERT(!PoolContains(p));
    pool_.push_back(p);
    if (size() > max_size())  // Statistics; not needed for operation.
      max_size_ = size();
  }

  int dim() const { return dim_; }
  int size() const { return static_cast<int>(pool_.size()); }
  int max_size() const { return max_size_; }

 private:
  static double* new_aligned(int sz) {
    return new(std::align_val_t(32)) double[sz];
  }
  static void delete_aligned(double* p) {
    operator delete[](p, std::align_val_t(32));
  }
  bool PoolContains(double* p) const {
    return std::find(pool_.begin(), pool_.end(), p) != pool_.end();
  }

  const int dim_;
  int max_size_{0};
  std::vector<double*> pool_;

  static constexpr double kMagic = 1234.5;
  static constexpr int kOffset = 4;
};

/* Pretends to be a VectorXd but allocates memory from a local pool. */
class PoolVectorXd {
 public:
  PoolVectorXd() {}

  explicit PoolVectorXd(size_t size) : data_(my_new(size)), size_(size) {
    setZero();
  }

  ~PoolVectorXd() {
    my_delete(data_);
  }

  // Copy constructor.
  PoolVectorXd(const PoolVectorXd& source)
      : PoolVectorXd(source.data(), source.size()) {
  }

  // Copy constructor from unaligned raw data.
  PoolVectorXd(const double* inu, ptrdiff_t inu_size) :
    data_(my_new(inu_size)), size_(inu_size) {
    SetFromData(inu, inu_size);
  }

  // Copy constructor from VectorXd.
  explicit PoolVectorXd(const Eigen::VectorXd& source)
      : PoolVectorXd(source.data(), source.size()) {}

  // Copy assignment.
  PoolVectorXd& operator=(const PoolVectorXd& source) {
    return SetFromData(source.data(), source.size());
  }

  // Copy assignment from unaligned raw data.
  PoolVectorXd& SetFromData4(const double* inu, ptrdiff_t inu_size) {
    DRAKE_ASSERT(inu == nullptr || inu != data_ || inu_size == size_);
    resize(inu_size);  // Does nothing if already the right size.
    double* data = data_;
    while (data != this_end4()) {
      const __m256d in = _mm256_loadu_pd(inu);  // Next 4 input values.
      _mm256_store_pd(data, in);  // Aligned.
      data += 4; inu += 4;
    }
    while (data != this_end()) {
      *data++ = *inu++;
    }
    return *this;
  }

  // Copy assignment from unaligned raw data.
  /*__attribute__((always_inline))*/ PoolVectorXd&
  SetFromData(const double* inu, ptrdiff_t inu_size) {
    DRAKE_ASSERT(inu == nullptr || inu != data_ || inu_size == size_);
    resize(inu_size);  // Does nothing if already the right size.
    double* out = data_;
    const double* end32 = this_end32();
    while (out != end32) {
      __m256d* d = reinterpret_cast<__m256d*>(out);

      *(d + 0) = _mm256_loadu_pd(inu + 0);  // Next 4 input values.
      *(d + 1) = _mm256_loadu_pd(inu + 4);
      *(d + 2) = _mm256_loadu_pd(inu + 8);
      *(d + 3) = _mm256_loadu_pd(inu + 12);
      *(d + 4) = _mm256_loadu_pd(inu + 16);
      *(d + 5) = _mm256_loadu_pd(inu + 20);
      *(d + 6) = _mm256_loadu_pd(inu + 24);
      *(d + 7) = _mm256_loadu_pd(inu + 28);

      out += 32; inu += 32;
    }

    // Handle the next (inu_size % 32) / 8 entries (max 3 iterations).
    const double* end8 = this_end8();
    while (out != end8) {
      __m256d* d = reinterpret_cast<__m256d*>(out);
      *(d + 0) = _mm256_loadu_pd(inu + 0);
      *(d + 1) = _mm256_loadu_pd(inu + 4);
      out += 8; inu += 8;
    }

    // Finish off the last inu_size % 8 entries (max 7 iterations).
    const double* end = this_end();
    while (out != end) {
      *out++ = *inu++;
    }

    return *this;
  }

  // Copy assignment from unaligned raw data.
  __attribute__((always_inline)) PoolVectorXd&
  SetFromData16(const double* inu, ptrdiff_t inu_size) {
    DRAKE_ASSERT(inu == nullptr || inu != data_ || inu_size == size_);
    resize(inu_size);  // Does nothing if already the right size.
    double* out = data_;
    const double* end16 = this_end16();
    while (out != end16) {
      __m256d* d = reinterpret_cast<__m256d*>(out);

      *(d + 0) = _mm256_loadu_pd(inu + 0);  // Next 4 input values.
      *(d + 1) = _mm256_loadu_pd(inu + 4);
      *(d + 2) = _mm256_loadu_pd(inu + 8);
      *(d + 3) = _mm256_loadu_pd(inu + 12);

      out += 16; inu += 16;
    }

    // Finish off the last inu_size % 16 entries (max 15 iterations).
    const double* end = this_end();
    while (out != end) {
      *out++ = *inu++;
    }

    return *this;
  }

  // Copy assignment from unaligned raw data.
  PoolVectorXd& SetFromDataUsingCopy(const double* inu, ptrdiff_t inu_size) {
    DRAKE_ASSERT(inu == nullptr || inu != data_ || inu_size == size_);
    resize(inu_size);  // Does nothing if already the right size.
    std::copy(inu, inu + inu_size, data_);
    return *this;
  }

  // Assignment from scaled unaligned raw data.
  // *this = scale * inu
  PoolVectorXd& SetFromDataScaled(const double* inu, ptrdiff_t inu_size,
                                  double scale) {
    DRAKE_ASSERT(inu == nullptr || inu != data_);
    resize(inu_size);  // Does nothing if already the right size.
    const __m256d scale4 = _mm256_broadcast_sd(&scale);
    double* data = data_;  // Aligned.
    const double* end16 = this_end16();
    while (data != end16) {
      __m256d* d = reinterpret_cast<__m256d*>(data);

      __m256d in = _mm256_loadu_pd(inu);  // Next 4 input values.
      *d = _mm256_mul_pd(scale4, in);

      in = _mm256_loadu_pd(inu + 4);
      *(d + 1) = _mm256_mul_pd(scale4, in);

      in = _mm256_loadu_pd(inu + 8);
      *(d + 2)  = _mm256_mul_pd(scale4, in);

      in = _mm256_loadu_pd(inu + 12);
      *(d + 3)  = _mm256_mul_pd(scale4, in);

      data += 16; inu += 16;
    }
    while (data != this_end()) {
      *data++ = scale * *inu++;
    }
    return *this;
  }

  // Move constructor.
  PoolVectorXd(PoolVectorXd&& source)
      : data_(source.data_), size_(source.size_) {
    source.data_ = nullptr;
    source.size_ = 0;
  }

  // Move assignment.
  PoolVectorXd& operator=(PoolVectorXd&& source) {
    std::swap(data_, source.data_);
    std::swap(size_, source.size_);
    return *this;
  }

  // Copy constructor from anything that looks like a VectorXd.
  PoolVectorXd(Eigen::Ref<const Eigen::VectorXd> source, bool) :
     PoolVectorXd(source.data(), source.size()) {}

  // Copy assignment from a VectorXd.
  PoolVectorXd& operator=(const Eigen::VectorXd& source) {
    SetFromData(source.data(), source.size());
    return *this;
  }

  // Copy assignment from anything that looks like a VectorXd.
  PoolVectorXd& SetFromAny(Eigen::Ref<const Eigen::VectorXd> source) {
    return SetFromData(source.data(), source.size());
  }

  // Assignment from anything that looks like a VectorXd, scaled.
  PoolVectorXd& SetFromAnyScaled(Eigen::Ref<const Eigen::VectorXd> source,
                                 double scale) {
    return SetFromDataScaled(source.data(), source.size(), scale);
  }

  ptrdiff_t size() const { return size_; }
  double* data() { return data_; }
  const double* data() const { return data_; }

  double& coeffRef(ptrdiff_t i) {
    DRAKE_ASSERT(i < size());
    return data_[i];
  }

  const double& coeff(ptrdiff_t i) const {
    DRAKE_ASSERT(i < size());
    return data_[i];
  }

  const double& operator[](ptrdiff_t i) const { return coeff(i); }
  const double& operator()(ptrdiff_t i) const { return coeff(i); }

  double& operator[](ptrdiff_t i) { return coeffRef(i); }
  double& operator()(ptrdiff_t i) { return coeffRef(i); }

  void setZero() {
    memset(data_, 0, size() * sizeof(double));
  }

  void resize(ptrdiff_t dsize) {
    if (size_ != dsize) {
      my_delete(data_);
      data_ = my_new(dsize);
      size_ = dsize;
    }
  }

  void resize(ptrdiff_t dsize, int ncols) {
    DRAKE_DEMAND(ncols == 1);
    resize(dsize);
  }

  PoolVectorXd& ScaleInPlace4(const double& scale) {
    const __m256d scale4 = _mm256_broadcast_sd(&scale);

    double* data = data_;  // Aligned data.
    while (data != this_end4()) {
      const __m256d in = _mm256_load_pd(data);  // Next 4 values.
      const __m256d result = _mm256_mul_pd(scale4, in);
      _mm256_store_pd(data, result);
      data += 4;
    }
    while (data != this_end()) {
      *data++ *= scale;
    }
    return *this;
  }

  PoolVectorXd& ScaleInPlace(const double& scale) {
    const __m256d scale4 = _mm256_broadcast_sd(&scale);

    double* data = data_;  // Aligned data.
    const double* end16 = this_end16();
    while (data != end16) {
      __m256d* d = reinterpret_cast<__m256d*>(data);
      *d = _mm256_mul_pd(scale4, *d);
      *(d + 1) = _mm256_mul_pd(scale4, *(d + 1));
      *(d + 2) = _mm256_mul_pd(scale4, *(d + 2));
      *(d + 3) = _mm256_mul_pd(scale4, *(d + 3));
      data += 16;
    }
    while (data != this_end()) {
      *data++ *= scale;
    }
    return *this;
  }

  PoolVectorXd& operator*=(const double& scale) { return ScaleInPlace(scale); }

  inline friend PoolVectorXd operator*(PoolVectorXd pvx, double d) {
    return pvx *= d;
  }

  inline friend PoolVectorXd operator*(double d, PoolVectorXd pvx) {
    return pvx *= d;
  }

  PoolVectorXd& operator/=(const double& d) {
    const double ood = 1. / d;
    return *this *= ood;
  }

  PoolVectorXd& PlusEqDataScaled4(const double* in, ptrdiff_t in_size,
                                 double scale) {
    DRAKE_ASSERT(in_size == size());
    const __m256d scale4 = _mm256_broadcast_sd(&scale);
    double* data = data_;  // Aligned data.
    while (data != this_end4()) {
      const __m256d inn = _mm256_loadu_pd(in);  // May be unaligned.
      __m256d outt = _mm256_load_pd(data);  // Aligned.
      outt = _mm256_fmadd_pd(scale4, inn, outt);  // outt = s*in + outt
      _mm256_store_pd(data, outt);  // *data = outt
      data += 4; in += 4;
    }
    while (data != this_end()) {
      *data++ += scale * *in++;
    }
    return *this;
  }

  PoolVectorXd& PlusEqDataScaled(const double* in, ptrdiff_t in_size,
                                 double scale) {
    DRAKE_ASSERT(in_size == size());
    const __m256d scale4 = _mm256_broadcast_sd(&scale);
    double* data = data_;  // Aligned data.
    const double* end16 = this_end16();
    while (data != end16) {
      __m256d* d = reinterpret_cast<__m256d*>(data);

      __m256d inn = _mm256_loadu_pd(in);  // May be unaligned.
      *d = _mm256_fmadd_pd(scale4, inn, *d);  // d = s*in + d

      inn = _mm256_loadu_pd(in + 4);
      *(d + 1) = _mm256_fmadd_pd(scale4, inn, *(d + 1));

      inn = _mm256_loadu_pd(in + 8);
      *(d + 2) = _mm256_fmadd_pd(scale4, inn, *(d + 2));

      inn = _mm256_loadu_pd(in + 12);
      *(d + 3) = _mm256_fmadd_pd(scale4, inn, *(d + 3));

      data += 16; in += 16;
    }
    while (data != this_end()) {
      *data++ += scale * *in++;
    }
    return *this;
  }

  template <int N>
  PoolVectorXd& PlusEqAnyScaled(const Eigen::Matrix<double, N, 1>& rhs,
                                double scale) {
    return PlusEqDataScaled(rhs.data(), rhs.size(), scale);
  }

  PoolVectorXd& PlusEqAnyScaled(const Eigen::VectorXd& rhs, double scale) {
    return PlusEqDataScaled(rhs.data(), rhs.size(), scale);
  }

  PoolVectorXd& PlusEqAnyScaled(Eigen::Ref<Eigen::VectorXd> rhs, double scale) {
    return PlusEqDataScaled(rhs.data(), rhs.size(), scale);
  }

  PoolVectorXd& PlusEqData4(const double* in, ptrdiff_t in_size) {
    DRAKE_ASSERT(in_size == size());
    double* data = data_;  // Aligned data.
    while (data != this_end4()) {
      const __m256d inn = _mm256_loadu_pd(in);  // May be unaligned.
      __m256d outt = _mm256_load_pd(data);  // Aligned.
      outt = _mm256_add_pd(outt, inn);  // outt = outt + inn
      _mm256_store_pd(data, outt);  // *data = outt
      data += 4; in += 4;
    }
    while (data != this_end()) {
      *data++ += *in++;
    }
    return *this;
  }

  PoolVectorXd& PlusEqData(const double* in, ptrdiff_t in_size) {
    DRAKE_ASSERT(in_size == size());
    double* data = data_;  // Aligned data.
    while (data != this_end16()) {
      __m256d* d = reinterpret_cast<__m256d*>(data);

      __m256d inn = _mm256_loadu_pd(in);  // May be unaligned.
      *d = _mm256_add_pd(*d, inn);  // d = d + inn

      inn = _mm256_loadu_pd(in + 4);
      *(d + 1) = _mm256_add_pd(*(d + 1), inn);

      inn = _mm256_loadu_pd(in + 8);
      *(d + 2) = _mm256_add_pd(*(d + 2), inn);

      inn = _mm256_loadu_pd(in + 12);
      *(d + 3) = _mm256_add_pd(*(d + 3), inn);

      data += 16; in += 16;
    }
    while (data != this_end()) {
      *data++ += *in++;
    }
    return *this;
  }

  template <int N>
  PoolVectorXd& PlusEqAny(const Eigen::Matrix<double, N, 1>& rhs) {
    return PlusEqData(rhs.data(), rhs.size());
  }

  PoolVectorXd& PlusEqAny(const Eigen::VectorXd& rhs) {
    return PlusEqData(rhs.data(), rhs.size());
  }

  PoolVectorXd& PlusEqAny(Eigen::Ref<Eigen::VectorXd> rhs) {
    return PlusEqData(rhs.data(), rhs.size());
  }

  PoolVectorXd& operator+=(const PoolVectorXd& pvx) {
    return PlusEqData(pvx.data(), pvx.size());
    return *this;
  }

  PoolVectorXd& MinusEqData4(const double* in, ptrdiff_t in_size) {
    DRAKE_ASSERT(in_size == size());
    double* data = data_;  // Aligned data.
    while (data != this_end4()) {
      const __m256d inn = _mm256_loadu_pd(in);  // May be unaligned.
      __m256d outt = _mm256_load_pd(data);  // Aligned.
      outt = _mm256_sub_pd(outt, inn);  // outt = outt - inn
      _mm256_store_pd(data, outt);  // *data = outt
      data += 4; in += 4;
    }
    while (data != this_end()) {
      *data++ -= *in++;
    }
    return *this;
  }

  PoolVectorXd& MinusEqData(const double* in, ptrdiff_t in_size) {
    DRAKE_ASSERT(in_size == size());
    double* data = data_;  // Aligned data.
    while (data != this_end16()) {
      __m256d* d = reinterpret_cast<__m256d*>(data);

      __m256d inn = _mm256_loadu_pd(in);  // May be unaligned.
      *d = _mm256_sub_pd(*d, inn);  // d = d - inn

      inn = _mm256_loadu_pd(in + 4);
      *(d + 1) = _mm256_sub_pd(*(d + 1), inn);

      inn = _mm256_loadu_pd(in + 8);
      *(d + 2) = _mm256_sub_pd(*(d + 2), inn);

      inn = _mm256_loadu_pd(in + 12);
      *(d + 3) = _mm256_sub_pd(*(d + 3), inn);

      data += 16; in += 16;
    }
    while (data != this_end()) {
      *data++ -= *in++;
    }
    return *this;
  }

  template <int N>
  PoolVectorXd& MinusEqAny(const Eigen::Matrix<double, N, 1>& rhs) {
    return MinusEqData(rhs.data(), rhs.size());
  }

  PoolVectorXd& MinusEqAny(const Eigen::VectorXd& rhs) {
    return MinusEqData(rhs.data(), rhs.size());
  }

  PoolVectorXd& MinusEqAny(Eigen::Ref<Eigen::VectorXd> rhs) {
    return MinusEqData(rhs.data(), rhs.size());
  }

  PoolVectorXd& operator-=(const PoolVectorXd& pvx) {
    return MinusEqData(pvx.data(), pvx.size());
  }

  PoolVectorXd operator-() const {
    return PoolVectorXd(*this) *= -1.;
  }

  const Eigen::VectorXd& to_eigen() const {
    return *reinterpret_cast<const Eigen::VectorXd*>(&data_);
  }

  inline friend bool operator==(const PoolVectorXd& lhs,
      const Eigen::VectorXd& rhs) {
    return lhs.to_eigen() == rhs;
  }

  inline friend bool operator==(const Eigen::VectorXd& lhs,
      const PoolVectorXd& rhs) {
    return lhs == rhs.to_eigen();
  }

  inline friend std::ostream& operator<<(std::ostream& os,
                                         const PoolVectorXd& pvxd) {
    return os << pvxd.to_eigen();
  }

  template <typename Archive>
  void Serialize(Archive* a) {
    // TODO(rpoyner-tri) implement this.
    DRAKE_UNREACHABLE();
  }

  /* Reinterpret a const VectorXd as a const PoolVectorXd. */
  static const PoolVectorXd& to_pool_vector(const Eigen::VectorXd& vxd) {
    static_assert(sizeof(PoolVectorXd) == sizeof(Eigen::VectorXd));
    return *reinterpret_cast<const PoolVectorXd*>(&vxd);
  }

  static int pool_max_size() {
    return pool().max_size();
  }

 private:
  static double* my_new(size_t sz) {
#ifdef DRAKE_AUTODIFF_USE_POOL
    return pool().alloc(sz);
#else
    return new(std::align_val_t(32)) double[sz];
#endif
  }

  static void my_delete(double* data) {
#ifdef DRAKE_AUTODIFF_USE_POOL
    pool().free(data);
#else
    operator delete[](data, std::align_val_t(32));
#endif
  }

  static inline Pool& pool() {
    if (!pool_) {
      pool_ = new Pool(128, 10);
    }
    return *pool_;
  }

  const double* this_end() const { return data_ + size_; }
  const double* this_end4()  const { return data_ + (size_ & ~0b11); }
  const double* this_end8()  const { return data_ + (size_ & ~0b111); }
  const double* this_end16() const { return data_ + (size_ & ~0b1111); }
  const double* this_end32() const { return data_ + (size_ & ~0b11111); }

  // This layout has to match Eigen's VectorXd in memory. (Enforced at compile
  // time via a static_assert above.)
  double* data_{};
  ptrdiff_t size_{0};

  static __thread Pool* pool_;
};

}  // namespace internal
}  // namespace drake

#undef DRAKE_AUTODIFF_USE_POOL

namespace Eigen {

#if !defined(DRAKE_DOXYGEN_CXX)
// Explicit template specializations of Eigen::AutoDiffScalar for VectorXd.
//
// AutoDiffScalar tries to call internal::make_coherent to promote empty
// derivatives. However, it fails to do the promotion when an operand is an
// expression tree (i.e. CwiseBinaryOp). Our solution is to provide special
// overloading for VectorXd and change the return types of its operators. With
// this change, the operators evaluate terms immediately and return an
// AutoDiffScalar<VectorXd> instead of expression trees (such as CwiseBinaryOp).
// Eigen's implementation of internal::make_coherent makes use of const_cast in
// order to promote zero sized derivatives. This however interferes badly with
// our caching system and produces unexpected behaviors. See #10971 for details.
// Therefore our implementation stops using internal::make_coherent and treats
// scalars with zero sized derivatives as constants, as it should.
//
// We also provide overloading of math functions for AutoDiffScalar<VectorXd>
// which return AutoDiffScalar<VectorXd> instead of an expression tree.
//
// See https://github.com/RobotLocomotion/drake/issues/6944 for more
// information. See also drake/common/autodiff_overloads.h.
//
// TODO(soonho-tri): Next time when we upgrade Eigen, please check if we still
// need these specializations.
//
// @note move-aware arithmetic
// Prior implementations of arithmetic overloads required construction of new
// objects at each operation, which induced costly heap allocations. In modern
// C++, it is possible to instead exploit move semantics to avoid allocation in
// many cases. In particular, the compiler can implicitly use moves to satisfy
// pass-by-value parameters in cases where moves are possible (move construction
// and assignment are available), and the storage in question is not needed
// afterward. This allows definitions of operators that pass and return by
// value, and only allocate when needed, as determined by the compiler. For C++
// considerations, see Scott Meyers' _Effective Modern C++_ Item 41. See #13985
// for more discussion of Drake considerations.
template <>
class AutoDiffScalar<VectorXd>
    : public internal::auto_diff_special_op<VectorXd, false> {
 public:
  typedef internal::auto_diff_special_op<VectorXd, false> Base;
  typedef typename internal::remove_all<VectorXd>::type DerType;
  typedef typename internal::traits<DerType>::Scalar Scalar;
  typedef typename NumTraits<Scalar>::Real Real;

  using Base::operator+;
  using Base::operator*;

  AutoDiffScalar() {}

  AutoDiffScalar(const Scalar& value, int nbDer, int derNumber)
      : m_value(value), m_derivatives(nbDer) {
    m_derivatives.coeffRef(derNumber) = Scalar(1);
  }

  // NOLINTNEXTLINE(runtime/explicit): Code from Eigen.
  AutoDiffScalar(const Real& value) : m_value(value) {}

  AutoDiffScalar(const Scalar& value, const DerType& der)
      : m_value(value), m_derivatives(der) {}

  template <typename OtherDerType>
  AutoDiffScalar(
      const AutoDiffScalar<OtherDerType>& other
#ifndef EIGEN_PARSED_BY_DOXYGEN
          ,
          typename internal::enable_if<
          internal::is_same<
              Scalar, typename internal::traits<typename internal::remove_all<
                          OtherDerType>::type>::Scalar>::value,
          void*>::type = 0
#endif
      )
      : m_value(other.value()),
      m_derivatives(other.derivatives(), true) {
  }

  friend std::ostream& operator<<(std::ostream& s, const AutoDiffScalar& a) {
    return s << a.value();
  }

  AutoDiffScalar(const AutoDiffScalar& other)
      : m_value(other.value()), m_derivatives(other.pool_derivatives()) {}

  // Move construction and assignment are trivial, but need to be explicitly
  // requested, since we have user-declared copy and assignment operators.
  AutoDiffScalar(AutoDiffScalar&&) = default;
  AutoDiffScalar& operator=(AutoDiffScalar&&) = default;

  template <typename OtherDerType>
  inline AutoDiffScalar& operator=(const AutoDiffScalar<OtherDerType>& other) {
    m_value = other.value();
    m_derivatives.SetFromAny(other.derivatives());
    return *this;
  }

  inline AutoDiffScalar& operator=(const AutoDiffScalar& other) {
    m_value = other.value();
    m_derivatives = other.pool_derivatives();
    return *this;
  }

  inline AutoDiffScalar& operator=(const Scalar& other) {
    m_value = other;
    if (m_derivatives.size() > 0) m_derivatives.setZero();
    return *this;
  }

  inline const Scalar& value() const { return m_value; }
  inline Scalar& value() { return m_value; }

  inline const DerType& derivatives() const { return const_derivatives(); }
  inline const DerType& const_derivatives() const {
    return m_derivatives.to_eigen();
  }
  inline drake::internal::PoolVectorXd& derivatives() { return m_derivatives; }

  inline const drake::internal::PoolVectorXd& pool_derivatives() const {
    return m_derivatives;
  }
  inline drake::internal::PoolVectorXd& pool_derivatives() {
    return m_derivatives;
  }

  inline bool operator<(const Scalar& other) const { return m_value < other; }
  inline bool operator<=(const Scalar& other) const { return m_value <= other; }
  inline bool operator>(const Scalar& other) const { return m_value > other; }
  inline bool operator>=(const Scalar& other) const { return m_value >= other; }
  inline bool operator==(const Scalar& other) const { return m_value == other; }
  inline bool operator!=(const Scalar& other) const { return m_value != other; }

  friend inline bool operator<(const Scalar& a, const AutoDiffScalar& b) {
    return a < b.value();
  }
  friend inline bool operator<=(const Scalar& a, const AutoDiffScalar& b) {
    return a <= b.value();
  }
  friend inline bool operator>(const Scalar& a, const AutoDiffScalar& b) {
    return a > b.value();
  }
  friend inline bool operator>=(const Scalar& a, const AutoDiffScalar& b) {
    return a >= b.value();
  }
  friend inline bool operator==(const Scalar& a, const AutoDiffScalar& b) {
    return a == b.value();
  }
  friend inline bool operator!=(const Scalar& a, const AutoDiffScalar& b) {
    return a != b.value();
  }

  template <typename OtherDerType>
  inline bool operator<(const AutoDiffScalar<OtherDerType>& b) const {
    return m_value < b.value();
  }
  template <typename OtherDerType>
  inline bool operator<=(const AutoDiffScalar<OtherDerType>& b) const {
    return m_value <= b.value();
  }
  template <typename OtherDerType>
  inline bool operator>(const AutoDiffScalar<OtherDerType>& b) const {
    return m_value > b.value();
  }
  template <typename OtherDerType>
  inline bool operator>=(const AutoDiffScalar<OtherDerType>& b) const {
    return m_value >= b.value();
  }
  template <typename OtherDerType>
  inline bool operator==(const AutoDiffScalar<OtherDerType>& b) const {
    return m_value == b.value();
  }
  template <typename OtherDerType>
  inline bool operator!=(const AutoDiffScalar<OtherDerType>& b) const {
    return m_value != b.value();
  }

  // The arithmetic operators below exploit move-awareness to avoid heap
  // allocations. See note `move-aware arithmetic` above. Particular details
  // will be called out below, the first time they appear.

  // Using a friend operator instead of a method allows the ADS parameter to be
  // used as storage when move optimizations are possible.
  friend inline AutoDiffScalar operator+(AutoDiffScalar a, const Scalar& b) {
    a += b;
    return a;
  }

  friend inline AutoDiffScalar operator+(const Scalar& a, AutoDiffScalar b) {
    b += a;
    return b;
  }

  // Compound assignment operators contain the primitive implementations, since
  // the choice of writable storage is clear. Binary operations invoke the
  // compound assignments.
  inline AutoDiffScalar& operator+=(const Scalar& other) {
    value() += other;
    return *this;
  }

  // It is possible that further overloads could exploit more move-awareness
  // here. However, overload ambiguities are difficult to resolve. Currently
  // only the left-hand operand is available for optimizations.  See #13985,
  // #14039 for discussion.
  template <typename OtherDerType>
  friend inline AutoDiffScalar<DerType> operator+(
      AutoDiffScalar<DerType> a, const AutoDiffScalar<OtherDerType>& b) {
    a += b;
    return a;
  }

  template <typename OtherDerType>
  inline AutoDiffScalar& operator+=(const AutoDiffScalar<OtherDerType>& other) {
    const bool has_this_der = m_derivatives.size() > 0;
    const bool has_both_der =
        has_this_der && (other.derivatives().size() > 0);
    m_value += other.value();
    if (has_both_der) {
      m_derivatives.PlusEqAny(other.derivatives());
    } else if (has_this_der) {
      // noop
    } else {
      m_derivatives.SetFromAny(other.derivatives());
    }
    return *this;
  }

  friend inline AutoDiffScalar operator-(AutoDiffScalar a, const Scalar& b) {
    a -= b;
    return a;
  }

  // Scalar-on-the-left non-commutative operations must also contain primitive
  // implementations.
  friend inline AutoDiffScalar operator-(const Scalar& a, AutoDiffScalar b) {
    b.value() = a - b.value();
    b.pool_derivatives() *= -1;
    return b;
  }

  inline AutoDiffScalar& operator-=(const Scalar& other) {
    m_value -= other;
    return *this;
  }

  template <typename OtherDerType>
  friend inline AutoDiffScalar<DerType> operator-(
      AutoDiffScalar<DerType> a, const AutoDiffScalar<OtherDerType>& other) {
    a -= other;
    return a;
  }

  template <typename OtherDerType>
  inline AutoDiffScalar& operator-=(const AutoDiffScalar<OtherDerType>& other) {
    const bool has_this_der = m_derivatives.size() > 0;
    const bool has_both_der =
        has_this_der && (other.derivatives().size() > 0);
    m_value -= other.value();
    if (has_both_der) {
      m_derivatives.MinusEqAny(other.derivatives());
    } else if (has_this_der) {
      // noop
    } else {
      m_derivatives.SetFromAnyScaled(other.derivatives(), -1.);
    }
    return *this;
  }

  // Phrasing unary negation as a value-passing friend permits some move
  // optimizations.
  friend inline AutoDiffScalar operator-(AutoDiffScalar a) {
    a.value() *= -1;
    a.pool_derivatives() *= -1;
    return a;
  }

  friend inline AutoDiffScalar operator*(AutoDiffScalar a, const Scalar& b) {
    a *= b;
    return a;
  }

  friend inline AutoDiffScalar operator*(const Scalar& a, AutoDiffScalar b) {
    b *= a;
    return b;
  }

  friend inline AutoDiffScalar operator/(AutoDiffScalar a, const Scalar& b) {
    a /= b;
    return a;
  }

  friend inline AutoDiffScalar operator/(const Scalar& a, AutoDiffScalar b) {
    b.pool_derivatives() *= Scalar(-a) / (b.value() * b.value());
    b.value() = a / b.value();
    return b;
  }

  template <typename OtherDerType>
  friend inline AutoDiffScalar<DerType> operator/(
      AutoDiffScalar<DerType> a, const AutoDiffScalar<OtherDerType>& b) {
    a /= b;
    return a;
  }

  template <typename OtherDerType>
  friend inline AutoDiffScalar<DerType> operator*(
      AutoDiffScalar<DerType> a, const AutoDiffScalar<OtherDerType>& b) {
    a *= b;
    return a;
  }

  inline AutoDiffScalar& operator*=(const Scalar& other) {
    m_value *= other;
    m_derivatives *= other;
    return *this;
  }

  template <typename OtherDerType>
  inline AutoDiffScalar& operator*=(const AutoDiffScalar<OtherDerType>& other) {
    const bool has_this_der = m_derivatives.size() > 0;
    const bool has_both_der = has_this_der && (other.derivatives().size() > 0);
    // Some of the math below may look tempting to rewrite using `*=`, but
    // performance measurement and analysis show that this formulation is
    // faster because it results in better expression tree optimization and
    // inlining.
    if (has_both_der) {
      m_derivatives *= other.value();
      m_derivatives.PlusEqAnyScaled(other.derivatives(), m_value);
    } else if (has_this_der) {
      m_derivatives = m_derivatives * other.value();
    } else {
      m_derivatives.SetFromAnyScaled(other.derivatives(), m_value);
    }
    m_value *= other.value();
    return *this;
  }

  inline AutoDiffScalar& operator/=(const Scalar& other) {
    m_value /= other;
    m_derivatives *= Scalar(1) / other;
    return *this;
  }

  template <typename OtherDerType>
  inline AutoDiffScalar& operator/=(const AutoDiffScalar<OtherDerType>& other) {
    auto& this_der = m_derivatives;
    const auto& other_der = other.derivatives();
    const bool has_this_der = m_derivatives.size() > 0;
    const bool has_both_der =
        has_this_der && (other_der.size() > 0);
    const Scalar scale = Scalar(1) / (other.value() * other.value());
    if (has_both_der) {
      this_der *= other.value();
      this_der.PlusEqAnyScaled(other_der, -m_value);
      this_der *= scale;
    } else if (has_this_der) {
      this_der *= Scalar(1) / other.value();
    } else {
      this_der.SetFromAnyScaled(other_der, -m_value * scale);
    }
    m_value /= other.value();
    return *this;
  }

 protected:
  Scalar m_value;
  drake::internal::PoolVectorXd m_derivatives;
};

#define DRAKE_EIGEN_AUTODIFFXD_DECLARE_GLOBAL_UNARY(FUNC, CODE) \
  inline AutoDiffScalar<VectorXd> FUNC(                         \
      AutoDiffScalar<VectorXd> x) {                             \
    EIGEN_UNUSED typedef double Scalar;                         \
    CODE;                                                       \
    return x;                                                   \
  }

DRAKE_EIGEN_AUTODIFFXD_DECLARE_GLOBAL_UNARY(
    abs, using std::abs;
    x.pool_derivatives() *= (x.value() < 0 ? -1 : 1);
    x.value() = abs(x.value());)

DRAKE_EIGEN_AUTODIFFXD_DECLARE_GLOBAL_UNARY(
    abs2, using numext::abs2;
    x.pool_derivatives() *= (Scalar(2) * x.value());
    x.value() = abs2(x.value());)

DRAKE_EIGEN_AUTODIFFXD_DECLARE_GLOBAL_UNARY(
    sqrt, using std::sqrt;
    Scalar sqrtx = sqrt(x.value());
    x.value() = sqrtx;
    x.pool_derivatives() *= (Scalar(0.5) / sqrtx);)

DRAKE_EIGEN_AUTODIFFXD_DECLARE_GLOBAL_UNARY(
    cos, using std::cos; using std::sin;
    x.pool_derivatives() *= -sin(x.value());
    x.value() = cos(x.value());)

DRAKE_EIGEN_AUTODIFFXD_DECLARE_GLOBAL_UNARY(
    sin, using std::sin; using std::cos;
    x.pool_derivatives() *= cos(x.value());
    x.value() = sin(x.value());)

DRAKE_EIGEN_AUTODIFFXD_DECLARE_GLOBAL_UNARY(
    exp, using std::exp;
    x.value() = exp(x.value());
    x.pool_derivatives() *= x.value();)

DRAKE_EIGEN_AUTODIFFXD_DECLARE_GLOBAL_UNARY(
    log, using std::log;
    x.pool_derivatives() *= Scalar(1) / x.value();
    x.value() = log(x.value());)

DRAKE_EIGEN_AUTODIFFXD_DECLARE_GLOBAL_UNARY(
    tan, using std::tan; using std::cos;
    x.pool_derivatives() *= Scalar(1) / numext::abs2(cos(x.value()));
    x.value() = tan(x.value());)

DRAKE_EIGEN_AUTODIFFXD_DECLARE_GLOBAL_UNARY(
    asin, using std::sqrt; using std::asin;
    x.pool_derivatives() *= Scalar(1) / sqrt(1 - numext::abs2(x.value()));
    x.value() = asin(x.value());)

DRAKE_EIGEN_AUTODIFFXD_DECLARE_GLOBAL_UNARY(
    acos, using std::sqrt; using std::acos;
    x.pool_derivatives() *= Scalar(-1) / sqrt(1 - numext::abs2(x.value()));
    x.value() = acos(x.value());)

DRAKE_EIGEN_AUTODIFFXD_DECLARE_GLOBAL_UNARY(
    // TODO(rpoyner-tri): implementation seems fishy --see #14051.
    atan, using std::atan;
    x.pool_derivatives() *= Scalar(1) / (1 + x.value() * x.value());
    x.value() = atan(x.value());)

DRAKE_EIGEN_AUTODIFFXD_DECLARE_GLOBAL_UNARY(
    tanh, using std::cosh; using std::tanh;
    x.pool_derivatives() *= Scalar(1) / numext::abs2(cosh(x.value()));
    x.value() = tanh(x.value());)

DRAKE_EIGEN_AUTODIFFXD_DECLARE_GLOBAL_UNARY(
    sinh, using std::sinh; using std::cosh;
    x.pool_derivatives() *= cosh(x.value());
    x.value() = sinh(x.value());)

DRAKE_EIGEN_AUTODIFFXD_DECLARE_GLOBAL_UNARY(
    cosh, using std::sinh; using std::cosh;
    x.pool_derivatives() *= sinh(x.value());
    x.value() = cosh(x.value());)

#undef DRAKE_EIGEN_AUTODIFFXD_DECLARE_GLOBAL_UNARY

// We have this specialization here because the Eigen-3.3.3's atan2
// implementation for AutoDiffScalar does not make a return with properly sized
// derivatives.
inline AutoDiffScalar<VectorXd> atan2(AutoDiffScalar<VectorXd> a,
                                      const AutoDiffScalar<VectorXd>& b) {
  const bool has_a_der = a.pool_derivatives().size() > 0;
  const bool has_both_der = has_a_der && (b.pool_derivatives().size() > 0);
  const double squared_hypot = a.value() * a.value() + b.value() * b.value();
  if (has_both_der) {
    a.pool_derivatives() *= b.value();
    a.pool_derivatives() -= a.value() * b.pool_derivatives();
  } else if (has_a_der) {
    a.pool_derivatives() *= b.value();
  } else {
    a.pool_derivatives() = -a.value() * b.pool_derivatives();
  }
  a.pool_derivatives() /= squared_hypot;
  a.value() = std::atan2(a.value(), b.value());
  return a;
}

// Right-hand pass-by-value optimizations for atan2() are blocked by code in
// Eigen; see #14039.

inline AutoDiffScalar<VectorXd> pow(AutoDiffScalar<VectorXd> a, double b) {
  // TODO(rpoyner-tri): implementation seems fishy --see #14052.
  using std::pow;
  a.pool_derivatives() *= b * pow(a.value(), b - 1);
  a.value() = pow(a.value(), b);
  return a;
}

// We have these implementations here because Eigen's implementations do not
// have consistent behavior when a == b. We enforce the following rules for that
// case:
// 1) If both a and b are ADS with non-empty derivatives, return a.
// 2) If both a and b are doubles, return a.
// 3) If one of a, b is a double, and the other is an ADS, return the ADS.
// 4) Treat ADS with empty derivatives as though they were doubles.
// Points (1) and (4) are handled here. Points (2) and (3) are already handled
// by Eigen's overloads.
// See https://gitlab.com/libeigen/eigen/-/issues/1870.
inline const AutoDiffScalar<VectorXd> min(const AutoDiffScalar<VectorXd>& a,
                                          const AutoDiffScalar<VectorXd>& b) {
  // If both a and b have derivatives, then their derivative sizes must match.
  DRAKE_ASSERT(
      a.pool_derivatives().size() == 0 || b.pool_derivatives().size() == 0 ||
      a.pool_derivatives().size() == b.pool_derivatives().size());
  // The smaller of a or b wins; ties go to a iff it has any derivatives.
  return ((a < b) || ((a == b) && (a.pool_derivatives().size() != 0))) ? a : b;
}

// NOLINTNEXTLINE(build/include_what_you_use)
inline const AutoDiffScalar<VectorXd> max(const AutoDiffScalar<VectorXd>& a,
                                          const AutoDiffScalar<VectorXd>& b) {
  // If both a and b have derivatives, then their derivative sizes must match.
  DRAKE_ASSERT(
      a.pool_derivatives().size() == 0 || b.pool_derivatives().size() == 0 ||
      a.pool_derivatives().size() == b.pool_derivatives().size());
  // The larger of a or b wins; ties go to a iff it has any derivatives.
  return ((a > b) || ((a == b) && (a.pool_derivatives().size() != 0))) ? a : b;
}

#endif

}  // namespace Eigen
