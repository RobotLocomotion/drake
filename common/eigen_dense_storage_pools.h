#pragma once

/// @file
/// This file contains a specialization of Eigen::DenseStorage to support
/// memory pools.


#ifndef DRAKE_COMMON_AUTODIFF_HEADER
#error Do not directly include this file. Include "drake/common/autodiff.h".
#endif

#include <algorithm>
#include <memory>
#include <type_traits>
#include <utility>

#include <Eigen/Core>

#include "drake/common/drake_assert.h"
#include "drake/common/pool_ladder_allocator.h"

namespace drake {
namespace internal {

/** The magic MaxRowsAtCompileTime value that invokes pools. */
constexpr int kMaxRowsAtCompileTimeThatTriggersPools = 5461;

}  // namespace internal
}  // namespace drake


namespace Eigen {

/** A specialization of a DenseStorage vector for use by drake::AutoDiffXd,
using storage from memory pools. */
template <>
class DenseStorage<
  /* T = */ double,
  /* Size = */ drake::internal::kMaxRowsAtCompileTimeThatTriggersPools,
  /* _Rows = */ Dynamic,
  /* _Cols = */ 1,
  /* _Options = */ 0> {
 public:
  using T = double;

  DenseStorage() = default;

  explicit DenseStorage(internal::constructor_without_unaligned_array_assert)
    : DenseStorage() {}

  DenseStorage(Index size, Index rows, Index cols) {
    this->resize(size, rows, cols);
  }

  DenseStorage(const DenseStorage& other) {
    const Index other_rows = other.rows();
    const T* const other_data = other.data();
    this->resize(other_rows, other_rows, 1);
    internal::smart_copy(other_data, other_data + other_rows, this->data());
  }

  DenseStorage& operator=(const DenseStorage& other) {
    if (this != &other) {
      const Index other_rows = other.rows();
      const T* const other_data = other.data();
      this->resize(other_rows, other_rows, 1);
      internal::smart_copy(other_data, other_data + other_rows, this->data());
    }
    return *this;
  }

  DenseStorage(DenseStorage&& other) noexcept {
    std::swap(this->heap_storage_, other.heap_storage_);
    std::swap(this->rows_, other.rows_);
  }

  DenseStorage& operator=(DenseStorage&& other) noexcept {
    if (this != &other) {
      std::swap(this->heap_storage_, other.heap_storage_);
      std::swap(this->rows_, other.rows_);
    }
    return *this;
  }

  void swap(DenseStorage& other) {
    if (this != &other) {
      std::swap(this->heap_storage_, other.heap_storage_);
      std::swap(this->rows_, other.rows_);
    }
  }

  Index rows() const { return rows_; }
  static Index cols() { return 1; }

  const T* data() const {
    return const_cast<DenseStorage*>(this)->data();
  }

  T* data() {
    return heap_storage_.get();
  }

  void resize(Index size, Index rows, Index cols) {
    DRAKE_ASSERT(size == rows);
    DRAKE_ASSERT(rows >= 0);
    DRAKE_ASSERT(cols == 1);
    if (rows_ == rows) { return; }

    heap_storage_.reset(rows ? ladder()->Get(rows) : nullptr);
    rows_ = rows;
  }

  void conservativeResize(Index size, Index rows, Index cols) {
    DRAKE_ASSERT(size == rows);
    DRAKE_ASSERT(rows >= 0);
    DRAKE_ASSERT(cols == 1);
    if (rows_ == rows) { return; }

    const Index min_rows = std::min(rows_, rows);
    // We are resizing from heap to heap.
    if (rows) {
      auto new_storage(ladder()->Get(rows));
      const T* const old_base = heap_storage_.get();
      internal::smart_copy(old_base, old_base + min_rows, new_storage);
      heap_storage_.reset(new_storage);
    } else {
      heap_storage_.reset();
    }
    rows_ = rows;
  }

 private:
  // Ladders are thread-local, to avoid synchronized bookkeeping.
  using Ladder = drake::internal::pool_ladder_allocator::Ladder;
  static __thread Ladder* s_ladder_;

  static void ladder_init() {
    DRAKE_ASSERT(!s_ladder_);
    s_ladder_ = new Ladder;
    s_reaper_.Awaken();
  }

  static inline Ladder* ladder() {
    if (!s_ladder_) {
      ladder_init();
    }
    return s_ladder_;
  }

  // Away from the fast path, monitor threads and clear up ladders at thread
  // shutdown.
  struct Reaper {
    // This method gives other code a way to force initialization of the Reaper
    // on a thread. Otherwise the object never gets constructed and hence the
    // destructor does nothing.
    void Awaken() {}

    ~Reaper() {
      delete s_ladder_;
      s_ladder_ = nullptr;
    }
  };
  static thread_local Reaper s_reaper_;

  // This function is a convenience wrapper to help construct a custom
  // unique_ptr type that knows to return storage to the pool.
  static inline void Delete(double* p) { ladder()->Put(p); }

  // Use some template magic to construct a custom unique_ptr type whose custom
  // deleter is fixed at compile time.  See also StackOverflow article here:
  // https://stackoverflow.com/questions/19053351/how-do-i-use-a-custom-deleter-with-a-stdunique-ptr-member
  template <auto fn>
  using DeleterFromFn = std::integral_constant<decltype(fn), fn>;
  template <typename T, auto fn>
  using DeleterUniquePtr = std::unique_ptr<T, DeleterFromFn<fn>>;
  using PoolPtr = DeleterUniquePtr<double, Delete>;
  static_assert(sizeof(PoolPtr) == sizeof(void*),
                "PoolPtr is wasting storage for the custom deleter.");

  PoolPtr heap_storage_;
  Index rows_{0};
};

}  // namespace Eigen
