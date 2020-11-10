#pragma once

/// @file
/// This file contains a specialization of Eigen::DenseStorage to support
/// memory pools.


#ifndef DRAKE_COMMON_AUTODIFF_HEADER
#error Do not directly include this file. Include "drake/common/autodiff.h".
#endif

#include <algorithm>
#include <memory>
#include <utility>
#include <vector>

#include <Eigen/Core>

#include "drake/common/drake_assert.h"
#include "drake/common/pool_ladder_allocator.h"

namespace drake {
namespace internal {

/** The magic MaxRowsAtCompileTime value that invokes pools. */
constexpr int kMaxRowsAtCompileTimeThatTriggersPools = 987'654'321;

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

    heap_storage_.reset(s_ladder_fn()->get(rows));
    rows_ = rows;
  }

  void conservativeResize(Index size, Index rows, Index cols) {
    DRAKE_ASSERT(size == rows);
    DRAKE_ASSERT(rows >= 0);
    DRAKE_ASSERT(cols == 1);
    if (rows_ == rows) { return; }

    const Index min_rows = std::min(rows_, rows);
    // We are resizing from heap to heap.
    auto new_storage(s_ladder_fn()->get(rows));
    const T* const old_base = heap_storage_.get();
    internal::smart_copy(old_base, old_base + min_rows, new_storage);
    heap_storage_.reset(new_storage);
    rows_ = rows;
  }

 private:
  // Ladders are thread-local, to avoid synchronized bookkeeping.
  using Ladder = drake::internal::pool_ladder_allocator::Ladder;
  static thread_local Ladder* s_ladder;

  // Letting the compiler protect thread-local access results in expensive
  // access functions at every use site. Instead, use an access function that
  // is expensive on the first access in a thread, and cheap thereafter.
  using LadderFn = Ladder* (*)();
  static thread_local LadderFn s_ladder_fn;

  // The cheap ladder access function.
  static Ladder* ladder() { return s_ladder; }

  // The expensive ladder initialization function. It builds the ladder and
  // replaces itself with the cheap function.
  static Ladder* ladder_init_thunk() {
    DRAKE_ASSERT(!s_ladder);
    s_ladder_fn = ladder;
    s_ladder = new Ladder;
    s_reaper.awaken();
    return s_ladder;
  }

  // Away from the fast path, monitor threads and clear up ladders at thread
  // shutdown.
  struct Reaper {
    // This method gives other code a way to force initialization of the Reaper
    // on a thread. Otherwise the object never gets constructed and hence the
    // destructor does nothing.
    void awaken() {}

    ~Reaper() {
      delete s_ladder;
      s_ladder = nullptr;
      s_ladder_fn = ladder_init_thunk;
    }
  };
  static thread_local Reaper s_reaper;

  // This function is a convenience wrapper to help construct a custom
  // unique_ptr type that knows to return storage to the pool.
  static inline void deleter(double* p) { s_ladder_fn()->put(p); }

  // Use some template magic to construct a custom unique_ptr type whose custom
  // deleter is fixed at compile time.  See also StackOverflow article here:
  // https://stackoverflow.com/questions/19053351/how-do-i-use-a-custom-deleter-with-a-stdunique-ptr-member
  template <auto fn>
  using DeleterFromFn = std::integral_constant<decltype(fn), fn>;
  template <typename T, auto fn>
  using DeleterUniquePtr = std::unique_ptr<T, DeleterFromFn<fn>>;
  using PoolPtr = DeleterUniquePtr<double, deleter>;
  static_assert(sizeof(PoolPtr) == sizeof(void*),
                "PoolPtr is wasting storage for the custom deleter.");

  PoolPtr heap_storage_;
  Index rows_{0};
};

}  // namespace Eigen
