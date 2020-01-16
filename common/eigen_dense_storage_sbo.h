#pragma once

#ifndef DRAKE_COMMON_AUTODIFF_HEADER
#error Do not directly include this file. Include "drake/common/autodiff.h".
#endif

#include <algorithm>
#include <memory>
#include <utility>

#include <Eigen/Core>

#include "drake/common/drake_assert.h"

namespace drake {
namespace internal {

/** The number of inline doubles we store in drake::AutoDiffXd. */
constexpr int kDenseStorageMaxInlineSize = 6;

/** The magic MaxRowsAtCompileTime value that invokes SBO. */
constexpr int kMaxRowsAtCompileTimeThatTriggersInlineStorage = 1234567;

}  // namespace internal
}  // namespace drake

namespace Eigen {

/** A specialization of a DenseStorage vector for use by drake::AutoDiffXd,
using the small-buffer optimization trick, aka "inline vector". */
template <>
class DenseStorage<
  /* T = */ double,
  /* Size = */ drake::internal::kMaxRowsAtCompileTimeThatTriggersInlineStorage,
  /* _Rows = */ Dynamic,
  /* _Cols = */ 1,
  /* _Options = */ 0> {
 public:
  using T = double;
  enum { kMaxInlineSize = drake::internal::kDenseStorageMaxInlineSize };

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
    if (const Index n = other.inline_rows(); n > 0) {
      const T* const other_data = other.inline_storage_;
      internal::smart_copy(other_data, other_data + n, this->inline_storage_);
    }
    std::swap(this->heap_storage_, other.heap_storage_);
    std::swap(this->rows_, other.rows_);
  }

  DenseStorage& operator=(DenseStorage&& other) noexcept {
    if (this != &other) {
      if (const Index n = other.inline_rows(); n > 0) {
        const T* const other_data = other.inline_storage_;
        internal::smart_copy(other_data, other_data + n, this->inline_storage_);
      }
      std::swap(this->heap_storage_, other.heap_storage_);
      std::swap(this->rows_, other.rows_);
    }
    return *this;
  }

  void swap(DenseStorage& other) {
    if (this != &other) {
      const Index max_inline_rows =
          std::max(this->inline_rows(), other.inline_rows());
      std::swap_ranges(
          this->inline_storage_, this->inline_storage_ + max_inline_rows,
          other.inline_storage_);
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
    return rows_ <= kMaxInlineSize ? inline_storage_ : heap_storage_.get();
  }

  void resize(Index size, Index rows, Index cols) {
    DRAKE_ASSERT(size == rows);
    DRAKE_ASSERT(rows >= 0);
    DRAKE_ASSERT(cols == 1);
    if (rows_ == rows) { return; }

    heap_storage_.reset(rows <= kMaxInlineSize ? nullptr : new T[rows]);
    rows_ = rows;
  }

  void conservativeResize(Index size, Index rows, Index cols) {
    DRAKE_ASSERT(size == rows);
    DRAKE_ASSERT(rows >= 0);
    DRAKE_ASSERT(cols == 1);
    if (rows_ == rows) { return; }

    const Index min_rows = std::min(rows_, rows);
    const bool old_inline = (rows_ <= kMaxInlineSize);
    const bool new_inline = (rows <= kMaxInlineSize);
    if (old_inline && new_inline) {
      // We are resizing from inline to inline, so no need to copy.
    } else if (old_inline) {
      // We are resizing from inline to heap.
      heap_storage_.reset(new T[rows]);
      const T* const old_base = inline_storage_;
      internal::smart_copy(old_base, old_base + min_rows, heap_storage_.get());
    } else if (new_inline) {
      // We are resizing from heap to inline.
      const T* const old_base = heap_storage_.get();
      internal::smart_copy(old_base, old_base + min_rows, inline_storage_);
      heap_storage_.reset();
    } else {
      // We are resizing from heap to heap.
      std::unique_ptr<T[]> new_storage(new T[rows]);
      const T* const old_base = heap_storage_.get();
      internal::smart_copy(old_base, old_base + min_rows, new_storage.get());
      heap_storage_ = std::move(new_storage);
    }
    rows_ = rows;
  }

 private:
  Index inline_rows() { return rows_ <= kMaxInlineSize ? rows_ : 0; }

  // Declare inline_storage_ first so that it and we are maximally aligned.
  // N.B We do not initialize inline_storage_, because rows_ == 0 by default.
  T inline_storage_[kMaxInlineSize];
  std::unique_ptr<T[]> heap_storage_;
  Index rows_{0};
};

}  // namespace Eigen
