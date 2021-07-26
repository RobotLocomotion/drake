#pragma once

#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"

namespace drake {
namespace systems {

/**
 This utility class serves as an in-memory cache of time-dependent vector
 values. Note that this is a standalone class, not a Drake System. It is
 primarily intended to support the Drake System primitive VectorLogSink, but
 can be used independently.

 @tparam_default_scalar
 */
template <typename T>
class VectorLog {
 public:
  /**
   * The default capacity of vector log allocation, expressed as a number of
   * samples. Chosen to be large enough that most systems won't need to
   * allocate during simulation advance steps.
   */
  static constexpr int64_t kDefaultCapacity = 1000;

  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(VectorLog)

  /** Constructs the vector log.
   @param input_size                Dimension of the per-time step data set.
  */
  explicit VectorLog(int input_size);

  /** Reports the size of the log's input vector. */
  int64_t get_input_size() const { return data_.rows(); }

  /** Returns the number of samples taken since construction or last Reset(). */
  int num_samples() const { return num_samples_; }

  /** Accesses the logged time stamps. */
  Eigen::VectorBlock<const VectorX<T>> sample_times() const {
    return const_cast<const VectorX<T>&>(sample_times_).head(num_samples_);
  }

  /** Accesses the logged data. */
  Eigen::Block<const MatrixX<T>, Eigen::Dynamic, Eigen::Dynamic, true> data()
  const {
    return const_cast<const MatrixX<T>&>(data_).leftCols(num_samples_);
  }

  /**
   * Reserve storage for at least `capacity` samples. At construction, there
   * will be at least `kDefaultCapacity`; use this method to reserve more.
   */
  void Reserve(int64_t capacity);

  /** Clears the logged data. */
  void Reset() {
    // Resetting num_samples_ is sufficient to have all future writes and
    // reads re-initialized to the beginning of the data.
    num_samples_ = 0;
  }

  /** Adds a `sample` to the data set with the associated `time` value.

   @param time      The time value for this sample.
   @param sample    A vector of data of the declared size for this log.
   */
  void AddData(T time, const VectorX<T>& sample);

 private:
  int64_t num_samples_{0};
  VectorX<T> sample_times_;
  MatrixX<T> data_;
};
}  // namespace systems
}  // namespace drake
