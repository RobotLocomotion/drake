#pragma once

#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"

namespace drake {
namespace systems {

/**
 This utility class serves as an in-memory cache of time-dependent vector
 values. Note that this is a standalone class, not a Drake System. It is
 primarily intended to support the Drake System primitive SignalLogger, but can
 be used independently.

 @tparam_default_scalar
 */
template <typename T>
class SignalLog {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(SignalLog)

  /** Constructs the signal log.
   @param input_size                Dimension of the per-time step data set.
   @param batch_allocation_size     Storage is (re)allocated in blocks of size
                                    (input_size X batch_allocation_size).
  */
  explicit SignalLog(int input_size, int batch_allocation_size = 1000);

  /** Returns the number of samples taken since construction or last reset(). */
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

  /** Clears the logged data. */
  void reset() {
    // Resetting num_samples_ is sufficient to have all future writes and
    // reads re-initialized to the beginning of the data.
    num_samples_ = 0;
  }

  /** Adds a `sample` to the data set with the associated `time` value.

   @param time      The time value for this sample.
   @param sample    A vector of data of the declared size for this log.
   */
  void AddData(T time, VectorX<T> sample);

  /** Reports the size of the log's input vector. */
  int64_t get_input_size() const { return data_.rows(); }

 private:
  const int batch_allocation_size_{1000};

  // Use mutable variables to hold the logged data.
  mutable int64_t num_samples_{0};
  mutable VectorX<T> sample_times_;
  mutable MatrixX<T> data_;
};
}  // namespace systems
}  // namespace drake
