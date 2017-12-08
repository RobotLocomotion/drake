#pragma once

#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"

namespace drake {
namespace systems {

/**
 This class serves as an in-memory cache of time-dependent vector values.

 @tparam T The vector element type, which must be a valid Eigen scalar.
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

  /** Accesses the logged time stamps. */
  Eigen::VectorBlock<const VectorX<T>> sample_times() const {
    return const_cast<const VectorX<T>&>(sample_times_).head(num_samples_);
  }

  /** Accesses the logged data. */
  Eigen::Block<const MatrixX<T>, Eigen::Dynamic, Eigen::Dynamic, true> data()
  const {
    return const_cast<const MatrixX<T>&>(data_).leftCols(num_samples_);
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
