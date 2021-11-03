#pragma once

#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/common/reset_after_move.h"

namespace drake {
namespace systems {

/**
 This utility class serves as an in-memory cache of time-dependent vector
 values. Note that this is a standalone class, not a Drake System. It is
 primarily intended to support the Drake System primitive VectorLogSink, but
 can be used independently.

 When the log becomes full, adding more data will cause the allocated space to
 double in size. If avoiding memory allocation during some performance-critical
 phase is desired, clients can call Reserve() to pre-allocate log storage.

 This object imposes no constraints on the stored data. For example, times
 passed to AddData() need not be increasing in order of insertion, values are
 allowed to be infinite, NaN, etc.

 @tparam_default_scalar
 */
template <typename T>
class VectorLog {
 public:
  /**
   The default capacity of vector log allocation, expressed as a number of
   samples. Chosen to be large enough that most systems won't need to allocate
   during simulation advance steps.
   */
  static constexpr int64_t kDefaultCapacity = 1000;

  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(VectorLog)

  /** Constructs the vector log.
   @param input_size                Dimension of the per-time step data set.
   */
  explicit VectorLog(int input_size);

  /** Reports the size of the log's input vector. */
  int64_t get_input_size() const { return data_.rows(); }

  /** Returns the number of samples taken since construction or last Clear(). */
  int num_samples() const { return num_samples_; }

  // The return type here must be a VectorBlock because only the leading
  // part of the sample_times_ vector contains meaningful data.
  /** Accesses the logged time stamps. */
  Eigen::VectorBlock<const VectorX<T>> sample_times() const {
    return const_cast<const VectorX<T>&>(sample_times_).head(num_samples_);
  }

  /** Accesses the logged data.

   The InnerPanel parameter of the return type indicates that the compiler can
   assume aligned access to the data.
   */
  Eigen::Block<const MatrixX<T>, Eigen::Dynamic, Eigen::Dynamic,
               true /* InnerPanel */>
  data() const {
    return data_.leftCols(num_samples_);
  }

  /**
   Reserve storage for at least `capacity` samples. At construction, there will
   be at least `kDefaultCapacity`; use this method to reserve more.
   */
  void Reserve(int64_t capacity);

  /** Clears the logged data. */
  void Clear() {
    DRAKE_ASSERT_VOID(CheckInvariants());
    // Resetting num_samples_ is sufficient to have all future writes and
    // reads re-initialized to the beginning of the data.
    num_samples_ = 0;
    DRAKE_ASSERT_VOID(CheckInvariants());
  }

  /** Adds a `sample` to the data set with the associated `time` value. The new
   * sample and time are added to the end of the log. No constraints are
   * imposed on the values of`time` or `sample`.

   @param time      The time value for this sample.
   @param sample    A vector of data of the declared size for this log.
   */
  void AddData(const T& time, const VectorX<T>& sample);

 private:
  void CheckInvariants() const;

  reset_after_move<int64_t> num_samples_{0};
  VectorX<T> sample_times_;
  MatrixX<T> data_;
};
}  // namespace systems
}  // namespace drake
