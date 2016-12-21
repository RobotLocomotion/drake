#pragma once

#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace systems {

/// A sink block which logs its input to memory.  This data is then retrievable
/// (e.g. after a simulation) via a handful of accessor methods.
///
/// @tparam T The vector element type, which must be a valid Eigen scalar.
///
/// Instantiated templates for the following kinds of T's are provided:
/// - double
/// - AutoDiffXd
///
/// @ingroup primitive_systems
template <typename T>
class SignalLogger : public LeafSystem<T> {
 public:
  /// Construct the signal logger system.
  /// @param input_size Dimension of the (single) input port.
  /// @param batch_allocation_size Storage is (re)allocated in blocks of
  /// input_size-by-batch_allocation_size.
  explicit SignalLogger(int input_size, int batch_allocation_size = 1000);

  // Non-copyable.
  SignalLogger(const SignalLogger<T>&) = delete;
  SignalLogger& operator=(const SignalLogger<T>&) = delete;

  /// Access the (simulation) time of the logged data.
  Eigen::VectorBlock<const VectorX<T>> sample_times() const {
    return const_cast<const VectorX<T>&>(sample_times_).head(num_samples_);
  }

  /// Access the logged data.
  Eigen::Block<const MatrixX<T>, Eigen::Dynamic, Eigen::Dynamic, true> data()
      const {
    return const_cast<const MatrixX<T>&>(data_).leftCols(num_samples_);
  }

 private:
  // No output.
  void DoCalcOutput(const Context<T>& context,
                    SystemOutput<T>* output) const override {}

  // Logging is done in this method.
  void DoPublish(const Context<T>& context) const override;

  const int batch_allocation_size_{1000};

  // Use mutable variables to hold the logged data.
  mutable int num_samples_{0};
  mutable VectorX<T> sample_times_;
  mutable MatrixX<T> data_;
};

}  // namespace systems
}  // namespace drake
