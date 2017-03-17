#pragma once

#include <Eigen/Dense>

#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/framework/output_port_value.h"
#include "drake/systems/primitives/signal_log.h"

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
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(SignalLogger)

  /// Construct the signal logger system.
  /// @param input_size Dimension of the (single) input port.
  /// @param batch_allocation_size Storage is (re)allocated in blocks of
  /// input_size-by-batch_allocation_size.
  explicit SignalLogger(int input_size, int batch_allocation_size = 1000);

  /// Access the (simulation) time of the logged data.
  Eigen::VectorBlock<const VectorX<T>> sample_times() const {
    return log_.sample_times();
  }

  /// Access the logged data.
  Eigen::Block<const MatrixX<T>, Eigen::Dynamic, Eigen::Dynamic, true> data()
      const {
    return log_.data();
  }

 private:
  // No output.
  void DoCalcOutput(const Context<T>& context,
                    SystemOutput<T>* output) const override {}

  // Logging is done in this method.
  void DoPublish(const Context<T>& context) const override;

  mutable SignalLog<T> log_;
};

}  // namespace systems
}  // namespace drake
