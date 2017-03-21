#pragma once

#include <memory>

#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/framework/output_port_value.h"
#include "drake/systems/framework/system_port_descriptor.h"

namespace drake {
namespace systems {

/// A base class that specializes LeafSystem for use with no input ports, and
/// only a single output port.  Subclasses should override the protected method
///   DoCalcOutput(const Context<T>&, Eigen::VectorBlock<VectorX<T>>*) const
///
/// @tparam T The vector element type, which must be a valid Eigen scalar.
template <typename T>
class SingleOutputVectorSource : public LeafSystem<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(SingleOutputVectorSource)

  /// Deleted default constructor.  The output vector size must be supplied to
  /// the single-argument constructor SingleOutputVectorSource(int).
  SingleOutputVectorSource() = delete;

  ~SingleOutputVectorSource() override = default;

  /// Returns the sole output port.
  const OutputPortDescriptor<T>& get_output_port() const {
    return LeafSystem<T>::get_output_port(0);
  }

  // Don't use the indexed get_output_port when calling this system directly.
  void get_output_port(int) = delete;

  // Confirms the single-output invariant when allocating the context.
  std::unique_ptr<Context<T>> AllocateContext() const override {
    DRAKE_DEMAND(this->get_num_input_ports() == 0);
    DRAKE_DEMAND(this->get_num_output_ports() == 1);
    return LeafSystem<T>::AllocateContext();
  }

 protected:
  /// Creates a source with the given sole output port configuration.
  explicit SingleOutputVectorSource(int size) {
    this->DeclareOutputPort(kVectorValued, size);
  }

  /// Converts the parameters to Eigen::VectorBlock form, then delegates to
  /// DoCalcVectorOutput().
  void DoCalcOutput(const Context<T>& context,
                    SystemOutput<T>* output) const final {
    Eigen::VectorBlock<VectorX<T>> block =
        System<T>::GetMutableOutputVector(output, 0);
    DoCalcVectorOutput(context, &block);
  }

  /// Provides a convenience method for %SingleOutputVectorSource subclasses.
  /// This method performs the same logical operation as System::DoCalcOutput
  /// but provides the single output's VectorBlock instead.  Subclasses should
  /// override this method, and not the base class method (which is `final`).
  virtual void DoCalcVectorOutput(
      const Context<T>& context,
      Eigen::VectorBlock<VectorX<T>>* output) const = 0;
};

}  // namespace systems
}  // namespace drake
