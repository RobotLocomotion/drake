#pragma once

#include <memory>

#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace systems {

/// A base class that specializes LeafSystem for use with no input ports, and
/// only a single, vector output port. Subclasses should override the protected
/// method
/// @code
/// void DoCalcOutput(const Context<T>&, Eigen::VectorBlock<VectorX<T>>*) const;
/// @endcode
///
/// @tparam T The vector element type, which must be a valid Eigen scalar.
template <typename T>
class SingleOutputVectorSource : public LeafSystem<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(SingleOutputVectorSource)

  /// Deleted default constructor.  Child classes must either supply the
  /// vector size to the single-argument constructor of `int`, or supply a model
  /// vector to the single-argument constructor of `const BasicVector<T>&`.
  SingleOutputVectorSource() = delete;

  ~SingleOutputVectorSource() override = default;

  /// Returns the sole output port.
  const OutputPort<T>& get_output_port() const {
    return LeafSystem<T>::get_output_port(0);
  }

  // Don't use the indexed get_output_port when calling this system directly.
  void get_output_port(int) = delete;

 protected:
  /// Creates a source with the given sole output port configuration.
  explicit SingleOutputVectorSource(int size)
      : SingleOutputVectorSource(BasicVector<T>(size)) {}

  /// Creates a source with output type and dimension of the @p model_vector.
  explicit SingleOutputVectorSource(const BasicVector<T>& model_vector) {
    this->DeclareVectorOutputPort(
        model_vector,
        &SingleOutputVectorSource<T>::CalcVectorOutput);
  }

  /// Provides a convenience method for %SingleOutputVectorSource subclasses.
  /// This method performs the same logical operation as System::DoCalcOutput
  /// but provides the single output's VectorBlock instead.  Subclasses should
  /// override this method, and not the base class method (which is `final`).
  virtual void DoCalcVectorOutput(
      const Context<T>& context,
      Eigen::VectorBlock<VectorX<T>>* output) const = 0;

 private:
  // Confirms the single-output invariant when allocating the context.
  void DoAcquireLeafContextResources(Context<T>*) const final {
    DRAKE_DEMAND(this->get_num_input_ports() == 0);
    DRAKE_DEMAND(this->get_num_output_ports() == 1);
  }

  // Converts the parameters to Eigen::VectorBlock form, then delegates to
  // DoCalcVectorOutput().
  void CalcVectorOutput(const Context<T>& context,
                        BasicVector<T>* output) const {
    Eigen::VectorBlock<VectorX<T>> block = output->get_mutable_value();
    DoCalcVectorOutput(context, &block);
  }
};

}  // namespace systems
}  // namespace drake
