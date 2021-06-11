#pragma once

#include <memory>
#include <utility>

#include "drake/common/default_scalars.h"
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
/// @tparam_default_scalar
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
  ///
  /// @note Objects created using this constructor overload do not support
  /// system scalar conversion.  See @ref system_scalar_conversion.  Use a
  /// different constructor overload if such conversion is desired.
  explicit SingleOutputVectorSource(int size)
      : SingleOutputVectorSource({}, size) {}

  /// Creates a source with output type and dimension of the @p model_vector.
  ///
  /// @note Objects created using this constructor overload do not support
  /// system scalar conversion.  See @ref system_scalar_conversion.  Use a
  /// different constructor overload if such conversion is desired.
  explicit SingleOutputVectorSource(const BasicVector<T>& model_vector)
      : SingleOutputVectorSource({}, model_vector) {}

  /// Creates a source with the given sole output port configuration.
  ///
  /// @note objects created using this constructor may support system scalar
  /// conversion. See @ref system_scalar_conversion.
  ///
  /// @param converter is per LeafSystem::LeafSystem constructor documentation;
  /// see that function documentation for details.
  SingleOutputVectorSource(SystemScalarConverter converter, int size)
      : SingleOutputVectorSource(std::move(converter), BasicVector<T>(size)) {}

  /// Creates a source with output type and dimension of the @p model_vector.
  ///
  /// @note objects created using this constructor may support system scalar
  /// conversion. See @ref system_scalar_conversion.
  ///
  /// @param converter is per LeafSystem::LeafSystem constructor documentation;
  /// see that function documentation for details.
  SingleOutputVectorSource(
      SystemScalarConverter converter, const BasicVector<T>& model_vector)
      : LeafSystem<T>(std::move(converter)) {
    this->DeclareVectorOutputPort(
        kUseDefaultName, model_vector,
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
  void DoValidateAllocatedLeafContext(const LeafContext<T>&) const final {
    DRAKE_DEMAND(this->num_input_ports() == 0);
    DRAKE_DEMAND(this->num_output_ports() == 1);
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

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::systems::SingleOutputVectorSource)
