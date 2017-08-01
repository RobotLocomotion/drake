#pragma once

#include <memory>
#include <vector>

#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/drake_throw.h"
#include "drake/common/eigen_types.h"
#include "drake/common/unused.h"
#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/framework/output_port_value.h"

namespace drake {
namespace systems {

/// A base class that specializes LeafSystem for use with only zero or one
/// vector input ports, and only zero or one vector output ports.
///
/// By default, this base class does not declare any state; subclasses may
/// optionally declare continuous or discrete state, but not both; subclasses
/// may not declare abstract state.
///
/// @tparam T The vector element type, which must be a valid Eigen scalar.
template <typename T>
class VectorSystem : public LeafSystem<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(VectorSystem)

  ~VectorSystem() override = default;

  /// Returns the sole input port.
  const InputPortDescriptor<T>& get_input_port() const {
    DRAKE_DEMAND(this->get_num_input_ports() == 1);
    return LeafSystem<T>::get_input_port(0);
  }

  // Don't use the indexed get_input_port when calling this system directly.
  void get_input_port(int) = delete;

  /// Returns the sole output port.
  const OutputPort<T>& get_output_port() const {
    DRAKE_DEMAND(this->get_num_output_ports() == 1);
    return LeafSystem<T>::get_output_port(0);
  }

  // Don't use the indexed get_output_port when calling this system directly.
  void get_output_port(int) = delete;

  // Confirms the VectorSystem invariants when allocating the context.
  std::unique_ptr<Context<T>> AllocateContext() const final {
    auto result = LeafSystem<T>::AllocateContext();

    // N.B. The DRAKE_THROW_UNLESS conditions can be triggered by subclass
    // mistakes, so are part of our unit tests.  The DRAKE_DEMAND conditions
    // should be invariants guaranteed by the framework, so are asserted.

    // Exactly one input and output.
    DRAKE_THROW_UNLESS(this->get_num_input_ports() <= 1);
    DRAKE_THROW_UNLESS(this->get_num_output_ports() <= 1);
    DRAKE_DEMAND(result->get_num_input_ports() <= 1);

    // At most one of either continuous xor discrete state.
    DRAKE_THROW_UNLESS(result->get_num_abstract_state_groups() == 0);
    const int continuous_size = result->get_continuous_state()->size();
    const int num_discrete_groups = result->get_num_discrete_state_groups();
    DRAKE_DEMAND(continuous_size >= 0);
    DRAKE_DEMAND(num_discrete_groups >= 0);
    DRAKE_THROW_UNLESS(num_discrete_groups <= 1);
    DRAKE_THROW_UNLESS((continuous_size == 0) || (num_discrete_groups == 0));

    return result;
  }

 protected:
  /// Creates a system with one input port and one output port of the given
  /// sizes.  Does not declare any state -- subclasses may optionally declare
  /// continuous or discrete state, but not both.
  VectorSystem(int input_size, int output_size)
      : LeafSystem<T>() {
    DoConstructorBody(input_size, output_size);
  }

  /// Like VectorSystem(int, int), but also declares that this System object is
  /// of dynamic type S, which enables conversion to other scalar-types such as
  /// AutoDiff or symbolic form.  Subclasses that wish to support conversion to
  /// other scalar types should use this constructor.
  ///
  /// Example:
  ///
  /// @code
  /// namespace sample {
  /// template <typename T>
  /// class MySystem : public VectorSystem<T> {
  ///  public:
  ///   DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(MySystem);
  ///
  ///   /// Default constructor.
  ///   MySystem() : VectorSystem<T>(SystemTypeTag<sample::MySystem>{}, 1, 1) {}
  ///
  ///   /// Scalar-converting copy constructor.
  ///   template <typename U>
  ///   explicit MySystem(const MySystem<U>&) : MySystem<T>() {}
  ///
  ///   ...
  /// @endcode
  template <template <typename> class S>
  VectorSystem(SystemTypeTag<S> tag, int input_size, int output_size)
      : LeafSystem<T>(tag) {
    DoConstructorBody(input_size, output_size);
  }

  /// Converts the parameters to Eigen::VectorBlock form, then delegates to
  /// DoCalcVectorTimeDerivatives().
  void DoCalcTimeDerivatives(const Context<T>& context,
                             ContinuousState<T>* derivatives) const final {
    // Short-circuit when there's no work to do.
    if (derivatives->size() == 0) {
      return;
    }

    // Obtain the block form of u (or the empty vector).
    const VectorX<T> empty_vector(0);
    const Eigen::VectorBlock<const VectorX<T>> input_block =
        (this->get_num_input_ports() > 0)
            ? this->EvalEigenVectorInput(context, 0)
            : Eigen::VectorBlock<const VectorX<T>>(empty_vector, 0, 0);

    // Obtain the block form of xc.
    DRAKE_ASSERT(context.has_only_continuous_state());
    const VectorBase<T>& state_vector = context.get_continuous_state_vector();
    const Eigen::VectorBlock<const VectorX<T>> state_block =
        dynamic_cast<const BasicVector<T>&>(state_vector).get_value();

    // Obtain the block form of xcdot.
    VectorBase<T>* const derivatives_vector = derivatives->get_mutable_vector();
    DRAKE_ASSERT(derivatives_vector != nullptr);
    Eigen::VectorBlock<VectorX<T>> derivatives_block =
        dynamic_cast<BasicVector<T>&>(*derivatives_vector).get_mutable_value();

    // Delegate to subclass.
    DoCalcVectorTimeDerivatives(context, input_block, state_block,
                                &derivatives_block);
  }

  /// Converts the parameters to Eigen::VectorBlock form, then delegates to
  /// DoCalcVectorDiscreteVariableUpdates().
  void DoCalcDiscreteVariableUpdates(
      const Context<T>& context,
      const std::vector<const DiscreteUpdateEvent<T>*>&,
      DiscreteValues<T>* discrete_state) const final {
    // Short-circuit when there's no work to do.
    if (discrete_state->num_groups() == 0) {
      return;
    }

    // Obtain the block form of u (or the empty vector).
    const VectorX<T> empty_vector(0);
    const Eigen::VectorBlock<const VectorX<T>> input_block =
        (this->get_num_input_ports() > 0)
        ? this->EvalEigenVectorInput(context, 0)
        : Eigen::VectorBlock<const VectorX<T>>(empty_vector, 0, 0);

    // Obtain the block form of xd before the update (i.e., the prior state).
    DRAKE_ASSERT(context.has_only_discrete_state());
    const BasicVector<T>* const state_vector = context.get_discrete_state(0);
    DRAKE_ASSERT(state_vector != nullptr);
    const Eigen::VectorBlock<const VectorX<T>> state_block =
        state_vector->get_value();

    // Obtain the block form of xd after the update (i.e., the next state).
    DRAKE_ASSERT(discrete_state != nullptr);
    BasicVector<T>* const discrete_update_vector =
        discrete_state->get_mutable_vector();
    DRAKE_ASSERT(discrete_update_vector != nullptr);
    Eigen::VectorBlock<VectorX<T>> discrete_update_block =
        discrete_update_vector->get_mutable_value();

    // Delegate to subclass.
    DoCalcVectorDiscreteVariableUpdates(context, input_block, state_block,
                                        &discrete_update_block);
  }

  /// Converts the parameters to Eigen::VectorBlock form, then delegates to
  /// DoCalcVectorOutput().
  void CalcVectorOutput(const Context<T>& context,
                        BasicVector<T>* output) const {
    // Should only get here if we've declared an output.
    DRAKE_ASSERT(this->get_num_output_ports() > 0);

    // Obtain the block form of u (or the empty vector).
    const VectorX<T> empty_vector(0);
    const Eigen::VectorBlock<const VectorX<T>> input_block =
        (this->get_num_input_ports() > 0)
        ? this->EvalEigenVectorInput(context, 0)
        : Eigen::VectorBlock<const VectorX<T>>(empty_vector, 0, 0);

    // Obtain the block form of xc or xd[n].
    DRAKE_ASSERT(context.get_num_abstract_state_groups() == 0);
    const BasicVector<T>* state_vector{};
    if (context.get_num_discrete_state_groups() == 0) {
      const VectorBase<T>& vector_base = context.get_continuous_state_vector();
      state_vector = dynamic_cast<const BasicVector<T>*>(&vector_base);
    } else {
      DRAKE_ASSERT(context.has_only_discrete_state());
      state_vector = context.get_discrete_state(0);
    }
    DRAKE_DEMAND(state_vector != nullptr);
    const Eigen::VectorBlock<const VectorX<T>> state_block =
        state_vector->get_value();

    // Obtain the block form of y.
    Eigen::VectorBlock<VectorX<T>> output_block = output->get_mutable_value();

    // Delegate to subclass.
    DoCalcVectorOutput(context, input_block, state_block, &output_block);
  }

  /// Provides a convenience method for %VectorSystem subclasses.  This
  /// method performs the same logical operation as System::DoCalcOutput but
  /// provides VectorBlocks to represent the input, state, and output.
  /// Subclasses with outputs should override this method, and not the base
  /// class method (which is `final`).
  ///
  /// The @p state will be either empty, the continuous state, or the discrete
  /// state, depending on which (or none) was declared at context-creation
  /// time.
  ///
  /// By default, this function does nothing if the @p output is empty,
  /// and throws an exception otherwise.
  virtual void DoCalcVectorOutput(
      const Context<T>& context,
      const Eigen::VectorBlock<const VectorX<T>>& input,
      const Eigen::VectorBlock<const VectorX<T>>& state,
      Eigen::VectorBlock<VectorX<T>>* output) const {
    unused(context, input, state);
    DRAKE_THROW_UNLESS(output->size() == 0);
  }

  /// Provides a convenience method for %VectorSystem subclasses.  This
  /// method performs the same logical operation as
  /// System::DoCalcTimeDerivatives but provides VectorBlocks to represent the
  /// input, continuous state, and derivatives.  Subclasses should override
  /// this method, and not the base class method (which is `final`).
  ///
  /// The @p state will be either empty or the continuous state, depending on
  /// whether continuous state was declared at context-creation time.
  ///
  /// By default, this function does nothing if the @p derivatives are empty,
  /// and throws an exception otherwise.
  virtual void DoCalcVectorTimeDerivatives(
      const Context<T>& context,
      const Eigen::VectorBlock<const VectorX<T>>& input,
      const Eigen::VectorBlock<const VectorX<T>>& state,
      Eigen::VectorBlock<VectorX<T>>* derivatives) const {
    unused(context, input, state);
    DRAKE_THROW_UNLESS(derivatives->size() == 0);
  }

  /// Provides a convenience method for %VectorSystem subclasses.  This
  /// method performs the same logical operation as
  /// System::DoCalcDiscreteVariableUpdates but provides VectorBlocks to
  /// represent the input, discrete state, and discrete updates.  Subclasses
  /// should override this method, and not the base class method (which is
  /// `final`).
  ///
  /// The @p state will be either empty or the discrete state, depending on
  /// whether discrete state was declared at context-creation time.
  ///
  /// By default, this function does nothing if the @p next_state is
  /// empty, and throws an exception otherwise.
  virtual void DoCalcVectorDiscreteVariableUpdates(
      const Context<T>& context,
      const Eigen::VectorBlock<const VectorX<T>>& input,
      const Eigen::VectorBlock<const VectorX<T>>& state,
      Eigen::VectorBlock<VectorX<T>>* next_state) const {
    unused(context, input, state);
    DRAKE_THROW_UNLESS(next_state->size() == 0);
  }

 private:
  // All constructors should call this method immediately after invoking the
  // base class constructor, as if this were using constructor delegation.
  ///
  // We cannot use C++'s constructor delegation, because we need to invoke a
  // different LeafSystem constructor from each of our constructors.
  void DoConstructorBody(int input_size, int output_size) {
    if (input_size > 0) {
      this->DeclareInputPort(kVectorValued, input_size);
    }
    if (output_size > 0) {
      this->DeclareVectorOutputPort(BasicVector<T>(output_size),
                                    &VectorSystem::CalcVectorOutput);
    }
  }
};

}  // namespace systems
}  // namespace drake
