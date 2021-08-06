#pragma once

#include <memory>
#include <optional>
#include <set>
#include <utility>
#include <vector>

#include "drake/common/default_scalars.h"
#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/drake_throw.h"
#include "drake/common/eigen_types.h"
#include "drake/common/never_destroyed.h"
#include "drake/common/unused.h"
#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace systems {

/// A base class that specializes LeafSystem for use with only zero or one
/// vector input ports, and only zero or one vector output ports.
///
/// By default, this base class does not declare any state; subclasses may
/// optionally declare continuous or discrete state, but not both; subclasses
/// may not declare abstract state.
///
/// @tparam_default_scalar
template <typename T>
class VectorSystem : public LeafSystem<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(VectorSystem)

  ~VectorSystem() override = default;

 protected:
  /// Creates a system with one input port and one output port of the given
  /// sizes, when the sizes are non-zero.  Either size can be zero, in which
  /// case no input (or output) port is created.
  ///
  /// The `direct_feedthrough` specifies whether the input port direct feeds
  /// through to the output port.  (See SystemBase::GetDirectFeedthroughs().)
  /// When not provided, assumes true (the output is direct feedthrough).
  /// When false, the DoCalcVectorOutput `input` will be empty (zero-sized).
  ///
  /// Does *not* declare scalar-type conversion support (AutoDiff, etc.).  To
  /// enable AutoDiff support, use the SystemScalarConverter-based constructor.
  /// (For that, see @ref system_scalar_conversion at the example titled
  /// "Example using drake::systems::VectorSystem as the base class".)
  VectorSystem(int input_size, int output_size,
               std::optional<bool> direct_feedthrough = std::nullopt)
      : VectorSystem(SystemScalarConverter{}, input_size, output_size,
                     direct_feedthrough) {}

  /// Creates a system with one input port and one output port of the given
  /// sizes, when the sizes are non-zero.  Either size can be zero, in which
  /// case no input (or output) port is created.  This constructor allows
  /// subclasses to declare scalar-type conversion support (AutoDiff, etc.).
  ///
  /// The `direct_feedthrough` specifies whether the input port direct feeds
  /// through to the output port.  (See SystemBase::GetDirectFeedthroughs().)
  /// When not provided, infers feedthrough from the symbolic form if
  /// available, or else assumes true (the output is direct feedthrough).
  /// When false, the DoCalcVectorOutput `input` will be empty (zero-sized).
  ///
  /// The scalar-type conversion support will use @p converter.
  /// To enable scalar-type conversion support, pass a `SystemTypeTag<S>{}`
  /// where `S` must be the exact class of `this` being constructed.
  ///
  /// See @ref system_scalar_conversion for detailed background and examples
  /// related to scalar-type conversion support, especially the example titled
  /// "Example using drake::systems::VectorSystem as the base class".
  VectorSystem(SystemScalarConverter converter, int input_size, int output_size,
               std::optional<bool> direct_feedthrough = std::nullopt)
      : LeafSystem<T>(std::move(converter)) {
    if (input_size > 0) {
      this->DeclareInputPort(kUseDefaultName, kVectorValued, input_size);
    }
    if (output_size > 0) {
      std::set<DependencyTicket> prerequisites_of_calc;
      if (direct_feedthrough.value_or(true)) {
        // Depend on everything.
        prerequisites_of_calc = {
          this->all_sources_ticket()
        };
      } else {
        // Depend on everything *except* for the inputs.
        prerequisites_of_calc = {
          this->time_ticket(),
          this->accuracy_ticket(),
          this->all_state_ticket(),
          this->all_parameters_ticket(),
        };
      }
      this->DeclareVectorOutputPort(
          kUseDefaultName, output_size,
          &VectorSystem::CalcVectorOutput, std::move(prerequisites_of_calc));
    }
  }

  /// Causes the vector-valued input port to become up-to-date, and returns
  /// the port's value as an %Eigen vector.  If the system has zero inputs,
  /// then returns an empty vector.
  Eigen::VectorBlock<const VectorX<T>> EvalVectorInput(
      const Context<T>& context) const {
    // Obtain the block form of u (or the empty vector).
    if (this->num_input_ports() > 0) {
      return this->get_input_port().Eval(context);
    }
    static const never_destroyed<VectorX<T>> empty_vector(0);
    return empty_vector.access().segment(0, 0);
  }

  /// Returns a reference to an %Eigen vector version of the state from within
  /// the %Context.
  Eigen::VectorBlock<const VectorX<T>> GetVectorState(
      const Context<T>& context) const {
    // Obtain the block form of xc or xd.
    DRAKE_ASSERT(context.num_abstract_states() == 0);
    const BasicVector<T>* state_vector{};
    if (context.num_discrete_state_groups() == 0) {
      const VectorBase<T>& vector_base = context.get_continuous_state_vector();
      state_vector = dynamic_cast<const BasicVector<T>*>(&vector_base);
    } else {
      DRAKE_ASSERT(context.has_only_discrete_state());
      state_vector = &context.get_discrete_state(0);
    }
    DRAKE_DEMAND(state_vector != nullptr);
    return state_vector->get_value();
  }

  /// Converts the parameters to Eigen::VectorBlock form, then delegates to
  /// DoCalcVectorTimeDerivatives().
  void DoCalcTimeDerivatives(const Context<T>& context,
                             ContinuousState<T>* derivatives) const final {
    // Short-circuit when there's no work to do.
    if (derivatives->size() == 0) {
      return;
    }

    const Eigen::VectorBlock<const VectorX<T>> input_block =
        EvalVectorInput(context);

    // Obtain the block form of xc.
    DRAKE_ASSERT(context.has_only_continuous_state());
    const VectorBase<T>& state_vector = context.get_continuous_state_vector();
    const Eigen::VectorBlock<const VectorX<T>> state_block =
        dynamic_cast<const BasicVector<T>&>(state_vector).get_value();

    // Obtain the block form of xcdot.
    VectorBase<T>& derivatives_vector = derivatives->get_mutable_vector();
    Eigen::VectorBlock<VectorX<T>> derivatives_block =
        dynamic_cast<BasicVector<T>&>(derivatives_vector).get_mutable_value();

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

    const Eigen::VectorBlock<const VectorX<T>> input_block =
        EvalVectorInput(context);

    // Obtain the block form of xd before the update (i.e., the prior state).
    DRAKE_ASSERT(context.has_only_discrete_state());
    const BasicVector<T>& state_vector = context.get_discrete_state(0);
    const Eigen::VectorBlock<const VectorX<T>> state_block =
        state_vector.get_value();

    // Obtain the block form of xd after the update (i.e., the next state).
    DRAKE_ASSERT(discrete_state != nullptr);
    Eigen::VectorBlock<VectorX<T>> discrete_update_block =
        discrete_state->get_mutable_value();

    // Delegate to subclass.
    DoCalcVectorDiscreteVariableUpdates(context, input_block, state_block,
                                        &discrete_update_block);
  }

  /// Converts the parameters to Eigen::VectorBlock form, then delegates to
  /// DoCalcVectorOutput().
  void CalcVectorOutput(const Context<T>& context,
                        BasicVector<T>* output) const {
    // Should only get here if we've declared an output.
    DRAKE_ASSERT(this->num_output_ports() > 0);

    // Decide whether we should evaluate our input port and pass its value to
    // our subclass's DoCalcVectorOutput method.  When should_eval_input is
    // false, we will pass an empty vector instead of pulling on our input.
    bool should_eval_input = false;
    if (this->num_input_ports() > 0) {
      // We have an input port, but when our subclass's DoCalcVectorOutput is
      // not direct-feedthrough, then evaluating the input port might cause a
      // computational loop.  We should only evaluate the input when this
      // System is declared to be direct-feedthrough (i.e., by asking a System
      // base class method such as HasAnyDirectFeedthrough).
      //
      // However, there is a Catch-22: our LeafSystem base class contains a
      // default implementation of feedthrough reporting that uses the
      // SystemSymbolicInspector to infer sparsity.  The inspector fixes the
      // input port to be a symbolic Variable, and then evaluates the output.
      // If during that output calculation, this method *again* asked to
      // compute the feedthrough, we would re-enter the inspection code and
      // cause infinite recursion.  We would have a Calc -> HasAny -> Calc ...
      // infinite loop.
      //
      // To break that recursion, we avoid HasAnyDirectFeedthrough when our
      // scalar type is a symbolic expression and the input port is fixed.  We
      // know the inspector must always used a fixed input port (it has no
      // diagram that it could use), so this will always bottom out the
      // recursion.  Even if not being evaluated by the symbolic inspector, if
      // the input port is fixed to a symbolic expression then it is no harm to
      // evaluate the input, even if the system is not supposed to have
      // feedthrough -- it is merely providing extra ignored data to the
      // DoCalcVectorOutput helper.
      constexpr bool is_symbolic = std::is_same_v<T, symbolic::Expression>;
      const bool is_fixed_input =
          (context.MaybeGetFixedInputPortValue(0) != nullptr);
      if (is_symbolic && is_fixed_input) {
        should_eval_input = true;
      } else {
        should_eval_input = this->HasAnyDirectFeedthrough();
      }
    }

    // Only provide input when direct feedthrough occurs; otherwise, we might
    // create a computational loop.
    static const never_destroyed<VectorX<T>> empty_vector(0);
    Eigen::VectorBlock<const VectorX<T>> input_block =
        should_eval_input ? EvalVectorInput(context) :
        empty_vector.access().segment(0, 0);

    // Obtain the block form of xc or xd.
    const Eigen::VectorBlock<const VectorX<T>> state_block =
        GetVectorState(context);

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
  /// The @p input will be empty (zero-sized) when this System is declared to
  /// be non-direct-feedthrough.
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
  // Confirms the VectorSystem invariants when allocating the context.
  void DoValidateAllocatedLeafContext(
      const LeafContext<T>& context) const final {
    // N.B. The DRAKE_THROW_UNLESS conditions can be triggered by subclass
    // mistakes, so are part of our unit tests.  The DRAKE_DEMAND conditions
    // should be invariants guaranteed by the framework, so are asserted.

    // Exactly one input and output.
    DRAKE_THROW_UNLESS(this->num_input_ports() <= 1);
    DRAKE_THROW_UNLESS(this->num_output_ports() <= 1);
    DRAKE_DEMAND(context.num_input_ports() <= 1);

    // At most one of either continuous or discrete state.
    DRAKE_THROW_UNLESS(context.num_abstract_states() == 0);
    const int continuous_size = context.num_continuous_states();
    const int num_discrete_groups = context.num_discrete_state_groups();
    DRAKE_DEMAND(continuous_size >= 0);
    DRAKE_DEMAND(num_discrete_groups >= 0);
    DRAKE_THROW_UNLESS(num_discrete_groups <= 1);
    DRAKE_THROW_UNLESS((continuous_size == 0) || (num_discrete_groups == 0));
  }
};

}  // namespace systems
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::systems::VectorSystem)
