#pragma once

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <type_traits>
#include <utility>
#include <vector>

#include "drake/common/autodiff_overloads.h"
#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/drake_deprecated.h"
#include "drake/common/eigen_types.h"
#include "drake/common/number_traits.h"
#include "drake/systems/framework/abstract_values.h"
#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/continuous_state.h"
#include "drake/systems/framework/discrete_state.h"
#include "drake/systems/framework/leaf_context.h"
#include "drake/systems/framework/model_values.h"
#include "drake/systems/framework/sparsity_matrix.h"
#include "drake/systems/framework/system.h"
#include "drake/systems/framework/system_output.h"
#include "drake/systems/framework/value.h"
#include "drake/systems/framework/value_checker.h"

namespace drake {
namespace systems {

/// A token describing an event that recurs on a fixed period.
///
/// @tparam T The vector element type, which must be a valid Eigen scalar.
template <typename T>
struct PeriodicEvent {
  /// The period with which this event should recur.
  double period_sec{0.0};
  /// The time after zero when this event should first occur.
  double offset_sec{0.0};
  /// The action that should be taken when this event occurs.
  DiscreteEvent<T> event;
};

/// A superclass template that extends System with some convenience utilities
/// that are not applicable to Diagrams.
///
/// @tparam T The vector element type, which must be a valid Eigen scalar.
template <typename T>
class LeafSystem : public System<T> {
 public:
  // LeafSystem objects are neither copyable nor moveable.
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(LeafSystem)

  ~LeafSystem() override {}

  // =========================================================================
  // Implementations of System<T> methods.

  std::unique_ptr<Context<T>> AllocateContext() const override {
    std::unique_ptr<LeafContext<T>> context(new LeafContext<T>);
    // Reserve inputs that have already been declared.
    context->SetNumInputPorts(this->get_num_input_ports());
    // Reserve continuous state via delegation to subclass.
    context->set_continuous_state(this->AllocateContinuousState());
    // Reserve discrete state via delegation to subclass.
    context->set_discrete_state(this->AllocateDiscreteState());
    context->set_abstract_state(this->AllocateAbstractState());
    // Reserve parameters via delegation to subclass.
    context->set_parameters(this->AllocateParameters());

    // Enforce some requirements on the fully-assembled Context.
    // -- The continuous state must be contiguous, i.e., a valid BasicVector.
    //    (In general, a System's Context's continuous state can be any kind of
    //    VectorBase including scatter-gather implementations like Supervector.
    //    But for a LeafSystem with LeafContext, we only allow BasicVectors,
    //    which are guaranteed to have a linear storage layout.)  If the xc is
    //    not BasicVector, the dynamic_cast will yield nullptr, and the
    //    invariant-checker will complain.
    const VectorBase<T>* const xc = &context->get_continuous_state_vector();
    detail::CheckBasicVectorInvariants(dynamic_cast<const BasicVector<T>*>(xc));
    // -- The discrete state must all be valid BasicVectors.
    for (const BasicVector<T>* group :
             context->get_state().get_discrete_state()->get_data()) {
      detail::CheckBasicVectorInvariants(group);
    }
    // -- The numeric parameters must all be valid BasicVectors.
    const int num_numeric_parameters = context->num_numeric_parameters();
    for (int i = 0; i < num_numeric_parameters; ++i) {
      const BasicVector<T>* const group = context->get_numeric_parameter(i);
      detail::CheckBasicVectorInvariants(group);
    }
    // Note that the outputs are not part of the Context, but instead are
    // checked by LeafSystemOutput::add_port.

    return std::unique_ptr<Context<T>>(context.release());
  }

  /// Default implementation: set all continuous and discrete state variables
  /// to zero.  It makes no attempt to set abstract state values. Overrides
  /// must not change the number of state variables.
  void SetDefaultState(const Context<T>& context,
                       State<T>* state) const override {
    DRAKE_DEMAND(state != nullptr);
    ContinuousState<T>* xc = state->get_mutable_continuous_state();
    xc->SetFromVector(VectorX<T>::Zero(xc->size()));
    DiscreteState<T>* xd = state->get_mutable_discrete_state();
    for (int i = 0; i < xd->size(); i++) {
      BasicVector<T>* s = xd->get_mutable_discrete_state(i);
      s->SetFromVector(VectorX<T>::Zero(s->size()));
    }
  }

  /// Default implementation: set all numeric parameters to one.  It makes no
  /// attempt to set abstract parameter values. Overrides must not change the
  /// number of parameters.
  virtual void SetDefaultParameters(const LeafContext<T>& context,
                                    Parameters<T>* parameters) const {
    for (int i = 0; i < parameters->num_numeric_parameters(); i++) {
      BasicVector<T>* p = parameters->get_mutable_numeric_parameter(i);
      p->SetFromVector(VectorX<T>::Constant(p->size(), 1.0));
    }
  }

  // Sets Context fields to their default values.
  void SetDefaults(Context<T>* context) const final {
    systems::LeafContext<T>* leaf_context =
        dynamic_cast<systems::LeafContext<T>*>(context);
    DRAKE_DEMAND(leaf_context != nullptr);

    // Set the default state, checking that the number of state variables does
    // not change.
    const int n_xc = context->get_continuous_state()->size();
    const int n_xd = context->get_num_discrete_state_groups();
    const int n_xm = context->get_num_abstract_state_groups();

    SetDefaultState(*context, context->get_mutable_state());

    DRAKE_DEMAND(n_xc == context->get_continuous_state()->size());
    DRAKE_DEMAND(n_xd == context->get_num_discrete_state_groups());
    DRAKE_DEMAND(n_xm == context->get_num_abstract_state_groups());

    // Set the default parameters, checking that the number of parameters does
    // not change.
    const int num_params = leaf_context->num_numeric_parameters();
    SetDefaultParameters(*leaf_context,
                         &leaf_context->get_mutable_parameters());
    DRAKE_DEMAND(num_params == leaf_context->num_numeric_parameters());
  }

  std::unique_ptr<SystemOutput<T>> AllocateOutput(
      const Context<T>& context) const final {
    std::unique_ptr<LeafSystemOutput<T>> output(new LeafSystemOutput<T>);
    for (int i = 0; i < this->get_num_output_ports(); ++i) {
      const OutputPortDescriptor<T>& descriptor = this->get_output_port(i);
      if (descriptor.get_data_type() == kVectorValued) {
        output->add_port(std::make_unique<OutputPortValue>(
            AllocateOutputVector(descriptor)));
      } else {
        output->add_port(std::make_unique<OutputPortValue>(
            AllocateOutputAbstract(descriptor)));
      }
    }
    return std::unique_ptr<SystemOutput<T>>(output.release());
  }

  /// Returns the AllocateContinuousState value, which must not be nullptr.
  std::unique_ptr<ContinuousState<T>> AllocateTimeDerivatives() const override {
    return AllocateContinuousState();
  }

  /// Returns the AllocateDiscreteState value, which must not be nullptr.
  std::unique_ptr<DiscreteState<T>> AllocateDiscreteVariables()
      const override {
    return AllocateDiscreteState();
  }

  /// Returns to `true` if any of the inputs to the system is directly
  /// fed through to any of its outputs and `false` otherwise.
  bool HasAnyDirectFeedthrough() const final {
    auto sparsity = MakeSparsityMatrix();
    for (int i = 0; i < this->get_num_input_ports(); ++i) {
      for (int j = 0; j < this->get_num_output_ports(); ++j) {
        if (DoHasDirectFeedthrough(sparsity.get(), i, j)) {
          return true;
        }
      }
    }
    return false;
  }

  /// Returns true if there is direct-feedthrough from any input port to the
  /// given @p output_port, and false otherwise.
  bool HasDirectFeedthrough(int output_port) const final {
    auto sparsity = MakeSparsityMatrix();
    for (int i = 0; i < this->get_num_input_ports(); ++i) {
      if (DoHasDirectFeedthrough(sparsity.get(), i, output_port)) {
        return true;
      }
    }
    return false;
  }

  /// Returns true if there is direct-feedthrough from the given @p input_port
  /// to the given @p output_port, and false otherwise.
  bool HasDirectFeedthrough(int input_port, int output_port) const final {
    auto sparsity = MakeSparsityMatrix();
    return DoHasDirectFeedthrough(sparsity.get(), input_port, output_port);
  }

 protected:
  LeafSystem() {}

  // =========================================================================
  // Implementations of System<T> methods.

  /// Computes the next update time based on the configured periodic events, for
  /// scalar types that are arithmetic, or aborts for scalar types that are not
  /// arithmetic. Subclasses that require aperiodic events should override.
  void DoCalcNextUpdateTime(const Context<T>& context,
                            UpdateActions<T>* events) const override {
    DoCalcNextUpdateTimeImpl(context, events);
  }

  /// Allocates a vector that is suitable as an input value for @p descriptor.
  /// The default implementation in this class either clones the model_vector
  /// (if the port was declared via DeclareVectorInputPort) or else allocates a
  /// BasicVector (if the port was declared via DeclareInputPort(kVectorValued,
  /// size).  Subclasses can override this method if the default behavior is
  /// not sufficient.
  BasicVector<T>* DoAllocateInputVector(
      const InputPortDescriptor<T>& descriptor) const override {
    std::unique_ptr<BasicVector<T>> model_result =
        model_input_values_.CloneVectorModel<T>(descriptor.get_index());
    if (model_result) {
      return model_result.release();
    }
    return new BasicVector<T>(descriptor.size());
  }

  /// Allocates a vector that is suitable as an input value for @p descriptor.
  /// The default implementation in this class either clones the model_value
  /// (if the port was declared via DeclareAbstractInputPort) or else aborts.
  ///
  /// Subclasses with abstract input ports must either provide a model_value
  /// when declaring the port, or else override this method.
  AbstractValue* DoAllocateInputAbstract(
      const InputPortDescriptor<T>& descriptor) const override {
    std::unique_ptr<AbstractValue> model_result =
        model_input_values_.CloneModel(descriptor.get_index());
    if (model_result) {
      return model_result.release();
    }
    DRAKE_ABORT_MSG("A concrete leaf system with abstract input ports should "
                    "pass a model_value to DeclareAbstractInputPort, or else "
                    "must override DoAllocateInputAbstract");
  }

  // =========================================================================
  // New methods for subclasses to override

  /// Returns a ContinuousState used to implement both CreateDefaultContext and
  /// AllocateTimeDerivatives. Allocates the state configured with
  /// DeclareContinuousState, or none by default. Systems with continuous state
  /// variables may override, but must ensure the ContinuousState vector is
  /// a subclass of BasicVector.
  virtual std::unique_ptr<ContinuousState<T>> AllocateContinuousState() const {
    if (model_continuous_state_vector_ != nullptr) {
      return std::make_unique<ContinuousState<T>>(
          model_continuous_state_vector_->Clone(), num_generalized_positions_,
          num_generalized_velocities_, num_misc_continuous_states_);
    }
    return std::make_unique<ContinuousState<T>>();
  }

  /// Reserves the discrete state as required by CreateDefaultContext. By
  /// default, reserves no state. Systems with discrete state should override.
  virtual std::unique_ptr<DiscreteState<T>> AllocateDiscreteState() const {
    if (model_discrete_state_vector_ != nullptr) {
      return std::make_unique<DiscreteState<T>>(
          model_discrete_state_vector_->Clone());
    }
    return std::make_unique<DiscreteState<T>>();
  }

  /// Reserves the abstract state as required by CreateDefaultContext. By
  /// default, reserves no state. Systems with abstract state should override.
  virtual std::unique_ptr<AbstractValues> AllocateAbstractState() const {
    return std::make_unique<AbstractValues>();
  }

  /// Reserves the parameters as required by CreateDefaultContext. By default,
  /// reserves no parameters. Systems with parameters should override.
  virtual std::unique_ptr<Parameters<T>> AllocateParameters() const {
    return std::make_unique<Parameters<T>>();
  }

  /// Given a port descriptor, allocates the vector storage.  The default
  /// implementation in this class either clones the model_vector (if the port
  /// was declared via DeclareVectorOutputPort) or else allocates a BasicVector
  /// (if the port was declared via DeclareOutputPort(kVectorValued, size).
  /// Subclasses can override this method if the default behavior is not
  /// sufficient.
  ///
  /// The descriptor must match a port declared via DeclareOutputPort or one of
  /// its wrappers, e.g., DeclareVectorOutputPort.
  virtual std::unique_ptr<BasicVector<T>> AllocateOutputVector(
      const OutputPortDescriptor<T>& descriptor) const {
    std::unique_ptr<BasicVector<T>> model_result =
        model_output_values_.CloneVectorModel<T>(descriptor.get_index());
    if (model_result) {
      return model_result;
    }
    return std::make_unique<BasicVector<T>>(descriptor.size());
  }

  /// Given a port descriptor, allocates the abstract storage.  The default
  /// implementation in this class either clones the model_value (if the port
  /// was declared via DeclareAbstractOutputPort) or aborts.
  ///
  /// Subclasses with abstract output ports must either provide a model_value
  /// when declaring the port, or else override this method.
  ///
  /// The descriptor must match a port declared via DeclareOutputPort or one of
  /// its wrappers, e.g., DeclareAbstractOutputPort.
  virtual std::unique_ptr<AbstractValue> AllocateOutputAbstract(
      const OutputPortDescriptor<T>& descriptor) const {
    std::unique_ptr<AbstractValue> model_result =
        model_output_values_.CloneModel(descriptor.get_index());
    if (model_result) {
      return model_result;
    }
    DRAKE_ABORT_MSG("A concrete leaf system with abstract output ports must "
                    "override AllocateOutputAbstract.");
  }

  /// Returns true if there is direct-feedthrough from the given @p input_port
  /// to the given @p output_port, and false otherwise, according to the given
  /// @p sparsity.
  ///
  /// If @p sparsity is nullptr, returns true, so that by default we assume
  /// there is direct feedthrough of values from every input to every output.
  /// This is a conservative assumption that ensures we detect and can prevent
  /// the formation of algebraic loops (implicit computations) in system
  /// Diagrams. Systems which do not have direct feedthrough may override that
  /// assumption in two ways:
  ///
  /// - Override DoToSymbolic, allowing this function to infer the sparsity
  ///   from the symbolic equations. This method is typically preferred for
  ///   systems that have a symbolic form, but should be avoided in certain
  ///   corner cases where fully descriptive symbolic analysis is impossible,
  ///   e.g. when the symbolic form depends on C++ native conditionals. For
  ///   additional discussion, consult the documentation for SparsityMatrix.
  ///
  /// - Override this function directly, reporting manual sparsity. This method
  ///   is recommended when ToSymbolic has not been implemented, or when its
  ///   output is not fully descriptive, as discussed above. Manually configured
  ///   sparsity must be conservative: if there is any Context for which an
  ///   input port is direct-feedthrough to an output port, this function must
  ///   return true for those two ports.
  virtual bool DoHasDirectFeedthrough(const SparsityMatrix* sparsity,
                                      int input_port,
                                      int output_port) const {
    DRAKE_ASSERT(input_port >= 0);
    DRAKE_ASSERT(input_port < this->get_num_input_ports());
    DRAKE_ASSERT(output_port >= 0);
    DRAKE_ASSERT(output_port < this->get_num_output_ports());

    // If this System has manually declared that it has no direct-feedthrough
    // by overriding the deprecated API has_any_direct_feedthrough, accept
    // that override and skip sparsity analysis.
    // TODO(david-german-tri): Remove overrides of has_any_direct_feedthrough,
    // then remove this check.
    if (!this->has_any_direct_feedthrough()) {
      return false;
    }

    // If no symbolic sparsity matrix is available, assume direct feedthrough
    // by default.
    if (sparsity == nullptr) {
      return true;
    }

    return sparsity->IsConnectedInputToOutput(input_port, output_port);
  }

  // =========================================================================
  // New methods for subclasses to use

  /// Extracts the numeric parameters of type U from the @p context at @p index.
  /// Asserts if the context is not a LeafContext, or if it does not have a
  /// vector-valued parameter of type U at @p index.
  template <template <typename> class U = BasicVector>
  const U<T>& GetNumericParameter(const Context<T>& context, int index) const {
    static_assert(std::is_base_of<BasicVector<T>, U<T>>::value,
                  "U must be a subclass of BasicVector.");
    const systems::LeafContext<T>& leaf_context =
        dynamic_cast<const systems::LeafContext<T>&>(context);
    const auto* const params =
        dynamic_cast<const U<T>*>(leaf_context.get_numeric_parameter(index));
    DRAKE_ASSERT(params != nullptr);
    return *params;
  }

  /// Declares that this System has a simple, fixed-period discrete action.
  /// The first tick will be at t = period_sec, and it will recur at every
  /// period_sec thereafter. On the discrete tick, the system may perform
  /// the given type of action.
  void DeclarePeriodicAction(double period_sec, double offset_sec,
      const typename DiscreteEvent<T>::ActionType& action) {
    PeriodicEvent<T> event;
    event.period_sec = period_sec;
    event.offset_sec = offset_sec;
    event.event.action = action;
    periodic_events_.push_back(event);
  }

  /// Declares that this System has a simple, fixed-period discrete update.
  /// The first tick will be at t = period_sec, and it will recur at every
  /// period_sec thereafter. On the discrete tick, the system may update
  /// the discrete state.
  void DeclareDiscreteUpdatePeriodSec(double period_sec) {
    DeclarePeriodicAction(period_sec, 0.0,
        DiscreteEvent<T>::kDiscreteUpdateAction);
  }

  /// Declares that this System has a simple, fixed-period discrete update.
  /// The first tick will be at t = offset_sec, and it will recur at every
  /// period_sec thereafter. On the discrete tick, the system may update the
  /// discrete state.
  void DeclarePeriodicDiscreteUpdate(double period_sec, double offset_sec) {
    DeclarePeriodicAction(period_sec, offset_sec,
        DiscreteEvent<T>::kDiscreteUpdateAction);
  }

  /// Declares that this System has a simple, fixed-period unrestricted state
  /// update. The first tick will be at t = offset_sec, and it will recur at
  /// every period_sec thereafter. On the discrete tick, the system may perform
  /// unrestricted updates.
  void DeclarePeriodicUnrestrictedUpdate(double period_sec, double offset_sec) {
    DeclarePeriodicAction(period_sec, offset_sec,
        DiscreteEvent<T>::kUnrestrictedUpdateAction);
  }

  /// Declares that this System has a simple, fixed-period publish.
  /// The first tick will be at t = period_sec, and it will recur at every
  /// period_sec thereafter. On the discrete tick, the system may update
  /// the discrete state.
  void DeclarePublishPeriodSec(double period_sec) {
    DeclarePeriodicAction(period_sec, 0, DiscreteEvent<T>::kPublishAction);
  }

  /// Declares that this System should reserve continuous state with
  /// @p num_state_variables state variables, which have no second-order
  /// structure. Has no effect if AllocateContinuousState is overridden.
  void DeclareContinuousState(int num_state_variables) {
    const int num_q = 0, num_v = 0;
    DeclareContinuousState(num_q, num_v, num_state_variables);
  }

  /// Declares that this System should reserve continuous state with @p num_q
  /// generalized positions, @p num_v generalized velocities, and @p num_z
  /// miscellaneous state variables.  Has no effect if AllocateContinuousState
  /// is overridden.
  void DeclareContinuousState(int num_q, int num_v, int num_z) {
    const int n = num_q + num_v + num_z;
    DeclareContinuousState(BasicVector<T>(n), num_q, num_v, num_z);
  }

  /// Declares that this System should reserve continuous state with
  /// @p model_vector.size() miscellaneous state variables, stored in a
  /// vector Cloned from @p model_vector.  Has no effect if
  /// AllocateContinuousState is overridden.
  void DeclareContinuousState(const BasicVector<T>& model_vector) {
    const int num_q = 0, num_v = 0;
    const int num_z = model_vector.size();
    DeclareContinuousState(model_vector, num_q, num_v, num_z);
  }

  /// Declares that this System should reserve continuous state with @p num_q
  /// generalized positions, @p num_v generalized velocities, and @p num_z
  /// miscellaneous state variables, stored in a vector Cloned from
  /// @p model_vector. Aborts if @p model_vector has the wrong size. Has no
  /// effect if AllocateContinuousState is overridden.
  void DeclareContinuousState(const BasicVector<T>& model_vector,
                              int num_q, int num_v, int num_z) {
    DRAKE_DEMAND(model_vector.size() == num_q + num_v + num_z);
    model_continuous_state_vector_ = model_vector.Clone();
    num_generalized_positions_ = num_q;
    num_generalized_velocities_ = num_v;
    num_misc_continuous_states_ = num_z;
  }

  /// Declares that this System should reserve continuous state with @p num_q
  /// generalized positions, @p num_v generalized velocities, and @p num_z
  /// miscellaneous state variables, stored in the a vector Cloned from
  /// @p model_vector. Aborts if @p model_vector is nullptr or has the wrong
  /// size. Has no effect if AllocateContinuousState is overridden.
  DRAKE_DEPRECATED("Use the const-reference model_vector overload instead")
  void DeclareContinuousState(std::unique_ptr<BasicVector<T>> model_vector,
                              int num_q, int num_v, int num_z) {
    DRAKE_DEMAND(model_vector != nullptr);
    DeclareContinuousState(*model_vector, num_q, num_v, num_z);
  }

  /// Declares that this System should reserve discrete state with
  /// @p num_state_variables state variables. Has no effect if
  /// AllocateDiscreteState is overridden.
  void DeclareDiscreteState(int num_state_variables) {
    model_discrete_state_vector_ =
        std::make_unique<BasicVector<T>>(num_state_variables);
  }

  /// Declares a vector-valued input port using the given @p model_vector.
  /// This is the best way to declare LeafSystem input ports that require
  /// subclasses of BasicVector.  The port's size will be model_vector.size(),
  /// and LeafSystem's default implementation of DoAllocateInputVector will be
  /// model_vector.Clone().
  const InputPortDescriptor<T>& DeclareVectorInputPort(
      const BasicVector<T>& model_vector) {
    const int size = model_vector.size();
    const int next_index = this->get_num_input_ports();
    model_input_values_.AddVectorModel(next_index, model_vector.Clone());
    return this->DeclareInputPort(kVectorValued, size);
  }

  // Avoid shadowing out the no-arg DeclareAbstractInputPort().
  using System<T>::DeclareAbstractInputPort;

  /// Declares an abstract-valued input port using the given @p model_value.
  /// This is the best way to declare LeafSystem abstract input ports.
  /// LeafSystem's default implementation of DoAllocateInputAbstract will be
  /// model_value.Clone().
  const InputPortDescriptor<T>& DeclareAbstractInputPort(
      const AbstractValue& model_value) {
    const int next_index = this->get_num_input_ports();
    model_input_values_.AddModel(next_index, model_value.Clone());
    return this->DeclareAbstractInputPort();
  }

  /// Declares a vector-valued output port using the given @p model_vector.
  /// This is the best way to declare LeafSystem output ports that require
  /// subclasses of BasicVector.  The port's size will be model_vector.size(),
  /// and LeafSystem's default implementation of DoAllocateOutputVector will be
  /// model_vector.Clone().
  const OutputPortDescriptor<T>& DeclareVectorOutputPort(
      const BasicVector<T>& model_vector) {
    const int size = model_vector.size();
    const int next_index = this->get_num_output_ports();
    model_output_values_.AddVectorModel(next_index, model_vector.Clone());
    return this->DeclareOutputPort(kVectorValued, size);
  }

  // Avoid shadowing out the no-arg DeclareAbstractOutputPort().
  using System<T>::DeclareAbstractOutputPort;

  /// Declares an abstract-valued output port using the given @p model_value.
  /// This is the best way to declare LeafSystem abstract output ports.
  /// LeafSystem's default implementation of DoAllocateOutputAbstract will be
  /// model_value.Clone().
  const OutputPortDescriptor<T>& DeclareAbstractOutputPort(
      const AbstractValue& model_value) {
    const int next_index = this->get_num_output_ports();
    model_output_values_.AddModel(next_index, model_value.Clone());
    return this->DeclareAbstractOutputPort();
  }

 private:
  // Aborts for scalar types that are not numeric, since there is no reasonable
  // definition of "next update time" outside of the real line.
  //
  // @tparam T1 SFINAE boilerplate for the scalar type. Do not set.
  template <typename T1 = T>
  typename std::enable_if<!is_numeric<T1>::value>::type
  DoCalcNextUpdateTimeImpl(const Context<T1>& context,
                           UpdateActions<T1>* events) const {
    DRAKE_ABORT_MSG(
        "The default implementation of LeafSystem<T>::DoCalcNextUpdateTime "
        "only works with types that are drake::is_numeric.");
  }

  // Computes the next update time across all the scheduled events, for
  // scalar types that are numeric.
  //
  // @tparam T1 SFINAE boilerplate for the scalar type. Do not set.
  template <typename T1 = T>
  typename std::enable_if<is_numeric<T1>::value>::type DoCalcNextUpdateTimeImpl(
      const Context<T1>& context, UpdateActions<T1>* actions) const {
    T1 min_time = std::numeric_limits<double>::infinity();
    if (periodic_events_.empty()) {
      // No discrete update.
      actions->time = min_time;
      return;
    }

    // Find the minimum next sample time across all registered events, and
    // the set of registered events that will occur at that time.
    std::vector<const PeriodicEvent<T>*> next_events;
    for (const PeriodicEvent<T>& event : periodic_events_) {
      T1 t = GetNextSampleTime(event, context.get_time());
      if (t < min_time) {
        min_time = t;
        next_events = {&event};
      } else if (t == min_time) {
        next_events.push_back(&event);
      }
    }

    // Write out the events that fire at min_time.
    actions->time = min_time;
    for (const PeriodicEvent<T>* event : next_events) {
      actions->events.push_back(event->event);
    }
  }

  // Returns the next sample time for the given @p event.
  static T GetNextSampleTime(const PeriodicEvent<T>& event,
                             const T& current_time_sec) {
    const double period = event.period_sec;
    DRAKE_ASSERT(period > 0);
    const double offset = event.offset_sec;
    DRAKE_ASSERT(offset >= 0);

    // If the first sample time hasn't arrived yet, then that is the next
    // sample time.
    if (current_time_sec < offset) {
      return offset;
    }

    // NOLINTNEXTLINE(build/namespaces): Needed for ADL of floor and ceil.
    using namespace std;

    // Compute the index in the sequence of samples for the next time to sample,
    // which should be greater than the present time.
    const T offset_time = current_time_sec - offset;
    const int64_t next_k = static_cast<int64_t>(ceil(offset_time / period));
    T next_t = offset + next_k * period;
    if (next_t <= current_time_sec) {
      next_t = offset + (next_k + 1) * period;
    }
    DRAKE_ASSERT(next_t > current_time_sec);
    return next_t;
  }

  /// Returns a SparsityMatrix for this system, or nullptr if a SparsityMatrix
  /// cannot be constructed because this System has no symbolic representation.
  std::unique_ptr<SparsityMatrix> MakeSparsityMatrix() const {
    std::unique_ptr<System<symbolic::Expression>> symbolic_system =
        this->ToSymbolic();
    if (symbolic_system) {
      return std::make_unique<SparsityMatrix>(*symbolic_system);
    } else {
      return nullptr;
    }
  }

  // Periodic Update or Publish events registered on this system.
  std::vector<PeriodicEvent<T>> periodic_events_;

  // A model continuous state to be used in AllocateDefaultContext.
  std::unique_ptr<BasicVector<T>> model_continuous_state_vector_;
  int num_generalized_positions_{0};
  int num_generalized_velocities_{0};
  int num_misc_continuous_states_{0};

  // A model discrete state to be used in AllocateDefaultContext.
  std::unique_ptr<BasicVector<T>> model_discrete_state_vector_;

  // Model inputs to be used in AllocateOutput{Vector,Abstract}.
  detail::ModelValues model_input_values_;

  // Model outputs to be used in AllocateOutput{Vector,Abstract}.
  detail::ModelValues model_output_values_;
};

}  // namespace systems
}  // namespace drake
