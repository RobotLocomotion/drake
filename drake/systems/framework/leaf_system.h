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
#include "drake/common/unused.h"
#include "drake/systems/framework/abstract_values.h"
#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/continuous_state.h"
#include "drake/systems/framework/discrete_values.h"
#include "drake/systems/framework/leaf_context.h"
#include "drake/systems/framework/leaf_output_port.h"
#include "drake/systems/framework/model_values.h"
#include "drake/systems/framework/output_port_value.h"
#include "drake/systems/framework/sparsity_matrix.h"
#include "drake/systems/framework/system.h"
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
    std::unique_ptr<LeafContext<T>> context = DoMakeContext();
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

    return std::move(context);
  }

  /// Default implementation: sets all continuous and discrete state variables
  /// to zero.  It makes no attempt to set abstract state values. Overrides
  /// must not change the number of state variables.
  void SetDefaultState(const Context<T>& context,
                       State<T>* state) const override {
    unused(context);
    DRAKE_DEMAND(state != nullptr);
    ContinuousState<T>* xc = state->get_mutable_continuous_state();
    xc->SetFromVector(VectorX<T>::Zero(xc->size()));
    DiscreteValues<T>* xd = state->get_mutable_discrete_state();
    for (int i = 0; i < xd->num_groups(); i++) {
      BasicVector<T>* s = xd->get_mutable_vector(i);
      s->SetFromVector(VectorX<T>::Zero(s->size()));
    }
  }

  /// Default implementation: sets all numeric parameters to the model vector
  /// given to DeclareNumericParameter, or else if no model was provided sets
  /// the numeric parameter to one.  It makes no attempt to set abstract
  /// parameter values.  Overrides must not change the number of parameters.
  virtual void SetDefaultParameters(const LeafContext<T>& context,
                                    Parameters<T>* parameters) const {
    unused(context);
    for (int i = 0; i < parameters->num_numeric_parameters(); i++) {
      BasicVector<T>* p = parameters->get_mutable_numeric_parameter(i);
      auto model_vector = model_numeric_parameters_.CloneVectorModel<T>(i);
      if (model_vector != nullptr) {
        p->SetFrom(*model_vector);
      } else {
        p->SetFromVector(VectorX<T>::Constant(p->size(), 1.0));
      }
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
    const int n_xa = context->get_num_abstract_state_groups();

    SetDefaultState(*context, context->get_mutable_state());

    DRAKE_DEMAND(n_xc == context->get_continuous_state()->size());
    DRAKE_DEMAND(n_xd == context->get_num_discrete_state_groups());
    DRAKE_DEMAND(n_xa == context->get_num_abstract_state_groups());

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
      const OutputPort<T>& port = this->get_output_port(i);
      output->add_port(std::make_unique<OutputPortValue>(
          port.Allocate(context)));
    }
    return std::move(output);
  }

  /// Returns the AllocateContinuousState value, which must not be nullptr.
  std::unique_ptr<ContinuousState<T>> AllocateTimeDerivatives() const override {
    return AllocateContinuousState();
  }

  /// Returns the AllocateDiscreteState value, which must not be nullptr.
  std::unique_ptr<DiscreteValues<T>> AllocateDiscreteVariables()
      const override {
    return AllocateDiscreteState();
  }

  /// Returns `true` if any of the inputs to the system is directly
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

  /// Provides a new instance of the leaf context for this system. Derived
  /// leaf systems with custom derived leaf system contexts should override this
  /// to provide a context of the appropriate type. The returned context should
  /// be "empty"; invoked by AllocateContext(), the caller will take the
  /// responsibility to initialize the core LeafContext data.
  // TODO(SeanCurtis-TRI): This currently assumes that derived LeafContext
  // classes do *not* add new data members. If that changes, e.g., with the
  // advent of the cache, this documentation should be changed to include the
  // initialization of the sub-class's *unique* data members.
  virtual std::unique_ptr<LeafContext<T>> DoMakeContext() const {
    return std::make_unique<LeafContext<T>>();
  }

  /// Returns the per step events declared through DeclarePerStepAction().
  const std::vector<DiscreteEvent<T>>& get_per_step_events() const {
    return per_step_events_;
  }

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

  /// Allocates an AbstractValue suitable as an input value for @p descriptor.
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


  /// Emits a graphviz fragment for this System. Leaf systems are visualized as
  /// records. For instance, a leaf system with 2 inputs and 1 output is:
  ///
  /// @verbatim
  /// 123456 [shape= record, label="name | {<u0> 0 |<y0> 0} | {<u1> 1 | }"];
  /// @endverbatim
  ///
  /// which looks like:
  ///
  /// @verbatim
  /// +------------+----+
  /// | name  | u0 | u1 |
  /// |       | y0 |    |
  /// +-------+----+----+
  /// @endverbatim
  void GetGraphvizFragment(std::stringstream *dot) const override {
    // Use the this pointer as a unique ID for the node in the dotfile.
    const int64_t id = this->GetGraphvizId();
    std::string name = this->get_name();
    if (name.empty()) {
      name = this->GetMemoryObjectName();
    }

    // Open the attributes and label.
    *dot << id << " [shape=record, label=\"" << name << "|{";

    // Append input ports to the label.
    // TODO(david-german-tri): Provide a way to customize port names.
    *dot << "{";
    for (int i = 0; i < this->get_num_input_ports(); ++i) {
      if (i != 0) *dot << "|";
      *dot << "<u" << i << ">u" << i;
    }
    *dot << "}";

    // Append output ports to the label.
    *dot << " | {";
    for (int i = 0; i < this->get_num_output_ports(); ++i) {
      if (i != 0) *dot << "|";
      *dot << "<y" << i << ">y" << i;
    }
    *dot << "}";

    // Close the label and attributes.
    *dot << "}\"];" << std::endl;
  }

  void GetGraphvizInputPortToken(const InputPortDescriptor<T>& port,
                                 std::stringstream *dot) const final {
    DRAKE_DEMAND(port.get_system() == this);
    *dot << this->GetGraphvizId() << ":u" << port.get_index();
  }

  void GetGraphvizOutputPortToken(const OutputPort<T>& port,
                                  std::stringstream *dot) const final {
    DRAKE_DEMAND(&port.get_system() == this);
    *dot << this->GetGraphvizId() << ":y" << port.get_index();
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
  virtual std::unique_ptr<DiscreteValues<T>> AllocateDiscreteState() const {
    if (model_discrete_state_vector_ != nullptr) {
      return std::make_unique<DiscreteValues<T>>(
          model_discrete_state_vector_->Clone());
    }
    return std::make_unique<DiscreteValues<T>>();
  }

  /// Reserves the abstract state as required by CreateDefaultContext. By
  /// default, it clones the abstract states declared through
  /// DeclareAbstractState() calls. Derived systems should override for
  /// different behaviors.
  virtual std::unique_ptr<AbstractValues> AllocateAbstractState() const {
    return std::make_unique<AbstractValues>(std::move(
        model_abstract_states_.CloneAllModels()));
  }

  /// Reserves the parameters as required by CreateDefaultContext.  The default
  /// implementation in this class clones the model_vector for all parameters
  /// declared via DeclareNumericParameter(), and so does not allocate any
  /// abstract parameters.  Subclasses can override this method if the default
  /// behavior is not sufficient.
  virtual std::unique_ptr<Parameters<T>> AllocateParameters() const {
    std::vector<std::unique_ptr<BasicVector<T>>> numeric_params;
    numeric_params.reserve(model_numeric_parameters_.size());
    for (int i = 0; i < model_numeric_parameters_.size(); ++i) {
      auto param = model_numeric_parameters_.CloneVectorModel<T>(i);
      DRAKE_ASSERT(param != nullptr);
      numeric_params.emplace_back(std::move(param));
    }
    return std::make_unique<Parameters<T>>(std::move(numeric_params));
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

    // If no symbolic sparsity matrix is available, assume direct feedthrough
    // by default.
    if (sparsity == nullptr) {
      return true;
    }
    return sparsity->IsConnectedInputToOutput(input_port, output_port);
  }

  // =========================================================================
  // New methods for subclasses to use

  /// Declares a numeric parameter using the given @p model_vector.  This is
  /// the best way to declare LeafSystem numeric parameters.  LeafSystem's
  /// default implementation of AllocateParameters uses model_vector.Clone(),
  /// and the default implementation of SetDefaultParameters() will reset
  /// parameters to their model vectors.  Returns the index of the new
  /// parameter.
  int DeclareNumericParameter(const BasicVector<T>& model_vector) {
    const int next_index = model_numeric_parameters_.size();
    model_numeric_parameters_.AddVectorModel(next_index, model_vector.Clone());
    return next_index;
  }

  /// Extracts the numeric parameters of type U from the @p context at @p index.
  /// Asserts if the context is not a LeafContext, or if it does not have a
  /// vector-valued parameter of type U at @p index.
  template <template <typename> class U = BasicVector>
  const U<T>& GetNumericParameter(const Context<T>& context, int index) const {
    static_assert(std::is_base_of<BasicVector<T>, U<T>>::value,
                  "U must be a subclass of BasicVector.");
    const auto& leaf_context = dynamic_cast<const systems::LeafContext<T>&>(
        context);
    const auto* const params = dynamic_cast<const U<T>*>(
        leaf_context.get_numeric_parameter(index));
    DRAKE_ASSERT(params != nullptr);
    return *params;
  }

  /// Extracts the numeric parameters of type U from the @p context at @p index.
  /// Asserts if the context is not a LeafContext, or if it does not have a
  /// vector-valued parameter of type U at @p index.
  template <template <typename> class U = BasicVector>
  U<T>* GetMutableNumericParameter(Context<T>* context, int index) const {
    static_assert(std::is_base_of<BasicVector<T>, U<T>>::value,
                  "U must be a subclass of BasicVector.");
    auto* leaf_context = dynamic_cast<systems::LeafContext<T>*>(context);
    DRAKE_ASSERT(leaf_context != nullptr);
    auto* params = dynamic_cast<U<T>*>(
        leaf_context->get_mutable_numeric_parameter(index));
    DRAKE_ASSERT(params != nullptr);
    return params;
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

  /// Declares a per step action using the default handlers given type
  /// @p action. This method aborts if the same type has already been declared.
  // TODO(siyuan): provide a API for declaration with custom handlers.
  void DeclarePerStepAction(
      const typename DiscreteEvent<T>::ActionType& action) {
    DiscreteEvent<T> event;
    event.action = action;
    for (const auto& declared_event : per_step_events_) {
      if (declared_event.action == action) {
        DRAKE_ABORT_MSG("Per step action has already been declared.");
      }
    }

    per_step_events_.push_back(event);
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

  /// Declares an abstract state.
  /// @param abstract_state The abstract state, its ownership is transfered.
  /// @return index of the declared abstract state.
  int DeclareAbstractState(std::unique_ptr<AbstractValue> abstract_state) {
    int index = model_abstract_states_.size();
    model_abstract_states_.AddModel(index, std::move(abstract_state));
    return index;
  }

  // =========================================================================
  /// @name                    Declare input ports
  /// Methods in this section are used by derived classes to declare their
  /// output ports, which may be vector valued or abstract valued.
  //@{

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
  //@}

  // =========================================================================
  /// @name                    Declare output ports
  /// Methods in this section are used by derived classes to declare their
  /// output ports, which may be vector valued or abstract valued. Every output
  /// port must have an _allocator_ function and
  /// a _calculator_ function. The allocator returns an object suitable for
  /// holding a value of the output port. The calculator uses the contents of
  /// a given Context to produce the output port's value, which is placed in
  /// an object of the type returned by the allocator.
  ///
  /// Although the allocator and calculator functions ultimately satisfy generic
  /// function signatures defined in LeafOutputPort, we provide a variety
  /// of `DeclareVectorOutputPort()` and `DeclareAbstractOutputPort()`
  /// signatures here for convenient specification, with mapping to the generic
  /// form handled invisibly. In particular, allocators are most easily defined
  /// by providing a model value that can be used to construct an
  /// allocator that copies the model when a new value object is needed.
  /// Alternatively a method can be provided that constructs a value object when
  /// invoked (those methods are conventionally, but not necessarily, named
  /// `MakeSomething()` where `Something` is replaced by the output port value
  /// type).
  ///
  /// Because output port values are ultimately stored in AbstractValue objects,
  /// the underlying types must be suitable. For vector ports, that means the
  /// type must be BasicVector or a class derived from BasicVector. For abstract
  /// ports, the type must be copy constructible or cloneable. For
  /// methods below that are not given an explicit model value or construction
  /// ("make") method, the underlying type must be default constructible.
  /// @see drake::systems::Value for more about abstract values.
  //@{

  /// Declares a vector-valued output port by specifying (1) a model vector of
  /// type BasicVectorSubtype derived from BasicVector and initialized to the
  /// correct size and desired initial value, and (2) a calculator function that
  /// is a class member function (method) with signature:
  /// @code
  /// void MySystem::CalcOutputVector(const Context<T>&,
  ///                                 BasicVectorSubtype*) const;
  /// @endcode
  /// where `MySystem` is a class derived from `LeafSystem<T>`. Template
  /// arguments will be deduced and do not need to be specified.
  template <class MySystem, typename BasicVectorSubtype>
  const OutputPort<T>& DeclareVectorOutputPort(
      const BasicVectorSubtype& model_vector,
      void (MySystem::*calc)(const Context<T>&, BasicVectorSubtype*) const) {
    static_assert(std::is_base_of<LeafSystem<T>, MySystem>::value,
                  "Expected to be invoked from a LeafSystem-derived System.");
    static_assert(std::is_base_of<BasicVector<T>, BasicVectorSubtype>::value,
                  "Expected vector type derived from BasicVector.");
    // We need to obtain a `this` pointer of the right derived type to capture
    // in the calculator functor, so that it will be able to invoke the given
    // mmember function `calc()`.
    auto this_ptr = dynamic_cast<const MySystem*>(this);
    DRAKE_DEMAND(this_ptr != nullptr);
    // Currently all vector ports in Drake require a fixed size that is known
    // at the time the port is declared.
    auto& port = CreateVectorLeafOutputPort(
        model_vector.size(),
        // Allocator function just clones the given model vector.
        MakeAllocCallback<BasicVector<T>>(model_vector),
        // Calculator function downcasts to specific vector type and invokes
        // the given member function.
        [this_ptr, calc](const Context<T>& context, BasicVector<T>* result) {
          auto typed_result = dynamic_cast<BasicVectorSubtype*>(result);
          DRAKE_DEMAND(typed_result != nullptr);
          (this_ptr->*calc)(context, typed_result);
        });
    return port;
  }

  /// Declares a vector-valued output port by specifying _only_ a calculator
  /// function that is a class member function (method) with signature:
  /// @code
  /// void MySystem::CalcOutputVector(const Context<T>&,
  ///                                 BasicVectorSubtype*) const;
  /// @endcode
  /// where `MySystem` is a class derived from `LeafSystem<T>` and
  /// `BasicVectorSubtype` is derived from `BasicVector<T>` and has a suitable
  /// default constructor that allocates a vector of the expected size. This
  /// will use `BasicVectorSubtype{}` (that is, the default constructor) to
  /// produce a model vector for the output port's value.
  /// Template arguments will be deduced and do not need to be specified.
  ///
  /// @note The default constructor will be called once immediately, and
  /// subsequent allocations will just copy the model value without invoking the
  /// constructor again. If you want the constructor invoked again at each
  /// allocation (not common), use one of the other signatures to explicitly
  /// provide a method for the allocator to call; that method can then invoke
  /// the `BasicVectorSubtype` default constructor.
  template <class MySystem, typename BasicVectorSubtype>
  const OutputPort<T>& DeclareVectorOutputPort(
      void (MySystem::*calc)(const Context<T>&, BasicVectorSubtype*) const) {
    static_assert(
        std::is_default_constructible<BasicVectorSubtype>::value,
        "LeafSystem::DeclareVectorOutputPort(calc): the one-argument form of "
        "this method requires that the output type has a default constructor");
    // Invokes the previous method.
    return DeclareVectorOutputPort(BasicVectorSubtype{}, calc);
  }

  /// (Advanced) Declares a vector-valued output port using the given
  /// `model_vector` and a function for calculating the port's value at runtime.
  /// The port's size will be model_vector.size(), and the default allocator for
  /// the port will be model_vector.Clone(). Note that this takes the calculator
  /// function in its most generic form; if you have a member function available
  /// use one of the other signatures.
  /// @see LeafOutputPort::CalcVectorCallback
  const OutputPort<T>& DeclareVectorOutputPort(
      const BasicVector<T>& model_vector,
      typename LeafOutputPort<T>::CalcVectorCallback vector_calc_function) {
    auto& port = CreateVectorLeafOutputPort(model_vector.size(),
                                            MakeAllocCallback(model_vector),
                                            vector_calc_function);
    return port;
  }

  /// Declares an abstract-valued output port by specifying a model value of
  /// concrete type `OutputType` and a calculator function that is a class
  /// member function (method) with signature:
  /// @code
  /// void MySystem::CalcOutputValue(const Context<T>&, OutputType*) const;
  /// @endcode
  /// where `MySystem` must be a class derived from `LeafSystem<T>`.
  /// `OutputType` must be such that `Value<OutputType>` is permitted.
  /// Template arguments will be deduced and do not need to be specified.
  /// @see drake::systems::Value
  template <class MySystem, typename OutputType>
  const OutputPort<T>& DeclareAbstractOutputPort(
      const OutputType& model_value,
      void (MySystem::*calc)(const Context<T>&, OutputType*) const) {
    auto this_ptr = dynamic_cast<const MySystem*>(this);
    DRAKE_DEMAND(this_ptr != nullptr);
    auto& port = CreateAbstractLeafOutputPort(
        MakeAllocCallback(model_value),
        [this_ptr, calc](const Context<T>& context, AbstractValue* result) {
          OutputType& typed_result = result->GetMutableValue<OutputType>();
          (this_ptr->*calc)(context, &typed_result);
        });
    return port;
  }

  /// Declares an abstract-valued output port by specifying only a calculator
  /// function that is a class member function (method) with signature:
  /// @code
  /// void MySystem::CalcOutputValue(const Context<T>&, OutputType*) const;
  /// @endcode
  /// where `MySystem` is a class derived from `LeafSystem<T>`. `OutputType`
  /// is a concrete type such that `Value<OutputType>` is permitted, and
  /// must be default constructible, so that we can create a model value using
  /// `Value<OutputType>{}` (value initialized so numerical types will be
  /// zeroed in the model).
  /// Template arguments will be deduced and do not need to be specified.
  ///
  /// @note The default constructor will be called once immediately, and
  /// subsequent allocations will just copy the model value without invoking the
  /// constructor again. If you want the constructor invoked again at each
  /// allocation (not common), use one of the other signatures to explicitly
  /// provide a method for the allocator to call; that method can then invoke
  /// the `OutputType` default constructor.
  /// @see drake::systems::Value
  template <class MySystem, typename OutputType>
  const OutputPort<T>& DeclareAbstractOutputPort(
      void (MySystem::*calc)(const Context<T>&, OutputType*) const) {
    static_assert(
        std::is_default_constructible<OutputType>::value,
        "LeafSystem::DeclareAbstractOutputPort(calc): the one-argument form of "
        "this method requires that the output type has a default constructor");
    // Note that value initialization {} is required here.
    return DeclareAbstractOutputPort(OutputType{}, calc);
  }

  /// Declares an abstract-valued output port by specifying member functions to
  /// use both for the allocator and calculator. The signatures are:
  /// @code
  /// OutputType MySystem::MakeOutputValue(const Context<T>&) const;
  /// void MySystem::CalcOutputValue(const Context<T>&, OutputType*) const;
  /// @endcode
  /// where `MySystem` is a class derived from `LeafSystem<T>` and `OutputType`
  /// is any concrete type such that `Value<OutputType>` is permitted. See
  /// alternate signature if your allocator method does not need a Context.
  /// Template arguments will be deduced and do not need to be specified.
  /// @see drake::systems::Value
  template <class MySystem, typename OutputType>
  const OutputPort<T>& DeclareAbstractOutputPort(
      OutputType (MySystem::*make)(const Context<T>&) const,
      void (MySystem::*calc)(const Context<T>&, OutputType*) const) {
    auto this_ptr = dynamic_cast<const MySystem*>(this);
    DRAKE_DEMAND(this_ptr != nullptr);
    auto& port = CreateAbstractLeafOutputPort(
        [this_ptr, make](const Context<T>& context) {
          return AbstractValue::Make((this_ptr->*make)(context));
        },
        [this_ptr, calc](const Context<T>& context, AbstractValue* result) {
          OutputType& typed_result = result->GetMutableValue<OutputType>();
          (this_ptr->*calc)(context, &typed_result);
        });
    return port;
  }

  /// Declares an abstract-valued output port by specifying member functions to
  /// use both for the allocator and calculator. The signatures are:
  /// @code
  /// OutputType MySystem::MakeOutputValue() const;
  /// void MySystem::CalcOutputValue(const Context<T>&, OutputType*) const;
  /// @endcode
  /// where `MySystem` is a class derived from `LeafSystem<T>` and `OutputType`
  /// may be any concrete type such that `Value<OutputType>` is permitted.
  /// See alternate signature if your allocator method needs a Context.
  /// Template arguments will be deduced and do not need to be specified.
  /// @see drake::systems::Value
  template <class MySystem, typename OutputType>
  const OutputPort<T>& DeclareAbstractOutputPort(
      OutputType (MySystem::*make)() const,
      void (MySystem::*calc)(const Context<T>&, OutputType*) const) {
    auto this_ptr = dynamic_cast<const MySystem*>(this);
    DRAKE_DEMAND(this_ptr != nullptr);
    auto& port = CreateAbstractLeafOutputPort(
        [this_ptr, make](const Context<T>&) {  // Swallow the context.
          return AbstractValue::Make((this_ptr->*make)());
        },
        [this_ptr, calc](const Context<T>& context, AbstractValue* result) {
          OutputType& typed_result = result->GetMutableValue<OutputType>();
          (this_ptr->*calc)(context, &typed_result);
        });
    return port;
  }

  /// (Advanced) Declares an abstract-valued output port using the given
  /// allocator and calculator functions provided in their most generic forms.
  /// If you have a member function available use one of the other signatures.
  /// @see LeafOutputPort::AllocCallback, LeafOutputPort::CalcCallback
  const OutputPort<T>& DeclareAbstractOutputPort(
      typename LeafOutputPort<T>::AllocCallback alloc_function,
      typename LeafOutputPort<T>::CalcCallback calc_function) {
    auto& port = CreateAbstractLeafOutputPort(alloc_function, calc_function);
    return port;
  }
  //@}

 private:
  void DoGetPerStepEvents(
      const Context<T>&,
      std::vector<DiscreteEvent<T>>* events) const override {
    *events = per_step_events_;
  }

  // Aborts for scalar types that are not numeric, since there is no reasonable
  // definition of "next update time" outside of the real line.
  //
  // @tparam T1 SFINAE boilerplate for the scalar type. Do not set.
  template <typename T1 = T>
  typename std::enable_if<!is_numeric<T1>::value>::type
  DoCalcNextUpdateTimeImpl(const Context<T1>&, UpdateActions<T1>*) const {
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
    // No periodic events events.
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

  // Returns a SparsityMatrix for this system, or nullptr if a SparsityMatrix
  // cannot be constructed because this System has no symbolic representation.
  std::unique_ptr<SparsityMatrix> MakeSparsityMatrix() const {
    std::unique_ptr<System<symbolic::Expression>> symbolic_system =
        this->ToSymbolic();
    if (symbolic_system) {
      return std::make_unique<SparsityMatrix>(*symbolic_system);
    } else {
      return nullptr;
    }
  }

  // Creates a new vector-valued LeafOutputPort in this LeafSystem and returns
  // a reference to it.
  LeafOutputPort<T>& CreateVectorLeafOutputPort(
      int fixed_size,
      typename LeafOutputPort<T>::AllocCallback vector_allocator,
      typename LeafOutputPort<T>::CalcVectorCallback vector_calculator) {
    auto port = std::make_unique<LeafOutputPort<T>>(
        *this, fixed_size, vector_allocator, vector_calculator);
    LeafOutputPort<T>* const port_ptr = port.get();
    this->CreateOutputPort(std::move(port));
    return *port_ptr;
  }

  // Creates a new abstract-valued LeafOutputPort in this LeafSystem and returns
  // a reference to it.
  LeafOutputPort<T>& CreateAbstractLeafOutputPort(
      typename LeafOutputPort<T>::AllocCallback allocator,
      typename LeafOutputPort<T>::CalcCallback calculator) {
    auto port =
        std::make_unique<LeafOutputPort<T>>(*this, allocator, calculator);
    LeafOutputPort<T>* const port_ptr = port.get();
    this->CreateOutputPort(std::move(port));
    return *port_ptr;
  }

  // Creates an abstract output port allocator function from an arbitrary type
  // model value.
  template <typename OutputType>
  static typename LeafOutputPort<T>::AllocCallback MakeAllocCallback(
      const OutputType& model_value) {
    // The given model value may have *either* a copy constructor or a Clone()
    // method, since it just has to be suitable for containing in an
    // AbstractValue. We need to create a functor that is copy constructible,
    // so need to wrap the model value to give it a copy constructor. Drake's
    // copyable_unique_ptr does just that, so is suitable for capture by the
    // allocator functor here.
    copyable_unique_ptr<AbstractValue> owned_model(
        new Value<OutputType>(model_value));
    return [model = std::move(owned_model)](const Context<T>&) {
      return model->Clone();
    };
  }

  // Periodic Update or Publish events registered on this system.
  std::vector<PeriodicEvent<T>> periodic_events_;

  // Update or Publish events registered on this system for every simulator
  // major time step.
  std::vector<DiscreteEvent<T>> per_step_events_;

  // A model continuous state to be used in AllocateDefaultContext.
  std::unique_ptr<BasicVector<T>> model_continuous_state_vector_;
  int num_generalized_positions_{0};
  int num_generalized_velocities_{0};
  int num_misc_continuous_states_{0};

  // A model discrete state to be used in AllocateDefaultContext.
  std::unique_ptr<BasicVector<T>> model_discrete_state_vector_;

  // A model abstract state to be used in AllocateAbstractState.
  detail::ModelValues model_abstract_states_;

  // Model inputs to be used in AllocateOutput{Vector,Abstract}.
  detail::ModelValues model_input_values_;

  // Model outputs to be used in AllocateParameters.
  detail::ModelValues model_numeric_parameters_;
};

}  // namespace systems
}  // namespace drake
