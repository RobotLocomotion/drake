#pragma once

#include <algorithm>
#include <functional>
#include <limits>
#include <map>
#include <memory>
#include <set>
#include <stdexcept>
#include <utility>
#include <vector>

#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/number_traits.h"
#include "drake/common/symbolic_expression.h"
#include "drake/common/text_logging.h"
#include "drake/systems/framework/cache.h"
#include "drake/systems/framework/diagram_context.h"
#include "drake/systems/framework/discrete_state.h"
#include "drake/systems/framework/state.h"
#include "drake/systems/framework/subvector.h"
#include "drake/systems/framework/system.h"
#include "drake/systems/framework/system_port_descriptor.h"

namespace drake {
namespace systems {

template <typename T>
class DiagramBuilder;

namespace internal {

/// Returns a vector of raw pointers that correspond placewise with the
/// unique_ptrs in the vector @p in.
template <typename U>
std::vector<U*> Unpack(const std::vector<std::unique_ptr<U>>& in) {
  std::vector<U*> out(in.size());
  std::transform(in.begin(), in.end(), out.begin(),
                 [](const std::unique_ptr<U>& p) { return p.get(); });
  return out;
}

/// Returns true if any of the events in @p actions has type @p type.
template <typename T>
bool HasEvent(const UpdateActions<T> actions,
              typename DiscreteEvent<T>::ActionType type) {
  for (const DiscreteEvent<T>& event : actions.events) {
    if (event.action == type) return true;
  }
  return false;
}

/// DiagramOutput is an implementation of SystemOutput that holds unowned
/// OutputPortValue pointers. It is used to expose the outputs of constituent
/// systems as outputs of a Diagram.
///
/// @tparam T The type of the output data. Must be a valid Eigen scalar.
template <typename T>
class DiagramOutput : public SystemOutput<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(DiagramOutput)

  DiagramOutput() = default;

  int get_num_ports() const override { return static_cast<int>(ports_.size()); }

  OutputPortValue* get_mutable_port_value(int index) override {
    DRAKE_DEMAND(index >= 0 && index < get_num_ports());
    return ports_[index];
  }

  const OutputPortValue& get_port_value(int index) const override {
    DRAKE_DEMAND(index >= 0 && index < get_num_ports());
    return *ports_[index];
  }

  std::vector<OutputPortValue*>* get_mutable_port_values() { return &ports_; }

 protected:
  // Returns a clone that has the same number of output ports, with values
  // set to nullptr.
  DiagramOutput<T>* DoClone() const override {
    DiagramOutput<T>* clone = new DiagramOutput<T>();
    clone->ports_.resize(get_num_ports());
    return clone;
  }

 private:
  std::vector<OutputPortValue*> ports_;
};

/// DiagramTimeDerivatives is a version of DiagramContinuousState that owns
/// the constituent continuous states. As the name implies, it is only useful
/// for the time derivatives.
template <typename T>
class DiagramTimeDerivatives : public DiagramContinuousState<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(DiagramTimeDerivatives)

  explicit DiagramTimeDerivatives(
      std::vector<std::unique_ptr<ContinuousState<T>>>&& substates)
      : DiagramContinuousState<T>(Unpack(substates)),
        substates_(std::move(substates)) {}

  ~DiagramTimeDerivatives() override {}

 private:
  std::vector<std::unique_ptr<ContinuousState<T>>> substates_;
};

/// DiagramDiscreteVariables is a version of DiscreteState that owns
/// the constituent discrete states. As the name implies, it is only useful
/// for the discrete updates.
template <typename T>
class DiagramDiscreteVariables : public DiscreteState<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(DiagramDiscreteVariables)

  explicit DiagramDiscreteVariables(
      std::vector<std::unique_ptr<DiscreteState<T>>>&& subdiscretes)
      : DiscreteState<T>(Flatten(Unpack(subdiscretes))),
        subdiscretes_(std::move(subdiscretes)) {}

  ~DiagramDiscreteVariables() override {}

  int num_subdifferences() const {
    return static_cast<int>(subdiscretes_.size());
  }

  DiscreteState<T>* get_mutable_subdifference(int index) {
    DRAKE_DEMAND(index >= 0 && index < num_subdifferences());
    return subdiscretes_[index].get();
  }

 private:
  std::vector<BasicVector<T>*> Flatten(
      const std::vector<DiscreteState<T>*>& in) const {
    std::vector<BasicVector<T>*> out;
    for (const DiscreteState<T>* xd : in) {
      const std::vector<BasicVector<T>*>& xd_data = xd->get_data();
      out.insert(out.end(), xd_data.begin(), xd_data.end());
    }
    return out;
  }

  std::vector<std::unique_ptr<DiscreteState<T>>> subdiscretes_;
};

}  // namespace internal

/// Diagram is a System composed of one or more constituent Systems, arranged
/// in a directed graph where the vertices are the constituent Systems
/// themselves, and the edges connect the output of one constituent System
/// to the input of another. To construct a Diagram, use a DiagramBuilder.
template <typename T>
class Diagram : public System<T>,
                public detail::InputPortEvaluatorInterface<T> {
 public:
  // Diagram objects are neither copyable nor moveable.
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(Diagram)

  typedef typename std::pair<const System<T>*, int> PortIdentifier;

  ~Diagram() override {}

  /// Returns the list of contained Systems.
  std::vector<const systems::System<T>*> GetSystems() const {
    std::vector<const systems::System<T>*> result;
    result.reserve(registered_systems_.size());
    for (const auto& system : registered_systems_) {
      result.push_back(system.get());
    }
    return result;
  }

  /// This method is DEPRECATED. Legacy overrides will be respected for now,
  /// but will be deleted as soon as automatic analysis can replicate them.
  /// There will thereafter be no manual override for the direct feedthrough
  /// properties of a Diagram: developers will be required to express the
  /// direct feedthrough properties of each constituent System correctly, which
  /// will enable the Diagram to deduce its own direct-feedthrough properties
  /// without manual intervention.
  ///
  /// Returns `true` as a conservative default.
  bool has_any_direct_feedthrough() const override {
    return true;
  }

  /// Returns true if any output of the Diagram might have direct-feedthrough
  /// from any input of the Diagram. The implementation is quite conservative:
  /// it will return true if there is any path on the directed acyclic graph
  /// of subsystems that begins at any input port to the Diagram, and ends at
  /// any System producing an output port of the Diagram, such that every System
  /// in that path HasAnyDirectFeedthrough.
  ///
  /// TODO(david-german-tri): Improve the implementation by inspecting the
  /// fine-grained HasDirectFeedthrough(input_port, output_port) relationships
  /// of the subsystems.
  bool HasAnyDirectFeedthrough() const final {
    // Respect legacy overrides.
    if (!has_any_direct_feedthrough()) return false;

    for (int i = 0; i < this->get_num_output_ports(); i++) {
      if (HasDirectFeedthrough(i)) {
        return true;
      }
    }
    return false;
  }

  /// Returns true if the given @p output_port of the Diagram might have
  /// direct-feedthrough from any input of the Diagram. The implementation is
  /// quite conservative: it will return true if there is any path on the
  /// directed acyclic graph of subsystems that begins at any input port to
  /// the Diagram, and ends at the system producing the given @p output_port,
  /// such that every System in that path HasAnyDirectFeedthrough.
  ///
  /// TODO(david-german-tri): Improve the implementation by inspecting the
  /// fine-grained HasDirectFeedthrough(input_port, output_port) relationships
  /// of the subsystems.
  bool HasDirectFeedthrough(int output_port) const final {
    // Respect legacy overrides.
    if (!has_any_direct_feedthrough()) return false;

    DRAKE_ASSERT(output_port >= 0);
    DRAKE_ASSERT(output_port < this->get_num_output_ports());
    return HasDirectFeedthroughFromAnyInput(output_port_ids_[output_port]);
  }

  /// Aborts.
  /// Once this is implemented, the following comment will apply:
  ///
  /// Returns true if there might be direct feedthrough from the given
  /// @p input_port of the Diagram to the given @p output_port of the Diagram.
  bool HasDirectFeedthrough(int input_port, int output_port) const final {
    DRAKE_ABORT_MSG("The two-argument version of HasDirectFeedthrough is "
                    "not yet implemented for Diagrams.");
  }

  std::unique_ptr<Context<T>> AllocateContext() const override {
    const int num_systems = num_subsystems();
    // Reserve inputs as specified during Diagram initialization.
    auto context = std::make_unique<DiagramContext<T>>(num_systems);

    // Add each constituent system to the Context.
    for (int i = 0; i < num_systems; ++i) {
      const System<T>* const sys = sorted_systems_[i];
      auto subcontext = sys->AllocateContext();
      auto suboutput = sys->AllocateOutput(*subcontext);
      context->AddSystem(i, std::move(subcontext), std::move(suboutput));
    }

    // Wire up the Diagram-internal inputs and outputs.
    for (const auto& connection : dependency_graph_) {
      const PortIdentifier& src = connection.second;
      const PortIdentifier& dest = connection.first;
      context->Connect(ConvertToContextPortIdentifier(src),
                       ConvertToContextPortIdentifier(dest));
    }

    // Declare the Diagram-external inputs.
    for (const PortIdentifier& id : input_port_ids_) {
      context->ExportInput(ConvertToContextPortIdentifier(id));
    }

    context->MakeState();
    context->MakeParameters();
    return std::unique_ptr<Context<T>>(context.release());
  }

  void SetDefaultState(const Context<T>& context,
                       State<T>* state) const override {
    auto diagram_context = dynamic_cast<const DiagramContext<T>*>(&context);
    DRAKE_DEMAND(diagram_context != nullptr);

    auto diagram_state = dynamic_cast<DiagramState<T>*>(state);
    DRAKE_DEMAND(diagram_state != nullptr);

    // Set default state of each constituent system.
    for (int i = 0; i < num_subsystems(); ++i) {
      auto subcontext = diagram_context->GetSubsystemContext(i);
      DRAKE_DEMAND(subcontext != nullptr);
      auto substate = diagram_state->get_mutable_substate(i);
      DRAKE_DEMAND(substate != nullptr);
      sorted_systems_[i]->SetDefaultState(*subcontext, substate);
    }
  }

  void SetDefaults(Context<T>* context) const final {
    auto diagram_context = dynamic_cast<DiagramContext<T>*>(context);
    DRAKE_DEMAND(diagram_context != nullptr);

    // Set defaults of each constituent system.
    for (int i = 0; i < num_subsystems(); ++i) {
      auto subcontext = diagram_context->GetMutableSubsystemContext(i);
      sorted_systems_[i]->SetDefaults(subcontext);
    }
  }

  std::unique_ptr<SystemOutput<T>> AllocateOutput(
      const Context<T>& context) const override {
    auto diagram_context = dynamic_cast<const DiagramContext<T>*>(&context);
    DRAKE_DEMAND(diagram_context != nullptr);

    // The output ports of this Diagram are output ports of its constituent
    // systems. Create a DiagramOutput with that many ports.
    auto output = std::make_unique<internal::DiagramOutput<T>>();
    output->get_mutable_port_values()->resize(output_port_ids_.size());
    ExposeSubsystemOutputs(*diagram_context, output.get());
    return std::unique_ptr<SystemOutput<T>>(output.release());
  }

  void DoCalcOutput(const Context<T>& context,
                    SystemOutput<T>* output) const override {
    // Down-cast the context and output to DiagramContext and DiagramOutput.
    auto diagram_context = dynamic_cast<const DiagramContext<T>*>(&context);
    DRAKE_DEMAND(diagram_context != nullptr);
    auto diagram_output = dynamic_cast<internal::DiagramOutput<T>*>(output);
    DRAKE_DEMAND(diagram_output != nullptr);

    // Populate the output with pointers to the appropriate subsystem outputs
    // in the DiagramContext. We do this on every call to CalcOutput, so
    // that the diagram_context and diagram_output are not tightly coupled.
    ExposeSubsystemOutputs(*diagram_context, diagram_output);

    // Since the diagram output now contains pointers to the subsystem outputs,
    // all we need to do is ask those subsystem outputs to evaluate themselves.
    // They will recursively evaluate any intermediate inputs that they need.
    for (const PortIdentifier& id : output_port_ids_) {
      EvaluateOutputPort(*diagram_context, id);
    }
  }

  /// Aggregates the time derivatives from each subsystem into a
  /// DiagramTimeDerivatives.
  std::unique_ptr<ContinuousState<T>> AllocateTimeDerivatives() const override {
    std::vector<std::unique_ptr<ContinuousState<T>>> sub_derivatives;
    for (const System<T>* const system : sorted_systems_) {
      sub_derivatives.push_back(system->AllocateTimeDerivatives());
    }
    return std::unique_ptr<ContinuousState<T>>(
        new internal::DiagramTimeDerivatives<T>(std::move(sub_derivatives)));
  }

  /// Aggregates the discrete update variables from each subsystem into a
  /// DiagramDiscreteVariables.
  std::unique_ptr<DiscreteState<T>> AllocateDiscreteVariables()
      const override {
    std::vector<std::unique_ptr<DiscreteState<T>>> sub_differences;
    for (const System<T>* const system : sorted_systems_) {
      sub_differences.push_back(system->AllocateDiscreteVariables());
    }
    return std::unique_ptr<DiscreteState<T>>(
        new internal::DiagramDiscreteVariables<T>(
            std::move(sub_differences)));
  }

  void DoCalcTimeDerivatives(const Context<T>& context,
                             ContinuousState<T>* derivatives) const override {
    auto diagram_context = dynamic_cast<const DiagramContext<T>*>(&context);
    DRAKE_DEMAND(diagram_context != nullptr);

    auto diagram_derivatives =
        dynamic_cast<DiagramContinuousState<T>*>(derivatives);
    DRAKE_DEMAND(diagram_derivatives != nullptr);
    const int n = diagram_derivatives->get_num_substates();
    DRAKE_DEMAND(num_subsystems() == n);

    // Evaluate the derivatives of each constituent system.
    for (int i = 0; i < n; ++i) {
      const Context<T>* subcontext = diagram_context->GetSubsystemContext(i);
      ContinuousState<T>* subderivatives =
          diagram_derivatives->get_mutable_substate(i);
      sorted_systems_[i]->CalcTimeDerivatives(*subcontext, subderivatives);
    }
  }

  /// Retrieves the state derivatives for a particular subsystem from the
  /// derivatives for the entire diagram. Aborts if @p subsystem is not
  /// actually a subsystem of this diagram. Returns nullptr if @p subsystem
  /// is stateless.
  const ContinuousState<T>* GetSubsystemDerivatives(
      const ContinuousState<T>& derivatives, const System<T>* subsystem) const {
    DRAKE_DEMAND(subsystem != nullptr);
    auto diagram_derivatives =
        dynamic_cast<const DiagramContinuousState<T>*>(&derivatives);
    DRAKE_DEMAND(diagram_derivatives != nullptr);
    const int i = GetSystemIndexOrAbort(subsystem);
    return diagram_derivatives->get_substate(i);
  }

  /// Returns a constant reference to the subcontext that corresponds to the
  /// system @p subsystem.
  /// Classes inheriting from %Diagram need access to this method in order to
  /// pass their constituent subsystems the apropriate subcontext. Aborts if
  /// @p subsystem is not actually a subsystem of this diagram.
  const Context<T>& GetSubsystemContext(const Context<T>& context,
                                        const System<T>* subsystem) const {
    DRAKE_DEMAND(subsystem != nullptr);
    auto& diagram_context = dynamic_cast<const DiagramContext<T>&>(context);
    const int i = GetSystemIndexOrAbort(subsystem);
    return *diagram_context.GetSubsystemContext(i);
  }

  /// Returns the subcontext that corresponds to the system @p subsystem.
  /// Classes inheriting from %Diagram need access to this method in order to
  /// pass their constituent subsystems the apropriate subcontext. Aborts if
  /// @p subsystem is not actually a subsystem of this diagram.
  Context<T>* GetMutableSubsystemContext(Context<T>* context,
                                         const System<T>* subsystem) const {
    DRAKE_DEMAND(context != nullptr);
    DRAKE_DEMAND(subsystem != nullptr);
    auto diagram_context = dynamic_cast<DiagramContext<T>*>(context);
    DRAKE_DEMAND(diagram_context != nullptr);
    const int i = GetSystemIndexOrAbort(subsystem);
    return diagram_context->GetMutableSubsystemContext(i);
  }

  /// Retrieves the state for a particular subsystem from the context for the
  /// entire diagram. Invalidates all entries in that subsystem's cache that
  /// depend on State. Aborts if @p subsystem is not actually a subsystem of
  /// this diagram.
  ///
  /// TODO(david-german-tri): Provide finer-grained accessors for finer-grained
  /// invalidation.
  State<T>* GetMutableSubsystemState(Context<T>* context,
                                     const System<T>* subsystem) const {
    Context<T>* subcontext = GetMutableSubsystemContext(context, subsystem);
    return subcontext->get_mutable_state();
  }

  /// Retrieves the state for a particular subsystem from the @p state for the
  /// entire diagram. Aborts if @p subsystem is not actually a subsystem of this
  /// diagram.
  State<T>* GetMutableSubsystemState(State<T>* state,
                                     const System<T>* subsystem) const {
    const int i = GetSystemIndexOrAbort(subsystem);
    auto diagram_state = dynamic_cast<DiagramState<T>*>(state);
    DRAKE_DEMAND(diagram_state != nullptr);
    return diagram_state->get_mutable_substate(i);
  }

  /// Returns the full path of this Diagram in the tree of Diagrams. Implemented
  /// here to satisfy InputPortEvaluatorInterface, although we want the exact
  /// same behavior as in System.
  void GetPath(std::stringstream* output) const override {
    return System<T>::GetPath(output);
  }

  /// Evaluates the value of the subsystem input port with the given @p id
  /// in the given @p context. Satisfies InputPortEvaluatorInterface.
  ///
  /// This is a framework implementation detail. User code should not call
  /// this function.
  void EvaluateSubsystemInputPort(
      const Context<T>* context,
      const InputPortDescriptor<T>& descriptor) const override {
    // Find the output port connected to the given input port.
    const PortIdentifier id{descriptor.get_system(), descriptor.get_index()};
    const auto upstream_it = dependency_graph_.find(id);

    auto diagram_context = dynamic_cast<const DiagramContext<T>*>(context);

    // If the upstream output port exists in this Diagram, evaluate it.
    // TODO(david-german-tri): Add online algebraic loop detection here.
    if (upstream_it != dependency_graph_.end()) {
      DRAKE_DEMAND(diagram_context != nullptr);
      const PortIdentifier& prerequisite = upstream_it->second;
      this->EvaluateOutputPort(*diagram_context, prerequisite);
    }

    // If the upstream output port is an input of this whole Diagram, ask our
    // parent to evaluate it.
    const auto external_it =
        std::find(input_port_ids_.begin(), input_port_ids_.end(), id);
    if (external_it != input_port_ids_.end()) {
      const int i = external_it - input_port_ids_.begin();
      this->EvalInputPort(*diagram_context, i);
    }
  }

 protected:
  /// Constructs an uninitialized Diagram. Subclasses that use this constructor
  /// are obligated to call DiagramBuilder::BuildInto(this).
  Diagram() {}

  void DoPublish(const Context<T>& context) const override {
    auto diagram_context = dynamic_cast<const DiagramContext<T>*>(&context);
    DRAKE_DEMAND(diagram_context != nullptr);

    for (const System<T>* const system : sorted_systems_) {
      const int i = GetSystemIndexOrAbort(system);
      system->Publish(*diagram_context->GetSubsystemContext(i));
    }
  }

  /// The @p generalized_velocity vector must have the same size and ordering as
  /// the generalized velocity in the ContinuousState that this Diagram reserves
  /// in its context.
  void DoMapVelocityToQDot(
      const Context<T>& context,
      const Eigen::Ref<const VectorX<T>>& generalized_velocity,
      VectorBase<T>* qdot) const override {
    // Check that the dimensions of the continuous state in the context match
    // the dimensions of the provided generalized velocity and configuration
    // derivatives.
    const ContinuousState<T>* xc = context.get_continuous_state();
    DRAKE_DEMAND(xc != nullptr);
    const int nq = xc->get_generalized_position().size();
    const int nv = xc->get_generalized_velocity().size();
    DRAKE_DEMAND(nq == qdot->size());
    DRAKE_DEMAND(nv == generalized_velocity.size());

    auto diagram_context = dynamic_cast<const DiagramContext<T>*>(&context);
    DRAKE_DEMAND(diagram_context != nullptr);

    // Iterate over the subsystems in sorted order, asking each subsystem to
    // map its subslice of velocity to configuration derivatives. This approach
    // is valid because the DiagramContinuousState guarantees that the subsystem
    // states are concatenated in sorted order.
    int v_index = 0;  // The next index to read in generalized_velocity.
    int q_index = 0;  // The next index to write in qdot.
    for (int i = 0; i < num_subsystems(); ++i) {
      // Find the continuous state of subsystem i.
      const Context<T>* subcontext = diagram_context->GetSubsystemContext(i);
      DRAKE_DEMAND(subcontext != nullptr);
      const ContinuousState<T>* sub_xc = subcontext->get_continuous_state();
      // If subsystem i is stateless, skip it.
      if (sub_xc == nullptr) continue;

      // Select the chunk of generalized_velocity belonging to subsystem i.
      const int num_v = sub_xc->get_generalized_velocity().size();
      const Eigen::Ref<const VectorX<T>>& v_slice =
          generalized_velocity.segment(v_index, num_v);

      // Select the chunk of qdot belonging to subsystem i.
      const int num_q = sub_xc->get_generalized_position().size();
      Subvector<T> dq_slice(qdot, q_index, num_q);

      // Delegate the actual mapping to subsystem i itself.
      sorted_systems_[i]->MapVelocityToQDot(*subcontext, v_slice, &dq_slice);

      // Advance the indices.
      v_index += num_v;
      q_index += num_q;
    }
  }

  /// The @p generalized_velocity vector must have the same size and ordering as
  /// the generalized velocity in the ContinuousState that this Diagram reserves
  /// in its context.
  void DoMapQDotToVelocity(
      const Context<T>& context,
      const Eigen::Ref<const VectorX<T>>& qdot,
      VectorBase<T>* generalized_velocity) const override {
    // Check that the dimensions of the continuous state in the context match
    // the dimensions of the provided generalized velocity and configuration
    // derivatives.
    const ContinuousState<T>* xc = context.get_continuous_state();
    DRAKE_DEMAND(xc != nullptr);
    const int nq = xc->get_generalized_position().size();
    const int nv = xc->get_generalized_velocity().size();
    DRAKE_DEMAND(nq == qdot.size());
    DRAKE_DEMAND(nv == generalized_velocity->size());

    auto diagram_context = dynamic_cast<const DiagramContext<T>*>(&context);
    DRAKE_DEMAND(diagram_context != nullptr);

    // Iterate over the subsystems in sorted order, asking each subsystem to
    // map its subslice of configuration derivatives to velocity. This approach
    // is valid because the DiagramContinuousState guarantees that the subsystem
    // states are concatenated in sorted order.
    int q_index = 0;  // The next index to read in qdot.
    int v_index = 0;  // The next index to write in generalized_velocity.
    for (int i = 0; i < num_subsystems(); ++i) {
      // Find the continuous state of subsystem i.
      const Context<T>* subcontext = diagram_context->GetSubsystemContext(i);
      DRAKE_DEMAND(subcontext != nullptr);
      const ContinuousState<T>* sub_xc = subcontext->get_continuous_state();
      // If subsystem i is stateless, skip it.
      if (sub_xc == nullptr) continue;

      // Select the chunk of qdot belonging to subsystem i.
      const int num_q = sub_xc->get_generalized_position().size();
      const Eigen::Ref<const VectorX<T>>& dq_slice =
        qdot.segment(q_index, num_q);

      // Select the chunk of generalized_velocity belonging to subsystem i.
      const int num_v = sub_xc->get_generalized_velocity().size();
      Subvector<T> v_slice(generalized_velocity, v_index, num_v);

      // Delegate the actual mapping to subsystem i itself.
      sorted_systems_[i]->MapQDotToVelocity(*subcontext, dq_slice, &v_slice);

      // Advance the indices.
      v_index += num_v;
      q_index += num_q;
    }
  }

  /// Computes the next update time based on the configured actions, for scalar
  /// types that are arithmetic, or aborts for scalar types that are not
  /// arithmetic.
  void DoCalcNextUpdateTime(const Context<T>& context,
                            UpdateActions<T>* actions) const override {
    DoCalcNextUpdateTimeImpl(context, actions);
  }

  /// Creates a deep copy of this Diagram<double>, converting the scalar type
  /// to AutoDiffXd, and preserving all internal structure. Subclasses
  /// may wish to override to initialize additional member data, or to return a
  /// more specific covariant type.
  /// This is the NVI implementation of ToAutoDiffXd.
  Diagram<AutoDiffXd>* DoToAutoDiffXd() const override {
    using FromType = System<double>;
    using ToType = std::unique_ptr<System<AutoDiffXd>>;
    std::function<ToType(const FromType&)> subsystem_converter{
      [](const FromType& subsystem) {
        return subsystem.ToAutoDiffXd();
      }};
    return ConvertScalarType<AutoDiffXd>(subsystem_converter).release();
  }

  /// Creates a deep copy of this Diagram<double>, converting the scalar type
  /// to symbolic::Expression, and preserving all internal structure. Subclasses
  /// may wish to override to initialize additional member data, or to return a
  /// more specific covariant type.
  /// This is the NVI implementation of ToSymbolic.
  Diagram<symbolic::Expression>* DoToSymbolic() const override {
    using FromType = System<double>;
    using ToType = std::unique_ptr<System<symbolic::Expression>>;
    std::function<ToType(const FromType&)> subsystem_converter{
        [](const FromType& subsystem) {
          return subsystem.ToSymbolic();
        }};
    return ConvertScalarType<symbolic::Expression>(
        subsystem_converter).release();
  }

  BasicVector<T>* DoAllocateInputVector(
      const InputPortDescriptor<T>& descriptor) const override {
    // Ask the subsystem to perform the allocation.
    const PortIdentifier& id = input_port_ids_[descriptor.get_index()];
    const System<T>* subsystem = id.first;
    const int subindex = id.second;
    return subsystem->AllocateInputVector(
        subsystem->get_input_port(subindex)).release();
  }

  AbstractValue* DoAllocateInputAbstract(
      const InputPortDescriptor<T>& descriptor) const override {
    // Ask the subsystem to perform the allocation.
    const PortIdentifier& id = input_port_ids_[descriptor.get_index()];
    const System<T>* subsystem = id.first;
    const int subindex = id.second;
    return subsystem->AllocateInputAbstract(
        subsystem->get_input_port(subindex)).release();
  }

 private:
  /// Uses this Diagram<double> to manufacture a Diagram<NewType>, given a
  /// @p converter for subsystems from System<double> to System<NewType>.
  /// SFINAE overload for std::is_same<T, double>.
  ///
  /// @tparam NewType The scalar type to which to convert.
  /// @tparam T1 SFINAE boilerplate.
  template <typename NewType, typename T1 = T>
  std::unique_ptr<Diagram<NewType>> ConvertScalarType(
      std::function<std::unique_ptr<System<NewType>>(
          const System<
              std::enable_if_t<std::is_same<T1, double>::value, double>>&)>
          converter) const {
    std::vector<std::unique_ptr<System<NewType>>> new_systems;
    // Recursively convert all the subsystems.
    std::map<const System<T1>*, const System<NewType>*> old_to_new_map;
    for (const auto& old_system : registered_systems_) {
      new_systems.push_back(converter(*old_system));
      old_to_new_map[old_system.get()] = new_systems.back().get();
    }

    // Set up the blueprint.
    typename Diagram<NewType>::Blueprint blueprint;
    // Make all the inputs and outputs.
    for (const PortIdentifier& id : input_port_ids_) {
      const System<NewType>* new_system = old_to_new_map[id.first];
      const int port = id.second;
      blueprint.input_port_ids.emplace_back(new_system, port);
    }
    for (const PortIdentifier& id : output_port_ids_) {
      const System<NewType>* new_system = old_to_new_map[id.first];
      const int port = id.second;
      blueprint.output_port_ids.emplace_back(new_system, port);
    }
    // Make all the connections.
    for (const auto& edge : dependency_graph_) {
      const PortIdentifier& old_dest = edge.first;
      const System<NewType>* const dest_system = old_to_new_map[old_dest.first];
      const int dest_port = old_dest.second;
      const typename Diagram<NewType>::PortIdentifier new_dest{dest_system,
                                                               dest_port};

      const PortIdentifier& old_src = edge.second;
      const System<NewType>* const src_system = old_to_new_map[old_src.first];
      const int src_port = old_src.second;
      const typename Diagram<NewType>::PortIdentifier new_src{src_system,
                                                              src_port};

      blueprint.dependency_graph[new_dest] = new_src;
    }
    // Preserve the sort order.
    for (const System<T1>* system : sorted_systems_) {
      blueprint.sorted_systems.push_back(old_to_new_map[system]);
    }

    // Construct a new Diagram of type NewType from the blueprint.
    std::unique_ptr<Diagram<NewType>> new_diagram(
        new Diagram<NewType>(blueprint));
    new_diagram->Own(std::move(new_systems));
    return std::move(new_diagram);
  }

  /// Aborts at runtime.
  /// SFINAE overload for !std::is_same<T, double>.
  ///
  /// @tparam NewType The scalar type to which to convert.
  /// @tparam T1 SFINAE boilerplate.
  template <typename NewType, typename T1 = T>
  std::unique_ptr<Diagram<NewType>> ConvertScalarType(
      std::function<std::unique_ptr<System<NewType>>(
          const System<
              std::enable_if_t<!std::is_same<T1, double>::value, double>>&)>
          converter) const {
    DRAKE_ABORT_MSG(
        "Scalar type conversion is only supported from Diagram<double>.");
  }

  // Aborts for scalar types that are not numeric, since there is no reasonable
  // definition of "next update time" outside of the real line.
  //
  // @tparam T1 SFINAE boilerplate for the scalar type. Do not set.
  template <typename T1 = T>
  typename std::enable_if<!is_numeric<T1>::value>::type
  DoCalcNextUpdateTimeImpl(const Context<T1>& context,
                           UpdateActions<T1>* actions) const {
    DRAKE_ABORT_MSG(
        "The default implementation of Diagram<T>::DoCalcNextUpdateTime "
        "only works with types that are drake::is_numeric.");
  }

  // Computes the next update time across all the scheduled events, for
  // scalar types that are numeric.
  //
  // @tparam T1 SFINAE boilerplate for the scalar type. Do not set.
  template <typename T1 = T>
  typename std::enable_if<is_numeric<T1>::value>::type DoCalcNextUpdateTimeImpl(
      const Context<T1>& context, UpdateActions<T1>* actions) const {
    auto diagram_context = dynamic_cast<const DiagramContext<T1>*>(&context);
    DRAKE_DEMAND(diagram_context != nullptr);

    actions->time = std::numeric_limits<T1>::infinity();

    // Iterate over the subsystems in sorted order, and harvest the most
    // imminent updates.
    std::vector<UpdateActions<T1>> sub_actions(num_subsystems());
    for (int i = 0; i < num_subsystems(); ++i) {
      const Context<T1>* subcontext = diagram_context->GetSubsystemContext(i);
      DRAKE_DEMAND(subcontext != nullptr);
      const T1 time =
          sorted_systems_[i]->CalcNextUpdateTime(*subcontext, &sub_actions[i]);
      if (time < actions->time) {
        actions->time = time;
      }
    }

    // If no discrete actions are needed, bail early.
    if (actions->time == std::numeric_limits<T1>::infinity()) {
      return;
    }

    std::vector<std::pair<int, UpdateActions<T1>>> publishers;
    std::vector<std::pair<int, UpdateActions<T1>>> updaters;
    std::vector<std::pair<int, UpdateActions<T1>>> unrestricted_updaters;
    for (int i = 0; i < num_subsystems(); i++) {
      // Ignore the subsystems that aren't among the most imminent updates.
      if (sub_actions[i].time > actions->time) continue;
      if (internal::HasEvent(sub_actions[i],
                             DiscreteEvent<T1>::kPublishAction)) {
        publishers.emplace_back(i, sub_actions[i]);
      }
      if (internal::HasEvent(sub_actions[i],
                             DiscreteEvent<T1>::kDiscreteUpdateAction)) {
        updaters.emplace_back(i, sub_actions[i]);
      }
      if (internal::HasEvent(sub_actions[i],
                             DiscreteEvent<T1>::kUnrestrictedUpdateAction)) {
        unrestricted_updaters.emplace_back(i, sub_actions[i]);
      }
    }
    DRAKE_ASSERT(!publishers.empty() || !updaters.empty() ||
                 !unrestricted_updaters.empty());

    // Request a publish event, if our subsystems want it.
    if (!publishers.empty()) {
      DiscreteEvent<T1> event;
      event.action = DiscreteEvent<T1>::kPublishAction;
      event.do_publish = std::bind(&Diagram<T1>::HandlePublish, this,
                                   std::placeholders::_1, /* context */
                                   publishers);
      actions->events.push_back(event);
    }

    // Request an update event, if our subsystems want it.
    if (!updaters.empty()) {
      DiscreteEvent<T1> event;
      event.action = DiscreteEvent<T1>::kDiscreteUpdateAction;
      event.do_calc_discrete_variable_update = std::bind(
                                  &Diagram<T1>::HandleUpdate,
                                  this,
                                  std::placeholders::_1, /* context */
                                  std::placeholders::_2, /* difference state */
                                  updaters);
      actions->events.push_back(event);
    }

    // Request an unrestricted update event, if our subsystems want it.
    if (!unrestricted_updaters.empty()) {
      DiscreteEvent<T1> event;
      event.action = DiscreteEvent<T1>::kUnrestrictedUpdateAction;
      event.do_unrestricted_update = std::bind(
                                  &Diagram<T1>::HandleUnrestrictedUpdate,
                                  this,
                                  std::placeholders::_1, /* context */
                                  std::placeholders::_2, /* state */
                                  unrestricted_updaters);
      actions->events.push_back(event);
    }
  }

  // A structural outline of a Diagram, produced by DiagramBuilder.
  struct Blueprint {
    // The ordered subsystem ports that are inputs to the entire diagram.
    std::vector<PortIdentifier> input_port_ids;
    // The ordered subsystem ports that are outputs of the entire diagram.
    std::vector<PortIdentifier> output_port_ids;
    // A map from the input ports of constituent systems to the output ports
    // on which they depend. This graph is possibly cyclic, but must not
    // contain an algebraic loop.
    std::map<PortIdentifier, PortIdentifier> dependency_graph;
    // A list of the systems in the dependency graph in a valid, sorted
    // execution order, such that if EvalOutput is called on each system in
    // succession, every system will have valid inputs by the time its turn
    // comes.
    std::vector<const System<T>*> sorted_systems;
  };

  // Constructs a Diagram from the Blueprint that a DiagramBuilder produces.
  // This constructor is private because only DiagramBuilder calls it.
  explicit Diagram(const Blueprint& blueprint) { Initialize(blueprint); }

  // Validates the given @p blueprint and sets up the Diagram accordingly.
  void Initialize(const Blueprint& blueprint) {
    // The Diagram must not already be initialized.
    DRAKE_DEMAND(sorted_systems_.empty());
    // The initialization must be nontrivial.
    DRAKE_DEMAND(!blueprint.sorted_systems.empty());

    // Copy the data from the blueprint into private member variables.
    dependency_graph_ = blueprint.dependency_graph;
    sorted_systems_ = blueprint.sorted_systems;
    input_port_ids_ = blueprint.input_port_ids;
    output_port_ids_ = blueprint.output_port_ids;

    // Generate a map from the System pointer to its index in the sort order.
    for (int i = 0; i < num_subsystems(); ++i) {
      sorted_systems_map_[sorted_systems_[i]] = i;
    }

    // Every system must appear in the sort order exactly once.
    DRAKE_DEMAND(sorted_systems_.size() == sorted_systems_map_.size());
    // Every port named in the dependency_graph_ must actually exist.
    DRAKE_ASSERT(PortsAreValid());
    // The sort order must square with the dependency_graph_.
    DRAKE_ASSERT(SortOrderIsCorrect());

    // Add the inputs to the Diagram topology, and check their invariants.
    for (const PortIdentifier& id : input_port_ids_) {
      ExportInput(id);
    }
    for (const PortIdentifier& id : output_port_ids_) {
      ExportOutput(id);
    }
  }

  // Takes ownership of the @p registered_systems from DiagramBuilder.
  void Own(std::vector<std::unique_ptr<System<T>>> registered_systems) {
    // We must be given something to own.
    DRAKE_DEMAND(!registered_systems.empty());
    // We must not already own any subsystems.
    DRAKE_DEMAND(registered_systems_.empty());
    // The subsystems we are being given to own must be exactly the set of
    // subsystems for which we have an execution order.
    DRAKE_DEMAND(registered_systems.size() == sorted_systems_.size());
    for (const auto& system : registered_systems) {
      const auto it = sorted_systems_map_.find(system.get());
      DRAKE_DEMAND(it != sorted_systems_map_.end());
    }
    // All of those checks having passed, take ownership of the subsystems.
    registered_systems_ = std::move(registered_systems);
    // Inform the constituent system it's bound to this Diagram.
    for (auto& system : registered_systems_) {
      system->set_parent(this);
    }
  }

  // Exposes the given port as an input of the Diagram.
  void ExportInput(const PortIdentifier& port) {
    const System<T>* const sys = port.first;
    const int port_index = port.second;
    // Fail quickly if this system is not part of the sort order.
    GetSystemIndexOrAbort(sys);

    // Add this port to our externally visible topology.
    const auto& subsystem_descriptor = sys->get_input_port(port_index);
    this->DeclareInputPort(subsystem_descriptor.get_data_type(),
                           subsystem_descriptor.size());
  }

  // Exposes the given port as an output of the Diagram.
  void ExportOutput(const PortIdentifier& port) {
    const System<T>* const sys = port.first;
    const int port_index = port.second;
    // Fail quickly if this system is not part of the sort order.
    GetSystemIndexOrAbort(sys);

    // Add this port to our externally visible topology.
    const auto& subsystem_descriptor = sys->get_output_port(port_index);
    this->DeclareOutputPort(subsystem_descriptor.get_data_type(),
                            subsystem_descriptor.size());
  }

  // Evaluates the value of the output port with the given @p id in the given
  // @p context.
  //
  // TODO(david-german-tri): Add Diagram-level cache entries to keep track of
  // whether a given output port has already been evaluated.  Right now, we
  // are recomputing every intermediate output to satisfy every system that
  // depends on it, recursively. This is O(N^2 * M), where M is the number of
  // output ports the Diagram exposes, and N is the number of intermediate
  // output ports the Diagram contains.
  void EvaluateOutputPort(const DiagramContext<T>& context,
                          const PortIdentifier& id) const {
    const System<T>* const system = id.first;
    const int i = GetSystemIndexOrAbort(system);
    SPDLOG_TRACE(log(), "Evaluating output for subsystem {}, port {}",
                 system->GetPath(), id.second);
    const Context<T>* subsystem_context = context.GetSubsystemContext(i);
    SystemOutput<T>* subsystem_output = context.GetSubsystemOutput(i);
    // TODO(david-german-tri): Once #2890 is resolved, only evaluate the
    // particular port specified in id.second.
    system->CalcOutput(*subsystem_context, subsystem_output);
  }

  // Returns the index of the given @p sys in the sorted order of this diagram,
  // or aborts if @p sys is not a member of the diagram.
  int GetSystemIndexOrAbort(const System<T>* sys) const {
    auto it = sorted_systems_map_.find(sys);
    DRAKE_DEMAND(it != sorted_systems_map_.end());
    return it->second;
  }

  // Converts a PortIdentifier to a DiagramContext::PortIdentifier.
  // The DiagramContext::PortIdentifier contains the index of the System in the
  // sorted order of the diagram, instead of an actual pointer to the System.
  typename DiagramContext<T>::PortIdentifier ConvertToContextPortIdentifier(
      const PortIdentifier& id) const {
    typename DiagramContext<T>::PortIdentifier output;
    output.first = GetSystemIndexOrAbort(id.first);
    output.second = id.second;
    return output;
  }

  // Sets up the OutputPortValue pointers in @p output to point to the subsystem
  // output values, found in @p context, that are the outputs of this Diagram.
  void ExposeSubsystemOutputs(const DiagramContext<T>& context,
                              internal::DiagramOutput<T>* output) const {
    // The number of output ports of this diagram must equal the number of
    // ports in the provided DiagramOutput.
    const int num_ports = static_cast<int>(output_port_ids_.size());
    DRAKE_DEMAND(output->get_num_ports() == num_ports);

    for (int i = 0; i < num_ports; ++i) {
      const PortIdentifier& id = output_port_ids_[i];
      // For each configured output port ID, obtain from the DiagramContext the
      // actual OutputPortValue that supplies its value.
      const int sys_index = GetSystemIndexOrAbort(id.first);
      const int port_index = id.second;
      SystemOutput<T>* subsystem_output = context.GetSubsystemOutput(sys_index);
      OutputPortValue* output_port_value =
          subsystem_output->get_mutable_port_value(port_index);

      // Then, put a pointer to that OutputPortValue in the DiagramOutput.
      (*output->get_mutable_port_values())[i] = output_port_value;
    }
  }

  // Returns true if every port mentioned in the dependency_graph_ exists.
  bool PortsAreValid() const {
    for (const auto& entry : dependency_graph_) {
      const PortIdentifier& dest = entry.first;
      const PortIdentifier& src = entry.second;
      if (dest.second < 0 || dest.second >= dest.first->get_num_input_ports()) {
        return false;
      }
      if (src.second < 0 || src.second >= src.first->get_num_output_ports()) {
        return false;
      }
    }
    return true;
  }

  // Returns true if every System precedes all its dependents in
  // sorted_systems_.
  bool SortOrderIsCorrect() const {
    for (const auto& entry : dependency_graph_) {
      const System<T>* const dest = entry.first.first;
      const System<T>* const src = entry.second.first;
      // If the destination system has no direct feedthrough, it does not
      // matter whether it is sorted before or after the systems on which
      // it depends.
      if (!dest->HasAnyDirectFeedthrough()) {
        continue;
      }
      if (GetSystemIndexOrAbort(dest) <= GetSystemIndexOrAbort(src)) {
        return false;
      }
    }
    return true;
  }

  // Returns true if any input port of the Diagram might feed directly through
  // to the given @p output_port_id, which is an output port of some subsystem
  // of the Diagram.
  bool HasDirectFeedthroughFromAnyInput(
      const PortIdentifier& output_port_id) const {
    // TODO(david-german-tri): Make this less conservative by checking only
    // HasAnyDirectFeedthrough(input, output), i.e. inspecting the graph of
    // ports, not the graph of systems.

    // If the system producing output_port_id has no direct-feedthrough, then
    // there is definitely no direct-feedthrough to output_port_id.
    const System<T>* system = output_port_id.first;
    if (!system->HasAnyDirectFeedthrough()) {
      return false;
    }

    // Otherwise, we need to check each of the system's input ports.
    for (int i = 0; i < system->get_num_input_ports(); ++i) {
      PortIdentifier input_port_id{system, i};

      // If input_port_id is an input port of the entire Diagram,
      // there may be direct-feedthrough to output_port_id. Since we don't have
      // a full sparsity matrix yet, we err on the side of caution and report
      // direct-feedthrough.
      //
      // TODO(david-german-tri): This should be an O(1) lookup, not O(N).
      for (const PortIdentifier& diagram_input_id : input_port_ids_) {
        if (diagram_input_id == input_port_id) {
          return true;
        }
      }

      // If input_port_id is connected to some other System's output port,
      // there is direct feedthrough to output_port_id if there is
      // direct-feedthrough to the upstream output port. Check recursively.
      auto upstream_it = dependency_graph_.find(input_port_id);
      if (upstream_it != dependency_graph_.end()) {
        const PortIdentifier& upstream_port = upstream_it->second;
        if (HasDirectFeedthroughFromAnyInput(upstream_port)) {
          return true;
        }
      }
    }

    // If none of the system's input ports create a direct-feedthrough path
    // back to an input of the Diagram, there is no direct-feedthrough to
    // output_port_id.
    return false;
  }

  /// Handles Publish callbacks that were registered in DoCalcNextUpdateTime.
  /// Dispatches the Publish events to the subsystems that requested them.
  void HandlePublish(
      const Context<T>& context,
      const std::vector<std::pair<int, UpdateActions<T>>>& sub_actions) const {
    auto diagram_context = dynamic_cast<const DiagramContext<T>*>(&context);
    DRAKE_DEMAND(diagram_context != nullptr);

    for (const auto& action : sub_actions) {
      const int index = action.first;
      const UpdateActions<T>& action_details = action.second;
      DRAKE_DEMAND(index >= 0 && index < num_subsystems());

      const Context<T>* subcontext =
          diagram_context->GetSubsystemContext(index);
      DRAKE_DEMAND(subcontext != nullptr);
      for (const DiscreteEvent<T>& event : action_details.events) {
        if (event.action == DiscreteEvent<T>::kPublishAction) {
          sorted_systems_[index]->Publish(*subcontext, event);
        }
      }
    }
  }

  /// Handles Update callbacks that were registered in DoCalcNextUpdateTime.
  /// Dispatches the Publish events to the subsystems that requested them.
  void HandleUpdate(
      const Context<T>& context, DiscreteState<T>* update,
      const std::vector<std::pair<int, UpdateActions<T>>>& sub_actions) const {
    auto diagram_context = dynamic_cast<const DiagramContext<T>*>(&context);
    DRAKE_DEMAND(diagram_context != nullptr);
    auto diagram_differences =
        dynamic_cast<internal::DiagramDiscreteVariables<T>*>(update);
    DRAKE_DEMAND(diagram_differences != nullptr);

    // As a baseline, initialize all the difference variables to their
    // current values.
    for (int i = 0; i < diagram_differences->size(); ++i) {
      diagram_differences->get_mutable_discrete_state(i)->set_value(
          context.get_discrete_state(i)->get_value());
    }

    // Then, allow the systems that wanted to update a difference variable
    // to do so.
    for (const auto& action : sub_actions) {
      const int index = action.first;
      const UpdateActions<T>& action_details = action.second;
      DRAKE_DEMAND(index >= 0 && index < num_subsystems());

      // Get the context and the difference state for the specified system.
      const Context<T>* subcontext =
          diagram_context->GetSubsystemContext(index);
      DRAKE_DEMAND(subcontext != nullptr);
      DiscreteState<T>* subdifference =
          diagram_differences->get_mutable_subdifference(index);
      DRAKE_DEMAND(subdifference != nullptr);

      // Do that system's update actions.
      for (const DiscreteEvent<T>& event : action_details.events) {
        if (event.action == DiscreteEvent<T>::kDiscreteUpdateAction) {
          sorted_systems_[index]->CalcDiscreteVariableUpdates(*subcontext,
                                                              event,
                                                              subdifference);
        }
      }
    }
  }

  /// Handles Update callbacks that were registered in DoCalcNextUpdateTime.
  /// Dispatches the UnrestrictedUpdate events to the subsystems that requested
  /// them.
  void HandleUnrestrictedUpdate(
      const Context<T>& context, State<T>* state,
      const std::vector<std::pair<int, UpdateActions<T>>>& sub_actions) const {
    auto diagram_context = dynamic_cast<const DiagramContext<T>*>(&context);
    DRAKE_DEMAND(diagram_context != nullptr);
    auto diagram_state = dynamic_cast<DiagramState<T>*>(state);
    DRAKE_DEMAND(diagram_state != nullptr);

    // No need to set state to context's state, since it has already been done
    // in System::CalcUnrestrictedUpdate().

    for (const auto& action : sub_actions) {
      const int index = action.first;
      const UpdateActions<T>& action_details = action.second;
      DRAKE_DEMAND(index >= 0 && index < num_subsystems());

      // Get the context and the state for the specified system.
      const Context<T>* subcontext =
          diagram_context->GetSubsystemContext(index);
      DRAKE_DEMAND(subcontext != nullptr);
      State<T>* substate = diagram_state->get_mutable_substate(index);
      DRAKE_DEMAND(substate != nullptr);

      // Do that system's update actions.
      for (const DiscreteEvent<T>& event : action_details.events) {
        if (event.action == DiscreteEvent<T>::kUnrestrictedUpdateAction) {
          sorted_systems_[index]->CalcUnrestrictedUpdate(*subcontext,
                                                         event,
                                                         substate);
        }
      }
    }
  }

  int num_subsystems() const {
    return static_cast<int>(sorted_systems_.size());
  }

  // A map from the input ports of constituent systems, to the output ports of
  // the systems on which they depend.
  std::map<PortIdentifier, PortIdentifier> dependency_graph_;

  // The topologically sorted list of Systems in this Diagram.
  std::vector<const System<T>*> sorted_systems_;

  // The Systems in this Diagram, which are owned by this Diagram, in the order
  // they were registered.
  std::vector<std::unique_ptr<System<T>>> registered_systems_;

  // For fast conversion queries: what is the index of this System in the
  // sorted order?
  std::map<const System<T>*, int> sorted_systems_map_;

  // The ordered inputs and outputs of this Diagram.
  std::vector<PortIdentifier> input_port_ids_;
  std::vector<PortIdentifier> output_port_ids_;

  // For all T, Diagram<T> considers DiagramBuilder<T> a friend, so that the
  // builder can set the internal state correctly.
  friend class DiagramBuilder<T>;

  // For all T, Diagram<T> considers Diagram<double> a friend, so that
  // Diagram<double> can provide transmogrification methods to more flavorful
  // scalar types.  See Diagram<T>::ConvertScalarType.
  friend class Diagram<double>;
};

}  // namespace systems
}  // namespace drake
