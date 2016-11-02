#pragma once

#include <algorithm>
#include <functional>
#include <map>
#include <set>
#include <stdexcept>
#include <utility>
#include <vector>

#include "drake/common/drake_assert.h"
#include "drake/common/text_logging.h"
#include "drake/systems/framework/cache.h"
#include "drake/systems/framework/diagram_context.h"
#include "drake/systems/framework/leaf_context.h"
#include "drake/systems/framework/state.h"
#include "drake/systems/framework/subvector.h"
#include "drake/systems/framework/system.h"
#include "drake/systems/framework/system_port_descriptor.h"

namespace drake {
namespace systems {

template <typename T>
class DiagramBuilder;

namespace internal {

/// DiagramOutput is an implementation of SystemOutput that holds unowned
/// OutputPort pointers. It is used to expose the outputs of constituent
/// systems as outputs of a Diagram.
///
/// @tparam T The type of the output data. Must be a valid Eigen scalar.
template <typename T>
class DiagramOutput : public SystemOutput<T> {
 public:
  int get_num_ports() const override { return static_cast<int>(ports_.size()); }

  OutputPort* get_mutable_port(int index) override {
    DRAKE_DEMAND(index >= 0 && index < get_num_ports());
    return ports_[index];
  }

  const OutputPort& get_port(int index) const override {
    DRAKE_DEMAND(index >= 0 && index < get_num_ports());
    return *ports_[index];
  }

  std::vector<OutputPort*>* get_mutable_ports() { return &ports_; }

 protected:
  // Returns a clone that has the same number of output ports, set to nullptr.
  DiagramOutput<T>* DoClone() const override {
    DiagramOutput<T>* clone = new DiagramOutput<T>();
    clone->ports_.resize(get_num_ports());
    return clone;
  }

 private:
  std::vector<OutputPort*> ports_;
};

/// DiagramTimeDerivatives is a version of DiagramContinuousState that owns
/// the constituent continuous states. As the name implies, it is only useful
/// for the time derivatives.
template <typename T>
class DiagramTimeDerivatives : public DiagramContinuousState<T> {
 public:
  explicit DiagramTimeDerivatives(
      std::vector<std::unique_ptr<ContinuousState<T>>> substates)
      : DiagramContinuousState<T>(Unpack(substates)),
        substates_(std::move(substates)) {}

  ~DiagramTimeDerivatives() override {}

 private:
  template <typename U>
  std::vector<U*> Unpack(const std::vector<std::unique_ptr<U>>& in) {
    std::vector<U*> out(in.size());
    std::transform(in.begin(), in.end(), out.begin(),
                   [](const std::unique_ptr<U>& p) { return p.get(); });
    return out;
  }

  std::vector<std::unique_ptr<ContinuousState<T>>> substates_;
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

  /// Returns true if any output of the Diagram might have direct-feedthrough
  /// from any input of the Diagram.
  bool has_any_direct_feedthrough() const override {
    // TODO(david-german-tri, bradking): Make this less conservative once the
    // sparsity matrix is available.

    // For each output, see whether it has direct feedthrough all the way back
    // to any input.
    for (const auto& output_port_id : output_port_ids_) {
      if (HasDirectFeedthroughFromAnyInput(output_port_id)) {
        return true;
      }
    }
    return false;
  }

  std::unique_ptr<Context<T>> CreateDefaultContext() const override {
    const int num_systems = static_cast<int>(sorted_systems_.size());
    // Reserve inputs as specified during Diagram initialization.
    auto context = std::make_unique<DiagramContext<T>>(num_systems);

    // Add each constituent system to the Context.
    for (int i = 0; i < num_systems; ++i) {
      const System<T>* const sys = sorted_systems_[i];
      auto subcontext = sys->CreateDefaultContext();
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
    return std::unique_ptr<Context<T>>(context.release());
  }

  std::unique_ptr<SystemOutput<T>> AllocateOutput(
      const Context<T>& context) const override {
    auto diagram_context = dynamic_cast<const DiagramContext<T>*>(&context);
    DRAKE_DEMAND(diagram_context != nullptr);

    // The output ports of this Diagram are output ports of its constituent
    // systems. Create a DiagramOutput with that many ports.
    auto output = std::make_unique<internal::DiagramOutput<T>>();
    output->get_mutable_ports()->resize(output_port_ids_.size());
    ExposeSubsystemOutputs(*diagram_context, output.get());
    return std::unique_ptr<SystemOutput<T>>(output.release());
  }

  void EvalOutput(const Context<T>& context,
                  SystemOutput<T>* output) const override {
    // Down-cast the context and output to DiagramContext and DiagramOutput.
    auto diagram_context = dynamic_cast<const DiagramContext<T>*>(&context);
    DRAKE_DEMAND(diagram_context != nullptr);
    auto diagram_output = dynamic_cast<internal::DiagramOutput<T>*>(output);
    DRAKE_DEMAND(diagram_output != nullptr);

    // Populate the output with pointers to the appropriate subsystem outputs
    // in the DiagramContext. We do this on every call to EvalOutput, so
    // that the diagram_context and diagram_output are not tightly coupled.
    ExposeSubsystemOutputs(*diagram_context, diagram_output);

    // Since the diagram output now contains pointers to the subsystem outputs,
    // all we need to do is ask those subsystem outputs to evaluate themselves.
    // They will recursively evaluate any intermediate inputs that they need.
    for (const PortIdentifier& id : output_port_ids_) {
      EvaluateOutputPort(*diagram_context, id);
    }
  }

  std::unique_ptr<ContinuousState<T>> AllocateTimeDerivatives() const override {
    std::vector<std::unique_ptr<ContinuousState<T>>> sub_derivatives;
    for (const System<T>* const system : sorted_systems_) {
      sub_derivatives.push_back(system->AllocateTimeDerivatives());
    }
    return std::unique_ptr<ContinuousState<T>>(
        new internal::DiagramTimeDerivatives<T>(std::move(sub_derivatives)));
  }

  void EvalTimeDerivatives(const Context<T>& context,
                           ContinuousState<T>* derivatives) const override {
    auto diagram_context = dynamic_cast<const DiagramContext<T>*>(&context);
    DRAKE_DEMAND(diagram_context != nullptr);

    auto diagram_derivatives =
        dynamic_cast<DiagramContinuousState<T>*>(derivatives);
    DRAKE_DEMAND(diagram_derivatives != nullptr);
    const int n = diagram_derivatives->get_num_substates();
    DRAKE_DEMAND(static_cast<int>(sorted_systems_.size()) == n);

    // Evaluate the derivatives of each constituent system.
    for (int i = 0; i < n; ++i) {
      const Context<T>* subcontext = diagram_context->GetSubsystemContext(i);
      ContinuousState<T>* subderivatives =
          diagram_derivatives->get_mutable_substate(i);
      sorted_systems_[i]->EvalTimeDerivatives(*subcontext, subderivatives);
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
      const SystemPortDescriptor<T>& descriptor) const override {
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
  void DoMapVelocityToConfigurationDerivatives(
      const Context<T>& context,
      const Eigen::Ref<const VectorX<T>>& generalized_velocity,
      VectorBase<T>* configuration_derivatives) const override {
    // Check that the dimensions of the continuous state in the context match
    // the dimensions of the provided generalized velocity and configuration
    // derivatives.
    const ContinuousState<T>* xc = context.get_continuous_state();
    DRAKE_DEMAND(xc != nullptr);
    const int nq = xc->get_generalized_position().size();
    const int nv = xc->get_generalized_velocity().size();
    DRAKE_DEMAND(nq == configuration_derivatives->size());
    DRAKE_DEMAND(nv == generalized_velocity.size());

    auto diagram_context = dynamic_cast<const DiagramContext<T>*>(&context);
    DRAKE_DEMAND(diagram_context != nullptr);

    // Iterate over the subsystems in sorted order, asking each subsystem to
    // map its subslice of velocity to configuration derivatives. This approach
    // is valid because the DiagramContinuousState guarantees that the subsystem
    // states are concatenated in sorted order.
    int v_index = 0;  // The next index to read in generalized_velocity.
    int q_index = 0;  // The next index to write in configuration_derivatives.
    for (int i = 0; i < static_cast<int>(sorted_systems_.size()); ++i) {
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

      // Select the chunk of configuration_derivatives belonging to subsystem i.
      const int num_q = sub_xc->get_generalized_position().size();
      Subvector<T> dq_slice(configuration_derivatives, q_index, num_q);

      // Delegate the actual mapping to subsystem i itself.
      sorted_systems_[i]->MapVelocityToConfigurationDerivatives(
          *subcontext, v_slice, &dq_slice);

      // Advance the indices.
      v_index += num_v;
      q_index += num_q;
    }
  }

 private:
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
    for (int i = 0; i < static_cast<int>(sorted_systems_.size()); ++i) {
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
    const auto& subsystem_ports = sys->get_input_ports();
    if (port_index < 0 ||
        port_index >= static_cast<int>(subsystem_ports.size())) {
      throw std::out_of_range("Input port out of range.");
    }
    const auto& subsystem_descriptor = subsystem_ports[port_index];
    SystemPortDescriptor<T> descriptor(
        this, kInputPort, this->get_num_input_ports(),
        subsystem_descriptor.get_data_type(), subsystem_descriptor.get_size(),
        subsystem_descriptor.get_sampling());
    this->DeclareInputPort(descriptor);
  }

  // Exposes the given port as an output of the Diagram.
  void ExportOutput(const PortIdentifier& port) {
    const System<T>* const sys = port.first;
    const int port_index = port.second;
    // Fail quickly if this system is not part of the sort order.
    GetSystemIndexOrAbort(sys);

    // Add this port to our externally visible topology.
    const auto& subsystem_ports = sys->get_output_ports();
    if (port_index < 0 ||
        port_index >= static_cast<int>(subsystem_ports.size())) {
      throw std::out_of_range("Output port out of range.");
    }
    const auto& subsystem_descriptor = subsystem_ports[port_index];
    SystemPortDescriptor<T> descriptor(
        this, kOutputPort, this->get_num_output_ports(),
        subsystem_descriptor.get_data_type(), subsystem_descriptor.get_size(),
        subsystem_descriptor.get_sampling());
    this->DeclareOutputPort(descriptor);
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
    log()->trace("Evaluating output for subsystem {}, port {}",
                 system->GetPath(), id.second);
    const Context<T>* subsystem_context = context.GetSubsystemContext(i);
    SystemOutput<T>* subsystem_output = context.GetSubsystemOutput(i);
    // TODO(david-german-tri): Once #2890 is resolved, only evaluate the
    // particular port specified in id.second.
    system->EvalOutput(*subsystem_context, subsystem_output);
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

  // Sets up the OutputPort pointers in @p output to point to the subsystem
  // outputs, found in @p context, that are the outputs of this Diagram.
  void ExposeSubsystemOutputs(const DiagramContext<T>& context,
                              internal::DiagramOutput<T>* output) const {
    // The number of output ports of this diagram must equal the number of
    // ports in the provided DiagramOutput.
    const int num_ports = static_cast<int>(output_port_ids_.size());
    DRAKE_DEMAND(output->get_num_ports() == num_ports);

    for (int i = 0; i < num_ports; ++i) {
      const PortIdentifier& id = output_port_ids_[i];
      // For each configured output port ID, obtain from the DiagramContext the
      // actual OutputPort that produces it.
      const int sys_index = GetSystemIndexOrAbort(id.first);
      const int port_index = id.second;
      SystemOutput<T>* subsystem_output = context.GetSubsystemOutput(sys_index);
      OutputPort* output_port = subsystem_output->get_mutable_port(port_index);

      // Then, put a pointer to that OutputPort in the DiagramOutput.
      (*output->get_mutable_ports())[i] = output_port;
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
      if (!dest->has_any_direct_feedthrough()) {
        continue;
      }
      if (GetSystemIndexOrAbort(dest) <= GetSystemIndexOrAbort(src)) {
        return false;
      }
    }
    return true;
  }

  // Checks whether any input port of the Diagram feeds directly through to the
  // given @p output_port_id.
  bool HasDirectFeedthroughFromAnyInput(
      const PortIdentifier& output_port_id) const {
    // TODO(david-german-tri, bradking): This can be made less conservative
    // once the sparsity matrix is available.

    // If the system producing output_port_id has no direct-feedthrough, then
    // there is definitely no direct-feedthrough to output_port_id.
    const System<T>* system = output_port_id.first;
    if (!system->has_any_direct_feedthrough()) {
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

  // Diagram objects are neither copyable nor moveable.
  Diagram(const Diagram<T>& other) = delete;
  Diagram& operator=(const Diagram<T>& other) = delete;
  Diagram(Diagram<T>&& other) = delete;
  Diagram& operator=(Diagram<T>&& other) = delete;

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

  friend class DiagramBuilder<T>;
};

}  // namespace systems
}  // namespace drake
