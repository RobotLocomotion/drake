#pragma once

#include <algorithm>
#include <functional>
#include <map>
#include <set>
#include <stdexcept>
#include <vector>

#include "drake/common/drake_assert.h"
#include "drake/systems/framework/cache.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/diagram_context.h"
#include "drake/systems/framework/state.h"
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
    DRAKE_ABORT_UNLESS(index >= 0 && index < get_num_ports());
    return ports_[index];
  }

  const OutputPort& get_port(int index) const override {
    DRAKE_ABORT_UNLESS(index >= 0 && index < get_num_ports());
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
                   [](auto& p) { return p.get(); });
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
class Diagram : public System<T> {
 public:
  typedef typename std::pair<const System<T>*, int> PortIdentifier;

  ~Diagram() override {}

  std::unique_ptr<ContextBase<T>> CreateDefaultContext() const override {
    // Reserve inputs as specified during Diagram initialization.
    auto context = std::make_unique<DiagramContext<T>>();

    // Add each constituent system to the Context.
    for (int i = 0; i < static_cast<int>(sorted_systems_.size()); ++i) {
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
    return std::unique_ptr<ContextBase<T>>(context.release());
  }

  std::unique_ptr<SystemOutput<T>> AllocateOutput(
      const ContextBase<T>& context) const override {
    auto diagram_context = dynamic_cast<const DiagramContext<T>*>(&context);
    DRAKE_ABORT_UNLESS(diagram_context != nullptr);

    // The output ports of this Diagram are output ports of its constituent
    // systems. Create a DiagramOutput with that many ports.
    auto output = std::make_unique<internal::DiagramOutput<T>>();
    output->get_mutable_ports()->resize(output_port_ids_.size());
    ExposeSubsystemOutputs(*diagram_context, output.get());
    return std::unique_ptr<SystemOutput<T>>(output.release());
  }

  void EvalOutput(const ContextBase<T>& context,
                  SystemOutput<T>* output) const override {
    // Down-cast the context and output to DiagramContext and DiagramOutput.
    auto diagram_context = dynamic_cast<const DiagramContext<T>*>(&context);
    DRAKE_ABORT_UNLESS(diagram_context != nullptr);
    auto diagram_output = dynamic_cast<internal::DiagramOutput<T>*>(output);
    DRAKE_ABORT_UNLESS(diagram_output != nullptr);

    // Populate the output with pointers to the appropriate subsystem outputs
    // in the DiagramContext. We do this on every call to EvalOutput, so
    // that the diagram_context and diagram_output are not tightly coupled.
    ExposeSubsystemOutputs(*diagram_context, diagram_output);

    // Since the diagram output now contains pointers to the subsystem outputs,
    // all we need to do is compute all the subsystem outputs in sorted order.
    //
    // TODO(david-german-tri): This can be made less conservative. We don't need
    // to compute intermediate outputs that don't affect the diagram outputs.
    ComputeAllSubsystemOutputs(diagram_context);
  }

  std::unique_ptr<ContinuousState<T>> AllocateTimeDerivatives() const override {
    std::vector<std::unique_ptr<ContinuousState<T>>> sub_derivatives;
    for (const System<T>* const system : sorted_systems_) {
      sub_derivatives.push_back(system->AllocateTimeDerivatives());
    }
    return std::unique_ptr<ContinuousState<T>>(
        new internal::DiagramTimeDerivatives<T>(std::move(sub_derivatives)));
  }

  void EvalTimeDerivatives(const ContextBase<T>& context,
                           ContinuousState<T>* derivatives) const override {
    // Freshen all the subsystem inputs to match the provided context.
    //
    // TODO(david-german-tri): This can be made less conservative: we don't
    // need to freshen inputs to subsystems with no state.
    auto diagram_context = dynamic_cast<const DiagramContext<T>*>(&context);
    DRAKE_ABORT_UNLESS(diagram_context != nullptr);
    ComputeAllSubsystemOutputs(diagram_context);

    auto diagram_derivatives =
        dynamic_cast<DiagramContinuousState<T>*>(derivatives);
    DRAKE_ABORT_UNLESS(diagram_derivatives != nullptr);
    const int n = diagram_derivatives->get_num_substates();
    DRAKE_ABORT_UNLESS(static_cast<int>(sorted_systems_.size()) == n);

    // Evaluate the derivatives of each constituent system.
    for (int i = 0; i < n; ++i) {
      const ContextBase<T>* subcontext =
          diagram_context->GetSubsystemContext(i);
      ContinuousState<T>* subderivatives =
          diagram_derivatives->get_mutable_substate(i);
      sorted_systems_[i]->EvalTimeDerivatives(*subcontext, subderivatives);
    }
  }

  void MapVelocityToConfigurationDerivatives(
      const ContextBase<T>& context, const StateVector<T>& generalized_velocity,
      StateVector<T>* configuration_derivatives) const override {
    // TODO(david-german-tri): Actually map velocity to derivatives.
  }

  /// Retrieves the state derivatives for a particular subsystem from the
  /// derivatives for the entire diagram.
  const ContinuousState<T>& GetSubsystemDerivatives(
      const ContinuousState<T>& derivatives, const System<T>* subsystem) const {
    DRAKE_ABORT_UNLESS(subsystem != nullptr);
    auto diagram_derivatives =
        dynamic_cast<const DiagramContinuousState<T>*>(&derivatives);
    DRAKE_ABORT_UNLESS(diagram_derivatives != nullptr);
    auto substate =
        diagram_derivatives->get_substate(GetSystemIndex(subsystem));
    // TODO(david-german-tri): We should fail softer than this for stateless
    // systems.
    DRAKE_ABORT_UNLESS(substate != nullptr);
    return *substate;
  }

  /// Returns the sub-context that corresponds to the system @p subsystem.
  /// Classes inheriting from %Diagram need access to this method in order to
  /// pass their constituent subsystem's the apropriate subcontext.
  ContextBase<T>* GetMutableSubsystemContext(ContextBase<T>* context,
                                             const System<T>* subsystem) const {
    DRAKE_ABORT_UNLESS(context != nullptr);
    DRAKE_ABORT_UNLESS(subsystem != nullptr);
    auto diagram_context = dynamic_cast<DiagramContext<T>*>(context);
    DRAKE_ABORT_UNLESS(diagram_context != nullptr);
    const int i = GetSystemIndex(subsystem);
    return diagram_context->GetMutableSubsystemContext(i);
  }

  /// Retrieves the state for a particular subsystem from the context for the
  /// entire diagram. Invalidates all entries in that subsystem's cache that
  /// depend on State. Returns nullptr if the subsystem is not part of the
  /// diagram.
  ///
  /// TODO(david-german-tri): Provide finer-grained accessors for finer-grained
  /// invalidation.
  State<T>* GetMutableSubsystemState(ContextBase<T>* context,
                                     const System<T>* subsystem) const {
    return GetMutableSubsystemContext(context, subsystem)->get_mutable_state();
  }

 protected:
  /// Constructs an uninitialized Diagram. Subclasses that use this constructor
  /// are obligated to call DiagramBuilder::BuildInto(this).
  Diagram() {}

  void DoPublish(const ContextBase<T>& context) const override {
    // Freshen all the subsystem inputs to match the provided context.
    //
    // TODO(david-german-tri): This can be made less conservative: we don't
    // need to freshen inputs to subsystems that don't Publish.
    auto diagram_context = dynamic_cast<const DiagramContext<T>*>(&context);
    DRAKE_ABORT_UNLESS(diagram_context != nullptr);
    ComputeAllSubsystemOutputs(diagram_context);

    for (const System<T>* const system : sorted_systems_) {
      const int index = GetSystemIndex(system);
      const ContextBase<T>* subsystem_context =
          diagram_context->GetSubsystemContext(index);
      system->Publish(*subsystem_context);
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
    DRAKE_ABORT_UNLESS(sorted_systems_.empty());
    // The initialization must be nontrivial.
    DRAKE_ABORT_UNLESS(!blueprint.sorted_systems.empty());

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
    DRAKE_ABORT_UNLESS(sorted_systems_.size() == sorted_systems_map_.size());
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

  // Exposes the given port as an input of the Diagram.
  void ExportInput(const PortIdentifier& port) {
    const System<T>* const sys = port.first;
    const int port_index = port.second;
    // Fail quickly if this system is not part of the sort order.
    DRAKE_ABORT_UNLESS(GetSystemIndex(sys) >= 0);

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
    DRAKE_ABORT_UNLESS(GetSystemIndex(sys) >= 0);

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

  int GetSystemIndex(const System<T>* sys) const {
    auto it = sorted_systems_map_.find(sys);
    DRAKE_ABORT_UNLESS(it != sorted_systems_map_.end());
    return it->second;
  }

  // Converts a PortIdentifier to a DiagramContext::PortIdentifier.
  // The DiagramContext::PortIdentifier contains the index of the System in the
  // sorted order of the diagram, instead of an actual pointer to the System.
  typename DiagramContext<T>::PortIdentifier ConvertToContextPortIdentifier(
      const PortIdentifier& id) const {
    typename DiagramContext<T>::PortIdentifier output;
    output.first = GetSystemIndex(id.first);
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
    DRAKE_ABORT_UNLESS(output->get_num_ports() == num_ports);

    for (int i = 0; i < num_ports; ++i) {
      const PortIdentifier& id = output_port_ids_[i];
      // For each configured output port ID, obtain from the DiagramContext the
      // actual OutputPort that produces it.
      const int sys_index = GetSystemIndex(id.first);
      const int port_index = id.second;
      SystemOutput<T>* subsystem_output = context.GetSubsystemOutput(sys_index);
      OutputPort* output_port = subsystem_output->get_mutable_port(port_index);

      // Then, put a pointer to that OutputPort in the DiagramOutput.
      (*output->get_mutable_ports())[i] = output_port;
    }
  }

  // In sorted order, compute the outputs for all subsystems. This is also a
  // blunt way to update the inputs for all subsystems to match the given
  // @p context.
  void ComputeAllSubsystemOutputs(const DiagramContext<T>* context) const {
    DRAKE_ABORT_UNLESS(context != nullptr);
    // TODO(david-german-tri): Use the diagram-level cache to skip systems that
    // are already fresh.
    for (const System<T>* const system : sorted_systems_) {
      const int index = GetSystemIndex(system);
      const ContextBase<T>* subsystem_context =
          context->GetSubsystemContext(index);
      SystemOutput<T>* subsystem_output = context->GetSubsystemOutput(index);
      system->EvalOutput(*subsystem_context, subsystem_output);
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
      if (GetSystemIndex(dest) <= GetSystemIndex(src)) {
        return false;
      }
    }
    return true;
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
