#pragma once

#include <map>
#include <memory>
#include <set>
#include <stdexcept>
#include <utility>
#include <vector>

#include "drake/systems/framework/context_base.h"
#include "drake/systems/framework/state.h"
#include "drake/systems/framework/state_supervector.h"
#include "drake/systems/framework/system_input.h"
#include "drake/systems/framework/system_output.h"

namespace drake {
namespace systems {

/// The DiagramContext is a container for all of the data necessary to uniquely
/// determine the computations performed by a Diagram. Specifically, a
/// DiagramContext contains contexts and outputs for all the constituent
/// Systems,
/// wired up as specified by calls to `DiagramContext::Connect`.
///
/// @tparam T The mathematical type of the context, which must be a valid Eigen
///           scalar.
template <typename T>
class DiagramContext : public ContextBase<T> {
 public:
  typedef int SystemIndex;
  typedef int PortIndex;
  typedef std::pair<SystemIndex, PortIndex> PortIdentifier;

  DiagramContext() {}

  /// Declares a new subsystem in the DiagramContext. Subsystems are identified
  /// by number.
  ///
  /// User code should not call this method. It is for use during Diagram
  /// context allocation only.
  void AddSystem(SystemIndex sys, std::unique_ptr<ContextBase<T>> context,
                 std::unique_ptr<SystemOutput<T>> output) {
    DRAKE_ASSERT(contexts_.find(sys) == contexts_.end());
    contexts_[sys] = std::move(context);

    DRAKE_ASSERT(outputs_.find(sys) == outputs_.end());
    outputs_[sys] = std::move(output);
  }

  /// Declares that a particular input port of a particular subsystem is an
  /// input to the entire Diagram that allocates this Context.
  ///
  /// User code should not call this method. It is for use during Diagram
  /// context allocation only.
  void ExportInput(const PortIdentifier& id) {
    const SystemIndex system_index = id.first;
    if (contexts_.find(system_index) == contexts_.end()) {
      throw std::runtime_error(
          "Cannot add a System as an input until the "
          "System itself has been added.");
    }
    input_ids_.emplace_back(id);
  }

  /// Declares that the output port specified by @p src is connected to the
  /// input port specified by @p dest.
  ///
  /// User code should not call this method. It is for use during Diagram
  /// context allocation only.
  void Connect(const PortIdentifier& src, const PortIdentifier& dest) {
    // Validate and identify the source port.
    SystemIndex src_system_index = src.first;
    PortIndex src_port_index = src.second;
    SystemOutput<T>* src_ports = outputs_[src_system_index].get();
    if (src_port_index < 0 || src_port_index >= src_ports->get_num_ports()) {
      throw std::out_of_range("Source port out of range.");
    }
    OutputPort<T>* output_port = src_ports->get_mutable_port(src_port_index);

    // Validate, construct, and install the destination port.
    SystemIndex dest_system_index = dest.first;
    PortIndex dest_port_index = dest.second;
    ContextBase<T>* dest_context = contexts_[dest_system_index].get();
    if (dest_port_index < 0 ||
        dest_port_index >= dest_context->get_num_input_ports()) {
      throw std::out_of_range("Destination port out of range.");
    }
    auto input_port = std::make_unique<DependentInputPort<T>>(output_port);
    dest_context->SetInputPort(dest_port_index, std::move(input_port));

    // Remember the graph structure. We need it in DoClone().
    dependency_graph_[dest] = src;
  }

  /// Generates the state vector for the entire diagram by wrapping the states
  /// of all the constituent diagrams.
  ///
  /// User code should not call this method. It is for use during Diagram
  /// context allocation only.
  void MakeState() {
    // Generate the continuous state supervectors.
    std::vector<StateVector<T>*> continuous_states;
    std::vector<StateVector<T>*> continuous_qs;
    std::vector<StateVector<T>*> continuous_vs;
    std::vector<StateVector<T>*> continuous_zs;

    for (auto& it : contexts_) {
      ContinuousState<T>* xc =
          it.second->get_mutable_state()->continuous_state.get();

      // Skip subsystems that have no continuous state.
      if (xc == nullptr) {
        continue;
      }

      continuous_states.push_back(xc->get_mutable_state());
      continuous_qs.push_back(xc->get_mutable_generalized_position());
      continuous_vs.push_back(xc->get_mutable_generalized_velocity());
      continuous_zs.push_back(xc->get_mutable_misc_continuous_state());
    }

    state_.continuous_state = std::make_unique<ContinuousState<T>>(
        std::make_unique<StateSupervector<T>>(continuous_states),
        std::make_unique<StateSupervector<T>>(continuous_qs),
        std::make_unique<StateSupervector<T>>(continuous_vs),
        std::make_unique<StateSupervector<T>>(continuous_zs));
  }

  /// Returns the output structure for a given constituent system @p sys, or
  /// nullptr if @p sys is not a constituent system.
  SystemOutput<T>* GetSubsystemOutput(SystemIndex sys) const {
    auto it = outputs_.find(sys);
    if (it == outputs_.end()) {
      return nullptr;
    }
    return (*it).second.get();
  }

  /// Returns the context structure for a given constituent system @p sys, or
  /// nullptr if @p sys is not a constituent system.
  const ContextBase<T>* GetSubsystemContext(SystemIndex sys) const {
    auto it = contexts_.find(sys);
    if (it == contexts_.end()) {
      return nullptr;
    }
    return (*it).second.get();
  }

  int get_num_input_ports() const override { return input_ids_.size(); }

  void SetInputPort(int index, std::unique_ptr<InputPort<T>> port) override {
    if (index < 0 || index >= get_num_input_ports()) {
      throw std::out_of_range("Input port out of range.");
    }
    const PortIdentifier& id = input_ids_[index];
    SystemIndex system_index = id.first;
    PortIndex port_index = id.second;
    const auto it = contexts_.find(system_index);
    DRAKE_ASSERT(it != contexts_.end());
    it->second->SetInputPort(port_index, std::move(port));
    // TODO(david-german-tri): Set invalidation callbacks.
  }

  const VectorInterface<T>* get_vector_input(int index) const override {
    if (index >= get_num_input_ports()) {
      throw std::out_of_range("Input port out of range.");
    }
    const PortIdentifier& id = input_ids_[index];
    SystemIndex system_index = id.first;
    PortIndex port_index = id.second;
    const auto it = contexts_.find(system_index);
    if (it == contexts_.end()) {
      return nullptr;
    }
    return it->second->get_vector_input(port_index);
  }

  const State<T>& get_state() const override { return state_; }

  State<T>* get_mutable_state() override { return &state_; }

 protected:
  DiagramContext<T>* DoClone() const override {
    DiagramContext<T>* clone = new DiagramContext();

    // Clone all the subsystem contexts and outputs.
    for (const auto& subcontext : contexts_) {
      // When a leaf context is cloned, it will clone the data that currently
      // appears on each of its input ports into a FreestandingInputPort.
      clone->contexts_[subcontext.first] = subcontext.second->Clone();
    }
    for (const auto& suboutput : outputs_) {
      clone->outputs_[suboutput.first] = suboutput.second->Clone();
    }

    // Build a superstate over the subsystem contexts.
    clone->MakeState();

    // Clone the internal graph structure. After this is done, the clone will
    // still have FreestandingInputPorts at the inputs to the Diagram itself,
    // but all of the intermediate nodes will have DependentInputPorts.
    for (const auto& connection : dependency_graph_) {
      const PortIdentifier& src = connection.second;
      const PortIdentifier& dest = connection.first;
      clone->Connect(src, dest);
    }

    // Clone the external input structure.
    for (const PortIdentifier& id : input_ids_) {
      clone->ExportInput(id);
    }

    // Make deep copies of everything else using the default copy constructors.
    *clone->get_mutable_step_info() = this->get_step_info();

    return clone;
  }

 private:
  std::vector<PortIdentifier> input_ids_;

  // Order is critical: Output ports must outlive the DependentInputPorts that
  // depend on them, because the input ports unregister themselves from output
  // port change notifications at destruction time. Thus, outputs_ must be
  // declared before contexts_, so that contexts_ is destroyed before outputs_.
  std::map<SystemIndex, std::unique_ptr<SystemOutput<T>>> outputs_;
  std::map<SystemIndex, std::unique_ptr<ContextBase<T>>> contexts_;

  // A map from the input ports of constituent systems, to the output ports of
  // the systems on which they depend.
  std::map<PortIdentifier, PortIdentifier> dependency_graph_;

  // The internal state of the System.
  State<T> state_;
};

}  // namespace systems
}  // namespace drake
