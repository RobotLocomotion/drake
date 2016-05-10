#pragma once

#include <map>
#include <memory>
#include <set>
#include <stdexcept>
#include <utility>
#include <vector>

#include "drake/systems/framework/abstract_context.h"
#include "drake/systems/framework/state.h"
#include "drake/systems/framework/system_input.h"
#include "drake/systems/framework/system_interface.h"
#include "drake/systems/framework/system_output.h"

namespace drake {
namespace systems {

template <typename T>
using PortIdentifier = std::pair<const SystemInterface<T>*, int>;

/// The DiagramContext is a container for all of the data necessary to uniquely
/// determine the computations performed by a Diagram. Specifically, a
/// DiagramContext contains contexts and outputs for all the constituent
/// Systems,
/// wired up as specified by calls to `DiagramContext::Connect`.
///
/// @tparam T The mathematical type of the context, which must be a valid Eigen
///           scalar.
template <typename T>
class DiagramContext : public AbstractContext<T> {
 public:
  /// @param input_ids_ Identifiers for some system ports that are inputs
  ///                   to the Diagram that manufactures this context.
  explicit DiagramContext(const std::vector<PortIdentifier<T>>& input_ids)
      : input_ids_(input_ids) {
    // Any system whose input port is an input port of the diagram is
    // necessarily a constituent of the diagram.
    for (const auto& id : input_ids_) {
      AddConstituentSystem(id.first);
    }
  }

  /// Declares a new subsystem in the DiagramContext.
  ///
  /// User code should not call this method. It is for use during Diagram
  /// context allocation only.
  void AddConstituentSystem(const SystemInterface<T>* sys) {
    contexts_[sys] = sys->CreateDefaultContext();
    outputs_[sys] = sys->AllocateOutput(*contexts_[sys]);
  }

  /// Declares that the output port specified by @p src is connected to the
  /// input port specified by @p dest.
  ///
  /// User code should not call this method. It is for use during Diagram
  /// context allocation only.
  void Connect(const PortIdentifier<T>& src, const PortIdentifier<T>& dest) {
    // Validate and identify the source port.
    SystemOutputInterface<T>* src_ports = outputs_[src.first].get();
    const int src_port_index = src.second;
    if (src_port_index < 0 || src_port_index >= src_ports->get_num_ports()) {
      throw std::out_of_range("Source port out of range.");
    }
    OutputPort<T>* output_port = src_ports->get_mutable_port(src_port_index);

    // Validate, construct, and install the destination port.
    const int dest_port_index = dest.second;
    AbstractContext<T>* dest_context = contexts_[dest.first].get();
    if (dest_port_index < 0 ||
        dest_port_index >= dest_context->get_num_input_ports()) {
      throw std::out_of_range("Destination port out of range.");
    }
    // TODO(david-german-tri): Set the sample rate.
    auto input_port = std::make_unique<DependentInputPort<T>>(output_port, 0.0);
    dest_context->SetInputPort(dest_port_index, std::move(input_port));

    // Remember the graph structure. We need it in DoClone().
    dependency_graph_[dest] = src;
  }

  /// Returns the output structure for a given constituent system @p sys, or
  /// nullptr if @p sys is not a constituent system.
  SystemOutputInterface<T>* GetSubsystemOutput(
      const SystemInterface<T>* sys) const {
    auto it = outputs_.find(sys);
    if (it == outputs_.end()) {
      return nullptr;
    }
    return (*it).second.get();
  }

  /// Returns the context structure for a given constituent system @p sys, or
  /// nullptr if @p sys is not a constituent system.
  const AbstractContext<T>* GetSubsystemContext(
      const SystemInterface<T>* sys) const {
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
    const PortIdentifier<T>& id = input_ids_[index];
    const auto it = contexts_.find(id.first);
    DRAKE_ASSERT(it != contexts_.end());
    it->second->SetInputPort(id.second, std::move(port));
    // TODO(david-german-tri): Set invalidation callbacks.
  }

  const VectorInterface<T>* get_vector_input(int index) const override {
    if (index >= get_num_input_ports()) {
      throw std::out_of_range("Input port out of range.");
    }
    const PortIdentifier<T>& id = input_ids_[index];
    const auto it = contexts_.find(id.first);
    if (it == contexts_.end()) {
      return nullptr;
    }
    return it->second->get_vector_input(id.second);
  }

  const State<T>& get_state() const override { return state_; }

  State<T>* get_mutable_state() override { return &state_; }

 protected:
  DiagramContext<T>* DoClone() const override {
    DiagramContext<T>* clone = new DiagramContext(input_ids_);

    // Clone all the subsystem contexts. They will make deep copies of the
    // input ports that they care about.
    for (const auto& subcontext : contexts_) {
      clone->contexts_[subcontext.first] = subcontext.second->Clone();
    }

    // Construct fresh subsystem outputs based on the new contexts.
    for (const auto& suboutput : outputs_) {
      const SystemInterface<T>* sys = suboutput.first;
      clone->outputs_[sys] = sys->AllocateOutput(*clone->contexts_[sys]);
    }

    // Clone the internal graph structure. After this is done, the clone will
    // have FreestandingInputPorts at the inputs to the Diagram itself, and
    // DependentInputPorts for the intermediate nodes.
    for (const auto& connection : dependency_graph_) {
      const PortIdentifier<T>& src = connection.second;
      const PortIdentifier<T>& dest = connection.first;
      clone->Connect(src, dest);
    }

    // Make deep copies of everything else using the default copy constructors.
    *clone->get_mutable_step_info() = this->get_step_info();

    // TODO(david-german-tri): Clone the superstate.
    return clone;
  }

 private:
  const std::vector<PortIdentifier<T>> input_ids_;

  // Order is critical: Output ports must outlive the DependentInputPorts that
  // depend on them, because the input ports unregister themselves from output
  // port change notifications at destruction time. Thus, outputs_ must be
  // declared before contexts_, so that contexts_ is destroyed before outputs_.
  std::map<const SystemInterface<T>*, std::unique_ptr<SystemOutputInterface<T>>>
      outputs_;
  std::map<const SystemInterface<T>*, std::unique_ptr<AbstractContext<T>>>
      contexts_;

  // A map from the input ports of constituent systems, to the output ports of
  // the systems on which they depend.
  std::map<PortIdentifier<T>, PortIdentifier<T>> dependency_graph_;

  // The internal state of the System.
  State<T> state_;
};

}  // namespace systems
}  // namespace drake
