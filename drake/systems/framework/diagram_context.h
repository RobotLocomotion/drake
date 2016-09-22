#pragma once

#include <map>
#include <memory>
#include <set>
#include <stdexcept>
#include <utility>
#include <vector>

#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/state.h"
#include "drake/systems/framework/state_supervector.h"
#include "drake/systems/framework/system_input.h"
#include "drake/systems/framework/system_output.h"

namespace drake {
namespace systems {

/// DiagramContinuousState is a ContinuousState consisting of StateSupervectors
/// over a set of constituent ContinuousStates.
///
/// @tparam T The type of the output data. Must be a valid Eigen scalar.
template <typename T>
class DiagramContinuousState : public ContinuousState<T> {
 public:
  /// Constructs a ContinuousState that is composed of other ContinuousStates,
  /// which are not owned by this object and must outlive it. Some of the
  /// subsystem states may be nullptr if the system is stateless.
  ///
  /// The DiagramContinuousState vector will have the same order as the
  /// @p states parameter, which should be the sort order of the Diagram itself.
  explicit DiagramContinuousState(std::vector<ContinuousState<T>*> substates)
      : ContinuousState<T>(
            Span(substates, x_selector), Span(substates, q_selector),
            Span(substates, v_selector), Span(substates, z_selector)),
        substates_(std::move(substates)) {}

  ~DiagramContinuousState() override {}

  int get_num_substates() const { return static_cast<int>(substates_.size()); }

  /// Returns the continuous state at the given @p index, or nullptr if that
  /// system is stateless. Aborts if @p index is out-of-bounds.
  const ContinuousState<T>* get_substate(int index) const {
    DRAKE_DEMAND(index >= 0 && index < get_num_substates());
    return substates_[index];
  }

  /// Returns the continuous state at the given @p index, or nullptr if that
  /// system is stateless. Aborts if @p index is out-of-bounds.
  ContinuousState<T>* get_mutable_substate(int index) {
    DRAKE_DEMAND(index >= 0 && index < get_num_substates());
    return substates_[index];
  }

 private:
  // Returns a StateSupervector over the x, q, v, or z components of each
  // substate in @p substates, as indicated by @p selector.
  static std::unique_ptr<VectorBase<T>> Span(
      const std::vector<ContinuousState<T>*>& substates,
      std::function<VectorBase<T>*(ContinuousState<T>&)> selector) {
    std::vector<VectorBase<T>*> sub_xs;
    for (const auto& substate : substates) {
      if (substate != nullptr) {
        sub_xs.push_back(selector(*substate));
      }
    }
    return std::make_unique<StateSupervector<T>>(sub_xs);
  }

  // Returns the entire state vector in @p xc.
  static VectorBase<T>* x_selector(ContinuousState<T>& xc) {
    return xc.get_mutable_state();
  }
  // Returns the generalized position vector in @p xc.
  static VectorBase<T>* q_selector(ContinuousState<T>& xc) {
    return xc.get_mutable_generalized_position();
  }
  // Returns the generalized velocity vector in @p xc.
  static VectorBase<T>* v_selector(ContinuousState<T>& xc) {
    return xc.get_mutable_generalized_velocity();
  }
  // Returns the misc continuous state vector in @p xc.
  static VectorBase<T>* z_selector(ContinuousState<T>& xc) {
    return xc.get_mutable_misc_continuous_state();
  }

  std::vector<ContinuousState<T>*> substates_;
};

/// The DiagramContext is a container for all of the data necessary to uniquely
/// determine the computations performed by a Diagram. Specifically, a
/// DiagramContext contains contexts and outputs for all the constituent
/// Systems, wired up as specified by calls to `DiagramContext::Connect`.
///
/// In general, users should not need to interact with a DiagramContext
/// directly. Use the accessors on Diagram instead.
///
/// @tparam T The mathematical type of the context, which must be a valid Eigen
///           scalar.
template <typename T>
class DiagramContext : public Context<T> {
 public:
  typedef int SystemIndex;
  typedef int PortIndex;
  typedef std::pair<SystemIndex, PortIndex> PortIdentifier;

  /// Constructs a DiagramContext with the given @p num_subsystems, which is
  /// final: you cannot resize a DiagramContext after construction.
  explicit DiagramContext(const int num_subsystems)
      : outputs_(num_subsystems), contexts_(num_subsystems) {}

  /// Declares a new subsystem in the DiagramContext. Subsystems are identified
  /// by number. If the subsystem has already been declared, aborts.
  ///
  /// User code should not call this method. It is for use during Diagram
  /// context allocation only.
  void AddSystem(SystemIndex index, std::unique_ptr<Context<T>> context,
                 std::unique_ptr<SystemOutput<T>> output) {
    DRAKE_DEMAND(contexts_[index] == nullptr);
    DRAKE_DEMAND(outputs_[index] == nullptr);
    contexts_[index] = std::move(context);
    outputs_[index] = std::move(output);
  }

  /// Declares that a particular input port of a particular subsystem is an
  /// input to the entire Diagram that allocates this Context. Aborts if the
  /// subsystem has not been added to the DiagramContext.
  ///
  /// User code should not call this method. It is for use during Diagram
  /// context allocation only.
  void ExportInput(const PortIdentifier& id) {
    const SystemIndex system_index = id.first;
    DRAKE_DEMAND(contexts_[system_index] != nullptr);
    input_ids_.emplace_back(id);
  }

  /// Declares that the output port specified by @p src is connected to the
  /// input port specified by @p dest.
  ///
  /// User code should not call this method. It is for use during Diagram
  /// context allocation only.
  void Connect(const PortIdentifier& src, const PortIdentifier& dest) {
    // Identify and validate the source port.
    SystemIndex src_system_index = src.first;
    PortIndex src_port_index = src.second;
    SystemOutput<T>* src_ports = GetSubsystemOutput(src_system_index);
    DRAKE_DEMAND(src_port_index >= 0);
    DRAKE_DEMAND(src_port_index < src_ports->get_num_ports());
    OutputPort* output_port = src_ports->get_mutable_port(src_port_index);

    // Identify and validate the destination port.
    SystemIndex dest_system_index = dest.first;
    PortIndex dest_port_index = dest.second;
    Context<T>* dest_context = GetMutableSubsystemContext(dest_system_index);
    DRAKE_DEMAND(dest_port_index >= 0);
    DRAKE_DEMAND(dest_port_index < dest_context->get_num_input_ports());

    // Construct and install the destination port.
    auto input_port = std::make_unique<DependentInputPort>(output_port);
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
    std::vector<ContinuousState<T>*> substates;
    for (auto& context : contexts_) {
      substates.push_back(context->get_mutable_state()->continuous_state.get());
    }
    state_.continuous_state.reset(new DiagramContinuousState<T>(substates));
  }

  /// Returns the output structure for a given constituent system at @p index.
  /// Aborts if @p index is out of bounds, or if no system has been added to the
  /// DiagramContext at that index.
  SystemOutput<T>* GetSubsystemOutput(SystemIndex index) const {
    const int num_outputs = static_cast<int>(outputs_.size());
    DRAKE_DEMAND(index >= 0 && index < num_outputs);
    DRAKE_DEMAND(outputs_[index] != nullptr);
    return outputs_[index].get();
  }

  /// Returns the context structure for a given constituent system @p index.
  /// Aborts if @p index is out of bounds, or if no system has been added to the
  /// DiagramContext at that index.
  const Context<T>* GetSubsystemContext(SystemIndex index) const {
    const int num_contexts = static_cast<int>(contexts_.size());
    DRAKE_DEMAND(index >= 0 && index < num_contexts);
    DRAKE_DEMAND(contexts_[index] != nullptr);
    return contexts_[index].get();
  }

  /// Returns the context structure for a given subsystem @p index.
  /// Aborts if @p index is out of bounds, or if no system has been added to the
  /// DiagramContext at that index.
  Context<T>* GetMutableSubsystemContext(SystemIndex index) {
    const int num_contexts = static_cast<int>(contexts_.size());
    DRAKE_DEMAND(index >= 0 && index < num_contexts);
    DRAKE_DEMAND(contexts_[index] != nullptr);
    return contexts_[index].get();
  }

  /// Recursively sets the time on this context and all subcontexts.
  void set_time(const T& time_sec) override {
    Context<T>::set_time(time_sec);
    for (auto& subcontext : contexts_) {
      if (subcontext != nullptr) {
        subcontext->set_time(time_sec);
      }
    }
  }

  int get_num_input_ports() const override {
    return static_cast<int>(input_ids_.size());
  }

  void SetInputPort(int index, std::unique_ptr<InputPort> port) override {
    if (index < 0 || index >= get_num_input_ports()) {
      throw std::out_of_range("Input port out of range.");
    }
    const PortIdentifier& id = input_ids_[index];
    SystemIndex system_index = id.first;
    PortIndex port_index = id.second;
    GetMutableSubsystemContext(system_index)
        ->SetInputPort(port_index, std::move(port));
    // TODO(david-german-tri): Set invalidation callbacks.
  }

  const BasicVector<T>* get_vector_input(int index) const override {
    if (index >= get_num_input_ports()) {
      throw std::out_of_range("Input port out of range.");
    }
    const PortIdentifier& id = input_ids_[index];
    SystemIndex system_index = id.first;
    PortIndex port_index = id.second;
    return GetSubsystemContext(system_index)->get_vector_input(port_index);
  }

  const AbstractValue* get_abstract_input(int index) const override {
    if (index >= get_num_input_ports()) {
      throw std::out_of_range("Input port out of range.");
    }
    const PortIdentifier& id = input_ids_[index];
    SystemIndex system_index = id.first;
    PortIndex port_index = id.second;
    return GetSubsystemContext(system_index)->get_abstract_input(port_index);
  }

  const State<T>& get_state() const override { return state_; }

  State<T>* get_mutable_state() override { return &state_; }

 protected:
  DiagramContext<T>* DoClone() const override {
    DRAKE_ASSERT(contexts_.size() == outputs_.size());
    const int num_subsystems = static_cast<int>(contexts_.size());
    DiagramContext<T>* clone = new DiagramContext(num_subsystems);

    // Clone all the subsystem contexts and outputs.
    for (int i = 0; i < num_subsystems; ++i) {
      DRAKE_DEMAND(contexts_[i] != nullptr);
      DRAKE_DEMAND(outputs_[i] != nullptr);
      // When a leaf context is cloned, it will clone the data that currently
      // appears on each of its input ports into a FreestandingInputPort.
      clone->contexts_[i] = contexts_[i]->Clone();
      clone->outputs_[i] = outputs_[i]->Clone();
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

  std::vector<std::unique_ptr<SystemOutput<T>>> outputs_;
  std::vector<std::unique_ptr<Context<T>>> contexts_;

  // A map from the input ports of constituent systems, to the output ports of
  // the systems on which they depend.
  std::map<PortIdentifier, PortIdentifier> dependency_graph_;

  // The internal state of the System.
  State<T> state_;
};

}  // namespace systems
}  // namespace drake
