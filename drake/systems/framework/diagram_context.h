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
  explicit DiagramContinuousState(
      std::vector<ContinuousState<T>*> substates)
      : ContinuousState<T>(
            Span(substates, x_selector), Span(substates, q_selector),
            Span(substates, v_selector), Span(substates, z_selector)),
        substates_(std::move(substates)) {}

  ~DiagramContinuousState() override {}

  int get_num_substates() const { return static_cast<int>(substates_.size()); }

  /// Returns the continuous state at the given @p index, or nullptr if that
  /// system is stateless.
  const ContinuousState<T>* get_substate(int index) const {
    return substates_[index];
  }

  /// Returns the continuous state at the given @p index, or nullptr if that
  /// system is stateless.
  ContinuousState<T>* get_mutable_substate(int index) {
    return substates_[index];
  }

 private:
  // Returns a StateSupervector over the x, q, v, or z components of each
  // substate in @p substates, as indicated by @p selector.
  static std::unique_ptr<StateVector<T>> Span(
      const std::vector<ContinuousState<T>*>& substates,
      std::function<StateVector<T>*(ContinuousState<T>&)> selector) {
    std::vector<StateVector<T>*> sub_xs;
    for (const auto& substate : substates) {
      if (substate != nullptr) {
        sub_xs.push_back(selector(*substate));
      }
    }
    return std::make_unique<StateSupervector<T>>(sub_xs);
  }

  // Returns the entire state vector in @p xc.
  static StateVector<T>* x_selector(ContinuousState<T>& xc) {
    return xc.get_mutable_state();
  }
  // Returns the generalized position vector in @p xc.
  static StateVector<T>* q_selector(ContinuousState<T>& xc) {
    return xc.get_mutable_generalized_position();
  }
  // Returns the generalized velocity vector in @p xc.
  static StateVector<T>* v_selector(ContinuousState<T>& xc) {
    return xc.get_mutable_generalized_velocity();
  }
  // Returns the misc continuous state vector in @p xc.
  static StateVector<T>* z_selector(ContinuousState<T>& xc) {
    return xc.get_mutable_misc_continuous_state();
  }

  std::vector<ContinuousState<T>*> substates_;
};

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
    // Identify and validate the source port.
    SystemIndex src_system_index = src.first;
    PortIndex src_port_index = src.second;
    SystemOutput<T>* src_ports = outputs_[src_system_index].get();
    DRAKE_ABORT_UNLESS(src_port_index >= 0);
    DRAKE_ABORT_UNLESS(src_port_index < src_ports->get_num_ports());
    OutputPort* output_port = src_ports->get_mutable_port(src_port_index);

    // Identify and validate the destination port.
    SystemIndex dest_system_index = dest.first;
    PortIndex dest_port_index = dest.second;
    ContextBase<T>* dest_context = contexts_[dest_system_index].get();
    DRAKE_ABORT_UNLESS(dest_port_index >= 0);
    DRAKE_ABORT_UNLESS(dest_port_index < dest_context->get_num_input_ports());

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
    for (auto& it : contexts_) {
      substates.push_back(
          it.second->get_mutable_state()->continuous_state.get());
    }
    state_.continuous_state.reset(new DiagramContinuousState<T>(substates));
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

  /// Returns the context structure for a given subsystem @p sys, or
  /// nullptr if @p sys is not a subsystem.
  ContextBase<T>* GetMutableSubsystemContext(SystemIndex sys) {
    auto it = contexts_.find(sys);
    if (it == contexts_.end()) {
      return nullptr;
    }
    return (*it).second.get();
  }

  /// Recursively sets the time on this context and all subcontexts.
  void set_time(const T& time_sec) override {
    ContextBase<T>::set_time(time_sec);
    for (auto& kv : contexts_) {
      ContextBase<T>* subcontext = kv.second.get();
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
    const auto it = contexts_.find(system_index);
    DRAKE_ASSERT(it != contexts_.end());
    it->second->SetInputPort(port_index, std::move(port));
    // TODO(david-german-tri): Set invalidation callbacks.
  }

  const VectorBase<T>* get_vector_input(int index) const override {
    if (index >= get_num_input_ports()) {
      throw std::out_of_range("Input port out of range.");
    }
    const PortIdentifier& id = input_ids_[index];
    SystemIndex system_index = id.first;
    PortIndex port_index = id.second;
    const ContextBase<T>* subsystem_context = GetSubsystemContext(system_index);
    return subsystem_context->get_vector_input(port_index);
  }

  const AbstractValue* get_abstract_input(int index) const override {
    if (index >= get_num_input_ports()) {
      throw std::out_of_range("Input port out of range.");
    }
    const PortIdentifier& id = input_ids_[index];
    SystemIndex system_index = id.first;
    PortIndex port_index = id.second;
    const ContextBase<T>* subsystem_context = GetSubsystemContext(system_index);
    return subsystem_context->get_abstract_input(port_index);
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
