#pragma once

#include <algorithm>
#include <map>
#include <memory>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_set>
#include <utility>
#include <variant>
#include <vector>

#include "drake/common/default_scalars.h"
#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/drake_throw.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/system.h"

namespace drake {
namespace systems {

/// DiagramBuilder is a factory class for Diagram. It is single
/// use: after calling Build or BuildInto, DiagramBuilder gives up ownership
/// of the constituent systems, and should therefore be discarded.
///
/// A system must be added to the DiagramBuilder with AddSystem before it can
/// be wired up in any way. Every system must have a unique, non-empty name.
template <typename T>
class DiagramBuilder {
 public:
  // DiagramBuilder objects are neither copyable nor moveable.
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(DiagramBuilder)

  DiagramBuilder() {}
  virtual ~DiagramBuilder() {}

  // TODO(sherm1) The AddSystem methods (or some variant) should take the system
  // name as their first parameter. See discussion in issue #5895.

  /// Takes ownership of @p system and adds it to the builder. Returns a bare
  /// pointer to the System, which will remain valid for the lifetime of the
  /// Diagram built by this builder.
  ///
  /// If the system's name is unset, sets it to System::GetMemoryObjectName()
  /// as a default in order to have unique names within the diagram.
  ///
  /// @code
  ///   DiagramBuilder<T> builder;
  ///   auto foo = builder.AddSystem(std::make_unique<Foo<T>>());
  /// @endcode
  ///
  /// @tparam S The type of system to add.
  template<class S>
  S* AddSystem(std::unique_ptr<S> system) {
    if (system->get_name().empty()) {
      system->set_name(system->GetMemoryObjectName());
    }
    S* raw_sys_ptr = system.get();
    systems_.insert(raw_sys_ptr);
    registered_systems_.push_back(std::move(system));
    return raw_sys_ptr;
  }

  /// Constructs a new system with the given @p args, and adds it to the
  /// builder, which retains ownership. Returns a bare pointer to the System,
  /// which will remain valid for the lifetime of the Diagram built by this
  /// builder.
  ///
  /// @code
  ///   DiagramBuilder<double> builder;
  ///   auto foo = builder.AddSystem<Foo<double>>("name", 3.14);
  /// @endcode
  ///
  /// Note that for dependent names you must use the template keyword:
  ///
  /// @code
  ///   DiagramBuilder<T> builder;
  ///   auto foo = builder.template AddSystem<Foo<T>>("name", 3.14);
  /// @endcode
  ///
  /// You may prefer the `unique_ptr` variant instead.
  ///
  ///
  /// @tparam S The type of System to construct. Must subclass System<T>.
  ///
  /// @exclude_from_pydrake_mkdoc{Not bound in pydrake -- emplacement while
  /// specifying <T> doesn't make sense for that language.}
  template<class S, typename... Args>
  S* AddSystem(Args&&... args) {
    return AddSystem(std::make_unique<S>(std::forward<Args>(args)...));
  }

  /// Constructs a new system with the given @p args, and adds it to the
  /// builder, which retains ownership. Returns a bare pointer to the System,
  /// which will remain valid for the lifetime of the Diagram built by this
  /// builder.
  ///
  /// @code
  ///   DiagramBuilder<double> builder;
  ///   // Foo must be a template.
  ///   auto foo = builder.AddSystem<Foo>("name", 3.14);
  /// @endcode
  ///
  /// Note that for dependent names you must use the template keyword:
  ///
  /// @code
  ///   DiagramBuilder<T> builder;
  ///   auto foo = builder.template AddSystem<Foo>("name", 3.14);
  /// @endcode
  ///
  /// You may prefer the `unique_ptr` variant instead.
  ///
  /// @tparam S A template for the type of System to construct. The template
  /// will be specialized on the scalar type T of this builder.
  ///
  /// @exclude_from_pydrake_mkdoc{Not bound in pydrake -- emplacement while
  /// specifying <T> doesn't make sense for that language.}
  template<template<typename Scalar> class S, typename... Args>
  S<T>* AddSystem(Args&&... args) {
    return AddSystem(std::make_unique<S<T>>(std::forward<Args>(args)...));
  }

  /// Returns whether any Systems have been added yet.
  bool empty() const { return registered_systems_.empty(); }

  /// Returns the list of contained Systems.
  std::vector<systems::System<T>*> GetMutableSystems() {
    std::vector<systems::System<T>*> result;
    result.reserve(registered_systems_.size());
    for (const auto& system : registered_systems_) {
      result.push_back(system.get());
    }
    return result;
  }

  /// Declares that input port @p dest is connected to output port @p src.
  /// @note The connection created between @p src and @p dest via a call to
  /// this method can be effectively overridden by any subsequent call to
  /// Context::FixInputPort(). That is, calling Context::FixInputPort() on an
  /// already connected input port causes the resultant
  /// FixedInputPortValue to override any other value present on that
  /// port.
  void Connect(const OutputPort<T>& src,
               const InputPort<T>& dest) {
    InputPortLocator dest_id{&dest.get_system(), dest.get_index()};
    OutputPortLocator src_id{&src.get_system(), src.get_index()};
    ThrowIfSystemNotRegistered(&src.get_system());
    ThrowIfSystemNotRegistered(&dest.get_system());
    ThrowIfInputAlreadyWired(dest_id);
    if (src.get_data_type() != dest.get_data_type()) {
      throw std::logic_error(fmt::format(
          "DiagramBuilder::Connect: Cannot mix vector-valued and abstract-"
          "valued ports while connecting output port {} of System {} to "
          "input port {} of System {}",
          src.get_name(), src.get_system().get_name(),
          dest.get_name(), dest.get_system().get_name()));
    }
    if ((src.get_data_type() != kAbstractValued) &&
        (src.size() != dest.size())) {
      throw std::logic_error(fmt::format(
          "DiagramBuilder::Connect: Mismatched vector sizes while connecting "
          "output port {} of System {} (size {}) to "
          "input port {} of System {} (size {})",
          src.get_name(), src.get_system().get_name(), src.size(),
          dest.get_name(), dest.get_system().get_name(), dest.size()));
    }
    if (src.get_data_type() == kAbstractValued) {
      auto model_output = src.Allocate();
      auto model_input = dest.get_system().AllocateInputAbstract(dest);
      const std::type_info& output_type = model_output->static_type_info();
      const std::type_info& input_type = model_input->static_type_info();
      if (output_type != input_type) {
        throw std::logic_error(fmt::format(
            "DiagramBuilder::Connect: Mismatched value types while connecting "
            "output port {} of System {} (type {}) to "
            "input port {} of System {} (type {})",
            src.get_name(), src.get_system().get_name(),
            NiceTypeName::Get(output_type),
            dest.get_name(), dest.get_system().get_name(),
            NiceTypeName::Get(input_type)));
      }
    }
    connection_map_[dest_id] = src_id;
  }

  /// Declares that sole input port on the @p dest system is connected to sole
  /// output port on the @p src system.
  /// @note The connection created between @p src and @p dest via a call to
  /// this method can be effectively overridden by any subsequent call to
  /// Context::FixInputPort(). That is, calling Context::FixInputPort() on an
  /// already connected input port causes the resultant
  /// FixedInputPortValue to override any other value present on that
  /// port.
  /// @throws std::exception if the sole-port precondition is not met (i.e.,
  /// if @p dest has no input ports, or @p dest has more than one input port,
  /// or @p src has no output ports, or @p src has more than one output port).
  ///
  /// @exclude_from_pydrake_mkdoc{Not bound in pydrake.}
  void Connect(const System<T>& src, const System<T>& dest) {
    DRAKE_THROW_UNLESS(src.num_output_ports() == 1);
    DRAKE_THROW_UNLESS(dest.num_input_ports() == 1);
    Connect(src.get_output_port(0), dest.get_input_port(0));
  }

  /// Cascades @p src and @p dest.  The sole input port on the @p dest system
  /// is connected to sole output port on the @p src system.
  /// @throws std::exception if the sole-port precondition is not met (i.e., if
  /// @p dest has no input ports, or @p dest has more than one input port, or
  /// @p src has no output ports, or @p src has more than one output port).
  void Cascade(const System<T>& src, const System<T>& dest) {
    Connect(src, dest);
  }

  /// Declares that the given @p input port of a constituent system is an input
  /// to the entire Diagram.  @p name is an optional name for the input port;
  /// if it is unspecified, then a default name will be provided.
  /// @pre If supplied at all, @p name must not be empty.
  /// @return The index of the exported input port of the entire diagram.
  InputPortIndex ExportInput(
      const InputPort<T>& input,
      std::variant<std::string, UseDefaultName> name = kUseDefaultName) {
    InputPortLocator id{&input.get_system(), input.get_index()};
    ThrowIfInputAlreadyWired(id);
    ThrowIfSystemNotRegistered(&input.get_system());
    InputPortIndex return_id(input_port_ids_.size());
    input_port_ids_.push_back(id);

    // The requirement that subsystem names are unique guarantees uniqueness
    // of the port names.
    std::string port_name =
        name == kUseDefaultName
            ? input.get_system().get_name() + "_" + input.get_name()
            : std::get<std::string>(std::move(name));
    DRAKE_DEMAND(!port_name.empty());
    input_port_names_.emplace_back(std::move(port_name));

    diagram_input_set_.insert(id);
    return return_id;
  }

  /// Declares that the given @p output port of a constituent system is an
  /// output of the entire diagram.  @p name is an optional name for the output
  /// port; if it is unspecified, then a default name will be provided.
  /// @pre If supplied at all, @p name must not be empty.
  /// @return The index of the exported output port of the entire diagram.
  OutputPortIndex ExportOutput(
      const OutputPort<T>& output,
      std::variant<std::string, UseDefaultName> name = kUseDefaultName) {
    ThrowIfSystemNotRegistered(&output.get_system());
    OutputPortIndex return_id(output_port_ids_.size());
    output_port_ids_.push_back(
        OutputPortLocator{&output.get_system(), output.get_index()});

    // The requirement that subsystem names are unique guarantees uniqueness
    // of the port names.
    std::string port_name =
        name == kUseDefaultName
            ? output.get_system().get_name() + "_" + output.get_name()
            : std::get<std::string>(std::move(name));
    DRAKE_DEMAND(!port_name.empty());
    output_port_names_.emplace_back(std::move(port_name));

    return return_id;
  }

  /// Builds the Diagram that has been described by the calls to Connect,
  /// ExportInput, and ExportOutput.
  /// @throws std::logic_error if the graph is not buildable.
  std::unique_ptr<Diagram<T>> Build() {
    std::unique_ptr<Diagram<T>> diagram(new Diagram<T>(Compile()));
    return diagram;
  }

  /// Configures @p target to have the topology that has been described by
  /// the calls to Connect, ExportInput, and ExportOutput.
  /// @throws std::logic_error if the graph is not buildable.
  ///
  /// Only Diagram subclasses should call this method. The target must not
  /// already be initialized.
  void BuildInto(Diagram<T>* target) {
    target->Initialize(Compile());
  }

 private:
  using InputPortLocator = typename Diagram<T>::InputPortLocator;
  using OutputPortLocator = typename Diagram<T>::OutputPortLocator;

  // Throws if the given input port (belonging to a child subsystem) has
  // already been connected to an output port, or exported to be an input
  // port of the whole diagram.
  void ThrowIfInputAlreadyWired(const InputPortLocator& id) const {
    if (connection_map_.find(id) != connection_map_.end() ||
        diagram_input_set_.find(id) != diagram_input_set_.end()) {
      throw std::logic_error("Input port is already wired.");
    }
  }

  void ThrowIfSystemNotRegistered(const System<T>* system) const {
    DRAKE_DEMAND(system != nullptr);
    if (systems_.count(system) == 0) {
      throw std::logic_error(fmt::format(
          "DiagramBuilder: Cannot operate on ports of System {} "
          "until it has been registered using AddSystem",
          system->get_name()));
    }
  }

  // (Defined lower down in this file.)
  void ThrowIfAlgebraicLoopsExist() const;

  // TODO(russt): Implement AddRandomSources method to wire up all dangling
  // random input ports with a compatible RandomSource system.

  // Produces the Blueprint that has been described by the calls to
  // Connect, ExportInput, and ExportOutput. Throws std::logic_error if the
  // graph is empty or contains algebraic loops.
  // The DiagramBuilder passes ownership of the registered systems to the
  // blueprint.
  std::unique_ptr<typename Diagram<T>::Blueprint> Compile() {
    if (registered_systems_.size() == 0) {
      throw std::logic_error("Cannot Compile an empty DiagramBuilder.");
    }
    ThrowIfAlgebraicLoopsExist();

    auto blueprint = std::make_unique<typename Diagram<T>::Blueprint>();
    blueprint->input_port_ids = input_port_ids_;
    blueprint->input_port_names = input_port_names_;
    blueprint->output_port_ids = output_port_ids_;
    blueprint->output_port_names = output_port_names_;
    blueprint->connection_map = connection_map_;
    blueprint->systems = std::move(registered_systems_);

    return blueprint;
  }

  // The ordered inputs and outputs of the Diagram to be built.
  std::vector<InputPortLocator> input_port_ids_;
  std::vector<std::string> input_port_names_;
  std::vector<OutputPortLocator> output_port_ids_;
  std::vector<std::string> output_port_names_;

  // For fast membership queries: has this input port already been declared?
  std::set<InputPortLocator> diagram_input_set_;

  // A map from the input ports of constituent systems, to the output ports of
  // the systems from which they get their values.
  std::map<InputPortLocator, OutputPortLocator> connection_map_;

  // A mirror on the systems in the diagram. Should have the same values as
  // registered_systems_. Used for fast membership queries.
  std::unordered_set<const System<T>*> systems_;
  // The Systems in this DiagramBuilder, in the order they were registered.
  internal::OwnedSystems<T> registered_systems_;

  friend int AddRandomInputs(double, systems::DiagramBuilder<double>*);
};

#ifndef DRAKE_DOXYGEN_CXX
namespace internal {
// A non-templated helper class so that we can isolate code in our *.cc file.
class DiagramBuilderImpl {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(DiagramBuilderImpl)
  DiagramBuilderImpl() = delete;

  // Examines the graph of port dependencies (including connections between
  // output ports and input ports and direct feedthrough connections between
  // input ports and output ports) and if an algebraic loop is detected throws
  // a std::logic_error.
  static void ThrowIfAlgebraicLoopsExist(
      const std::vector<const SystemBase*>& systems,
      const std::map<
        std::pair<const SystemBase*, InputPortIndex>,
        std::pair<const SystemBase*, OutputPortIndex>>& connection_map);
};

}  // namespace internal
#endif  // DRAKE_DOXYGEN_CXX

template <typename T>
void DiagramBuilder<T>::ThrowIfAlgebraicLoopsExist() const {
  // Delegate to our Impl after upcasting the pointers to SystemBase.
  std::vector<const SystemBase*> systems;
  systems.reserve(registered_systems_.size());
  for (const auto& item : registered_systems_) {
    systems.push_back(item.get());
  }
  internal::DiagramBuilderImpl::ThrowIfAlgebraicLoopsExist(
      systems, {connection_map_.begin(), connection_map_.end()});
}

}  // namespace systems
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::systems::DiagramBuilder)
