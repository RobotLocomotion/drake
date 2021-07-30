#pragma once

#include <map>
#include <memory>
#include <set>
#include <string>
#include <unordered_set>
#include <utility>
#include <variant>
#include <vector>

#include "drake/common/default_scalars.h"
#include "drake/common/drake_copyable.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/system.h"

namespace drake {
namespace systems {

/// DiagramBuilder is a factory class for Diagram.
///
/// It is single use: after calling Build or BuildInto, DiagramBuilder gives up
/// ownership of the constituent systems, and should therefore be discarded.
///
/// When a Diagram (or DiagramBuilder) that owns systems is destroyed, the
/// systems will be destroyed in the reverse of the order they were added.
///
/// A system must be added to the DiagramBuilder with AddSystem or
/// AddNamedSystem before it can be wired up in any way. Every system must have
/// a unique, non-empty name.
template <typename T>
class DiagramBuilder {
 public:
  // DiagramBuilder objects are neither copyable nor moveable.
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(DiagramBuilder)

  DiagramBuilder();
  virtual ~DiagramBuilder();

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

  /// Takes ownership of @p system, applies @p name to it, and adds it to the
  /// builder. Returns a bare pointer to the System, which will remain valid
  /// for the lifetime of the Diagram built by this builder.
  ///
  /// @code
  ///   DiagramBuilder<T> builder;
  ///   auto bar = builder.AddNamedSystem("bar", std::make_unique<Bar<T>>());
  /// @endcode
  ///
  /// @tparam S The type of system to add.
  /// @post The system's name is @p name.
  template<class S>
  S* AddNamedSystem(const std::string& name, std::unique_ptr<S> system) {
    system->set_name(name);
    return AddSystem(std::move(system));
  }

  /// Constructs a new system with the given @p args, applies @p name to it,
  /// and adds it to the builder, which retains ownership. Returns a bare
  /// pointer to the System, which will remain valid for the lifetime of the
  /// Diagram built by this builder.
  ///
  /// @code
  ///   DiagramBuilder<double> builder;
  ///   auto bar = builder.AddNamedSystem<Bar<double>>("bar", 3.14);
  /// @endcode
  ///
  /// Note that for dependent names you must use the template keyword:
  ///
  /// @code
  ///   DiagramBuilder<T> builder;
  ///   auto bar = builder.template AddNamedSystem<Bar<T>>("bar", 3.14);
  /// @endcode
  ///
  /// You may prefer the `unique_ptr` variant instead.
  ///
  /// @tparam S The type of System to construct. Must subclass System<T>.
  /// @post The newly constructed system's name is @p name.
  ///
  /// @exclude_from_pydrake_mkdoc{Not bound in pydrake -- emplacement while
  /// specifying <T> doesn't make sense for that language.}
  template<class S, typename... Args>
  S* AddNamedSystem(const std::string& name, Args&&... args) {
    return AddNamedSystem(
        name, std::make_unique<S>(std::forward<Args>(args)...));
  }

  /// Constructs a new system with the given @p args, applies @p name to it,
  /// and adds it to the builder, which retains ownership. Returns a bare
  /// pointer to the System, which will remain valid for the lifetime of the
  /// Diagram built by this builder.
  ///
  /// @code
  ///   DiagramBuilder<double> builder;
  ///   // Bar must be a template.
  ///   auto bar = builder.AddNamedSystem<Bar>("bar", 3.14);
  /// @endcode
  ///
  /// Note that for dependent names you must use the template keyword:
  ///
  /// @code
  ///   DiagramBuilder<T> builder;
  ///   auto bar = builder.template AddNamedSystem<Bar>("bar", 3.14);
  /// @endcode
  ///
  /// You may prefer the `unique_ptr` variant instead.
  ///
  /// @tparam S A template for the type of System to construct. The template
  /// will be specialized on the scalar type T of this builder.
  /// @post The newly constructed system's name is @p name.
  ///
  /// @exclude_from_pydrake_mkdoc{Not bound in pydrake -- emplacement while
  /// specifying <T> doesn't make sense for that language.}
  template<template<typename Scalar> class S, typename... Args>
  S<T>* AddNamedSystem(const std::string& name, Args&&... args) {
    return AddNamedSystem(
        name, std::make_unique<S<T>>(std::forward<Args>(args)...));
  }

  /// Returns whether any Systems have been added yet.
  bool empty() const { return registered_systems_.empty(); }

  /// Returns the list of contained Systems.
  /// See also GetMutableSystems().
  std::vector<const System<T>*> GetSystems() const;

  /// Returns the list of contained Systems.
  /// See also GetSystems().
  std::vector<System<T>*> GetMutableSystems();

  /// Declares that input port @p dest is connected to output port @p src.
  /// @note The connection created between @p src and @p dest via a call to
  /// this method can be effectively overridden by any subsequent call to
  /// InputPort::FixValue(). That is, calling InputPort::FixValue() on an
  /// already connected input port causes the resultant
  /// FixedInputPortValue to override any other value present on that
  /// port.
  void Connect(const OutputPort<T>& src, const InputPort<T>& dest);

  /// Declares that sole input port on the @p dest system is connected to sole
  /// output port on the @p src system.
  /// @note The connection created between @p src and @p dest via a call to
  /// this method can be effectively overridden by any subsequent call to
  /// InputPort::FixValue(). That is, calling InputPort::FixValue() on an
  /// already connected input port causes the resultant
  /// FixedInputPortValue to override any other value present on that
  /// port.
  /// @throws std::exception if the sole-port precondition is not met (i.e.,
  /// if @p dest has no input ports, or @p dest has more than one input port,
  /// or @p src has no output ports, or @p src has more than one output port).
  ///
  /// @exclude_from_pydrake_mkdoc{Not bound in pydrake.}
  void Connect(const System<T>& src, const System<T>& dest);

  /// Cascades @p src and @p dest.  The sole input port on the @p dest system
  /// is connected to sole output port on the @p src system.
  /// @throws std::exception if the sole-port precondition is not met (i.e., if
  /// @p dest has no input ports, or @p dest has more than one input port, or
  /// @p src has no output ports, or @p src has more than one output port).
  void Cascade(const System<T>& src, const System<T>& dest);

  /// Declares that the given @p input port of a constituent system is
  /// connected to a new input to the entire Diagram.  @p name is an optional
  /// name for the new input port; if it is unspecified, then a default name
  /// will be provided.
  /// @pre If supplied at all, @p name must not be empty.
  /// @pre A port indicated by the resolution of @p name must not exist.
  /// @post @p input is connected to the new exported input port.
  /// @return The index of the exported input port of the entire diagram.
  InputPortIndex ExportInput(
      const InputPort<T>& input,
      std::variant<std::string, UseDefaultName> name = kUseDefaultName);

  /// Connects an input to the entire Diagram, indicated by @p
  /// diagram_port_name, to the given @p input port of a constituent system.
  /// @pre The Diagram input indicated by @p diagram_port_name must have been
  /// previously built via ExportInput().
  /// @post @p input is connected to the indicated Diagram input port.
  void ConnectInput(
      const std::string& diagram_port_name, const InputPort<T>& input);

  /// Connects an input to the entire Diagram, indicated by @p
  /// diagram_port_index, to the given @p input port of a constituent system.
  /// @pre The Diagram input indicated by @p diagram_port_index must have been
  /// previously built via ExportInput().
  /// @post @p input is connected to the indicated Diagram input port.
  void ConnectInput(
      InputPortIndex diagram_port_index, const InputPort<T>& input);

  /// Declares that the given @p output port of a constituent system is an
  /// output of the entire diagram.  @p name is an optional name for the output
  /// port; if it is unspecified, then a default name will be provided.
  /// @pre If supplied at all, @p name must not be empty.
  /// @return The index of the exported output port of the entire diagram.
  OutputPortIndex ExportOutput(
      const OutputPort<T>& output,
      std::variant<std::string, UseDefaultName> name = kUseDefaultName);

  /// Builds the Diagram that has been described by the calls to Connect,
  /// ExportInput, and ExportOutput.
  /// @throws std::exception if the graph is not buildable.
  std::unique_ptr<Diagram<T>> Build();

  /// Configures @p target to have the topology that has been described by
  /// the calls to Connect, ExportInput, and ExportOutput.
  /// @throws std::exception if the graph is not buildable.
  ///
  /// Only Diagram subclasses should call this method. The target must not
  /// already be initialized.
  void BuildInto(Diagram<T>* target);

  /// Returns true iff the given input @p port of a constituent system is either
  /// connected to another constituent system or exported as a diagram input.
  bool IsConnectedOrExported(const InputPort<T>& port) const;

 private:
  using InputPortLocator = typename Diagram<T>::InputPortLocator;
  using OutputPortLocator = typename Diagram<T>::OutputPortLocator;

  // Declares a new input to the entire Diagram, using @p model_input to
  // supply the data type. @p name is an optional name for the input port; if
  // it is unspecified, then a default name will be provided.
  // @pre @p model_input must be a port of a system within the diagram.
  // @pre If supplied at all, @p name must not be empty.
  // @pre A port indicated by the resolution of @p name must not exist.
  // @post @p model_input is *not* connected to the new exported input port.
  // @return The index of the exported input port of the entire diagram.
  InputPortIndex DeclareInput(
      const InputPort<T>& model_input,
      std::variant<std::string, UseDefaultName> name = kUseDefaultName);

  // Throws if the given input port (belonging to a child subsystem) has
  // already been connected to an output port, or exported to be an input
  // port of the whole diagram.
  void ThrowIfInputAlreadyWired(const InputPortLocator& id) const;

  void ThrowIfSystemNotRegistered(const System<T>* system) const;

  // (Defined lower down in this file.)
  void ThrowIfAlgebraicLoopsExist() const;

  // Produces the Blueprint that has been described by the calls to
  // Connect, ExportInput, and ExportOutput. Throws std::exception if the
  // graph is empty or contains algebraic loops.
  // The DiagramBuilder passes ownership of the registered systems to the
  // blueprint.
  std::unique_ptr<typename Diagram<T>::Blueprint> Compile();

  // The ordered inputs and outputs of the Diagram to be built.
  std::vector<InputPortLocator> input_port_ids_;
  std::vector<std::string> input_port_names_;
  std::vector<OutputPortLocator> output_port_ids_;
  std::vector<std::string> output_port_names_;

  // For fast membership queries: has this input port already been wired?
  std::set<InputPortLocator> diagram_input_set_;

  // A vector of data about exported input ports.
  struct ExportedInputData {
    // Which port to use for data type comparisons at connection time.
    InputPortLocator model_input;
    // The name of the port.
    std::string name;
  };
  std::vector<ExportedInputData> diagram_input_data_;

  // The InputPort fan-out API requires name lookup in some cases.
  std::map<std::string, InputPortIndex> diagram_input_indices_;

  // A map from the input ports of constituent systems, to the output ports of
  // the systems from which they get their values.
  std::map<InputPortLocator, OutputPortLocator> connection_map_;

  // A mirror on the systems in the diagram. Should have the same values as
  // registered_systems_. Used for fast membership queries.
  std::unordered_set<const System<T>*> systems_;
  // The Systems in this DiagramBuilder, in the order they were registered.
  internal::OwnedSystems<T> registered_systems_;
};

}  // namespace systems
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::systems::DiagramBuilder)
