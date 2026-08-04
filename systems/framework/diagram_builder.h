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
#include "drake/common/hash.h"
#include "drake/common/pointer_cast.h"
#include "drake/common/string_map.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/system.h"

namespace drake {
namespace systems {

/// DiagramBuilder is a factory class for Diagram.
///
/// It is single use: after calling Build or BuildInto, DiagramBuilder gives up
/// ownership of the constituent systems, and should therefore be discarded;
/// all member functions will throw an exception after this point.
///
/// When a Diagram (or DiagramBuilder) that owns systems is destroyed, the
/// systems will be destroyed in the reverse of the order they were added.
///
/// A system must be added to the DiagramBuilder with AddSystem or
/// AddNamedSystem before it can be wired up in any way. Every system must have
/// a unique, non-empty name.
///
/// @anchor DiagramBuilder_feedthrough
/// <h2>Building large Diagrams</h2>
///
/// When building large Diagrams with many added systems and input-output port
/// connections, the runtime performance of DiagramBuilder::Build() might become
/// relevant.
///
/// As part of its correctness checks, the DiagramBuilder::Build() function
/// performs a graph search of the diagram's dependencies. In the graph, the
/// nodes are the child systems that have been added to the diagram, and the
/// edges are the diagram connections from one child's output port to another
/// child's input port(s). The builder must confirm that the graph is acyclic;
/// a cycle would imply an infinite loop in an output calculation function.
/// With a large graph, this check can be computationally expensive. To speed
/// it up, ensure that your output ports do not gratuitously depend on
/// irrelevant input ports.
///
/// The dependencies are supplied via the `prerequisites_of_calc` argument to
/// DeclareOutputPort family of functions.  If the default value is used (i.e.,
/// when no prerequisites are provided), the default is to assume the output
/// port value is dependent on all possible sources.
///
/// Refer to the
/// @ref DeclareLeafOutputPort_feedthrough "Direct feedthrough"
/// documentation for additional details and examples.  In particular, the
/// SystemBase::all_sources_except_input_ports_ticket() is a convenient
/// shortcut for outputs that do not depend on any inputs.
template <typename T>
class DiagramBuilder {
 public:
  // DiagramBuilder objects are neither copyable nor moveable.
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(DiagramBuilder);

  /// A designator for a "system + input port" pair, to uniquely refer to
  /// some input port on one of this builder's subsystems.
  using InputPortLocator = typename Diagram<T>::InputPortLocator;

  /// A designator for a "system + output port" pair, to uniquely refer to
  /// some output port on one of this builder's subsystems.
  using OutputPortLocator = typename Diagram<T>::OutputPortLocator;

  DiagramBuilder();
  virtual ~DiagramBuilder();

  /// Takes ownership of `system` and adds it to the builder. Returns a bare
  /// pointer to the System, which will remain valid for the lifetime of the
  /// Diagram built by this builder.
  ///
  /// If the system's name is unset, sets it to System::GetMemoryObjectName()
  /// as a default in order to have unique names within the diagram.
  ///
  /// @warning a System may only be added to at most one DiagramBuilder.
  /// Multiple Diagram instances cannot share the same System.
  template <class S>
  S* AddSystem(std::shared_ptr<S> system) {
    S* result = system.get();
    this->AddSystemImpl(std::move(system));
    return result;
  }

  /// Takes ownership of `system` and adds it to the builder. Returns a bare
  /// pointer to the System, which will remain valid for the lifetime of the
  /// Diagram built by this builder.
  ///
  /// If the system's name is unset, sets it to System::GetMemoryObjectName()
  /// as a default in order to have unique names within the diagram.
  ///
  /// @exclude_from_pydrake_mkdoc{Not bound in pydrake -- pydrake only uses the
  /// shared_ptr overload.}
  template <class S>
  S* AddSystem(std::unique_ptr<S> system) {
    S* result = system.get();
    this->AddSystemImpl(std::move(system));
    return result;
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
  /// @tparam S The type of System to construct. Must subclass System<T>.
  ///
  /// @exclude_from_pydrake_mkdoc{Not bound in pydrake -- emplacement while
  /// specifying <T> doesn't make sense for that language.}
  template <class S, typename... Args>
  S* AddSystem(Args&&... args) {
    auto system = std::make_shared<S>(std::forward<Args>(args)...);
    S* result = system.get();
    this->AddSystemImpl(std::move(system));
    return result;
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
  /// @tparam S A template for the type of System to construct. The template
  /// will be specialized on the scalar type T of this builder.
  ///
  /// @exclude_from_pydrake_mkdoc{Not bound in pydrake -- emplacement while
  /// specifying <T> doesn't make sense for that language.}
  template <template <typename Scalar> class S, typename... Args>
  S<T>* AddSystem(Args&&... args) {
    auto system = std::make_shared<S<T>>(std::forward<Args>(args)...);
    S<T>* result = system.get();
    this->AddSystemImpl(std::move(system));
    return result;
  }

  /// Takes ownership of `system`, sets its name to `name`, and adds it to the
  /// builder. Returns a bare pointer to the System, which will remain valid
  /// for the lifetime of the Diagram built by this builder.
  ///
  /// @warning a System may only be added to at most one DiagramBuilder.
  /// Multiple Diagram instances cannot share the same System.
  template <class S>
  S* AddNamedSystem(const std::string& name, std::shared_ptr<S> system) {
    S* result = system.get();
    this->AddNamedSystemImpl(name, std::move(system));
    return result;
  }

  /// Takes ownership of `system`, sets its name to `name`, and adds it to the
  /// builder. Returns a bare pointer to the System, which will remain valid
  /// for the lifetime of the Diagram built by this builder.
  ///
  /// @exclude_from_pydrake_mkdoc{Not bound in pydrake -- pydrake only uses the
  /// shared_ptr overload.}
  template <class S>
  S* AddNamedSystem(const std::string& name, std::unique_ptr<S> system) {
    S* result = system.get();
    this->AddNamedSystemImpl(name, std::move(system));
    return result;
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
  template <class S, typename... Args>
  S* AddNamedSystem(const std::string& name, Args&&... args) {
    auto system = std::make_shared<S>(std::forward<Args>(args)...);
    S* result = system.get();
    this->AddNamedSystemImpl(name, std::move(system));
    return result;
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
  template <template <typename Scalar> class S, typename... Args>
  S<T>* AddNamedSystem(const std::string& name, Args&&... args) {
    auto system = std::make_shared<S<T>>(std::forward<Args>(args)...);
    S<T>* result = system.get();
    this->AddNamedSystemImpl(name, std::move(system));
    return result;
  }

  /// Removes the given system from this builder and disconnects any connections
  /// or exported ports associated with it.
  ///
  /// Note that un-exporting this system's ports might have a ripple effect on
  /// other exported port index assignments. The relative order will remain
  /// intact, but any "holes" created by this removal will be filled in by
  /// decrementing the indices of all higher-numbered ports that remain.
  ///
  /// @warning Because a DiagramBuilder owns the objects it contains, the system
  /// will be deleted.
  void RemoveSystem(const System<T>& system);

  /// Returns whether any Systems have been added yet.
  bool empty() const {
    ThrowIfAlreadyBuilt();
    return registered_systems_.empty();
  }

  /// Returns true iff Build() or BuildInto() has been called on this Builder,
  /// in which case it's an error to call any member function other than the
  /// the destructor.
  bool already_built() const { return already_built_; }

  /// Returns the list of contained Systems.
  /// @see GetSubsystemByName()
  /// @see GetMutableSystems()
  std::vector<const System<T>*> GetSystems() const;

  /// Returns the list of contained Systems.
  /// @see GetMutableSubsystemByName()
  /// @see GetSystems()
  std::vector<System<T>*> GetMutableSystems();

  /// Returns true iff this contains a subsystem with the given name.
  /// @see GetSubsystemByName()
  bool HasSubsystemNamed(std::string_view name) const;

  /// Retrieves a const reference to the subsystem with name @p name returned
  /// by get_name().
  /// @throws std::exception if a unique match cannot be found.
  /// @see System<T>::get_name()
  /// @see GetMutableSubsystemByName()
  /// @see GetDowncastSubsystemByName()
  const System<T>& GetSubsystemByName(std::string_view name) const;

  /// Retrieves a mutable reference to the subsystem with name @p name returned
  /// by get_name().
  /// @throws std::exception if a unique match cannot be found.
  /// @see System<T>::get_name()
  /// @see GetSubsystemByName()
  /// @see GetMutableDowncastSubsystemByName()
  System<T>& GetMutableSubsystemByName(std::string_view name);

  /// Retrieves a const reference to the subsystem with name @p name returned
  /// by get_name(), downcast to the type provided as a template argument.
  /// @tparam MySystem is the downcast type, e.g., drake::systems::Adder
  /// @throws std::exception if a unique match cannot be found.
  /// @see GetMutableDowncastSubsystemByName()
  /// @see GetSubsystemByName()
  template <template <typename> class MySystem>
  const MySystem<T>& GetDowncastSubsystemByName(std::string_view name) const {
    return GetDowncastSubsystemByName<MySystem<T>>(name);
  }

  /// Retrieves a mutable reference to the subsystem with name @p name returned
  /// by get_name(), downcast to the type provided as a template argument.
  /// @tparam MySystem is the downcast type, e.g., drake::systems::Adder
  /// @throws std::exception if a unique match cannot be found.
  /// @see GetDowncastSubsystemByName()
  /// @see GetMutableSubsystemByName()
  template <template <typename> class MySystem>
  MySystem<T>& GetMutableDowncastSubsystemByName(std::string_view name) {
    return GetMutableDowncastSubsystemByName<MySystem<T>>(name);
  }

#ifndef DRAKE_DOXYGEN_CXX
  // We're omitting this from doxygen as the details are unhelpful.

  // Variants of Get[Mutable]DowncastSubsystemByName that allow for leaf
  // systems that are not templatized.
  // The requested LeafSystem must still have the same underlying scalar type
  // as this builder.
  template <class MyUntemplatizedSystem>
  const MyUntemplatizedSystem& GetDowncastSubsystemByName(
      std::string_view name) const {
    static_assert(std::is_same_v<typename MyUntemplatizedSystem::Scalar, T>,
                  "Scalar type of untemplatized System doesn't match the "
                  "DiagramBuilder's.");
    const System<T>& subsystem = this->GetSubsystemByName(name);
    return *dynamic_pointer_cast_or_throw<const MyUntemplatizedSystem>(
        &subsystem);
  }

  template <class MyUntemplatizedSystem>
  MyUntemplatizedSystem& GetMutableDowncastSubsystemByName(
      std::string_view name) {
    static_assert(std::is_same_v<typename MyUntemplatizedSystem::Scalar, T>,
                  "Scalar type of untemplatized System doesn't match the "
                  "DiagramBuilder's.");
    System<T>& subsystem = this->GetMutableSubsystemByName(name);
    return *dynamic_pointer_cast_or_throw<MyUntemplatizedSystem>(&subsystem);
  }
#endif

  /// (Advanced) Returns a reference to the map of connections between Systems.
  /// The reference becomes invalid upon any call to Build or BuildInto.
  const std::map<InputPortLocator, OutputPortLocator>& connection_map() const;

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
  /// This function ignores deprecated ports, unless there is only one port in
  /// which case it will use the deprecated port.
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
  void ConnectInput(std::string_view diagram_port_name,
                    const InputPort<T>& input);

  /// Connects an input to the entire Diagram, indicated by @p
  /// diagram_port_index, to the given @p input port of a constituent system.
  /// @pre The Diagram input indicated by @p diagram_port_index must have been
  /// previously built via ExportInput().
  /// @post @p input is connected to the indicated Diagram input port.
  void ConnectInput(InputPortIndex diagram_port_index,
                    const InputPort<T>& input);

  /// Connects @p dest to the same source as @p exemplar is connected to.
  ///
  /// If @p exemplar was connected to an output port, then @p dest is connected
  /// to that same output. Or, if @p exemplar was exported as an input of this
  /// diagram, then @p dest will be connected to that same diagram input. Or,
  /// if @p exemplar was neither connected or exported, then this function is
  /// a no-op.
  ///
  /// Both @p exemplar and @p dest must be ports of constituent systems that
  /// have already been added to this diagram.
  ///
  /// @return True iff any connection or was made; or false when a no-op.
  bool ConnectToSame(const InputPort<T>& exemplar, const InputPort<T>& dest);

  /// Declares that the given @p output port of a constituent system is an
  /// output of the entire diagram.  @p name is an optional name for the output
  /// port; if it is unspecified, then a default name will be provided.
  /// @pre If supplied at all, @p name must not be empty.
  /// @return The index of the exported output port of the entire diagram.
  OutputPortIndex ExportOutput(
      const OutputPort<T>& output,
      std::variant<std::string, UseDefaultName> name = kUseDefaultName);

  /// Undoes a Connect() by disconnecting the given subsystem ports.
  /// @see connection_map()
  /// @throws std::exception if the ports were not already connected.
  void Disconnect(const OutputPort<T>& source, const InputPort<T>& dest);

  /// Builds the Diagram that has been described by the calls to Connect,
  /// ExportInput, and ExportOutput.
  /// @throws std::exception if the graph is not buildable.
  ///
  /// See @ref DiagramBuilder_feedthrough "Building large Diagrams" for tips
  /// on improving runtime performance of this function.
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

  /// Returns the current number of diagram input ports. The count may change
  /// as more ports are exported.
  int num_input_ports() const;

  /// Returns the current number of diagram output outputs. The count may change
  /// as more ports are exported.
  int num_output_ports() const;

  /// (Internal use only). Returns a mutable reference to life support data for
  /// the diagram. The data will be moved to the diagram at Build() time. Data
  /// stored here will have a life-cycle that is the union of the builder and
  /// the diagram.
  internal::DiagramLifeSupport& get_mutable_life_support() {
    return life_support_;
  }

 private:
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

  void ThrowIfAlreadyBuilt() const;

  void AddSystemImpl(std::shared_ptr<System<T>>&& system);
  void AddNamedSystemImpl(const std::string& name,
                          std::shared_ptr<System<T>>&& system);

  // Throws if the given input port (belonging to a child subsystem) has
  // already been connected to an output port, or exported to be an input
  // port of the whole diagram.
  void ThrowIfInputAlreadyWired(const InputPortLocator& id) const;

  void ThrowIfSystemNotRegistered(const System<T>* system) const;

  void ThrowIfAlgebraicLoopsExist() const;

  void CheckInvariants() const;

  // Produces the Blueprint that has been described by the calls to
  // Connect, ExportInput, and ExportOutput. Throws std::exception if the
  // graph is empty or contains algebraic loops.
  // The DiagramBuilder passes ownership of the registered systems to the
  // blueprint.
  std::unique_ptr<typename Diagram<T>::Blueprint> Compile();

  // Whether or not Build() or BuildInto() has been called yet.
  bool already_built_{false};

  // The ordered inputs and outputs of the Diagram to be built.
  std::vector<InputPortLocator> input_port_ids_;
  std::vector<std::string> input_port_names_;
  std::vector<OutputPortLocator> output_port_ids_;
  std::vector<std::string> output_port_names_;

  // For fast membership queries: has this input port already been wired?
  std::unordered_set<InputPortLocator, DefaultHash> diagram_input_set_;

  // A vector of data about exported input ports.
  struct ExportedInputData {
    // Which port to use for data type comparisons at connection time.
    InputPortLocator model_input;
    // The name of the port.
    std::string name;
  };
  std::vector<ExportedInputData> diagram_input_data_;

  // The InputPort fan-out API requires name lookup in some cases.
  string_map<InputPortIndex> diagram_input_indices_;

  // A map from the input ports of constituent systems, to the output ports of
  // the systems from which they get their values.
  std::map<InputPortLocator, OutputPortLocator> connection_map_;

  // A mirror on the systems in the diagram. Should have the same values as
  // registered_systems_. Used for fast membership queries.
  std::unordered_set<const System<T>*> systems_;
  // The Systems in this DiagramBuilder, in the order they were registered.
  internal::OwnedSystems<T> registered_systems_;

  internal::DiagramLifeSupport life_support_;
};

}  // namespace systems
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::systems::DiagramBuilder);
