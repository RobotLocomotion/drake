#include "drake/systems/framework/system_base.h"
#include <pybind11/iostream.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

class SystemBase_publicist : public ::drake::systems::SystemBase {
public:
  using ::drake::systems::SystemBase::AddAbstractParameter;
  using ::drake::systems::SystemBase::AddAbstractState;
  using ::drake::systems::SystemBase::AddDiscreteStateGroup;
  using ::drake::systems::SystemBase::AddInputPort;
  using ::drake::systems::SystemBase::AddNumericParameter;
  using ::drake::systems::SystemBase::AddOutputPort;
  using ::drake::systems::SystemBase::assign_next_dependency_ticket;
  using ::drake::systems::SystemBase::ContextSizes;
  using ::drake::systems::SystemBase::DeclareCacheEntryWithKnownTicket;
  using ::drake::systems::SystemBase::DoAllocateContext;
  using ::drake::systems::SystemBase::EvalAbstractInputImpl;
  using ::drake::systems::SystemBase::get_parent_service;
  using ::drake::systems::SystemBase::get_system_id;
  using ::drake::systems::SystemBase::GetInputPortBaseOrThrow;
  using ::drake::systems::SystemBase::GetOutputPortBaseOrThrow;
  using ::drake::systems::SystemBase::InitializeContextBase;
  using ::drake::systems::SystemBase::MakeFixInputPortTypeChecker;
  using ::drake::systems::SystemBase::NextInputPortName;
  using ::drake::systems::SystemBase::NextOutputPortName;
  using ::drake::systems::SystemBase::
      set_implicit_time_derivatives_residual_size;
  using ::drake::systems::SystemBase::set_parent_service;
  using ::drake::systems::SystemBase::ThrowCantEvaluateInputPort;
  using ::drake::systems::SystemBase::ThrowInputPortHasWrongType;
  using ::drake::systems::SystemBase::ThrowInputPortIndexOutOfRange;
  using ::drake::systems::SystemBase::ThrowNegativePortIndex;
  using ::drake::systems::SystemBase::ThrowNotAVectorInputPort;
  using ::drake::systems::SystemBase::ThrowOutputPortIndexOutOfRange;
  using ::drake::systems::SystemBase::ThrowValidateContextMismatch;
};

namespace py = pybind11;
void apb11_pydrake_SystemBase_py_register(py::module &m) {
  static bool called = false;
  if (called) {
    return;
  }
  called = true;
  using namespace drake::systems;

  py::class_<SystemBase, internal::SystemMessageInterface> PySystemBase(
      m, "SystemBase",
      R"""(/** Provides non-templatized functionality shared by the templatized System 
classes. 
 
Terminology: in general a Drake System is a tree structure composed of 
"subsystems", which are themselves System objects. The corresponding Context is 
a parallel tree structure composed of "subcontexts", which are themselves 
Context objects. There is a one-to-one correspondence between subsystems and 
subcontexts. Within a given System (Context), its child subsystems (subcontexts) 
are indexed using a SubsystemIndex; there is no separate SubcontextIndex since 
the numbering must be identical. */)""");

  PySystemBase
      .def(
          "AddAbstractParameter",
          static_cast<void (SystemBase::*)(AbstractParameterIndex)>(
              &SystemBase_publicist::AddAbstractParameter),
          py::arg("index"),
          R"""(/** (Internal use only) Assigns a ticket to a new abstract parameter with 
the given `index`. 
@pre The supplied index must be the next available one; that is, indexes 
     must be assigned sequentially. */)""")
      .def(
          "AddAbstractState",
          static_cast<void (SystemBase::*)(AbstractStateIndex)>(
              &SystemBase_publicist::AddAbstractState),
          py::arg("index"),
          R"""(/** (Internal use only) Assigns a ticket to a new abstract state variable with 
the given `index`. 
@pre The supplied index must be the next available one; that is, indexes 
     must be assigned sequentially. */)""")
      .def(
          "AddDiscreteStateGroup",
          static_cast<void (SystemBase::*)(DiscreteStateIndex)>(
              &SystemBase_publicist::AddDiscreteStateGroup),
          py::arg("index"),
          R"""(/** (Internal use only) Assigns a ticket to a new discrete variable group 
with the given `index`. 
@pre The supplied index must be the next available one; that is, indexes 
     must be assigned sequentially. */)""")
      .def(
          "AddInputPort",
          [](SystemBase &self, drake::systems::InputPortBase port) {
            self.AddInputPort(
                std::make_unique<drake::systems::InputPortBase>(port));
          },
          R"""(/** (Internal use only) Adds an already-constructed input port to this System. 
Insists that the port already contains a reference to this System, and that 
the port's index is already set to the next available input port index for 
this System, that the port name is unique (just within this System), and that 
the port name is non-empty. */)""")
      .def(
          "AddNumericParameter",
          static_cast<void (SystemBase::*)(NumericParameterIndex)>(
              &SystemBase_publicist::AddNumericParameter),
          py::arg("index"),
          R"""(/** (Internal use only) Assigns a ticket to a new numeric parameter with 
the given `index`. 
@pre The supplied index must be the next available one; that is, indexes 
     must be assigned sequentially. */)""")
      .def(
          "AddOutputPort",
          [](SystemBase &self, drake::systems::OutputPortBase port) {
            self.AddOutputPort(
                std::make_unique<drake::systems::OutputPortBase>(port));
          },
          R"""(/** (Internal use only) Adds an already-constructed output port to this 
System. Insists that the port already contains a reference to this System, and 
that the port's index is already set to the next available output port index 
for this System, and that the name of the port is unique. 
@throws std::logic_error if the name of the output port is not unique. */)""")
      .def(
          "AllocateContext",
          static_cast<::std::unique_ptr<
              drake::systems::ContextBase,
              std::default_delete<drake::systems::ContextBase>> (
              SystemBase::*)() const>(&SystemBase::AllocateContext),
          R"""(/** Returns a Context suitable for use with this System. Context resources 
are allocated based on resource requests that were made during System 
construction. */)""")
      .def(
          "DeclareCacheEntry",
          [](SystemBase &self, ::std::string description,
             CacheEntry::AllocCallback alloc_function,
             CacheEntry::CalcCallback calc_function,
             ::std::set<
                 drake::TypeSafeIndex<drake::systems::DependencyTag>,
                 std::less<drake::TypeSafeIndex<drake::systems::DependencyTag>>,
                 std::allocator<
                     drake::TypeSafeIndex<drake::systems::DependencyTag>>>
                 prerequisites_of_calc) {
            return self.DeclareCacheEntry(description, alloc_function,
                                          calc_function, prerequisites_of_calc);
          },
          R"""(/// @anchor DeclareCacheEntry_primary 
/** Declares a new %CacheEntry in this System using the least-restrictive 
definitions for the associated functions. Prefer one of the more-convenient 
signatures below if you can. The new cache entry is assigned a unique 
CacheIndex and DependencyTicket, which can be obtained from the returned 
%CacheEntry. The function signatures here are: 
@code 
  std::unique_ptr<AbstractValue> Alloc(); 
  void Calc(const ContextBase&, AbstractValue*); 
@endcode 
where the AbstractValue objects must resolve to the same concrete type. 
 
@param[in] description 
  A human-readable description of this cache entry, most useful for debugging 
  and documentation. Not interpreted in any way by Drake; it is retained 
  by the cache entry and used to generate the description for the 
  corresponding CacheEntryValue in the Context. 
@param[in] alloc_function 
  Given a Context, returns a heap-allocated AbstractValue object suitable for 
  holding a value for this cache entry. 
@param[in] calc_function 
  Provides the computation that maps from a given Context to the current 
  value that this cache entry should have, and writes that value to a given 
  object of the type returned by `alloc_function`. 
@param[in] prerequisites_of_calc 
  Provides the DependencyTicket list containing a ticket for _every_ Context 
  value on which `calc_function` may depend when it computes its result. 
  Defaults to `{all_sources_ticket()}` if unspecified. If the cache value 
  is truly independent of the Context (rare!) say so explicitly by providing 
  the list `{nothing_ticket()}`; an explicitly empty list `{}` is forbidden. 
@returns a reference to the newly-created %CacheEntry. 
@throws std::logic_error if given an explicitly empty prerequisite list. */)""")
      .def(
          "DeclareCacheEntryWithKnownTicket",
          [](SystemBase &self, DependencyTicket known_ticket,
             ::std::string description,
             CacheEntry::AllocCallback alloc_function,
             CacheEntry::CalcCallback calc_function,
             ::std::set<
                 drake::TypeSafeIndex<drake::systems::DependencyTag>,
                 std::less<drake::TypeSafeIndex<drake::systems::DependencyTag>>,
                 std::allocator<
                     drake::TypeSafeIndex<drake::systems::DependencyTag>>>
                 prerequisites_of_calc) {
            return self.DeclareCacheEntryWithKnownTicket(
                known_ticket, description, alloc_function, calc_function,
                prerequisites_of_calc);
          },
          R"""(/** (Internal use only) This is for cache entries associated with pre-defined 
tickets, for example the cache entry for time derivatives. See the public API 
for the most-general DeclareCacheEntry() signature for the meanings of the 
other parameters here. */)""")
      .def(
          "DoAllocateContext",
          static_cast<::std::unique_ptr<
              drake::systems::ContextBase,
              std::default_delete<drake::systems::ContextBase>> (
              SystemBase::*)() const>(&SystemBase_publicist::DoAllocateContext),
          R"""(/** Derived class implementations should allocate a suitable concrete Context 
type, then invoke the above InitializeContextBase() method. A Diagram must 
then invoke AllocateContext() to obtain each of the subcontexts for its 
DiagramContext, and must set up inter-subcontext dependencies among its 
children and between itself and its children. Then context resources such as 
parameters and state should be allocated. */)""")
      .def(
          "EvalAbstractInput",
          static_cast<::drake::AbstractValue const *(
              SystemBase::*)(ContextBase const &, int)const>(
              &SystemBase::EvalAbstractInput),
          py::arg("context"), py::arg("port_index"),
          R"""(/** Returns the value of the input port with the given `port_index` as an 
AbstractValue, which is permitted for ports of any type. Causes the value to 
become up to date first if necessary, delegating to our parent Diagram. 
Returns a pointer to the port's value, or nullptr if the port is not 
connected. If you know the actual type, use one of the more-specific 
signatures. 
 
@pre `port_index` selects an existing input port of this System. 
 
@see EvalInputValue(), System::EvalVectorInput(), 
     System::EvalEigenVectorInput() */)""")
      .def(
          "EvalAbstractInputImpl",
          static_cast<::drake::AbstractValue const *(
              SystemBase::*)(char const *, ContextBase const &, InputPortIndex)
                          const>(&SystemBase_publicist::EvalAbstractInputImpl),
          py::arg("func"), py::arg("context"), py::arg("port_index"),
          R"""(/** (Internal use only) Shared code for updating an input port and returning a 
pointer to its abstract value, or nullptr if the port is not connected. `func` 
should be the user-visible API function name obtained with __func__. */)""")
      .def(
          "GetDirectFeedthroughs",
          static_cast<
              ::std::multimap<int, int, std::less<int>,
                              std::allocator<std::pair<const int, int>>> (
                  SystemBase::*)() const>(&SystemBase::GetDirectFeedthroughs),
          R"""(/** Reports all direct feedthroughs from input ports to output ports. For 
a system with m input ports: `I = i₀, i₁, ..., iₘ₋₁`, and n output ports, 
`O = o₀, o₁, ..., oₙ₋₁`, the return map will contain pairs (u, v) such that 
 
- 0 ≤ u < m, 
- 0 ≤ v < n, 
- and there _might_ be a direct feedthrough from input iᵤ to each output oᵥ. 
 
See @ref DeclareLeafOutputPort_feedthrough "DeclareLeafOutputPort" 
documentation for how leaf systems can report their feedthrough. 
*/)""")
      .def(
          "GetInputPortBaseOrThrow",
          static_cast<InputPortBase const &(SystemBase::*)(char const *,
                                                           int)const>(
              &SystemBase_publicist::GetInputPortBaseOrThrow),
          py::arg("func"), py::arg("port_index"),
          R"""(/** (Internal use only) Returns the InputPortBase at index `port_index`, 
throwing std::out_of_range we don't like the port index. The name of the 
public API method that received the bad index is provided in `func` and is 
included in the error message. */)""")
      .def(
          "GetOutputPortBaseOrThrow",
          static_cast<OutputPortBase const &(SystemBase::*)(char const *,
                                                            int)const>(
              &SystemBase_publicist::GetOutputPortBaseOrThrow),
          py::arg("func"), py::arg("port_index"),
          R"""(/** (Internal use only) Returns the OutputPortBase at index `port_index`, 
throwing std::out_of_range if we don't like the port index. The name of the 
public API method that received the bad index is provided in `func` and is 
included in the error message. */)""")
      .def(
          "GetSystemName",
          static_cast<::std::string const &(SystemBase::*)() const>(
              &SystemBase::GetSystemName),
          R"""(/** Returns a human-readable name for this system, for use in messages and 
logging. This will be the same as returned by get_name(), unless that would 
be an empty string. In that case we return a non-unique placeholder name, 
currently just "_" (a lone underscore). */)""")
      .def(
          "GetSystemPathname",
          static_cast<::std::string (SystemBase::*)() const>(
              &SystemBase::GetSystemPathname),
          R"""(/** Generates and returns a human-readable full path name of this subsystem, 
for use in messages and logging. The name starts from the root System, with 
"::" delimiters between parent and child subsystems, with the individual 
subsystems represented by their names as returned by GetSystemName(). */)""")
      .def(
          "GetSystemType",
          static_cast<::std::string (SystemBase::*)() const>(
              &SystemBase::GetSystemType),
          R"""(/** Returns the most-derived type of this concrete System object as a 
human-readable string suitable for use in error messages. The format is as 
generated by NiceTypeName and will include namespace qualification if 
present. 
@see NiceTypeName for more specifics. */)""")
      .def(
          "InitializeContextBase",
          static_cast<void (SystemBase::*)(ContextBase *) const>(
              &SystemBase_publicist::InitializeContextBase),
          py::arg("context"),
          R"""(/** This method must be invoked from within derived class DoAllocateContext() 
implementations right after the concrete Context object has been allocated. 
It allocates cache entries, sets up all intra-Context dependencies, and marks 
the ContextBase as initialized so that we can verify proper derived-class 
behavior. 
@pre The supplied context must not be null and must not already have been 
     initialized. */)""")
      .def(
          "MakeFixInputPortTypeChecker",
          static_cast<::std::function<void(const drake::AbstractValue &)> (
              SystemBase::*)(InputPortIndex) const>(
              &SystemBase_publicist::MakeFixInputPortTypeChecker),
          py::arg("port_index"),
          R"""(/** (Internal use only) Given a `port_index`, returns a function to be called 
when validating Context::FixInputPort requests. The function should attempt 
to throw an exception if the input AbstractValue is invalid, so that errors 
can be reported at Fix-time instead of EvalInput-time.*/)""")
      .def(
          "NextInputPortName",
          static_cast<::std::string (SystemBase::*)(
              ::std::variant<std::basic_string<char, std::char_traits<char>,
                                               std::allocator<char>>,
                             drake::systems::UseDefaultName>) const>(
              &SystemBase_publicist::NextInputPortName),
          py::arg("given_name"),
          R"""(/** (Internal use only) Returns a name for the next input port, using the 
given name if it isn't kUseDefaultName, otherwise making up a name like "u3" 
from the next available input port index. 
@pre `given_name` must not be empty. */)""")
      .def(
          "NextOutputPortName",
          static_cast<::std::string (SystemBase::*)(
              ::std::variant<std::basic_string<char, std::char_traits<char>,
                                               std::allocator<char>>,
                             drake::systems::UseDefaultName>) const>(
              &SystemBase_publicist::NextOutputPortName),
          py::arg("given_name"),
          R"""(/** (Internal use only) Returns a name for the next output port, using the 
given name if it isn't kUseDefaultName, otherwise making up a name like "y3" 
from the next available output port index. 
@pre `given_name` must not be empty. */)""")
      .def(
          "ThrowCantEvaluateInputPort",
          static_cast<void (SystemBase::*)(char const *, InputPortIndex) const>(
              &SystemBase_publicist::ThrowCantEvaluateInputPort),
          py::arg("func"), py::arg("port_index"),
          R"""(/** Throws std::logic_error because someone called API method `func`, that 
requires this input port to be evaluatable, but the port was neither 
fixed nor connected. */)""")
      .def(
          "ThrowInputPortHasWrongType",
          static_cast<void (SystemBase::*)(char const *, InputPortIndex,
                                           ::std::string const &,
                                           ::std::string const &) const>(
              &SystemBase_publicist::ThrowInputPortHasWrongType),
          py::arg("func"), py::arg("port_index"), py::arg("expected_type"),
          py::arg("actual_type"),
          R"""(/** Throws std::logic_error because someone called API method `func` claiming 
the input port had some value type that was wrong. */)""")
      .def_static(
          "ThrowInputPortHasWrongType",
          static_cast<void (*)(char const *, ::std::string const &,
                               InputPortIndex, ::std::string const &,
                               ::std::string const &, ::std::string const &)>(
              &SystemBase_publicist::ThrowInputPortHasWrongType),
          py::arg("func"), py::arg("system_pathname"), py::arg("arg2"),
          py::arg("port_name"), py::arg("expected_type"),
          py::arg("actual_type"),
          R"""(/** Throws std::logic_error because someone called API method `func` claiming 
the input port had some value type that was wrong. */)""")
      .def(
          "ThrowInputPortIndexOutOfRange",
          static_cast<void (SystemBase::*)(char const *, InputPortIndex) const>(
              &SystemBase_publicist::ThrowInputPortIndexOutOfRange),
          py::arg("func"), py::arg("port_index"),
          R"""(/** Throws std::out_of_range to report bad input `port_index` that was passed 
to API method `func`. */)""")
      .def("ThrowNegativePortIndex",
           static_cast<void (SystemBase::*)(char const *, int) const>(
               &SystemBase_publicist::ThrowNegativePortIndex),
           py::arg("func"), py::arg("port_index"))
      .def(
          "ThrowNotAVectorInputPort",
          static_cast<void (SystemBase::*)(char const *, InputPortIndex) const>(
              &SystemBase_publicist::ThrowNotAVectorInputPort),
          py::arg("func"), py::arg("port_index"),
          R"""(/** Throws std::logic_error because someone misused API method `func`, that is 
only allowed for declared-vector input ports, on an abstract port whose 
index is given here. */)""")
      .def(
          "ThrowOutputPortIndexOutOfRange",
          static_cast<void (SystemBase::*)(char const *, OutputPortIndex)
                          const>(
              &SystemBase_publicist::ThrowOutputPortIndexOutOfRange),
          py::arg("func"), py::arg("port_index"),
          R"""(/** Throws std::out_of_range to report bad output `port_index` that was passed 
to API method `func`. */)""")
      .def(
          "ThrowValidateContextMismatch",
          static_cast<void (SystemBase::*)(ContextBase const &) const>(
              &SystemBase_publicist::ThrowValidateContextMismatch),
          py::arg("arg0"),
          R"""(/** (Internal use only) Throws std::exception with a message that the sanity 
check(s) given by ValidateContext have failed. */)""")
      .def(
          "ValidateContext",
          static_cast<void (SystemBase::*)(ContextBase const &) const>(
              &SystemBase::ValidateContext),
          py::arg("context"),
          R"""(/** Checks whether the given context was created for this system. 
@note This method is sufficiently fast for performance sensitive code. */)""")
      .def(
          "ValidateContext",
          static_cast<void (SystemBase::*)(ContextBase *) const>(
              &SystemBase::ValidateContext),
          py::arg("context"),
          R"""(/** Checks whether the given context was created for this system. 
@note This method is sufficiently fast for performance sensitive code. */)""")
      .def(
          "abstract_parameter_ticket",
          static_cast<DependencyTicket (SystemBase::*)(AbstractParameterIndex)
                          const>(&SystemBase::abstract_parameter_ticket),
          py::arg("index"),
          R"""(/** Returns a ticket indicating dependence on a particular abstract 
parameter paᵢ. 
@see pa_ticket() to obtain a ticket for _all_ abstract parameters. */)""")
      .def(
          "abstract_state_ticket",
          static_cast<DependencyTicket (SystemBase::*)(AbstractStateIndex)
                          const>(&SystemBase::abstract_state_ticket),
          py::arg("index"),
          R"""(/** Returns a ticket indicating dependence on a particular abstract state 
variable xaᵢ. 
@see xa_ticket() to obtain a ticket for _all_ abstract variables. */)""")
      .def_static(
          "accuracy_ticket",
          static_cast<DependencyTicket (*)()>(&SystemBase::accuracy_ticket),
          R"""(/** Returns a ticket indicating dependence on the accuracy setting in the 
Context. This is the same ticket for all systems and refers to the same 
accuracy value. */)""")
      .def_static(
          "all_input_ports_ticket",
          static_cast<DependencyTicket (*)()>(
              &SystemBase::all_input_ports_ticket),
          R"""(/** Returns a ticket indicating dependence on _all_ input ports u of this 
system. 
@see input_port_ticket() to obtain a ticket for just one input port. */)""")
      .def_static(
          "all_parameters_ticket",
          static_cast<DependencyTicket (*)()>(
              &SystemBase::all_parameters_ticket),
          R"""(/** Returns a ticket indicating dependence on _all_ parameters p in this 
system, including numeric parameters pn, and abstract parameters pa. */)""")
      .def_static(
          "all_sources_except_input_ports_ticket",
          static_cast<DependencyTicket (*)()>(
              &SystemBase::all_sources_except_input_ports_ticket),
          R"""(/** Returns a ticket indicating dependence on every possible independent 
source value _except_ input ports. This can be helpful in avoiding the 
incorrect appearance of algebraic loops in a Diagram (those always involve 
apparent input port dependencies). For an output port, use this ticket plus 
tickets for just the input ports on which the output computation _actually_ 
depends. The sources included in this ticket are: time, accuracy, state, 
and parameters. Note that dependencies on cache entries are _not_ included 
here. Usually that won't matter since cache entries typically depend on at 
least one of time, accuracy, state, or parameters so will be invalidated for 
the same reason the current computation is. However, for a computation that 
depends on a cache entry that depends only on input ports, be sure that 
you have included those input ports in the dependency list, or include a 
direct dependency on the cache entry. 
 
@see input_port_ticket() to obtain a ticket for an input port. 
@see cache_entry_ticket() to obtain a ticket for a cache entry. 
@see all_sources_ticket() to also include all input ports as dependencies. */)""")
      .def_static(
          "all_sources_ticket",
          static_cast<DependencyTicket (*)()>(&SystemBase::all_sources_ticket),
          R"""(/** Returns a ticket indicating dependence on every possible independent 
source value, including time, accuracy, state, input ports, and parameters 
(but not cache entries). This is the default dependency for computations that 
have not specified anything more refined. It is equivalent to the set 
`{all_sources_except_input_ports_ticket(), all_input_ports_ticket()}`. 
@see cache_entry_ticket() to obtain a ticket for a cache entry. */)""")
      .def_static(
          "all_state_ticket",
          static_cast<DependencyTicket (*)()>(&SystemBase::all_state_ticket),
          R"""(/** Returns a ticket indicating dependence on _all_ state variables x in this 
system, including continuous variables xc, discrete (numeric) variables xd, 
and abstract state variables xa. This does not imply dependence on time, 
accuracy, parameters, or inputs; those must be specified separately. If you 
mean to express dependence on all possible value sources, use 
all_sources_ticket() instead. */)""")
      .def(
          "assign_next_dependency_ticket",
          static_cast<DependencyTicket (SystemBase::*)()>(
              &SystemBase_publicist::assign_next_dependency_ticket),
          R"""(/** (Internal use only) Assigns the next unused dependency ticket number, 
unique only within a particular system. Each call to this method increments 
the ticket number. */)""")
      .def(
          "cache_entry_ticket",
          static_cast<DependencyTicket (SystemBase::*)(CacheIndex) const>(
              &SystemBase::cache_entry_ticket),
          py::arg("index"),
          R"""(/** Returns a ticket indicating dependence on the cache entry indicated 
by `index`. Note that cache entries are _not_ included in the `all_sources` 
ticket so must be listed separately. 
@pre `index` selects an existing cache entry in this System. */)""")
      .def_static("configuration_ticket",
                  static_cast<DependencyTicket (*)()>(
                      &SystemBase::configuration_ticket))
      .def(
          "discrete_state_ticket",
          static_cast<DependencyTicket (SystemBase::*)(DiscreteStateIndex)
                          const>(&SystemBase::discrete_state_ticket),
          py::arg("index"),
          R"""(/** Returns a ticket indicating dependence on a particular discrete state 
variable xdᵢ (may be a vector). (We sometimes refer to this as a "discrete 
variable group".) 
@see xd_ticket() to obtain a ticket for _all_ discrete variables. */)""")
      .def(
          "get_cache_entry",
          static_cast<CacheEntry const &(SystemBase::*)(CacheIndex) const>(
              &SystemBase::get_cache_entry),
          py::arg("index"),
          R"""(/** Returns a reference to a CacheEntry given its `index`. */)""")
      .def("get_input_port_base",
           static_cast<InputPortBase const &(SystemBase::*)(InputPortIndex)
                           const>(&SystemBase::get_input_port_base),
           py::arg("port_index"),
           R"""(/** Returns a reference to an InputPort given its `port_index`. 
@pre `port_index` selects an existing input port of this System. */)""")
      .def(
          "get_mutable_cache_entry",
          static_cast<CacheEntry &(SystemBase::*)(CacheIndex)>(
              &SystemBase::get_mutable_cache_entry),
          py::arg("index"),
          R"""(/** (Advanced) Returns a mutable reference to a CacheEntry given its `index`. 
Note that you do not need mutable access to a CacheEntry to modify its value 
in a Context, so most users should not use this method. */)""")
      .def(
          "get_name",
          static_cast<::std::string const &(SystemBase::*)() const>(
              &SystemBase::get_name),
          R"""(/** Returns the name last supplied to set_name(), if any. Diagrams built with 
DiagramBuilder will always have a default name for every contained subsystem 
for which no user-provided name is available. Systems created by copying with 
a scalar type change have the same name as the source system. An empty string 
is returned if no name has been set. */)""")
      .def(
          "get_output_port_base",
          static_cast<OutputPortBase const &(SystemBase::*)(OutputPortIndex)
                          const>(&SystemBase::get_output_port_base),
          py::arg("port_index"),
          R"""(/** Returns a reference to an OutputPort given its `port_index`. 
@pre `port_index` selects an existing output port of this System. */)""")
      .def(
          "get_parent_service",
          static_cast<internal::SystemParentServiceInterface const *(
              SystemBase::*)() const>(
              &SystemBase_publicist::get_parent_service),
          R"""(/** Returns a pointer to the service interface of the immediately enclosing 
Diagram if one has been set, otherwise nullptr. */)""")
      .def(
          "get_system_id",
          static_cast<internal::SystemId (SystemBase::*)() const>(
              &SystemBase_publicist::get_system_id),
          R"""(/** (Internal) Gets the id used to tag context data as being created by this 
system. */)""")
      .def(
          "implicit_time_derivatives_residual_size",
          static_cast<int (SystemBase::*)() const>(
              &SystemBase::implicit_time_derivatives_residual_size),
          R"""(/** Returns the size of the implicit time derivatives residual vector. 
By default this is the same as num_continuous_states() but a LeafSystem 
can change it during construction via 
LeafSystem::DeclareImplicitTimeDerivativesResidualSize(). */)""")
      .def(
          "input_port_ticket",
          static_cast<DependencyTicket (SystemBase::*)(InputPortIndex) const>(
              &SystemBase::input_port_ticket),
          py::arg("index"),
          R"""(/** Returns a ticket indicating dependence on input port uᵢ indicated 
by `index`. 
@pre `index` selects an existing input port of this System. */)""")
      .def_static(
          "ke_ticket",
          static_cast<DependencyTicket (*)()>(&SystemBase::ke_ticket),
          R"""(/** Returns a ticket for the cache entry that holds the kinetic energy 
calculation. 
@see System::EvalKineticEnergy() */)""")
      .def_static("kinematics_ticket", static_cast<DependencyTicket (*)()>(
                                           &SystemBase::kinematics_ticket))
      .def_static(
          "nothing_ticket",
          static_cast<DependencyTicket (*)()>(&SystemBase::nothing_ticket),
          R"""(/** Returns a ticket indicating that a computation does not depend on *any* 
source value; that is, it is a constant. If this appears in a prerequisite 
list, it must be the only entry. */)""")
      .def("num_abstract_parameters",
           static_cast<int (SystemBase::*)() const>(
               &SystemBase::num_abstract_parameters),
           R"""(/** Returns the number of declared abstract parameters. */)""")
      .def(
          "num_abstract_states",
          static_cast<int (SystemBase::*)() const>(
              &SystemBase::num_abstract_states),
          R"""(/** Returns the number of declared abstract state variables. */)""")
      .def(
          "num_cache_entries",
          static_cast<int (SystemBase::*)() const>(
              &SystemBase::num_cache_entries),
          R"""(/** Returns the number nc of cache entries currently allocated in this System. 
These are indexed from 0 to nc-1. */)""")
      .def(
          "num_continuous_states",
          static_cast<int (SystemBase::*)() const>(
              &SystemBase::num_continuous_states),
          R"""(/** Returns the number of declared continuous state variables. */)""")
      .def(
          "num_discrete_state_groups",
          static_cast<int (SystemBase::*)() const>(
              &SystemBase::num_discrete_state_groups),
          R"""(/** Returns the number of declared discrete state groups (each group is 
a vector-valued discrete state variable). */)""")
      .def(
          "num_input_ports",
          static_cast<int (SystemBase::*)() const>(
              &SystemBase::num_input_ports),
          R"""(/** Returns the number of input ports currently allocated in this System. 
These are indexed from 0 to %num_input_ports()-1. */)""")
      .def(
          "num_numeric_parameter_groups",
          static_cast<int (SystemBase::*)() const>(
              &SystemBase::num_numeric_parameter_groups),
          R"""(/** Returns the number of declared numeric parameters (each of these is 
a vector-valued parameter). */)""")
      .def(
          "num_output_ports",
          static_cast<int (SystemBase::*)() const>(
              &SystemBase::num_output_ports),
          R"""(/** Returns the number of output ports currently allocated in this System. 
These are indexed from 0 to %num_output_ports()-1. */)""")
      .def(
          "num_total_inputs",
          static_cast<int (SystemBase::*)() const>(
              &SystemBase::num_total_inputs),
          R"""(/** Returns the total dimension of all of the vector-valued input ports (as if 
they were muxed). */)""")
      .def(
          "num_total_outputs",
          static_cast<int (SystemBase::*)() const>(
              &SystemBase::num_total_outputs),
          R"""(/** Returns the total dimension of all of the vector-valued output ports (as 
if they were muxed). */)""")
      .def(
          "numeric_parameter_ticket",
          static_cast<DependencyTicket (SystemBase::*)(NumericParameterIndex)
                          const>(&SystemBase::numeric_parameter_ticket),
          py::arg("index"),
          R"""(/** Returns a ticket indicating dependence on a particular numeric parameter 
pnᵢ (may be a vector). 
@see pn_ticket() to obtain a ticket for _all_ numeric parameters. */)""")
      .def(
          "output_port_ticket",
          static_cast<DependencyTicket (SystemBase::*)(OutputPortIndex) const>(
              &SystemBase::output_port_ticket),
          py::arg("index"),
          R"""(/** (Internal use only) Returns a ticket indicating dependence on the output 
port indicated by `index`. No user-definable quantities in a system can 
meaningfully depend on that system's own output ports. 
@pre `index` selects an existing output port of this System. */)""")
      .def_static(
          "pa_ticket",
          static_cast<DependencyTicket (*)()>(&SystemBase::pa_ticket),
          R"""(/** Returns a ticket indicating dependence on all of the abstract 
parameters pa in the current Context. 
@see abstract_parameter_ticket() to obtain a ticket for just one abstract 
     parameter. */)""")
      .def_static(
          "pc_ticket",
          static_cast<DependencyTicket (*)()>(&SystemBase::pc_ticket),
          R"""(/** Returns a ticket for the cache entry that holds the conservative power 
calculation. 
@see System::EvalConservativePower() */)""")
      .def_static(
          "pe_ticket",
          static_cast<DependencyTicket (*)()>(&SystemBase::pe_ticket),
          R"""(/** Returns a ticket for the cache entry that holds the potential energy 
calculation. 
@see System::EvalPotentialEnergy() */)""")
      .def_static(
          "pn_ticket",
          static_cast<DependencyTicket (*)()>(&SystemBase::pn_ticket),
          R"""(/** Returns a ticket indicating dependence on all of the numerical 
parameters in the current Context. 
@see numeric_parameter_ticket() to obtain a ticket for just one numeric 
     parameter. */)""")
      .def_static(
          "pnc_ticket",
          static_cast<DependencyTicket (*)()>(&SystemBase::pnc_ticket),
          R"""(/** Returns a ticket for the cache entry that holds the non-conservative 
power calculation. 
@see System::EvalNonConservativePower() */)""")
      .def_static(
          "q_ticket",
          static_cast<DependencyTicket (*)()>(&SystemBase::q_ticket),
          R"""(/** Returns a ticket indicating that a computation depends on configuration 
state variables q. There is no ticket representing just one of the state 
variables qᵢ. */)""")
      .def(
          "set_implicit_time_derivatives_residual_size",
          static_cast<void (SystemBase::*)(int)>(
              &SystemBase_publicist::
                  set_implicit_time_derivatives_residual_size),
          py::arg("n"),
          R"""(/** Allows a LeafSystem to override the default size for the implicit time 
derivatives residual and a Diagram to sum up the total size. If no value 
is set, the default size is n=num_continuous_states(). 
 
@param[in] n The size of the residual vector output argument of 
             System::CalcImplicitTimeDerivativesResidual(). If n <= 0 
             restore to the default, num_continuous_states(). 
 
@see implicit_time_derivatives_residual_size() 
@see LeafSystem::DeclareImplicitTimeDerivativesResidualSize() 
@see System::CalcImplicitTimeDerivativesResidual() */)""")
      .def(
          "set_name",
          static_cast<void (SystemBase::*)(::std::string const &)>(
              &SystemBase::set_name),
          py::arg("name"),
          R"""(/** Sets the name of the system. Do not use the path delimiter character ':' 
in the name. When creating a Diagram, names of sibling subsystems should be 
unique. DiagramBuilder uses this method to assign a unique default name if 
none is provided. */)""")
      .def_static(
          "set_parent_service",
          static_cast<void (*)(SystemBase *,
                               internal::SystemParentServiceInterface const *)>(
              &SystemBase_publicist::set_parent_service),
          py::arg("child"), py::arg("parent_service"),
          R"""(/** (Internal use only) Declares that `parent_service` is the service 
interface of the Diagram that owns this subsystem. Aborts if the parent 
service has already been set to something else. */)""")
      .def_static(
          "time_ticket",
          static_cast<DependencyTicket (*)()>(&SystemBase::time_ticket),
          R"""(/** Returns a ticket indicating dependence on time. This is the same ticket 
for all systems and refers to the same time value. */)""")
      .def_static(
          "v_ticket",
          static_cast<DependencyTicket (*)()>(&SystemBase::v_ticket),
          R"""(/** Returns a ticket indicating dependence on velocity state variables v. This 
does _not_ also indicate a dependence on configuration variables q -- you must 
list that explicitly or use kinematics_ticket() instead. There is no ticket 
representing just one of the state variables vᵢ. */)""")
      .def_static(
          "xa_ticket",
          static_cast<DependencyTicket (*)()>(&SystemBase::xa_ticket),
          R"""(/** Returns a ticket indicating dependence on all of the abstract 
state variables in the current Context. 
@see abstract_state_ticket() to obtain a ticket for just one abstract 
     state variable. */)""")
      .def_static(
          "xc_ticket",
          static_cast<DependencyTicket (*)()>(&SystemBase::xc_ticket),
          R"""(/** Returns a ticket indicating dependence on _all_ of the continuous 
state variables q, v, or z. */)""")
      .def_static(
          "xcdot_ticket",
          static_cast<DependencyTicket (*)()>(&SystemBase::xcdot_ticket),
          R"""(/** Returns a ticket for the cache entry that holds time derivatives of 
the continuous variables. 
@see EvalTimeDerivatives() */)""")
      .def_static(
          "xd_ticket",
          static_cast<DependencyTicket (*)()>(&SystemBase::xd_ticket),
          R"""(/** Returns a ticket indicating dependence on all of the numerical 
discrete state variables, in any discrete variable group. 
@see discrete_state_ticket() to obtain a ticket for just one discrete 
     state variable. */)""")
      .def_static(
          "z_ticket",
          static_cast<DependencyTicket (*)()>(&SystemBase::z_ticket),
          R"""(/** Returns a ticket indicating dependence on any or all of the miscellaneous 
continuous state variables z. There is no ticket representing just one of 
the state variables zᵢ. */)""")

      ;
}
