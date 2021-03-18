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
      m, "SystemBase");

  PySystemBase
      .def("AddAbstractParameter",
           static_cast<void (SystemBase::*)(AbstractParameterIndex)>(
               &SystemBase_publicist::AddAbstractParameter),
           py::arg("index"))
      .def("AddAbstractState",
           static_cast<void (SystemBase::*)(AbstractStateIndex)>(
               &SystemBase_publicist::AddAbstractState),
           py::arg("index"))
      .def("AddDiscreteStateGroup",
           static_cast<void (SystemBase::*)(DiscreteStateIndex)>(
               &SystemBase_publicist::AddDiscreteStateGroup),
           py::arg("index"))
      .def("AddInputPort",
           [](SystemBase &self, InputPortBase port) {
             self.AddInputPort(std::make_unique<InputPortBase>(port));
           })
      .def("AddNumericParameter",
           static_cast<void (SystemBase::*)(NumericParameterIndex)>(
               &SystemBase_publicist::AddNumericParameter),
           py::arg("index"))
      .def("AddOutputPort",
           [](SystemBase &self, OutputPortBase port) {
             self.AddOutputPort(std::make_unique<OutputPortBase>(port));
           })
      .def(
          "AllocateContext",
          static_cast<
              ::std::unique_ptr<ContextBase, std::default_delete<ContextBase>> (
                  SystemBase::*)() const>(&SystemBase::AllocateContext))
      .def("DeclareCacheEntry",
           [](SystemBase &self, ::std::string description,
              CacheEntry::AllocCallback alloc_function,
              CacheEntry::CalcCallback calc_function,
              ::std::set<drake::TypeSafeIndex<DependencyTag>,
                         std::less<drake::TypeSafeIndex<DependencyTag>>,
                         std::allocator<drake::TypeSafeIndex<DependencyTag>>>
                  prerequisites_of_calc) {
             return self.DeclareCacheEntry(description, alloc_function,
                                           calc_function,
                                           prerequisites_of_calc);
           })
      .def("DeclareCacheEntryWithKnownTicket",
           [](SystemBase &self, DependencyTicket known_ticket,
              ::std::string description,
              CacheEntry::AllocCallback alloc_function,
              CacheEntry::CalcCallback calc_function,
              ::std::set<drake::TypeSafeIndex<DependencyTag>,
                         std::less<drake::TypeSafeIndex<DependencyTag>>,
                         std::allocator<drake::TypeSafeIndex<DependencyTag>>>
                  prerequisites_of_calc) {
             return self.DeclareCacheEntryWithKnownTicket(
                 known_ticket, description, alloc_function, calc_function,
                 prerequisites_of_calc);
           })
      .def(
          "DoAllocateContext",
          static_cast<
              ::std::unique_ptr<ContextBase, std::default_delete<ContextBase>> (
                  SystemBase::*)() const>(
              &SystemBase_publicist::DoAllocateContext))
      .def("EvalAbstractInput",
           static_cast<::drake::AbstractValue const *(
               SystemBase::*)(ContextBase const &, int)const>(
               &SystemBase::EvalAbstractInput),
           py::arg("context"), py::arg("port_index"))
      .def("EvalAbstractInputImpl",
           static_cast<::drake::AbstractValue const *(
               SystemBase::*)(char const *, ContextBase const &, InputPortIndex)
                           const>(&SystemBase_publicist::EvalAbstractInputImpl),
           py::arg("func"), py::arg("context"), py::arg("port_index"))
      .def("GetDirectFeedthroughs",
           static_cast<
               ::std::multimap<int, int, std::less<int>,
                               std::allocator<std::pair<const int, int>>> (
                   SystemBase::*)() const>(&SystemBase::GetDirectFeedthroughs))
      .def("GetInputPortBaseOrThrow",
           static_cast<InputPortBase const &(SystemBase::*)(char const *,
                                                            int)const>(
               &SystemBase_publicist::GetInputPortBaseOrThrow),
           py::arg("func"), py::arg("port_index"))
      .def("GetOutputPortBaseOrThrow",
           static_cast<OutputPortBase const &(SystemBase::*)(char const *,
                                                             int)const>(
               &SystemBase_publicist::GetOutputPortBaseOrThrow),
           py::arg("func"), py::arg("port_index"))
      .def("GetSystemName",
           static_cast<::std::string const &(SystemBase::*)() const>(
               &SystemBase::GetSystemName))
      .def("GetSystemPathname",
           static_cast<::std::string (SystemBase::*)() const>(
               &SystemBase::GetSystemPathname))
      .def("GetSystemType", static_cast<::std::string (SystemBase::*)() const>(
                                &SystemBase::GetSystemType))
      .def("InitializeContextBase",
           static_cast<void (SystemBase::*)(ContextBase *) const>(
               &SystemBase_publicist::InitializeContextBase),
           py::arg("context"))
      .def("MakeFixInputPortTypeChecker",
           static_cast<::std::function<void(const drake::AbstractValue &)> (
               SystemBase::*)(InputPortIndex) const>(
               &SystemBase_publicist::MakeFixInputPortTypeChecker),
           py::arg("port_index"))
      .def("NextInputPortName",
           static_cast<::std::string (SystemBase::*)(
               ::std::variant<std::basic_string<char, std::char_traits<char>,
                                                std::allocator<char>>,
                              UseDefaultName>) const>(
               &SystemBase_publicist::NextInputPortName),
           py::arg("given_name"))
      .def("NextOutputPortName",
           static_cast<::std::string (SystemBase::*)(
               ::std::variant<std::basic_string<char, std::char_traits<char>,
                                                std::allocator<char>>,
                              UseDefaultName>) const>(
               &SystemBase_publicist::NextOutputPortName),
           py::arg("given_name"))
      .def(
          "ThrowCantEvaluateInputPort",
          static_cast<void (SystemBase::*)(char const *, InputPortIndex) const>(
              &SystemBase_publicist::ThrowCantEvaluateInputPort),
          py::arg("func"), py::arg("port_index"))
      .def("ThrowInputPortHasWrongType",
           static_cast<void (SystemBase::*)(char const *, InputPortIndex,
                                            ::std::string const &,
                                            ::std::string const &) const>(
               &SystemBase_publicist::ThrowInputPortHasWrongType),
           py::arg("func"), py::arg("port_index"), py::arg("expected_type"),
           py::arg("actual_type"))
      .def_static(
          "ThrowInputPortHasWrongType",
          static_cast<void (*)(char const *, ::std::string const &,
                               InputPortIndex, ::std::string const &,
                               ::std::string const &, ::std::string const &)>(
              &SystemBase_publicist::ThrowInputPortHasWrongType),
          py::arg("func"), py::arg("system_pathname"), py::arg("arg2"),
          py::arg("port_name"), py::arg("expected_type"),
          py::arg("actual_type"))
      .def(
          "ThrowInputPortIndexOutOfRange",
          static_cast<void (SystemBase::*)(char const *, InputPortIndex) const>(
              &SystemBase_publicist::ThrowInputPortIndexOutOfRange),
          py::arg("func"), py::arg("port_index"))
      .def("ThrowNegativePortIndex",
           static_cast<void (SystemBase::*)(char const *, int) const>(
               &SystemBase_publicist::ThrowNegativePortIndex),
           py::arg("func"), py::arg("port_index"))
      .def(
          "ThrowNotAVectorInputPort",
          static_cast<void (SystemBase::*)(char const *, InputPortIndex) const>(
              &SystemBase_publicist::ThrowNotAVectorInputPort),
          py::arg("func"), py::arg("port_index"))
      .def("ThrowOutputPortIndexOutOfRange",
           static_cast<void (SystemBase::*)(char const *, OutputPortIndex)
                           const>(
               &SystemBase_publicist::ThrowOutputPortIndexOutOfRange),
           py::arg("func"), py::arg("port_index"))
      .def("ThrowValidateContextMismatch",
           static_cast<void (SystemBase::*)(ContextBase const &) const>(
               &SystemBase_publicist::ThrowValidateContextMismatch),
           py::arg("arg0"))
      .def("ValidateContext",
           static_cast<void (SystemBase::*)(ContextBase const &) const>(
               &SystemBase::ValidateContext),
           py::arg("context"))
      .def("ValidateContext",
           static_cast<void (SystemBase::*)(ContextBase *) const>(
               &SystemBase::ValidateContext),
           py::arg("context"))
      .def("abstract_parameter_ticket",
           static_cast<DependencyTicket (SystemBase::*)(AbstractParameterIndex)
                           const>(&SystemBase::abstract_parameter_ticket),
           py::arg("index"))
      .def("abstract_state_ticket",
           static_cast<DependencyTicket (SystemBase::*)(AbstractStateIndex)
                           const>(&SystemBase::abstract_state_ticket),
           py::arg("index"))
      .def_static("accuracy_ticket", static_cast<DependencyTicket (*)()>(
                                         &SystemBase::accuracy_ticket))
      .def_static("all_input_ports_ticket",
                  static_cast<DependencyTicket (*)()>(
                      &SystemBase::all_input_ports_ticket))
      .def_static("all_parameters_ticket",
                  static_cast<DependencyTicket (*)()>(
                      &SystemBase::all_parameters_ticket))
      .def_static("all_sources_except_input_ports_ticket",
                  static_cast<DependencyTicket (*)()>(
                      &SystemBase::all_sources_except_input_ports_ticket))
      .def_static("all_sources_ticket", static_cast<DependencyTicket (*)()>(
                                            &SystemBase::all_sources_ticket))
      .def_static("all_state_ticket", static_cast<DependencyTicket (*)()>(
                                          &SystemBase::all_state_ticket))
      .def("assign_next_dependency_ticket",
           static_cast<DependencyTicket (SystemBase::*)()>(
               &SystemBase_publicist::assign_next_dependency_ticket))
      .def("cache_entry_ticket",
           static_cast<DependencyTicket (SystemBase::*)(CacheIndex) const>(
               &SystemBase::cache_entry_ticket),
           py::arg("index"))
      .def_static("configuration_ticket",
                  static_cast<DependencyTicket (*)()>(
                      &SystemBase::configuration_ticket))
      .def("discrete_state_ticket",
           static_cast<DependencyTicket (SystemBase::*)(DiscreteStateIndex)
                           const>(&SystemBase::discrete_state_ticket),
           py::arg("index"))
      .def("get_cache_entry",
           static_cast<CacheEntry const &(SystemBase::*)(CacheIndex) const>(
               &SystemBase::get_cache_entry),
           py::arg("index"))
      .def("get_input_port_base",
           static_cast<InputPortBase const &(SystemBase::*)(InputPortIndex)
                           const>(&SystemBase::get_input_port_base),
           py::arg("port_index"))
      .def("get_mutable_cache_entry",
           static_cast<CacheEntry &(SystemBase::*)(CacheIndex)>(
               &SystemBase::get_mutable_cache_entry),
           py::arg("index"))
      .def("get_name",
           static_cast<::std::string const &(SystemBase::*)() const>(
               &SystemBase::get_name))
      .def("get_output_port_base",
           static_cast<OutputPortBase const &(SystemBase::*)(OutputPortIndex)
                           const>(&SystemBase::get_output_port_base),
           py::arg("port_index"))
      .def("get_parent_service",
           static_cast<internal::SystemParentServiceInterface const *(
               SystemBase::*)() const>(
               &SystemBase_publicist::get_parent_service))
      .def("get_system_id",
           static_cast<internal::SystemId (SystemBase::*)() const>(
               &SystemBase_publicist::get_system_id))
      .def("implicit_time_derivatives_residual_size",
           static_cast<int (SystemBase::*)() const>(
               &SystemBase::implicit_time_derivatives_residual_size))
      .def("input_port_ticket",
           static_cast<DependencyTicket (SystemBase::*)(InputPortIndex) const>(
               &SystemBase::input_port_ticket),
           py::arg("index"))
      .def_static("ke_ticket",
                  static_cast<DependencyTicket (*)()>(&SystemBase::ke_ticket))
      .def_static("kinematics_ticket", static_cast<DependencyTicket (*)()>(
                                           &SystemBase::kinematics_ticket))
      .def_static("nothing_ticket", static_cast<DependencyTicket (*)()>(
                                        &SystemBase::nothing_ticket))
      .def("num_abstract_parameters", static_cast<int (SystemBase::*)() const>(
                                          &SystemBase::num_abstract_parameters))
      .def("num_abstract_states", static_cast<int (SystemBase::*)() const>(
                                      &SystemBase::num_abstract_states))
      .def("num_cache_entries", static_cast<int (SystemBase::*)() const>(
                                    &SystemBase::num_cache_entries))
      .def("num_continuous_states", static_cast<int (SystemBase::*)() const>(
                                        &SystemBase::num_continuous_states))
      .def("num_discrete_state_groups",
           static_cast<int (SystemBase::*)() const>(
               &SystemBase::num_discrete_state_groups))
      .def("num_input_ports", static_cast<int (SystemBase::*)() const>(
                                  &SystemBase::num_input_ports))
      .def("num_numeric_parameter_groups",
           static_cast<int (SystemBase::*)() const>(
               &SystemBase::num_numeric_parameter_groups))
      .def("num_output_ports", static_cast<int (SystemBase::*)() const>(
                                   &SystemBase::num_output_ports))
      .def("num_total_inputs", static_cast<int (SystemBase::*)() const>(
                                   &SystemBase::num_total_inputs))
      .def("num_total_outputs", static_cast<int (SystemBase::*)() const>(
                                    &SystemBase::num_total_outputs))
      .def("numeric_parameter_ticket",
           static_cast<DependencyTicket (SystemBase::*)(NumericParameterIndex)
                           const>(&SystemBase::numeric_parameter_ticket),
           py::arg("index"))
      .def("output_port_ticket",
           static_cast<DependencyTicket (SystemBase::*)(OutputPortIndex) const>(
               &SystemBase::output_port_ticket),
           py::arg("index"))
      .def_static("pa_ticket",
                  static_cast<DependencyTicket (*)()>(&SystemBase::pa_ticket))
      .def_static("pc_ticket",
                  static_cast<DependencyTicket (*)()>(&SystemBase::pc_ticket))
      .def_static("pe_ticket",
                  static_cast<DependencyTicket (*)()>(&SystemBase::pe_ticket))
      .def_static("pn_ticket",
                  static_cast<DependencyTicket (*)()>(&SystemBase::pn_ticket))
      .def_static("pnc_ticket",
                  static_cast<DependencyTicket (*)()>(&SystemBase::pnc_ticket))
      .def_static("q_ticket",
                  static_cast<DependencyTicket (*)()>(&SystemBase::q_ticket))
      .def("set_implicit_time_derivatives_residual_size",
           static_cast<void (SystemBase::*)(int)>(
               &SystemBase_publicist::
                   set_implicit_time_derivatives_residual_size),
           py::arg("n"))
      .def("set_name",
           static_cast<void (SystemBase::*)(::std::string const &)>(
               &SystemBase::set_name),
           py::arg("name"))
      .def_static(
          "set_parent_service",
          static_cast<void (*)(SystemBase *,
                               internal::SystemParentServiceInterface const *)>(
              &SystemBase_publicist::set_parent_service),
          py::arg("child"), py::arg("parent_service"))
      .def_static("time_ticket",
                  static_cast<DependencyTicket (*)()>(&SystemBase::time_ticket))
      .def_static("v_ticket",
                  static_cast<DependencyTicket (*)()>(&SystemBase::v_ticket))
      .def_static("xa_ticket",
                  static_cast<DependencyTicket (*)()>(&SystemBase::xa_ticket))
      .def_static("xc_ticket",
                  static_cast<DependencyTicket (*)()>(&SystemBase::xc_ticket))
      .def_static("xcdot_ticket", static_cast<DependencyTicket (*)()>(
                                      &SystemBase::xcdot_ticket))
      .def_static("xd_ticket",
                  static_cast<DependencyTicket (*)()>(&SystemBase::xd_ticket))
      .def_static("z_ticket",
                  static_cast<DependencyTicket (*)()>(&SystemBase::z_ticket))

      ;
}
