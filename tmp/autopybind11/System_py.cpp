#include "drake/systems/framework/system.h"
#include <pybind11/eigen.h>
#include <pybind11/iostream.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

using namespace drake::systems;

class System_double_publicist : public System<double> {
public:
  using System<double>::AddConstraint;
  using System<double>::DeclareInputPort;
  using System<double>::DeclareInputPort;
  using System<double>::DispatchDiscreteVariableUpdateHandler;
  using System<double>::DispatchPublishHandler;
  using System<double>::DispatchUnrestrictedUpdateHandler;
  using System<double>::DoApplyDiscreteVariableUpdate;
  using System<double>::DoApplyUnrestrictedUpdate;
  using System<double>::DoCalcConservativePower;
  using System<double>::DoCalcImplicitTimeDerivativesResidual;
  using System<double>::DoCalcKineticEnergy;
  using System<double>::DoCalcNextUpdateTime;
  using System<double>::DoCalcNonConservativePower;
  using System<double>::DoCalcPotentialEnergy;
  using System<double>::DoCalcTimeDerivatives;
  using System<double>::DoCalcWitnessValue;
  using System<double>::DoGetInitializationEvents;
  using System<double>::DoGetPerStepEvents;
  using System<double>::DoGetPeriodicEvents;
  using System<double>::DoGetWitnessFunctions;
  using System<double>::DoMapQDotToVelocity;
  using System<double>::DoMapVelocityToQDot;
  using System<double>::GetMutableOutputVector;
  using System<double>::forced_discrete_update_events_exist;
  using System<double>::forced_publish_events_exist;
  using System<double>::forced_unrestricted_update_events_exist;
  using System<double>::get_forced_discrete_update_events;
  using System<double>::get_forced_publish_events;
  using System<double>::get_forced_unrestricted_update_events;
  using System<double>::get_mutable_forced_discrete_update_events;
  using System<double>::get_mutable_forced_publish_events;
  using System<double>::get_mutable_forced_unrestricted_update_events;
  using System<double>::set_forced_discrete_update_events;
  using System<double>::set_forced_publish_events;
  using System<double>::set_forced_unrestricted_update_events;
};

class System_Eigen_AutoDiffScalar_Eigen_VectorXd_publicist
    : public System<drake::AutoDiffXd> {
public:
  using System<drake::AutoDiffXd>::AddConstraint;
  using System<drake::AutoDiffXd>::DeclareInputPort;
  using System<drake::AutoDiffXd>::DeclareInputPort;
  using System<drake::AutoDiffXd>::DispatchDiscreteVariableUpdateHandler;
  using System<drake::AutoDiffXd>::DispatchPublishHandler;
  using System<drake::AutoDiffXd>::DispatchUnrestrictedUpdateHandler;
  using System<drake::AutoDiffXd>::DoApplyDiscreteVariableUpdate;
  using System<drake::AutoDiffXd>::DoApplyUnrestrictedUpdate;
  using System<drake::AutoDiffXd>::DoCalcConservativePower;
  using System<drake::AutoDiffXd>::DoCalcImplicitTimeDerivativesResidual;
  using System<drake::AutoDiffXd>::DoCalcKineticEnergy;
  using System<drake::AutoDiffXd>::DoCalcNextUpdateTime;
  using System<drake::AutoDiffXd>::DoCalcNonConservativePower;
  using System<drake::AutoDiffXd>::DoCalcPotentialEnergy;
  using System<drake::AutoDiffXd>::DoCalcTimeDerivatives;
  using System<drake::AutoDiffXd>::DoCalcWitnessValue;
  using System<drake::AutoDiffXd>::DoGetInitializationEvents;
  using System<drake::AutoDiffXd>::DoGetPerStepEvents;
  using System<drake::AutoDiffXd>::DoGetPeriodicEvents;
  using System<drake::AutoDiffXd>::DoGetWitnessFunctions;
  using System<drake::AutoDiffXd>::DoMapQDotToVelocity;
  using System<drake::AutoDiffXd>::DoMapVelocityToQDot;
  using System<drake::AutoDiffXd>::GetMutableOutputVector;
  using System<drake::AutoDiffXd>::forced_discrete_update_events_exist;
  using System<drake::AutoDiffXd>::forced_publish_events_exist;
  using System<drake::AutoDiffXd>::forced_unrestricted_update_events_exist;
  using System<drake::AutoDiffXd>::get_forced_discrete_update_events;
  using System<drake::AutoDiffXd>::get_forced_publish_events;
  using System<drake::AutoDiffXd>::get_forced_unrestricted_update_events;
  using System<drake::AutoDiffXd>::get_mutable_forced_discrete_update_events;
  using System<drake::AutoDiffXd>::get_mutable_forced_publish_events;
  using System<
      drake::AutoDiffXd>::get_mutable_forced_unrestricted_update_events;
  using System<drake::AutoDiffXd>::set_forced_discrete_update_events;
  using System<drake::AutoDiffXd>::set_forced_publish_events;
  using System<drake::AutoDiffXd>::set_forced_unrestricted_update_events;
};

namespace py = pybind11;
void apb11_pydrake_System_py_register(py::module &m) {
  py::class_<System<double>, SystemBase> PySystem_double(m, "System_double");

  PySystem_double
      .def("AddConstraint",
           [](System<double> &self, SystemConstraint<double> constraint) {
             return self.AddConstraint(
                 std::make_unique<SystemConstraint<double>>(constraint));
           })
      .def("AddExternalConstraint",
           static_cast<SystemConstraintIndex (System<double>::*)(
               ExternalSystemConstraint)>(
               &System<double>::AddExternalConstraint),
           py::arg("constraint"))
      .def("AddTriggeredWitnessFunctionToCompositeEventCollection",
           static_cast<void (System<double>::*)(
               Event<double> *, CompositeEventCollection<double> *) const>(
               &System<double>::
                   AddTriggeredWitnessFunctionToCompositeEventCollection),
           py::arg("event"), py::arg("events"))
      .def("AllocateCompositeEventCollection",
           static_cast<::std::unique_ptr<
               CompositeEventCollection<double>,
               std::default_delete<CompositeEventCollection<double>>> (
               System<double>::*)() const>(
               &System<double>::AllocateCompositeEventCollection))
      .def("AllocateContext",
           static_cast<::std::unique_ptr<Context<double>,
                                         std::default_delete<Context<double>>> (
               System<double>::*)() const>(&System<double>::AllocateContext))
      .def("AllocateDiscreteVariables",
           static_cast<
               ::std::unique_ptr<DiscreteValues<double>,
                                 std::default_delete<DiscreteValues<double>>> (
                   System<double>::*)() const>(
               &System<double>::AllocateDiscreteVariables))
      .def("AllocateFixedInputs",
           static_cast<void (System<double>::*)(Context<double> *) const>(
               &System<double>::AllocateFixedInputs),
           py::arg("context"))
      .def("AllocateForcedDiscreteUpdateEventCollection",
           static_cast<::std::unique_ptr<
               EventCollection<DiscreteUpdateEvent<double>>,
               std::default_delete<EventCollection<
                   DiscreteUpdateEvent<double>>>> (System<double>::*)() const>(
               &System<double>::AllocateForcedDiscreteUpdateEventCollection))
      .def("AllocateForcedPublishEventCollection",
           static_cast<::std::unique_ptr<
               EventCollection<PublishEvent<double>>,
               std::default_delete<EventCollection<PublishEvent<double>>>> (
               System<double>::*)() const>(
               &System<double>::AllocateForcedPublishEventCollection))
      .def(
          "AllocateForcedUnrestrictedUpdateEventCollection",
          static_cast<::std::unique_ptr<
              EventCollection<UnrestrictedUpdateEvent<double>>,
              std::default_delete<
                  EventCollection<UnrestrictedUpdateEvent<double>>>> (
              System<double>::*)() const>(
              &System<double>::AllocateForcedUnrestrictedUpdateEventCollection))
      .def("AllocateImplicitTimeDerivativesResidual",
           static_cast<Eigen::VectorXd (System<double>::*)() const>(
               &System<double>::AllocateImplicitTimeDerivativesResidual))
      .def(
          "AllocateInputAbstract",
          static_cast<::std::unique_ptr<
              drake::AbstractValue, std::default_delete<drake::AbstractValue>> (
              System<double>::*)(InputPort<double> const &) const>(
              &System<double>::AllocateInputAbstract),
          py::arg("input_port"))
      .def("AllocateInputVector",
           static_cast<::std::unique_ptr<
               BasicVector<double>, std::default_delete<BasicVector<double>>> (
               System<double>::*)(InputPort<double> const &) const>(
               &System<double>::AllocateInputVector),
           py::arg("input_port"))
      .def(
          "AllocateOutput",
          static_cast<::std::unique_ptr<
              SystemOutput<double>, std::default_delete<SystemOutput<double>>> (
              System<double>::*)() const>(&System<double>::AllocateOutput))
      .def("AllocateTimeDerivatives",
           static_cast<
               ::std::unique_ptr<ContinuousState<double>,
                                 std::default_delete<ContinuousState<double>>> (
                   System<double>::*)() const>(
               &System<double>::AllocateTimeDerivatives))
      .def("ApplyDiscreteVariableUpdate",
           static_cast<void (System<double>::*)(
               EventCollection<DiscreteUpdateEvent<double>> const &,
               DiscreteValues<double> *, Context<double> *) const>(
               &System<double>::ApplyDiscreteVariableUpdate),
           py::arg("events"), py::arg("discrete_state"), py::arg("context"))
      .def("ApplyUnrestrictedUpdate",
           static_cast<void (System<double>::*)(
               EventCollection<UnrestrictedUpdateEvent<double>> const &,
               State<double> *, Context<double> *) const>(
               &System<double>::ApplyUnrestrictedUpdate),
           py::arg("events"), py::arg("state"), py::arg("context"))
      .def("CalcConservativePower",
           static_cast<double (System<double>::*)(Context<double> const &)
                           const>(&System<double>::CalcConservativePower),
           py::arg("context"))
      .def("CalcDiscreteVariableUpdates",
           static_cast<void (System<double>::*)(
               Context<double> const &,
               EventCollection<DiscreteUpdateEvent<double>> const &,
               DiscreteValues<double> *) const>(
               &System<double>::CalcDiscreteVariableUpdates),
           py::arg("context"), py::arg("events"), py::arg("discrete_state"))
      .def("CalcDiscreteVariableUpdates",
           static_cast<void (System<double>::*)(
               Context<double> const &, DiscreteValues<double> *) const>(
               &System<double>::CalcDiscreteVariableUpdates),
           py::arg("context"), py::arg("discrete_state"))
      .def("CalcImplicitTimeDerivativesResidual",
           static_cast<void (System<double>::*)(
               Context<double> const &, ContinuousState<double> const &,
               ::drake::EigenPtr<Eigen::VectorXd>) const>(
               &System<double>::CalcImplicitTimeDerivativesResidual),
           py::arg("context"), py::arg("proposed_derivatives"),
           py::arg("residual"))
      .def("CalcKineticEnergy",
           static_cast<double (System<double>::*)(Context<double> const &)
                           const>(&System<double>::CalcKineticEnergy),
           py::arg("context"))
      .def("CalcNextUpdateTime",
           static_cast<double (System<double>::*)(
               Context<double> const &, CompositeEventCollection<double> *)
                           const>(&System<double>::CalcNextUpdateTime),
           py::arg("context"), py::arg("events"))
      .def("CalcNonConservativePower",
           static_cast<double (System<double>::*)(Context<double> const &)
                           const>(&System<double>::CalcNonConservativePower),
           py::arg("context"))
      .def("CalcOutput",
           static_cast<void (System<double>::*)(Context<double> const &,
                                                SystemOutput<double> *) const>(
               &System<double>::CalcOutput),
           py::arg("context"), py::arg("outputs"))
      .def("CalcPotentialEnergy",
           static_cast<double (System<double>::*)(Context<double> const &)
                           const>(&System<double>::CalcPotentialEnergy),
           py::arg("context"))
      .def("CalcTimeDerivatives",
           static_cast<void (System<double>::*)(
               Context<double> const &, ContinuousState<double> *) const>(
               &System<double>::CalcTimeDerivatives),
           py::arg("context"), py::arg("derivatives"))
      .def("CalcUnrestrictedUpdate",
           static_cast<void (System<double>::*)(
               Context<double> const &,
               EventCollection<UnrestrictedUpdateEvent<double>> const &,
               State<double> *) const>(&System<double>::CalcUnrestrictedUpdate),
           py::arg("context"), py::arg("events"), py::arg("state"))
      .def("CalcUnrestrictedUpdate",
           static_cast<void (System<double>::*)(Context<double> const &,
                                                State<double> *) const>(
               &System<double>::CalcUnrestrictedUpdate),
           py::arg("context"), py::arg("state"))
      .def("CalcWitnessValue",
           static_cast<double (System<double>::*)(
               Context<double> const &, WitnessFunction<double> const &) const>(
               &System<double>::CalcWitnessValue),
           py::arg("context"), py::arg("witness_func"))
      .def("CheckSystemConstraintsSatisfied",
           static_cast<::drake::scalar_predicate<double>::type (
               System<double>::*)(Context<double> const &, double) const>(
               &System<double>::CheckSystemConstraintsSatisfied),
           py::arg("context"), py::arg("tol"))
      .def("CheckValidOutput",
           static_cast<void (System<double>::*)(SystemOutput<double> const *)
                           const>(&System<double>::CheckValidOutput),
           py::arg("output"))
      .def("CopyContinuousStateVector",
           static_cast<Eigen::VectorXd (System<double>::*)(
               Context<double> const &) const>(
               &System<double>::CopyContinuousStateVector),
           py::arg("context"))
      .def("CreateDefaultContext",
           static_cast<::std::unique_ptr<Context<double>,
                                         std::default_delete<Context<double>>> (
               System<double>::*)() const>(
               &System<double>::CreateDefaultContext))
      .def("DeclareInputPort",
           static_cast<InputPort<double> &(
               System<double>::*)(::std::variant<
                                      std::basic_string<char,
                                                        std::char_traits<char>,
                                                        std::allocator<char>>,
                                      UseDefaultName>,
                                  PortDataType, int,
                                  ::std::optional<drake::RandomDistribution>)>(
               &System_double_publicist::DeclareInputPort),
           py::arg("name"), py::arg("type"), py::arg("size"),
           py::arg("random_type") =
               ::std::optional<drake::RandomDistribution>(std::nullopt))
      .def("DeclareInputPort",
           static_cast<InputPort<double> &(
               System<double>::*)(PortDataType, int,
                                  ::std::optional<drake::RandomDistribution>)>(
               &System_double_publicist::DeclareInputPort),
           py::arg("type"), py::arg("size"),
           py::arg("random_type") =
               ::std::optional<drake::RandomDistribution>(std::nullopt))
      .def("DispatchDiscreteVariableUpdateHandler",
           static_cast<void (System<double>::*)(
               Context<double> const &,
               EventCollection<DiscreteUpdateEvent<double>> const &,
               DiscreteValues<double> *) const>(
               &System_double_publicist::DispatchDiscreteVariableUpdateHandler),
           py::arg("context"), py::arg("events"), py::arg("discrete_state"))
      .def("DispatchPublishHandler",
           static_cast<void (System<double>::*)(
               Context<double> const &,
               EventCollection<PublishEvent<double>> const &) const>(
               &System_double_publicist::DispatchPublishHandler),
           py::arg("context"), py::arg("events"))
      .def("DispatchUnrestrictedUpdateHandler",
           static_cast<void (System<double>::*)(
               Context<double> const &,
               EventCollection<UnrestrictedUpdateEvent<double>> const &,
               State<double> *) const>(
               &System_double_publicist::DispatchUnrestrictedUpdateHandler),
           py::arg("context"), py::arg("events"), py::arg("state"))
      .def("DoApplyDiscreteVariableUpdate",
           static_cast<void (System<double>::*)(
               EventCollection<DiscreteUpdateEvent<double>> const &,
               DiscreteValues<double> *, Context<double> *) const>(
               &System_double_publicist::DoApplyDiscreteVariableUpdate),
           py::arg("events"), py::arg("discrete_state"), py::arg("context"))
      .def("DoApplyUnrestrictedUpdate",
           static_cast<void (System<double>::*)(
               EventCollection<UnrestrictedUpdateEvent<double>> const &,
               State<double> *, Context<double> *) const>(
               &System_double_publicist::DoApplyUnrestrictedUpdate),
           py::arg("events"), py::arg("state"), py::arg("context"))
      .def("DoCalcConservativePower",
           static_cast<double (System<double>::*)(Context<double> const &)
                           const>(
               &System_double_publicist::DoCalcConservativePower),
           py::arg("context"))
      .def("DoCalcImplicitTimeDerivativesResidual",
           static_cast<void (System<double>::*)(
               Context<double> const &, ContinuousState<double> const &,
               ::drake::EigenPtr<Eigen::VectorXd>) const>(
               &System_double_publicist::DoCalcImplicitTimeDerivativesResidual),
           py::arg("context"), py::arg("proposed_derivatives"),
           py::arg("residual"))
      .def(
          "DoCalcKineticEnergy",
          static_cast<double (System<double>::*)(Context<double> const &)
                          const>(&System_double_publicist::DoCalcKineticEnergy),
          py::arg("context"))
      .def("DoCalcNextUpdateTime",
           static_cast<void (System<double>::*)(
               Context<double> const &, CompositeEventCollection<double> *,
               double *) const>(&System_double_publicist::DoCalcNextUpdateTime),
           py::arg("context"), py::arg("events"), py::arg("time"))
      .def("DoCalcNonConservativePower",
           static_cast<double (System<double>::*)(Context<double> const &)
                           const>(
               &System_double_publicist::DoCalcNonConservativePower),
           py::arg("context"))
      .def("DoCalcPotentialEnergy",
           static_cast<double (System<double>::*)(Context<double> const &)
                           const>(
               &System_double_publicist::DoCalcPotentialEnergy),
           py::arg("context"))
      .def("DoCalcTimeDerivatives",
           static_cast<void (System<double>::*)(
               Context<double> const &, ContinuousState<double> *) const>(
               &System_double_publicist::DoCalcTimeDerivatives),
           py::arg("context"), py::arg("derivatives"))
      .def("DoCalcWitnessValue",
           static_cast<double (System<double>::*)(
               Context<double> const &, WitnessFunction<double> const &) const>(
               &System_double_publicist::DoCalcWitnessValue),
           py::arg("context"), py::arg("witness_func"))
      .def("DoGetInitializationEvents",
           static_cast<void (System<double>::*)(
               Context<double> const &, CompositeEventCollection<double> *)
                           const>(
               &System_double_publicist::DoGetInitializationEvents),
           py::arg("context"), py::arg("events"))
      .def("DoGetMutableTargetSystemCompositeEventCollection",
           static_cast<CompositeEventCollection<double> *(
               System<double>::*)(System<double> const &,
                                  CompositeEventCollection<double> *)const>(
               &System<
                   double>::DoGetMutableTargetSystemCompositeEventCollection),
           py::arg("target_system"), py::arg("events"))
      .def(
          "DoGetMutableTargetSystemState",
          static_cast<State<double> *(System<double>::*)(System<double> const &,
                                                         State<double> *)const>(
              &System<double>::DoGetMutableTargetSystemState),
          py::arg("target_system"), py::arg("state"))
      .def("DoGetPerStepEvents",
           static_cast<void (System<double>::*)(
               Context<double> const &, CompositeEventCollection<double> *)
                           const>(&System_double_publicist::DoGetPerStepEvents),
           py::arg("context"), py::arg("events"))
      .def("DoGetPeriodicEvents",
           static_cast<::std::map<
               PeriodicEventData,
               std::vector<const Event<double> *,
                           std::allocator<const Event<double> *>>,
               PeriodicEventDataComparator,
               std::allocator<std::pair<
                   const PeriodicEventData,
                   std::vector<const Event<double> *,
                               std::allocator<const Event<double> *>>>>> (
               System<double>::*)() const>(
               &System_double_publicist::DoGetPeriodicEvents))
      .def("DoGetTargetSystemCompositeEventCollection",
           static_cast<CompositeEventCollection<double> const *(
               System<double>::*)(System<double> const &,
                                  CompositeEventCollection<double> const *)
                           const>(
               &System<double>::DoGetTargetSystemCompositeEventCollection),
           py::arg("target_system"), py::arg("events"))
      .def("DoGetTargetSystemContext",
           static_cast<Context<double> const *(
               System<double>::*)(System<double> const &,
                                  Context<double> const *)const>(
               &System<double>::DoGetTargetSystemContext),
           py::arg("target_system"), py::arg("context"))
      .def("DoGetTargetSystemContinuousState",
           static_cast<ContinuousState<double> const *(
               System<double>::*)(System<double> const &,
                                  ContinuousState<double> const *)const>(
               &System<double>::DoGetTargetSystemContinuousState),
           py::arg("target_system"), py::arg("xc"))
      .def("DoGetTargetSystemState",
           static_cast<State<double> const *(
               System<double>::*)(System<double> const &, State<double> const *)
                           const>(&System<double>::DoGetTargetSystemState),
           py::arg("target_system"), py::arg("state"))
      .def("DoGetWitnessFunctions",
           static_cast<void (System<double>::*)(
               Context<double> const &,
               ::std::vector<const WitnessFunction<double> *,
                             std::allocator<const WitnessFunction<double> *>> *)
                           const>(
               &System_double_publicist::DoGetWitnessFunctions),
           py::arg("arg0"), py::arg("arg1"))
      .def("DoMapQDotToVelocity",
           static_cast<void (System<double>::*)(
               Context<double> const &,
               ::Eigen::Ref<const Eigen::VectorXd, 0,
                            Eigen::InnerStride<1>> const &,
               VectorBase<double> *) const>(
               &System_double_publicist::DoMapQDotToVelocity),
           py::arg("context"), py::arg("qdot"), py::arg("generalized_velocity"))
      .def("DoMapVelocityToQDot",
           static_cast<void (System<double>::*)(
               Context<double> const &,
               ::Eigen::Ref<const Eigen::VectorXd, 0,
                            Eigen::InnerStride<1>> const &,
               VectorBase<double> *) const>(
               &System_double_publicist::DoMapVelocityToQDot),
           py::arg("context"), py::arg("generalized_velocity"), py::arg("qdot"))
      .def("EvalConservativePower",
           static_cast<double const &(
               System<double>::*)(Context<double> const &)const>(
               &System<double>::EvalConservativePower),
           py::arg("context"))
      .def("EvalEigenVectorInput",
           static_cast<::Eigen::VectorBlock<const Eigen::VectorXd, -1> (
               System<double>::*)(Context<double> const &, int) const>(
               &System<double>::EvalEigenVectorInput),
           py::arg("context"), py::arg("port_index"))
      .def("EvalKineticEnergy",
           static_cast<double const &(
               System<double>::*)(Context<double> const &)const>(
               &System<double>::EvalKineticEnergy),
           py::arg("context"))
      .def("EvalNonConservativePower",
           static_cast<double const &(
               System<double>::*)(Context<double> const &)const>(
               &System<double>::EvalNonConservativePower),
           py::arg("context"))
      .def("EvalPotentialEnergy",
           static_cast<double const &(
               System<double>::*)(Context<double> const &)const>(
               &System<double>::EvalPotentialEnergy),
           py::arg("context"))
      .def("EvalTimeDerivatives",
           static_cast<ContinuousState<double> const &(
               System<double>::*)(Context<double> const &)const>(
               &System<double>::EvalTimeDerivatives),
           py::arg("context"))
      .def("FixInputPortsFrom",
           static_cast<void (System<double>::*)(
               System<double> const &, Context<double> const &,
               Context<double> *) const>(&System<double>::FixInputPortsFrom),
           py::arg("other_system"), py::arg("other_context"),
           py::arg("target_context"))
      .def("GetGraphvizFragment",
           static_cast<void (System<double>::*)(int, ::std::stringstream *)
                           const>(&System<double>::GetGraphvizFragment),
           py::arg("max_depth"), py::arg("dot"))
      .def("GetGraphvizId", static_cast<::int64_t (System<double>::*)() const>(
                                &System<double>::GetGraphvizId))
      .def("GetGraphvizInputPortToken",
           static_cast<void (System<double>::*)(InputPort<double> const &, int,
                                                ::std::stringstream *) const>(
               &System<double>::GetGraphvizInputPortToken),
           py::arg("port"), py::arg("max_depth"), py::arg("dot"))
      .def("GetGraphvizOutputPortToken",
           static_cast<void (System<double>::*)(OutputPort<double> const &, int,
                                                ::std::stringstream *) const>(
               &System<double>::GetGraphvizOutputPortToken),
           py::arg("port"), py::arg("max_depth"), py::arg("dot"))
      .def("GetGraphvizString",
           static_cast<::std::string (System<double>::*)(int) const>(
               &System<double>::GetGraphvizString),
           py::arg("max_depth") = int(std::numeric_limits<int>::max()))
      .def("GetInitializationEvents",
           static_cast<void (System<double>::*)(
               Context<double> const &, CompositeEventCollection<double> *)
                           const>(&System<double>::GetInitializationEvents),
           py::arg("context"), py::arg("events"))
      .def("GetInputPort",
           static_cast<InputPort<double> const &(
               System<double>::*)(::std::string const &)const>(
               &System<double>::GetInputPort),
           py::arg("port_name"))
      .def("GetMemoryObjectName",
           static_cast<::std::string (System<double>::*)() const>(
               &System<double>::GetMemoryObjectName))
      .def("GetMutableOutputVector",
           static_cast<::Eigen::VectorBlock<Eigen::VectorXd, -1> (
               System<double>::*)(SystemOutput<double> *, int) const>(
               &System_double_publicist::GetMutableOutputVector),
           py::arg("output"), py::arg("port_index"))
      .def("GetMutableSubsystemContext",
           static_cast<Context<double> &(
               System<double>::*)(System<double> const &, Context<double> *)
                           const>(&System<double>::GetMutableSubsystemContext),
           py::arg("subsystem"), py::arg("context"))
      .def("GetMyContextFromRoot",
           static_cast<Context<double> const &(
               System<double>::*)(Context<double> const &)const>(
               &System<double>::GetMyContextFromRoot),
           py::arg("root_context"))
      .def("GetMyMutableContextFromRoot",
           static_cast<Context<double> &(System<double>::*)(Context<double> *)
                           const>(&System<double>::GetMyMutableContextFromRoot),
           py::arg("root_context"))
      .def("GetOutputPort",
           static_cast<OutputPort<double> const &(
               System<double>::*)(::std::string const &)const>(
               &System<double>::GetOutputPort),
           py::arg("port_name"))
      .def("GetPerStepEvents",
           static_cast<void (System<double>::*)(
               Context<double> const &, CompositeEventCollection<double> *)
                           const>(&System<double>::GetPerStepEvents),
           py::arg("context"), py::arg("events"))
      .def("GetPeriodicEvents",
           static_cast<::std::map<
               PeriodicEventData,
               std::vector<const Event<double> *,
                           std::allocator<const Event<double> *>>,
               PeriodicEventDataComparator,
               std::allocator<std::pair<
                   const PeriodicEventData,
                   std::vector<const Event<double> *,
                               std::allocator<const Event<double> *>>>>> (
               System<double>::*)() const>(&System<double>::GetPeriodicEvents))
      .def("GetSubsystemContext",
           static_cast<Context<double> const &(
               System<double>::*)(System<double> const &,
                                  Context<double> const &)const>(
               &System<double>::GetSubsystemContext),
           py::arg("subsystem"), py::arg("context"))
      .def("GetUniquePeriodicDiscreteUpdateAttribute",
           static_cast<::std::optional<PeriodicEventData> (System<double>::*)()
                           const>(
               &System<double>::GetUniquePeriodicDiscreteUpdateAttribute))
      .def("GetWitnessFunctions",
           static_cast<void (System<double>::*)(
               Context<double> const &,
               ::std::vector<const WitnessFunction<double> *,
                             std::allocator<const WitnessFunction<double> *>> *)
                           const>(&System<double>::GetWitnessFunctions),
           py::arg("context"), py::arg("w"))
      .def("HasAnyDirectFeedthrough",
           static_cast<bool (System<double>::*)() const>(
               &System<double>::HasAnyDirectFeedthrough))
      .def("HasDirectFeedthrough",
           static_cast<bool (System<double>::*)(int) const>(
               &System<double>::HasDirectFeedthrough),
           py::arg("output_port"))
      .def("HasDirectFeedthrough",
           static_cast<bool (System<double>::*)(int, int) const>(
               &System<double>::HasDirectFeedthrough),
           py::arg("input_port"), py::arg("output_port"))
      .def("HasInputPort",
           static_cast<bool (System<double>::*)(::std::string const &) const>(
               &System<double>::HasInputPort),
           py::arg("port_name"))
      .def("HasOutputPort",
           static_cast<bool (System<double>::*)(::std::string const &) const>(
               &System<double>::HasOutputPort),
           py::arg("port_name"))
      .def("IsDifferenceEquationSystem",
           static_cast<bool (System<double>::*)(double *) const>(
               &System<double>::IsDifferenceEquationSystem),
           py::arg("time_period") = (double *)nullptr)
      .def("MapQDotToVelocity",
           static_cast<void (System<double>::*)(
               Context<double> const &, VectorBase<double> const &,
               VectorBase<double> *) const>(&System<double>::MapQDotToVelocity),
           py::arg("context"), py::arg("qdot"), py::arg("generalized_velocity"))
      .def("MapQDotToVelocity",
           static_cast<void (System<double>::*)(
               Context<double> const &,
               ::Eigen::Ref<const Eigen::VectorXd, 0,
                            Eigen::InnerStride<1>> const &,
               VectorBase<double> *) const>(&System<double>::MapQDotToVelocity),
           py::arg("context"), py::arg("qdot"), py::arg("generalized_velocity"))
      .def("MapVelocityToQDot",
           static_cast<void (System<double>::*)(
               Context<double> const &, VectorBase<double> const &,
               VectorBase<double> *) const>(&System<double>::MapVelocityToQDot),
           py::arg("context"), py::arg("generalized_velocity"), py::arg("qdot"))
      .def("MapVelocityToQDot",
           static_cast<void (System<double>::*)(
               Context<double> const &,
               ::Eigen::Ref<const Eigen::VectorXd, 0,
                            Eigen::InnerStride<1>> const &,
               VectorBase<double> *) const>(&System<double>::MapVelocityToQDot),
           py::arg("context"), py::arg("generalized_velocity"), py::arg("qdot"))
      .def("Publish",
           static_cast<void (System<double>::*)(
               Context<double> const &,
               EventCollection<PublishEvent<double>> const &) const>(
               &System<double>::Publish),
           py::arg("context"), py::arg("events"))
      .def("Publish",
           static_cast<void (System<double>::*)(Context<double> const &) const>(
               &System<double>::Publish),
           py::arg("context"))
      .def("SetDefaultContext",
           static_cast<void (System<double>::*)(Context<double> *) const>(
               &System<double>::SetDefaultContext),
           py::arg("context"))
      .def("SetDefaultParameters",
           static_cast<void (System<double>::*)(Context<double> const &,
                                                Parameters<double> *) const>(
               &System<double>::SetDefaultParameters),
           py::arg("context"), py::arg("parameters"))
      .def("SetDefaultState",
           static_cast<void (System<double>::*)(Context<double> const &,
                                                State<double> *) const>(
               &System<double>::SetDefaultState),
           py::arg("context"), py::arg("state"))
      .def("SetRandomContext",
           static_cast<void (System<double>::*)(
               Context<double> *, ::drake::RandomGenerator *) const>(
               &System<double>::SetRandomContext),
           py::arg("context"), py::arg("generator"))
      .def("SetRandomParameters",
           static_cast<void (System<double>::*)(
               Context<double> const &, Parameters<double> *,
               ::drake::RandomGenerator *) const>(
               &System<double>::SetRandomParameters),
           py::arg("context"), py::arg("parameters"), py::arg("generator"))
      .def("SetRandomState",
           static_cast<void (System<double>::*)(
               Context<double> const &, State<double> *,
               ::drake::RandomGenerator *) const>(
               &System<double>::SetRandomState),
           py::arg("context"), py::arg("state"), py::arg("generator"))
      .def("ToAutoDiffXd",
           static_cast<::std::unique_ptr<
               System<::drake::AutoDiffXd>,
               std::default_delete<System<::drake::AutoDiffXd>>> (
               System<double>::*)() const>(&System<double>::ToAutoDiffXd))
      .def("ToAutoDiffXdMaybe",
           static_cast<::std::unique_ptr<
               System<::drake::AutoDiffXd>,
               std::default_delete<System<::drake::AutoDiffXd>>> (
               System<double>::*)() const>(&System<double>::ToAutoDiffXdMaybe))
      .def("ToSymbolic",
           static_cast<::std::unique_ptr<
               System<::drake::symbolic::Expression>,
               std::default_delete<System<::drake::symbolic::Expression>>> (
               System<double>::*)() const>(&System<double>::ToSymbolic))
      .def("ToSymbolicMaybe",
           static_cast<::std::unique_ptr<
               System<::drake::symbolic::Expression>,
               std::default_delete<System<::drake::symbolic::Expression>>> (
               System<double>::*)() const>(&System<double>::ToSymbolicMaybe))
      .def("forced_discrete_update_events_exist",
           static_cast<bool (System<double>::*)() const>(
               &System_double_publicist::forced_discrete_update_events_exist))
      .def("forced_publish_events_exist",
           static_cast<bool (System<double>::*)() const>(
               &System_double_publicist::forced_publish_events_exist))
      .def("forced_unrestricted_update_events_exist",
           static_cast<bool (System<double>::*)() const>(
               &System_double_publicist::
                   forced_unrestricted_update_events_exist))
      .def("get_constraint",
           static_cast<SystemConstraint<double> const &(
               System<double>::*)(SystemConstraintIndex) const>(
               &System<double>::get_constraint),
           py::arg("constraint_index"))
      .def("get_forced_discrete_update_events",
           static_cast<EventCollection<DiscreteUpdateEvent<double>> const &(
               System<double>::*)() const>(
               &System_double_publicist::get_forced_discrete_update_events))
      .def("get_forced_publish_events",
           static_cast<EventCollection<PublishEvent<double>> const &(
               System<double>::*)() const>(
               &System_double_publicist::get_forced_publish_events))
      .def("get_forced_unrestricted_update_events",
           static_cast<EventCollection<UnrestrictedUpdateEvent<double>> const &(
               System<double>::*)() const>(
               &System_double_publicist::get_forced_unrestricted_update_events))
      .def("get_input_port",
           static_cast<InputPort<double> const &(System<double>::*)(int)const>(
               &System<double>::get_input_port),
           py::arg("port_index"))
      .def("get_input_port",
           static_cast<InputPort<double> const &(System<double>::*)() const>(
               &System<double>::get_input_port))
      .def("get_input_port_selection",
           static_cast<InputPort<double> const *(
               System<double>::*)(::std::variant<
                                  InputPortSelection,
                                  drake::TypeSafeIndex<InputPortTag>>)const>(
               &System<double>::get_input_port_selection),
           py::arg("port_index"))
      .def("get_mutable_forced_discrete_update_events",
           static_cast<EventCollection<DiscreteUpdateEvent<double>> &(
               System<double>::*)()>(
               &System_double_publicist::
                   get_mutable_forced_discrete_update_events))
      .def("get_mutable_forced_publish_events",
           static_cast<EventCollection<PublishEvent<double>> &(
               System<double>::*)()>(
               &System_double_publicist::get_mutable_forced_publish_events))
      .def("get_mutable_forced_unrestricted_update_events",
           static_cast<EventCollection<UnrestrictedUpdateEvent<double>> &(
               System<double>::*)()>(
               &System_double_publicist::
                   get_mutable_forced_unrestricted_update_events))
      .def("get_output_port",
           static_cast<OutputPort<double> const &(System<double>::*)(int)const>(
               &System<double>::get_output_port),
           py::arg("port_index"))
      .def("get_output_port",
           static_cast<OutputPort<double> const &(System<double>::*)() const>(
               &System<double>::get_output_port))
      .def("get_output_port_selection",
           static_cast<OutputPort<double> const *(
               System<double>::*)(::std::variant<
                                  OutputPortSelection,
                                  drake::TypeSafeIndex<OutputPortTag>>)const>(
               &System<double>::get_output_port_selection),
           py::arg("port_index"))
      .def(
          "get_system_scalar_converter",
          static_cast<SystemScalarConverter const &(System<double>::*)() const>(
              &System<double>::get_system_scalar_converter))
      .def("get_time_derivatives_cache_entry",
           static_cast<CacheEntry const &(System<double>::*)() const>(
               &System<double>::get_time_derivatives_cache_entry))
      .def("num_constraints", static_cast<int (System<double>::*)() const>(
                                  &System<double>::num_constraints))
      .def("set_forced_discrete_update_events",
           [](System<double> &self,
              EventCollection<DiscreteUpdateEvent<double>> forced) {
             self.set_forced_discrete_update_events(
                 std::make_unique<EventCollection<DiscreteUpdateEvent<double>>>(
                     forced));
           })
      .def("set_forced_publish_events",
           [](System<double> &self,
              EventCollection<PublishEvent<double>> forced) {
             self.set_forced_publish_events(
                 std::make_unique<EventCollection<PublishEvent<double>>>(
                     forced));
           })
      .def("set_forced_unrestricted_update_events",
           [](System<double> &self,
              EventCollection<UnrestrictedUpdateEvent<double>> forced) {
             self.set_forced_unrestricted_update_events(
                 std::make_unique<
                     EventCollection<UnrestrictedUpdateEvent<double>>>(forced));
           })

      ;

  py::class_<System<drake::AutoDiffXd>, SystemBase>
      PySystem_Eigen_AutoDiffScalar_Eigen_VectorXd(
          m, "System_Eigen_AutoDiffScalar_Eigen_VectorXd");

  PySystem_Eigen_AutoDiffScalar_Eigen_VectorXd
      .def("Accept",
           static_cast<void (System<drake::AutoDiffXd>::*)(
               SystemVisitor<drake::AutoDiffXd> *) const>(
               &System<drake::AutoDiffXd>::Accept),
           py::arg("v"))
      .def("AddConstraint",
           [](System<drake::AutoDiffXd> &self,
              SystemConstraint<::drake::AutoDiffXd> constraint) {
             return self.AddConstraint(
                 std::make_unique<SystemConstraint<::drake::AutoDiffXd>>(
                     constraint));
           })
      .def("AddExternalConstraint",
           static_cast<SystemConstraintIndex (System<drake::AutoDiffXd>::*)(
               ExternalSystemConstraint)>(
               &System<drake::AutoDiffXd>::AddExternalConstraint),
           py::arg("constraint"))
      .def("AddTriggeredWitnessFunctionToCompositeEventCollection",
           static_cast<void (System<drake::AutoDiffXd>::*)(
               Event<drake::AutoDiffXd> *,
               CompositeEventCollection<drake::AutoDiffXd> *) const>(
               &System<drake::AutoDiffXd>::
                   AddTriggeredWitnessFunctionToCompositeEventCollection),
           py::arg("event"), py::arg("events"))
      .def("AllocateCompositeEventCollection",
           static_cast<
               ::std::unique_ptr<CompositeEventCollection<::drake::AutoDiffXd>,
                                 std::default_delete<CompositeEventCollection<
                                     ::drake::AutoDiffXd>>> (
                   System<drake::AutoDiffXd>::*)() const>(
               &System<drake::AutoDiffXd>::AllocateCompositeEventCollection))
      .def("AllocateContext",
           static_cast<::std::unique_ptr<
               Context<::drake::AutoDiffXd>,
               std::default_delete<Context<::drake::AutoDiffXd>>> (
               System<drake::AutoDiffXd>::*)() const>(
               &System<drake::AutoDiffXd>::AllocateContext))
      .def("AllocateDiscreteVariables",
           static_cast<::std::unique_ptr<
               DiscreteValues<::drake::AutoDiffXd>,
               std::default_delete<DiscreteValues<::drake::AutoDiffXd>>> (
               System<drake::AutoDiffXd>::*)() const>(
               &System<drake::AutoDiffXd>::AllocateDiscreteVariables))
      .def("AllocateFixedInputs",
           static_cast<void (System<drake::AutoDiffXd>::*)(
               Context<drake::AutoDiffXd> *) const>(
               &System<drake::AutoDiffXd>::AllocateFixedInputs),
           py::arg("context"))
      .def("AllocateForcedDiscreteUpdateEventCollection",
           static_cast<::std::unique_ptr<
               EventCollection<DiscreteUpdateEvent<::drake::AutoDiffXd>>,
               std::default_delete<
                   EventCollection<DiscreteUpdateEvent<::drake::AutoDiffXd>>>> (
               System<drake::AutoDiffXd>::*)() const>(
               &System<drake::AutoDiffXd>::
                   AllocateForcedDiscreteUpdateEventCollection))
      .def(
          "AllocateForcedPublishEventCollection",
          static_cast<::std::unique_ptr<
              EventCollection<PublishEvent<::drake::AutoDiffXd>>,
              std::default_delete<
                  EventCollection<PublishEvent<::drake::AutoDiffXd>>>> (
              System<drake::AutoDiffXd>::*)() const>(
              &System<drake::AutoDiffXd>::AllocateForcedPublishEventCollection))
      .def("AllocateForcedUnrestrictedUpdateEventCollection",
           static_cast<::std::unique_ptr<
               EventCollection<UnrestrictedUpdateEvent<::drake::AutoDiffXd>>,
               std::default_delete<EventCollection<
                   UnrestrictedUpdateEvent<::drake::AutoDiffXd>>>> (
               System<drake::AutoDiffXd>::*)() const>(
               &System<drake::AutoDiffXd>::
                   AllocateForcedUnrestrictedUpdateEventCollection))
      .def("AllocateImplicitTimeDerivativesResidual",
           static_cast<::Eigen::Matrix<drake::AutoDiffXd, -1, 1, 0, -1, 1> (
               System<drake::AutoDiffXd>::*)() const>(
               &System<
                   drake::AutoDiffXd>::AllocateImplicitTimeDerivativesResidual))
      .def(
          "AllocateInputAbstract",
          static_cast<::std::unique_ptr<
              drake::AbstractValue, std::default_delete<drake::AbstractValue>> (
              System<drake::AutoDiffXd>::*)(
              InputPort<drake::AutoDiffXd> const &) const>(
              &System<drake::AutoDiffXd>::AllocateInputAbstract),
          py::arg("input_port"))
      .def("AllocateInputVector",
           static_cast<::std::unique_ptr<
               BasicVector<::drake::AutoDiffXd>,
               std::default_delete<BasicVector<::drake::AutoDiffXd>>> (
               System<drake::AutoDiffXd>::*)(
               InputPort<drake::AutoDiffXd> const &) const>(
               &System<drake::AutoDiffXd>::AllocateInputVector),
           py::arg("input_port"))
      .def("AllocateOutput",
           static_cast<::std::unique_ptr<
               SystemOutput<::drake::AutoDiffXd>,
               std::default_delete<SystemOutput<::drake::AutoDiffXd>>> (
               System<drake::AutoDiffXd>::*)() const>(
               &System<drake::AutoDiffXd>::AllocateOutput))
      .def("AllocateTimeDerivatives",
           static_cast<::std::unique_ptr<
               ContinuousState<::drake::AutoDiffXd>,
               std::default_delete<ContinuousState<::drake::AutoDiffXd>>> (
               System<drake::AutoDiffXd>::*)() const>(
               &System<drake::AutoDiffXd>::AllocateTimeDerivatives))
      .def(
          "ApplyDiscreteVariableUpdate",
          static_cast<void (System<drake::AutoDiffXd>::*)(
              EventCollection<DiscreteUpdateEvent<::drake::AutoDiffXd>> const &,
              DiscreteValues<drake::AutoDiffXd> *, Context<drake::AutoDiffXd> *)
                          const>(
              &System<drake::AutoDiffXd>::ApplyDiscreteVariableUpdate),
          py::arg("events"), py::arg("discrete_state"), py::arg("context"))
      .def("ApplyUnrestrictedUpdate",
           static_cast<void (System<drake::AutoDiffXd>::*)(
               EventCollection<
                   UnrestrictedUpdateEvent<::drake::AutoDiffXd>> const &,
               State<drake::AutoDiffXd> *, Context<drake::AutoDiffXd> *) const>(
               &System<drake::AutoDiffXd>::ApplyUnrestrictedUpdate),
           py::arg("events"), py::arg("state"), py::arg("context"))
      .def("CalcConservativePower",
           static_cast<drake::AutoDiffXd (System<drake::AutoDiffXd>::*)(
               Context<drake::AutoDiffXd> const &) const>(
               &System<drake::AutoDiffXd>::CalcConservativePower),
           py::arg("context"))
      .def(
          "CalcDiscreteVariableUpdates",
          static_cast<void (System<drake::AutoDiffXd>::*)(
              Context<drake::AutoDiffXd> const &,
              EventCollection<DiscreteUpdateEvent<::drake::AutoDiffXd>> const &,
              DiscreteValues<drake::AutoDiffXd> *) const>(
              &System<drake::AutoDiffXd>::CalcDiscreteVariableUpdates),
          py::arg("context"), py::arg("events"), py::arg("discrete_state"))
      .def("CalcDiscreteVariableUpdates",
           static_cast<void (System<drake::AutoDiffXd>::*)(
               Context<drake::AutoDiffXd> const &,
               DiscreteValues<drake::AutoDiffXd> *) const>(
               &System<drake::AutoDiffXd>::CalcDiscreteVariableUpdates),
           py::arg("context"), py::arg("discrete_state"))
      .def("CalcImplicitTimeDerivativesResidual",
           static_cast<void (System<drake::AutoDiffXd>::*)(
               Context<drake::AutoDiffXd> const &,
               ContinuousState<drake::AutoDiffXd> const &,
               ::drake::EigenPtr<
                   Eigen::Matrix<drake::AutoDiffXd, -1, 1, 0, -1, 1>>) const>(
               &System<drake::AutoDiffXd>::CalcImplicitTimeDerivativesResidual),
           py::arg("context"), py::arg("proposed_derivatives"),
           py::arg("residual"))
      .def("CalcKineticEnergy",
           static_cast<drake::AutoDiffXd (System<drake::AutoDiffXd>::*)(
               Context<drake::AutoDiffXd> const &) const>(
               &System<drake::AutoDiffXd>::CalcKineticEnergy),
           py::arg("context"))
      .def("CalcNextUpdateTime",
           static_cast<drake::AutoDiffXd (System<drake::AutoDiffXd>::*)(
               Context<drake::AutoDiffXd> const &,
               CompositeEventCollection<drake::AutoDiffXd> *) const>(
               &System<drake::AutoDiffXd>::CalcNextUpdateTime),
           py::arg("context"), py::arg("events"))
      .def("CalcNonConservativePower",
           static_cast<drake::AutoDiffXd (System<drake::AutoDiffXd>::*)(
               Context<drake::AutoDiffXd> const &) const>(
               &System<drake::AutoDiffXd>::CalcNonConservativePower),
           py::arg("context"))
      .def("CalcOutput",
           static_cast<void (System<drake::AutoDiffXd>::*)(
               Context<drake::AutoDiffXd> const &,
               SystemOutput<drake::AutoDiffXd> *) const>(
               &System<drake::AutoDiffXd>::CalcOutput),
           py::arg("context"), py::arg("outputs"))
      .def("CalcPotentialEnergy",
           static_cast<drake::AutoDiffXd (System<drake::AutoDiffXd>::*)(
               Context<drake::AutoDiffXd> const &) const>(
               &System<drake::AutoDiffXd>::CalcPotentialEnergy),
           py::arg("context"))
      .def("CalcTimeDerivatives",
           static_cast<void (System<drake::AutoDiffXd>::*)(
               Context<drake::AutoDiffXd> const &,
               ContinuousState<drake::AutoDiffXd> *) const>(
               &System<drake::AutoDiffXd>::CalcTimeDerivatives),
           py::arg("context"), py::arg("derivatives"))
      .def("CalcUnrestrictedUpdate",
           static_cast<void (System<drake::AutoDiffXd>::*)(
               Context<drake::AutoDiffXd> const &,
               EventCollection<
                   UnrestrictedUpdateEvent<::drake::AutoDiffXd>> const &,
               State<drake::AutoDiffXd> *) const>(
               &System<drake::AutoDiffXd>::CalcUnrestrictedUpdate),
           py::arg("context"), py::arg("events"), py::arg("state"))
      .def("CalcUnrestrictedUpdate",
           static_cast<void (System<drake::AutoDiffXd>::*)(
               Context<drake::AutoDiffXd> const &, State<drake::AutoDiffXd> *)
                           const>(
               &System<drake::AutoDiffXd>::CalcUnrestrictedUpdate),
           py::arg("context"), py::arg("state"))
      .def("CalcWitnessValue",
           static_cast<drake::AutoDiffXd (System<drake::AutoDiffXd>::*)(
               Context<drake::AutoDiffXd> const &,
               WitnessFunction<drake::AutoDiffXd> const &) const>(
               &System<drake::AutoDiffXd>::CalcWitnessValue),
           py::arg("context"), py::arg("witness_func"))
      .def("CheckSystemConstraintsSatisfied",
           static_cast<::drake::scalar_predicate<drake::AutoDiffXd>::type (
               System<drake::AutoDiffXd>::*)(Context<drake::AutoDiffXd> const &,
                                             double) const>(
               &System<drake::AutoDiffXd>::CheckSystemConstraintsSatisfied),
           py::arg("context"), py::arg("tol"))
      .def("CheckValidOutput",
           static_cast<void (System<drake::AutoDiffXd>::*)(
               SystemOutput<drake::AutoDiffXd> const *) const>(
               &System<drake::AutoDiffXd>::CheckValidOutput),
           py::arg("output"))
      .def("CopyContinuousStateVector",
           static_cast<::Eigen::Matrix<drake::AutoDiffXd, -1, 1, 0, -1, 1> (
               System<drake::AutoDiffXd>::*)(Context<drake::AutoDiffXd> const &)
                           const>(
               &System<drake::AutoDiffXd>::CopyContinuousStateVector),
           py::arg("context"))
      .def("CreateDefaultContext",
           static_cast<::std::unique_ptr<
               Context<::drake::AutoDiffXd>,
               std::default_delete<Context<::drake::AutoDiffXd>>> (
               System<drake::AutoDiffXd>::*)() const>(
               &System<drake::AutoDiffXd>::CreateDefaultContext))
      .def("DeclareInputPort",
           static_cast<InputPort<drake::AutoDiffXd> &(
               System<drake::AutoDiffXd>::
                   *)(::std::variant<
                          std::basic_string<char, std::char_traits<char>,
                                            std::allocator<char>>,
                          UseDefaultName>,
                      PortDataType, int,
                      ::std::optional<drake::RandomDistribution>)>(
               &System_Eigen_AutoDiffScalar_Eigen_VectorXd_publicist::
                   DeclareInputPort),
           py::arg("name"), py::arg("type"), py::arg("size"),
           py::arg("random_type") =
               ::std::optional<drake::RandomDistribution>(std::nullopt))
      .def("DeclareInputPort",
           static_cast<InputPort<drake::AutoDiffXd> &(
               System<drake::AutoDiffXd>::*)(PortDataType, int,
                                             ::std::optional<
                                                 drake::RandomDistribution>)>(
               &System_Eigen_AutoDiffScalar_Eigen_VectorXd_publicist::
                   DeclareInputPort),
           py::arg("type"), py::arg("size"),
           py::arg("random_type") =
               ::std::optional<drake::RandomDistribution>(std::nullopt))
      .def(
          "DispatchDiscreteVariableUpdateHandler",
          static_cast<void (System<drake::AutoDiffXd>::*)(
              Context<drake::AutoDiffXd> const &,
              EventCollection<DiscreteUpdateEvent<::drake::AutoDiffXd>> const &,
              DiscreteValues<drake::AutoDiffXd> *) const>(
              &System_Eigen_AutoDiffScalar_Eigen_VectorXd_publicist::
                  DispatchDiscreteVariableUpdateHandler),
          py::arg("context"), py::arg("events"), py::arg("discrete_state"))
      .def("DispatchPublishHandler",
           static_cast<void (System<drake::AutoDiffXd>::*)(
               Context<drake::AutoDiffXd> const &,
               EventCollection<PublishEvent<::drake::AutoDiffXd>> const &)
                           const>(
               &System_Eigen_AutoDiffScalar_Eigen_VectorXd_publicist::
                   DispatchPublishHandler),
           py::arg("context"), py::arg("events"))
      .def("DispatchUnrestrictedUpdateHandler",
           static_cast<void (System<drake::AutoDiffXd>::*)(
               Context<drake::AutoDiffXd> const &,
               EventCollection<
                   UnrestrictedUpdateEvent<::drake::AutoDiffXd>> const &,
               State<drake::AutoDiffXd> *) const>(
               &System_Eigen_AutoDiffScalar_Eigen_VectorXd_publicist::
                   DispatchUnrestrictedUpdateHandler),
           py::arg("context"), py::arg("events"), py::arg("state"))
      .def(
          "DoApplyDiscreteVariableUpdate",
          static_cast<void (System<drake::AutoDiffXd>::*)(
              EventCollection<DiscreteUpdateEvent<::drake::AutoDiffXd>> const &,
              DiscreteValues<drake::AutoDiffXd> *, Context<drake::AutoDiffXd> *)
                          const>(
              &System_Eigen_AutoDiffScalar_Eigen_VectorXd_publicist::
                  DoApplyDiscreteVariableUpdate),
          py::arg("events"), py::arg("discrete_state"), py::arg("context"))
      .def("DoApplyUnrestrictedUpdate",
           static_cast<void (System<drake::AutoDiffXd>::*)(
               EventCollection<
                   UnrestrictedUpdateEvent<::drake::AutoDiffXd>> const &,
               State<drake::AutoDiffXd> *, Context<drake::AutoDiffXd> *) const>(
               &System_Eigen_AutoDiffScalar_Eigen_VectorXd_publicist::
                   DoApplyUnrestrictedUpdate),
           py::arg("events"), py::arg("state"), py::arg("context"))
      .def("DoCalcConservativePower",
           static_cast<drake::AutoDiffXd (System<drake::AutoDiffXd>::*)(
               Context<drake::AutoDiffXd> const &) const>(
               &System_Eigen_AutoDiffScalar_Eigen_VectorXd_publicist::
                   DoCalcConservativePower),
           py::arg("context"))
      .def("DoCalcImplicitTimeDerivativesResidual",
           static_cast<void (System<drake::AutoDiffXd>::*)(
               Context<drake::AutoDiffXd> const &,
               ContinuousState<drake::AutoDiffXd> const &,
               ::drake::EigenPtr<
                   Eigen::Matrix<drake::AutoDiffXd, -1, 1, 0, -1, 1>>) const>(
               &System_Eigen_AutoDiffScalar_Eigen_VectorXd_publicist::
                   DoCalcImplicitTimeDerivativesResidual),
           py::arg("context"), py::arg("proposed_derivatives"),
           py::arg("residual"))
      .def("DoCalcKineticEnergy",
           static_cast<drake::AutoDiffXd (System<drake::AutoDiffXd>::*)(
               Context<drake::AutoDiffXd> const &) const>(
               &System_Eigen_AutoDiffScalar_Eigen_VectorXd_publicist::
                   DoCalcKineticEnergy),
           py::arg("context"))
      .def("DoCalcNextUpdateTime",
           static_cast<void (System<drake::AutoDiffXd>::*)(
               Context<drake::AutoDiffXd> const &,
               CompositeEventCollection<drake::AutoDiffXd> *,
               drake::AutoDiffXd *) const>(
               &System_Eigen_AutoDiffScalar_Eigen_VectorXd_publicist::
                   DoCalcNextUpdateTime),
           py::arg("context"), py::arg("events"), py::arg("time"))
      .def("DoCalcNonConservativePower",
           static_cast<drake::AutoDiffXd (System<drake::AutoDiffXd>::*)(
               Context<drake::AutoDiffXd> const &) const>(
               &System_Eigen_AutoDiffScalar_Eigen_VectorXd_publicist::
                   DoCalcNonConservativePower),
           py::arg("context"))
      .def("DoCalcPotentialEnergy",
           static_cast<drake::AutoDiffXd (System<drake::AutoDiffXd>::*)(
               Context<drake::AutoDiffXd> const &) const>(
               &System_Eigen_AutoDiffScalar_Eigen_VectorXd_publicist::
                   DoCalcPotentialEnergy),
           py::arg("context"))
      .def("DoCalcTimeDerivatives",
           static_cast<void (System<drake::AutoDiffXd>::*)(
               Context<drake::AutoDiffXd> const &,
               ContinuousState<drake::AutoDiffXd> *) const>(
               &System_Eigen_AutoDiffScalar_Eigen_VectorXd_publicist::
                   DoCalcTimeDerivatives),
           py::arg("context"), py::arg("derivatives"))
      .def("DoCalcWitnessValue",
           static_cast<drake::AutoDiffXd (System<drake::AutoDiffXd>::*)(
               Context<drake::AutoDiffXd> const &,
               WitnessFunction<drake::AutoDiffXd> const &) const>(
               &System_Eigen_AutoDiffScalar_Eigen_VectorXd_publicist::
                   DoCalcWitnessValue),
           py::arg("context"), py::arg("witness_func"))
      .def("DoGetInitializationEvents",
           static_cast<void (System<drake::AutoDiffXd>::*)(
               Context<drake::AutoDiffXd> const &,
               CompositeEventCollection<drake::AutoDiffXd> *) const>(
               &System_Eigen_AutoDiffScalar_Eigen_VectorXd_publicist::
                   DoGetInitializationEvents),
           py::arg("context"), py::arg("events"))
      .def("DoGetMutableTargetSystemCompositeEventCollection",
           static_cast<CompositeEventCollection<drake::AutoDiffXd> *(
               System<drake::AutoDiffXd>::*)(System<drake::AutoDiffXd> const &,
                                             CompositeEventCollection<
                                                 drake::AutoDiffXd> *)const>(
               &System<drake::AutoDiffXd>::
                   DoGetMutableTargetSystemCompositeEventCollection),
           py::arg("target_system"), py::arg("events"))
      .def("DoGetMutableTargetSystemState",
           static_cast<State<drake::AutoDiffXd> *(
               System<drake::AutoDiffXd>::*)(System<drake::AutoDiffXd> const &,
                                             State<drake::AutoDiffXd> *)const>(
               &System<drake::AutoDiffXd>::DoGetMutableTargetSystemState),
           py::arg("target_system"), py::arg("state"))
      .def("DoGetPerStepEvents",
           static_cast<void (System<drake::AutoDiffXd>::*)(
               Context<drake::AutoDiffXd> const &,
               CompositeEventCollection<drake::AutoDiffXd> *) const>(
               &System_Eigen_AutoDiffScalar_Eigen_VectorXd_publicist::
                   DoGetPerStepEvents),
           py::arg("context"), py::arg("events"))
      .def("DoGetPeriodicEvents",
           static_cast<::std::map<
               PeriodicEventData,
               std::vector<const Event<::drake::AutoDiffXd> *,
                           std::allocator<const Event<::drake::AutoDiffXd> *>>,
               PeriodicEventDataComparator,
               std::allocator<std::pair<
                   const PeriodicEventData,
                   std::vector<
                       const Event<::drake::AutoDiffXd> *,
                       std::allocator<const Event<::drake::AutoDiffXd> *>>>>> (
               System<drake::AutoDiffXd>::*)() const>(
               &System_Eigen_AutoDiffScalar_Eigen_VectorXd_publicist::
                   DoGetPeriodicEvents))
      .def(
          "DoGetTargetSystemCompositeEventCollection",
          static_cast<CompositeEventCollection<drake::AutoDiffXd> const *(
              System<drake::AutoDiffXd>::
                  *)(System<drake::AutoDiffXd> const &,
                     CompositeEventCollection<drake::AutoDiffXd> const *)const>(
              &System<drake::AutoDiffXd>::
                  DoGetTargetSystemCompositeEventCollection),
          py::arg("target_system"), py::arg("events"))
      .def("DoGetTargetSystemContext",
           static_cast<Context<drake::AutoDiffXd> const *(
               System<drake::AutoDiffXd>::*)(System<drake::AutoDiffXd> const &,
                                             Context<drake::AutoDiffXd> const *)
                           const>(
               &System<drake::AutoDiffXd>::DoGetTargetSystemContext),
           py::arg("target_system"), py::arg("context"))
      .def("DoGetTargetSystemContinuousState",
           static_cast<ContinuousState<drake::AutoDiffXd> const *(
               System<drake::AutoDiffXd>::
                   *)(System<drake::AutoDiffXd> const &,
                      ContinuousState<drake::AutoDiffXd> const *)const>(
               &System<drake::AutoDiffXd>::DoGetTargetSystemContinuousState),
           py::arg("target_system"), py::arg("xc"))
      .def("DoGetTargetSystemState",
           static_cast<State<drake::AutoDiffXd> const *(
               System<drake::AutoDiffXd>::*)(System<drake::AutoDiffXd> const &,
                                             State<drake::AutoDiffXd> const *)
                           const>(
               &System<drake::AutoDiffXd>::DoGetTargetSystemState),
           py::arg("target_system"), py::arg("state"))
      .def("DoGetWitnessFunctions",
           static_cast<void (System<drake::AutoDiffXd>::*)(
               Context<drake::AutoDiffXd> const &,
               ::std::vector<const WitnessFunction<::drake::AutoDiffXd> *,
                             std::allocator<const WitnessFunction<
                                 ::drake::AutoDiffXd> *>> *) const>(
               &System_Eigen_AutoDiffScalar_Eigen_VectorXd_publicist::
                   DoGetWitnessFunctions),
           py::arg("arg0"), py::arg("arg1"))
      .def("DoMapQDotToVelocity",
           static_cast<void (System<drake::AutoDiffXd>::*)(
               Context<drake::AutoDiffXd> const &,
               ::Eigen::Ref<
                   const Eigen::Matrix<drake::AutoDiffXd, -1, 1, 0, -1, 1>, 0,
                   Eigen::InnerStride<1>> const &,
               VectorBase<drake::AutoDiffXd> *) const>(
               &System_Eigen_AutoDiffScalar_Eigen_VectorXd_publicist::
                   DoMapQDotToVelocity),
           py::arg("context"), py::arg("qdot"), py::arg("generalized_velocity"))
      .def("DoMapVelocityToQDot",
           static_cast<void (System<drake::AutoDiffXd>::*)(
               Context<drake::AutoDiffXd> const &,
               ::Eigen::Ref<
                   const Eigen::Matrix<drake::AutoDiffXd, -1, 1, 0, -1, 1>, 0,
                   Eigen::InnerStride<1>> const &,
               VectorBase<drake::AutoDiffXd> *) const>(
               &System_Eigen_AutoDiffScalar_Eigen_VectorXd_publicist::
                   DoMapVelocityToQDot),
           py::arg("context"), py::arg("generalized_velocity"), py::arg("qdot"))
      .def("EvalConservativePower",
           static_cast<drake::AutoDiffXd const &(
               System<drake::AutoDiffXd>::*)(Context<drake::AutoDiffXd> const &)
                           const>(
               &System<drake::AutoDiffXd>::EvalConservativePower),
           py::arg("context"))
      .def("EvalEigenVectorInput",
           static_cast<::Eigen::VectorBlock<
               const Eigen::Matrix<drake::AutoDiffXd, -1, 1, 0, -1, 1>, -1> (
               System<drake::AutoDiffXd>::*)(Context<drake::AutoDiffXd> const &,
                                             int) const>(
               &System<drake::AutoDiffXd>::EvalEigenVectorInput),
           py::arg("context"), py::arg("port_index"))
      .def(
          "EvalKineticEnergy",
          static_cast<drake::AutoDiffXd const &(
              System<drake::AutoDiffXd>::*)(Context<drake::AutoDiffXd> const &)
                          const>(&System<drake::AutoDiffXd>::EvalKineticEnergy),
          py::arg("context"))
      .def("EvalNonConservativePower",
           static_cast<drake::AutoDiffXd const &(
               System<drake::AutoDiffXd>::*)(Context<drake::AutoDiffXd> const &)
                           const>(
               &System<drake::AutoDiffXd>::EvalNonConservativePower),
           py::arg("context"))
      .def("EvalPotentialEnergy",
           static_cast<drake::AutoDiffXd const &(
               System<drake::AutoDiffXd>::*)(Context<drake::AutoDiffXd> const &)
                           const>(
               &System<drake::AutoDiffXd>::EvalPotentialEnergy),
           py::arg("context"))
      .def("EvalTimeDerivatives",
           static_cast<ContinuousState<drake::AutoDiffXd> const &(
               System<drake::AutoDiffXd>::*)(Context<drake::AutoDiffXd> const &)
                           const>(
               &System<drake::AutoDiffXd>::EvalTimeDerivatives),
           py::arg("context"))
      .def("FixInputPortsFrom",
           static_cast<void (System<drake::AutoDiffXd>::*)(
               System<double> const &, Context<double> const &,
               Context<drake::AutoDiffXd> *) const>(
               &System<drake::AutoDiffXd>::FixInputPortsFrom),
           py::arg("other_system"), py::arg("other_context"),
           py::arg("target_context"))
      .def("GetGraphvizFragment",
           static_cast<void (System<drake::AutoDiffXd>::*)(
               int, ::std::stringstream *) const>(
               &System<drake::AutoDiffXd>::GetGraphvizFragment),
           py::arg("max_depth"), py::arg("dot"))
      .def("GetGraphvizId",
           static_cast<::int64_t (System<drake::AutoDiffXd>::*)() const>(
               &System<drake::AutoDiffXd>::GetGraphvizId))
      .def("GetGraphvizInputPortToken",
           static_cast<void (System<drake::AutoDiffXd>::*)(
               InputPort<drake::AutoDiffXd> const &, int, ::std::stringstream *)
                           const>(
               &System<drake::AutoDiffXd>::GetGraphvizInputPortToken),
           py::arg("port"), py::arg("max_depth"), py::arg("dot"))
      .def("GetGraphvizOutputPortToken",
           static_cast<void (System<drake::AutoDiffXd>::*)(
               OutputPort<drake::AutoDiffXd> const &, int,
               ::std::stringstream *) const>(
               &System<drake::AutoDiffXd>::GetGraphvizOutputPortToken),
           py::arg("port"), py::arg("max_depth"), py::arg("dot"))
      .def("GetGraphvizString",
           static_cast<::std::string (System<drake::AutoDiffXd>::*)(int) const>(
               &System<drake::AutoDiffXd>::GetGraphvizString),
           py::arg("max_depth") = int(std::numeric_limits<int>::max()))
      .def("GetInitializationEvents",
           static_cast<void (System<drake::AutoDiffXd>::*)(
               Context<drake::AutoDiffXd> const &,
               CompositeEventCollection<drake::AutoDiffXd> *) const>(
               &System<drake::AutoDiffXd>::GetInitializationEvents),
           py::arg("context"), py::arg("events"))
      .def("GetInputPort",
           static_cast<InputPort<drake::AutoDiffXd> const &(
               System<drake::AutoDiffXd>::*)(::std::string const &)const>(
               &System<drake::AutoDiffXd>::GetInputPort),
           py::arg("port_name"))
      .def("GetMemoryObjectName",
           static_cast<::std::string (System<drake::AutoDiffXd>::*)() const>(
               &System<drake::AutoDiffXd>::GetMemoryObjectName))
      .def("GetMutableOutputVector",
           static_cast<::Eigen::VectorBlock<
               Eigen::Matrix<drake::AutoDiffXd, -1, 1, 0, -1, 1>, -1> (
               System<drake::AutoDiffXd>::*)(SystemOutput<drake::AutoDiffXd> *,
                                             int) const>(
               &System_Eigen_AutoDiffScalar_Eigen_VectorXd_publicist::
                   GetMutableOutputVector),
           py::arg("output"), py::arg("port_index"))
      .def(
          "GetMutableSubsystemContext",
          static_cast<Context<drake::AutoDiffXd> &(
              System<drake::AutoDiffXd>::*)(System<drake::AutoDiffXd> const &,
                                            Context<drake::AutoDiffXd> *)const>(
              &System<drake::AutoDiffXd>::GetMutableSubsystemContext),
          py::arg("subsystem"), py::arg("context"))
      .def("GetMyContextFromRoot",
           static_cast<Context<drake::AutoDiffXd> const &(
               System<drake::AutoDiffXd>::*)(Context<drake::AutoDiffXd> const &)
                           const>(
               &System<drake::AutoDiffXd>::GetMyContextFromRoot),
           py::arg("root_context"))
      .def(
          "GetMyMutableContextFromRoot",
          static_cast<Context<drake::AutoDiffXd> &(
              System<drake::AutoDiffXd>::*)(Context<drake::AutoDiffXd> *)const>(
              &System<drake::AutoDiffXd>::GetMyMutableContextFromRoot),
          py::arg("root_context"))
      .def("GetOutputPort",
           static_cast<OutputPort<drake::AutoDiffXd> const &(
               System<drake::AutoDiffXd>::*)(::std::string const &)const>(
               &System<drake::AutoDiffXd>::GetOutputPort),
           py::arg("port_name"))
      .def("GetPerStepEvents",
           static_cast<void (System<drake::AutoDiffXd>::*)(
               Context<drake::AutoDiffXd> const &,
               CompositeEventCollection<drake::AutoDiffXd> *) const>(
               &System<drake::AutoDiffXd>::GetPerStepEvents),
           py::arg("context"), py::arg("events"))
      .def("GetPeriodicEvents",
           static_cast<::std::map<
               PeriodicEventData,
               std::vector<const Event<::drake::AutoDiffXd> *,
                           std::allocator<const Event<::drake::AutoDiffXd> *>>,
               PeriodicEventDataComparator,
               std::allocator<std::pair<
                   const PeriodicEventData,
                   std::vector<
                       const Event<::drake::AutoDiffXd> *,
                       std::allocator<const Event<::drake::AutoDiffXd> *>>>>> (
               System<drake::AutoDiffXd>::*)() const>(
               &System<drake::AutoDiffXd>::GetPeriodicEvents))
      .def("GetSubsystemContext",
           static_cast<Context<drake::AutoDiffXd> const &(
               System<drake::AutoDiffXd>::*)(System<drake::AutoDiffXd> const &,
                                             Context<drake::AutoDiffXd> const &)
                           const>(
               &System<drake::AutoDiffXd>::GetSubsystemContext),
           py::arg("subsystem"), py::arg("context"))
      .def(
          "GetUniquePeriodicDiscreteUpdateAttribute",
          static_cast<::std::optional<PeriodicEventData> (
              System<drake::AutoDiffXd>::*)() const>(
              &System<
                  drake::AutoDiffXd>::GetUniquePeriodicDiscreteUpdateAttribute))
      .def("GetWitnessFunctions",
           static_cast<void (System<drake::AutoDiffXd>::*)(
               Context<drake::AutoDiffXd> const &,
               ::std::vector<
                   const WitnessFunction<::drake::AutoDiffXd> *,
                   std::allocator<const WitnessFunction<::drake::AutoDiffXd> *>>
                   *) const>(&System<drake::AutoDiffXd>::GetWitnessFunctions),
           py::arg("context"), py::arg("w"))
      .def("HasAnyDirectFeedthrough",
           static_cast<bool (System<drake::AutoDiffXd>::*)() const>(
               &System<drake::AutoDiffXd>::HasAnyDirectFeedthrough))
      .def("HasDirectFeedthrough",
           static_cast<bool (System<drake::AutoDiffXd>::*)(int) const>(
               &System<drake::AutoDiffXd>::HasDirectFeedthrough),
           py::arg("output_port"))
      .def("HasDirectFeedthrough",
           static_cast<bool (System<drake::AutoDiffXd>::*)(int, int) const>(
               &System<drake::AutoDiffXd>::HasDirectFeedthrough),
           py::arg("input_port"), py::arg("output_port"))
      .def(
          "HasInputPort",
          static_cast<bool (System<drake::AutoDiffXd>::*)(::std::string const &)
                          const>(&System<drake::AutoDiffXd>::HasInputPort),
          py::arg("port_name"))
      .def(
          "HasOutputPort",
          static_cast<bool (System<drake::AutoDiffXd>::*)(::std::string const &)
                          const>(&System<drake::AutoDiffXd>::HasOutputPort),
          py::arg("port_name"))
      .def("IsDifferenceEquationSystem",
           static_cast<bool (System<drake::AutoDiffXd>::*)(double *) const>(
               &System<drake::AutoDiffXd>::IsDifferenceEquationSystem),
           py::arg("time_period") = (double *)nullptr)
      .def("MapQDotToVelocity",
           static_cast<void (System<drake::AutoDiffXd>::*)(
               Context<drake::AutoDiffXd> const &,
               VectorBase<drake::AutoDiffXd> const &,
               VectorBase<drake::AutoDiffXd> *) const>(
               &System<drake::AutoDiffXd>::MapQDotToVelocity),
           py::arg("context"), py::arg("qdot"), py::arg("generalized_velocity"))
      .def("MapQDotToVelocity",
           static_cast<void (System<drake::AutoDiffXd>::*)(
               Context<drake::AutoDiffXd> const &,
               ::Eigen::Ref<
                   const Eigen::Matrix<drake::AutoDiffXd, -1, 1, 0, -1, 1>, 0,
                   Eigen::InnerStride<1>> const &,
               VectorBase<drake::AutoDiffXd> *) const>(
               &System<drake::AutoDiffXd>::MapQDotToVelocity),
           py::arg("context"), py::arg("qdot"), py::arg("generalized_velocity"))
      .def("MapVelocityToQDot",
           static_cast<void (System<drake::AutoDiffXd>::*)(
               Context<drake::AutoDiffXd> const &,
               VectorBase<drake::AutoDiffXd> const &,
               VectorBase<drake::AutoDiffXd> *) const>(
               &System<drake::AutoDiffXd>::MapVelocityToQDot),
           py::arg("context"), py::arg("generalized_velocity"), py::arg("qdot"))
      .def("MapVelocityToQDot",
           static_cast<void (System<drake::AutoDiffXd>::*)(
               Context<drake::AutoDiffXd> const &,
               ::Eigen::Ref<
                   const Eigen::Matrix<drake::AutoDiffXd, -1, 1, 0, -1, 1>, 0,
                   Eigen::InnerStride<1>> const &,
               VectorBase<drake::AutoDiffXd> *) const>(
               &System<drake::AutoDiffXd>::MapVelocityToQDot),
           py::arg("context"), py::arg("generalized_velocity"), py::arg("qdot"))
      .def("Publish",
           static_cast<void (System<drake::AutoDiffXd>::*)(
               Context<drake::AutoDiffXd> const &,
               EventCollection<PublishEvent<::drake::AutoDiffXd>> const &)
                           const>(&System<drake::AutoDiffXd>::Publish),
           py::arg("context"), py::arg("events"))
      .def("Publish",
           static_cast<void (System<drake::AutoDiffXd>::*)(
               Context<drake::AutoDiffXd> const &) const>(
               &System<drake::AutoDiffXd>::Publish),
           py::arg("context"))
      .def("SetDefaultContext",
           static_cast<void (System<drake::AutoDiffXd>::*)(
               Context<drake::AutoDiffXd> *) const>(
               &System<drake::AutoDiffXd>::SetDefaultContext),
           py::arg("context"))
      .def("SetDefaultParameters",
           static_cast<void (System<drake::AutoDiffXd>::*)(
               Context<drake::AutoDiffXd> const &,
               Parameters<drake::AutoDiffXd> *) const>(
               &System<drake::AutoDiffXd>::SetDefaultParameters),
           py::arg("context"), py::arg("parameters"))
      .def("SetDefaultState",
           static_cast<void (System<drake::AutoDiffXd>::*)(
               Context<drake::AutoDiffXd> const &, State<drake::AutoDiffXd> *)
                           const>(&System<drake::AutoDiffXd>::SetDefaultState),
           py::arg("context"), py::arg("state"))
      .def("SetRandomContext",
           static_cast<void (System<drake::AutoDiffXd>::*)(
               Context<drake::AutoDiffXd> *, ::drake::RandomGenerator *) const>(
               &System<drake::AutoDiffXd>::SetRandomContext),
           py::arg("context"), py::arg("generator"))
      .def("SetRandomParameters",
           static_cast<void (System<drake::AutoDiffXd>::*)(
               Context<drake::AutoDiffXd> const &,
               Parameters<drake::AutoDiffXd> *, ::drake::RandomGenerator *)
                           const>(
               &System<drake::AutoDiffXd>::SetRandomParameters),
           py::arg("context"), py::arg("parameters"), py::arg("generator"))
      .def("SetRandomState",
           static_cast<void (System<drake::AutoDiffXd>::*)(
               Context<drake::AutoDiffXd> const &, State<drake::AutoDiffXd> *,
               ::drake::RandomGenerator *) const>(
               &System<drake::AutoDiffXd>::SetRandomState),
           py::arg("context"), py::arg("state"), py::arg("generator"))
      .def("ToAutoDiffXd",
           static_cast<::std::unique_ptr<
               System<::drake::AutoDiffXd>,
               std::default_delete<System<::drake::AutoDiffXd>>> (
               System<drake::AutoDiffXd>::*)() const>(
               &System<drake::AutoDiffXd>::ToAutoDiffXd))
      .def("ToAutoDiffXdMaybe",
           static_cast<::std::unique_ptr<
               System<::drake::AutoDiffXd>,
               std::default_delete<System<::drake::AutoDiffXd>>> (
               System<drake::AutoDiffXd>::*)() const>(
               &System<drake::AutoDiffXd>::ToAutoDiffXdMaybe))
      .def("ToSymbolic",
           static_cast<::std::unique_ptr<
               System<::drake::symbolic::Expression>,
               std::default_delete<System<::drake::symbolic::Expression>>> (
               System<drake::AutoDiffXd>::*)() const>(
               &System<drake::AutoDiffXd>::ToSymbolic))
      .def("ToSymbolicMaybe",
           static_cast<::std::unique_ptr<
               System<::drake::symbolic::Expression>,
               std::default_delete<System<::drake::symbolic::Expression>>> (
               System<drake::AutoDiffXd>::*)() const>(
               &System<drake::AutoDiffXd>::ToSymbolicMaybe))
      .def("forced_discrete_update_events_exist",
           static_cast<bool (System<drake::AutoDiffXd>::*)() const>(
               &System_Eigen_AutoDiffScalar_Eigen_VectorXd_publicist::
                   forced_discrete_update_events_exist))
      .def("forced_publish_events_exist",
           static_cast<bool (System<drake::AutoDiffXd>::*)() const>(
               &System_Eigen_AutoDiffScalar_Eigen_VectorXd_publicist::
                   forced_publish_events_exist))
      .def("forced_unrestricted_update_events_exist",
           static_cast<bool (System<drake::AutoDiffXd>::*)() const>(
               &System_Eigen_AutoDiffScalar_Eigen_VectorXd_publicist::
                   forced_unrestricted_update_events_exist))
      .def("get_constraint",
           static_cast<SystemConstraint<drake::AutoDiffXd> const &(
               System<drake::AutoDiffXd>::*)(SystemConstraintIndex) const>(
               &System<drake::AutoDiffXd>::get_constraint),
           py::arg("constraint_index"))
      .def(
          "get_forced_discrete_update_events",
          static_cast<
              EventCollection<DiscreteUpdateEvent<::drake::AutoDiffXd>> const &(
                  System<drake::AutoDiffXd>::*)() const>(
              &System_Eigen_AutoDiffScalar_Eigen_VectorXd_publicist::
                  get_forced_discrete_update_events))
      .def("get_forced_publish_events",
           static_cast<EventCollection<PublishEvent<::drake::AutoDiffXd>> const
                           &(System<drake::AutoDiffXd>::*)() const>(
               &System_Eigen_AutoDiffScalar_Eigen_VectorXd_publicist::
                   get_forced_publish_events))
      .def("get_forced_unrestricted_update_events",
           static_cast<EventCollection<
               UnrestrictedUpdateEvent<::drake::AutoDiffXd>> const
                           &(System<drake::AutoDiffXd>::*)() const>(
               &System_Eigen_AutoDiffScalar_Eigen_VectorXd_publicist::
                   get_forced_unrestricted_update_events))
      .def("get_input_port",
           static_cast<InputPort<drake::AutoDiffXd> const &(
               System<drake::AutoDiffXd>::*)(int)const>(
               &System<drake::AutoDiffXd>::get_input_port),
           py::arg("port_index"))
      .def("get_input_port", static_cast<InputPort<drake::AutoDiffXd> const &(
                                 System<drake::AutoDiffXd>::*)() const>(
                                 &System<drake::AutoDiffXd>::get_input_port))
      .def("get_input_port_selection",
           static_cast<InputPort<drake::AutoDiffXd> const *(
               System<drake::AutoDiffXd>::
                   *)(::std::variant<InputPortSelection,
                                     drake::TypeSafeIndex<InputPortTag>>)const>(
               &System<drake::AutoDiffXd>::get_input_port_selection),
           py::arg("port_index"))
      .def("get_mutable_forced_discrete_update_events",
           static_cast<EventCollection<DiscreteUpdateEvent<::drake::AutoDiffXd>>
                           &(System<drake::AutoDiffXd>::*)()>(
               &System_Eigen_AutoDiffScalar_Eigen_VectorXd_publicist::
                   get_mutable_forced_discrete_update_events))
      .def("get_mutable_forced_publish_events",
           static_cast<EventCollection<PublishEvent<::drake::AutoDiffXd>> &(
               System<drake::AutoDiffXd>::*)()>(
               &System_Eigen_AutoDiffScalar_Eigen_VectorXd_publicist::
                   get_mutable_forced_publish_events))
      .def("get_mutable_forced_unrestricted_update_events",
           static_cast<
               EventCollection<UnrestrictedUpdateEvent<::drake::AutoDiffXd>> &(
                   System<drake::AutoDiffXd>::*)()>(
               &System_Eigen_AutoDiffScalar_Eigen_VectorXd_publicist::
                   get_mutable_forced_unrestricted_update_events))
      .def("get_output_port",
           static_cast<OutputPort<drake::AutoDiffXd> const &(
               System<drake::AutoDiffXd>::*)(int)const>(
               &System<drake::AutoDiffXd>::get_output_port),
           py::arg("port_index"))
      .def("get_output_port", static_cast<OutputPort<drake::AutoDiffXd> const &(
                                  System<drake::AutoDiffXd>::*)() const>(
                                  &System<drake::AutoDiffXd>::get_output_port))
      .def(
          "get_output_port_selection",
          static_cast<OutputPort<drake::AutoDiffXd> const *(
              System<drake::AutoDiffXd>::
                  *)(::std::variant<OutputPortSelection,
                                    drake::TypeSafeIndex<OutputPortTag>>)const>(
              &System<drake::AutoDiffXd>::get_output_port_selection),
          py::arg("port_index"))
      .def("get_system_scalar_converter",
           static_cast<SystemScalarConverter const &(
               System<drake::AutoDiffXd>::*)() const>(
               &System<drake::AutoDiffXd>::get_system_scalar_converter))
      .def(
          "get_time_derivatives_cache_entry",
          static_cast<CacheEntry const &(System<drake::AutoDiffXd>::*)() const>(
              &System<drake::AutoDiffXd>::get_time_derivatives_cache_entry))
      .def("num_constraints",
           static_cast<int (System<drake::AutoDiffXd>::*)() const>(
               &System<drake::AutoDiffXd>::num_constraints))
      .def(
          "set_forced_discrete_update_events",
          [](System<drake::AutoDiffXd> &self,
             EventCollection<DiscreteUpdateEvent<::drake::AutoDiffXd>> forced) {
            self.set_forced_discrete_update_events(
                std::make_unique<
                    EventCollection<DiscreteUpdateEvent<::drake::AutoDiffXd>>>(
                    forced));
          })
      .def("set_forced_publish_events",
           [](System<drake::AutoDiffXd> &self,
              EventCollection<PublishEvent<::drake::AutoDiffXd>> forced) {
             self.set_forced_publish_events(
                 std::make_unique<
                     EventCollection<PublishEvent<::drake::AutoDiffXd>>>(
                     forced));
           })
      .def("set_forced_unrestricted_update_events",
           [](System<drake::AutoDiffXd> &self,
              EventCollection<UnrestrictedUpdateEvent<::drake::AutoDiffXd>>
                  forced) {
             self.set_forced_unrestricted_update_events(
                 std::make_unique<EventCollection<
                     UnrestrictedUpdateEvent<::drake::AutoDiffXd>>>(forced));
           })

      ;
}
