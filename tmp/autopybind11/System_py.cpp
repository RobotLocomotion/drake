#include "drake/systems/framework/system.h"
#include <pybind11/eigen.h>
#include <pybind11/iostream.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

class System_double_publicist : public ::drake::systems::System<double> {
public:
  using ::drake::systems::System<double>::AddConstraint;
  using ::drake::systems::System<double>::DeclareInputPort;
  using ::drake::systems::System<double>::DeclareInputPort;
  using ::drake::systems::System<double>::DispatchDiscreteVariableUpdateHandler;
  using ::drake::systems::System<double>::DispatchPublishHandler;
  using ::drake::systems::System<double>::DispatchUnrestrictedUpdateHandler;
  using ::drake::systems::System<double>::DoApplyDiscreteVariableUpdate;
  using ::drake::systems::System<double>::DoApplyUnrestrictedUpdate;
  using ::drake::systems::System<double>::DoCalcConservativePower;
  using ::drake::systems::System<double>::DoCalcImplicitTimeDerivativesResidual;
  using ::drake::systems::System<double>::DoCalcKineticEnergy;
  using ::drake::systems::System<double>::DoCalcNextUpdateTime;
  using ::drake::systems::System<double>::DoCalcNonConservativePower;
  using ::drake::systems::System<double>::DoCalcPotentialEnergy;
  using ::drake::systems::System<double>::DoCalcTimeDerivatives;
  using ::drake::systems::System<double>::DoCalcWitnessValue;
  using ::drake::systems::System<double>::DoGetInitializationEvents;
  using ::drake::systems::System<double>::DoGetPerStepEvents;
  using ::drake::systems::System<double>::DoGetPeriodicEvents;
  using ::drake::systems::System<double>::DoGetWitnessFunctions;
  using ::drake::systems::System<double>::DoMapQDotToVelocity;
  using ::drake::systems::System<double>::DoMapVelocityToQDot;
  using ::drake::systems::System<double>::GetMutableOutputVector;
  using ::drake::systems::System<double>::forced_discrete_update_events_exist;
  using ::drake::systems::System<double>::forced_publish_events_exist;
  using ::drake::systems::System<
      double>::forced_unrestricted_update_events_exist;
  using ::drake::systems::System<double>::get_forced_discrete_update_events;
  using ::drake::systems::System<double>::get_forced_publish_events;
  using ::drake::systems::System<double>::get_forced_unrestricted_update_events;
  using ::drake::systems::System<
      double>::get_mutable_forced_discrete_update_events;
  using ::drake::systems::System<double>::get_mutable_forced_publish_events;
  using ::drake::systems::System<
      double>::get_mutable_forced_unrestricted_update_events;
  using ::drake::systems::System<double>::set_forced_discrete_update_events;
  using ::drake::systems::System<double>::set_forced_publish_events;
  using ::drake::systems::System<double>::set_forced_unrestricted_update_events;
};

class System_Eigen_AutoDiffScalar_Eigen_VectorXd_publicist
    : public ::drake::systems::System<Eigen::AutoDiffScalar<Eigen::VectorXd>> {
public:
  using ::drake::systems::System<
      Eigen::AutoDiffScalar<Eigen::VectorXd>>::AddConstraint;
  using ::drake::systems::System<
      Eigen::AutoDiffScalar<Eigen::VectorXd>>::DeclareInputPort;
  using ::drake::systems::System<
      Eigen::AutoDiffScalar<Eigen::VectorXd>>::DeclareInputPort;
  using ::drake::systems::System<Eigen::AutoDiffScalar<Eigen::VectorXd>>::
      DispatchDiscreteVariableUpdateHandler;
  using ::drake::systems::System<
      Eigen::AutoDiffScalar<Eigen::VectorXd>>::DispatchPublishHandler;
  using ::drake::systems::System<Eigen::AutoDiffScalar<Eigen::VectorXd>>::
      DispatchUnrestrictedUpdateHandler;
  using ::drake::systems::System<
      Eigen::AutoDiffScalar<Eigen::VectorXd>>::DoApplyDiscreteVariableUpdate;
  using ::drake::systems::System<
      Eigen::AutoDiffScalar<Eigen::VectorXd>>::DoApplyUnrestrictedUpdate;
  using ::drake::systems::System<
      Eigen::AutoDiffScalar<Eigen::VectorXd>>::DoCalcConservativePower;
  using ::drake::systems::System<Eigen::AutoDiffScalar<Eigen::VectorXd>>::
      DoCalcImplicitTimeDerivativesResidual;
  using ::drake::systems::System<
      Eigen::AutoDiffScalar<Eigen::VectorXd>>::DoCalcKineticEnergy;
  using ::drake::systems::System<
      Eigen::AutoDiffScalar<Eigen::VectorXd>>::DoCalcNextUpdateTime;
  using ::drake::systems::System<
      Eigen::AutoDiffScalar<Eigen::VectorXd>>::DoCalcNonConservativePower;
  using ::drake::systems::System<
      Eigen::AutoDiffScalar<Eigen::VectorXd>>::DoCalcPotentialEnergy;
  using ::drake::systems::System<
      Eigen::AutoDiffScalar<Eigen::VectorXd>>::DoCalcTimeDerivatives;
  using ::drake::systems::System<
      Eigen::AutoDiffScalar<Eigen::VectorXd>>::DoCalcWitnessValue;
  using ::drake::systems::System<
      Eigen::AutoDiffScalar<Eigen::VectorXd>>::DoGetInitializationEvents;
  using ::drake::systems::System<
      Eigen::AutoDiffScalar<Eigen::VectorXd>>::DoGetPerStepEvents;
  using ::drake::systems::System<
      Eigen::AutoDiffScalar<Eigen::VectorXd>>::DoGetPeriodicEvents;
  using ::drake::systems::System<
      Eigen::AutoDiffScalar<Eigen::VectorXd>>::DoGetWitnessFunctions;
  using ::drake::systems::System<
      Eigen::AutoDiffScalar<Eigen::VectorXd>>::DoMapQDotToVelocity;
  using ::drake::systems::System<
      Eigen::AutoDiffScalar<Eigen::VectorXd>>::DoMapVelocityToQDot;
  using ::drake::systems::System<
      Eigen::AutoDiffScalar<Eigen::VectorXd>>::GetMutableOutputVector;
  using ::drake::systems::System<Eigen::AutoDiffScalar<Eigen::VectorXd>>::
      forced_discrete_update_events_exist;
  using ::drake::systems::System<
      Eigen::AutoDiffScalar<Eigen::VectorXd>>::forced_publish_events_exist;
  using ::drake::systems::System<Eigen::AutoDiffScalar<Eigen::VectorXd>>::
      forced_unrestricted_update_events_exist;
  using ::drake::systems::System<Eigen::AutoDiffScalar<Eigen::VectorXd>>::
      get_forced_discrete_update_events;
  using ::drake::systems::System<
      Eigen::AutoDiffScalar<Eigen::VectorXd>>::get_forced_publish_events;
  using ::drake::systems::System<Eigen::AutoDiffScalar<Eigen::VectorXd>>::
      get_forced_unrestricted_update_events;
  using ::drake::systems::System<Eigen::AutoDiffScalar<Eigen::VectorXd>>::
      get_mutable_forced_discrete_update_events;
  using ::drake::systems::System<Eigen::AutoDiffScalar<Eigen::VectorXd>>::
      get_mutable_forced_publish_events;
  using ::drake::systems::System<Eigen::AutoDiffScalar<Eigen::VectorXd>>::
      get_mutable_forced_unrestricted_update_events;
  using ::drake::systems::System<Eigen::AutoDiffScalar<Eigen::VectorXd>>::
      set_forced_discrete_update_events;
  using ::drake::systems::System<
      Eigen::AutoDiffScalar<Eigen::VectorXd>>::set_forced_publish_events;
  using ::drake::systems::System<Eigen::AutoDiffScalar<Eigen::VectorXd>>::
      set_forced_unrestricted_update_events;
};

namespace py = pybind11;
void apb11_pydrake_System_py_register(py::module &m) {
  static bool called = false;
  if (called) {
    return;
  }
  called = true;
  using namespace drake::systems;

  using PySystem_double_0 = double;

  py::class_<System<PySystem_double_0>, SystemBase> PySystem_double(
      m, "System_double");

  PySystem_double
      .def("AddConstraint",
           [](System<PySystem_double_0> &self,
              SystemConstraint<PySystem_double_0> constraint) {
             return self.AddConstraint(
                 std::make_unique<SystemConstraint<PySystem_double_0>>(
                     constraint));
           })
      .def("AddExternalConstraint",
           static_cast<SystemConstraintIndex (System<PySystem_double_0>::*)(
               ExternalSystemConstraint)>(
               &System<PySystem_double_0>::AddExternalConstraint),
           py::arg("constraint"))
      .def("AddTriggeredWitnessFunctionToCompositeEventCollection",
           static_cast<void (System<PySystem_double_0>::*)(
               Event<PySystem_double_0> *,
               CompositeEventCollection<PySystem_double_0> *) const>(
               &System<PySystem_double_0>::
                   AddTriggeredWitnessFunctionToCompositeEventCollection),
           py::arg("event"), py::arg("events"))
      .def("AllocateCompositeEventCollection",
           static_cast<::std::unique_ptr<
               CompositeEventCollection<PySystem_double_0>,
               std::default_delete<CompositeEventCollection<
                   PySystem_double_0>>> (System<PySystem_double_0>::*)() const>(
               &System<PySystem_double_0>::AllocateCompositeEventCollection))
      .def("AllocateContext",
           static_cast<::std::unique_ptr<
               Context<PySystem_double_0>,
               std::default_delete<Context<PySystem_double_0>>> (
               System<PySystem_double_0>::*)() const>(
               &System<PySystem_double_0>::AllocateContext))
      .def("AllocateDiscreteVariables",
           static_cast<::std::unique_ptr<
               DiscreteValues<PySystem_double_0>,
               std::default_delete<DiscreteValues<PySystem_double_0>>> (
               System<PySystem_double_0>::*)() const>(
               &System<PySystem_double_0>::AllocateDiscreteVariables))
      .def("AllocateFixedInputs",
           static_cast<void (System<PySystem_double_0>::*)(
               Context<PySystem_double_0> *) const>(
               &System<PySystem_double_0>::AllocateFixedInputs),
           py::arg("context"))
      .def("AllocateForcedDiscreteUpdateEventCollection",
           static_cast<::std::unique_ptr<
               EventCollection<DiscreteUpdateEvent<PySystem_double_0>>,
               std::default_delete<
                   EventCollection<DiscreteUpdateEvent<PySystem_double_0>>>> (
               System<PySystem_double_0>::*)() const>(
               &System<PySystem_double_0>::
                   AllocateForcedDiscreteUpdateEventCollection))
      .def(
          "AllocateForcedPublishEventCollection",
          static_cast<::std::unique_ptr<
              EventCollection<PublishEvent<PySystem_double_0>>,
              std::default_delete<
                  EventCollection<PublishEvent<PySystem_double_0>>>> (
              System<PySystem_double_0>::*)() const>(
              &System<PySystem_double_0>::AllocateForcedPublishEventCollection))
      .def(
          "AllocateForcedUnrestrictedUpdateEventCollection",
          static_cast<::std::unique_ptr<
              EventCollection<UnrestrictedUpdateEvent<PySystem_double_0>>,
              std::default_delete<EventCollection<UnrestrictedUpdateEvent<
                  PySystem_double_0>>>> (System<PySystem_double_0>::*)() const>(
              &System<PySystem_double_0>::
                  AllocateForcedUnrestrictedUpdateEventCollection))
      .def("AllocateImplicitTimeDerivativesResidual",
           static_cast<::Eigen::Matrix<double, -1, 1, 0, -1, 1> (
               System<PySystem_double_0>::*)() const>(
               &System<
                   PySystem_double_0>::AllocateImplicitTimeDerivativesResidual))
      .def(
          "AllocateInputAbstract",
          static_cast<::std::unique_ptr<
              drake::AbstractValue, std::default_delete<drake::AbstractValue>> (
              System<PySystem_double_0>::*)(
              InputPort<PySystem_double_0> const &) const>(
              &System<PySystem_double_0>::AllocateInputAbstract),
          py::arg("input_port"))
      .def("AllocateInputVector",
           static_cast<::std::unique_ptr<
               BasicVector<PySystem_double_0>,
               std::default_delete<BasicVector<PySystem_double_0>>> (
               System<PySystem_double_0>::*)(
               InputPort<PySystem_double_0> const &) const>(
               &System<PySystem_double_0>::AllocateInputVector),
           py::arg("input_port"))
      .def("AllocateOutput",
           static_cast<::std::unique_ptr<
               SystemOutput<PySystem_double_0>,
               std::default_delete<SystemOutput<PySystem_double_0>>> (
               System<PySystem_double_0>::*)() const>(
               &System<PySystem_double_0>::AllocateOutput))
      .def("AllocateTimeDerivatives",
           static_cast<::std::unique_ptr<
               ContinuousState<PySystem_double_0>,
               std::default_delete<ContinuousState<PySystem_double_0>>> (
               System<PySystem_double_0>::*)() const>(
               &System<PySystem_double_0>::AllocateTimeDerivatives))
      .def("ApplyDiscreteVariableUpdate",
           static_cast<void (System<PySystem_double_0>::*)(
               EventCollection<DiscreteUpdateEvent<PySystem_double_0>> const &,
               DiscreteValues<PySystem_double_0> *,
               Context<PySystem_double_0> *) const>(
               &System<PySystem_double_0>::ApplyDiscreteVariableUpdate),
           py::arg("events"), py::arg("discrete_state"), py::arg("context"))
      .def("ApplyUnrestrictedUpdate",
           static_cast<void (System<PySystem_double_0>::*)(
               EventCollection<UnrestrictedUpdateEvent<PySystem_double_0>> const
                   &,
               State<PySystem_double_0> *, Context<PySystem_double_0> *) const>(
               &System<PySystem_double_0>::ApplyUnrestrictedUpdate),
           py::arg("events"), py::arg("state"), py::arg("context"))
      .def("CalcConservativePower",
           static_cast<double (System<PySystem_double_0>::*)(
               Context<PySystem_double_0> const &) const>(
               &System<PySystem_double_0>::CalcConservativePower),
           py::arg("context"))
      .def("CalcDiscreteVariableUpdates",
           static_cast<void (System<PySystem_double_0>::*)(
               Context<PySystem_double_0> const &,
               EventCollection<DiscreteUpdateEvent<PySystem_double_0>> const &,
               DiscreteValues<PySystem_double_0> *) const>(
               &System<PySystem_double_0>::CalcDiscreteVariableUpdates),
           py::arg("context"), py::arg("events"), py::arg("discrete_state"))
      .def("CalcDiscreteVariableUpdates",
           static_cast<void (System<PySystem_double_0>::*)(
               Context<PySystem_double_0> const &,
               DiscreteValues<PySystem_double_0> *) const>(
               &System<PySystem_double_0>::CalcDiscreteVariableUpdates),
           py::arg("context"), py::arg("discrete_state"))
      .def("CalcImplicitTimeDerivativesResidual",
           [](System<PySystem_double_0> &self,
              Context<PySystem_double_0> const &context,
              ContinuousState<PySystem_double_0> const &proposed_derivatives,
              Eigen::Ref<
                  ::drake::EigenPtr<Eigen::Matrix<double, -1, 1, 0, -1, 1>>, 0,
                  Eigen::Stride<Eigen::Dynamic, Eigen::Dynamic>>
                  residual) {
             return self.CalcImplicitTimeDerivativesResidual(
                 context, proposed_derivatives, residual);
           })
      .def("CalcKineticEnergy",
           static_cast<double (System<PySystem_double_0>::*)(
               Context<PySystem_double_0> const &) const>(
               &System<PySystem_double_0>::CalcKineticEnergy),
           py::arg("context"))
      .def("CalcNextUpdateTime",
           static_cast<double (System<PySystem_double_0>::*)(
               Context<PySystem_double_0> const &,
               CompositeEventCollection<PySystem_double_0> *) const>(
               &System<PySystem_double_0>::CalcNextUpdateTime),
           py::arg("context"), py::arg("events"))
      .def("CalcNonConservativePower",
           static_cast<double (System<PySystem_double_0>::*)(
               Context<PySystem_double_0> const &) const>(
               &System<PySystem_double_0>::CalcNonConservativePower),
           py::arg("context"))
      .def("CalcOutput",
           static_cast<void (System<PySystem_double_0>::*)(
               Context<PySystem_double_0> const &,
               SystemOutput<PySystem_double_0> *) const>(
               &System<PySystem_double_0>::CalcOutput),
           py::arg("context"), py::arg("outputs"))
      .def("CalcPotentialEnergy",
           static_cast<double (System<PySystem_double_0>::*)(
               Context<PySystem_double_0> const &) const>(
               &System<PySystem_double_0>::CalcPotentialEnergy),
           py::arg("context"))
      .def("CalcTimeDerivatives",
           static_cast<void (System<PySystem_double_0>::*)(
               Context<PySystem_double_0> const &,
               ContinuousState<PySystem_double_0> *) const>(
               &System<PySystem_double_0>::CalcTimeDerivatives),
           py::arg("context"), py::arg("derivatives"))
      .def("CalcUnrestrictedUpdate",
           static_cast<void (System<PySystem_double_0>::*)(
               Context<PySystem_double_0> const &,
               EventCollection<UnrestrictedUpdateEvent<PySystem_double_0>> const
                   &,
               State<PySystem_double_0> *) const>(
               &System<PySystem_double_0>::CalcUnrestrictedUpdate),
           py::arg("context"), py::arg("events"), py::arg("state"))
      .def("CalcUnrestrictedUpdate",
           static_cast<void (System<PySystem_double_0>::*)(
               Context<PySystem_double_0> const &, State<PySystem_double_0> *)
                           const>(
               &System<PySystem_double_0>::CalcUnrestrictedUpdate),
           py::arg("context"), py::arg("state"))
      .def("CalcWitnessValue",
           static_cast<double (System<PySystem_double_0>::*)(
               Context<PySystem_double_0> const &,
               WitnessFunction<PySystem_double_0> const &) const>(
               &System<PySystem_double_0>::CalcWitnessValue),
           py::arg("context"), py::arg("witness_func"))
      .def("CheckSystemConstraintsSatisfied",
           static_cast<::drake::scalar_predicate<PySystem_double_0>::type (
               System<PySystem_double_0>::*)(Context<PySystem_double_0> const &,
                                             double) const>(
               &System<PySystem_double_0>::CheckSystemConstraintsSatisfied),
           py::arg("context"), py::arg("tol"))
      .def("CheckValidOutput",
           static_cast<void (System<PySystem_double_0>::*)(
               SystemOutput<PySystem_double_0> const *) const>(
               &System<PySystem_double_0>::CheckValidOutput),
           py::arg("output"))
      .def("CopyContinuousStateVector",
           static_cast<::Eigen::Matrix<double, -1, 1, 0, -1, 1> (
               System<PySystem_double_0>::*)(Context<PySystem_double_0> const &)
                           const>(
               &System<PySystem_double_0>::CopyContinuousStateVector),
           py::arg("context"))
      .def("CreateDefaultContext",
           static_cast<::std::unique_ptr<
               Context<PySystem_double_0>,
               std::default_delete<Context<PySystem_double_0>>> (
               System<PySystem_double_0>::*)() const>(
               &System<PySystem_double_0>::CreateDefaultContext))
      .def("DeclareInputPort",
           static_cast<InputPort<PySystem_double_0> &(
               System<PySystem_double_0>::
                   *)(::std::variant<
                          std::basic_string<char, std::char_traits<char>,
                                            std::allocator<char>>,
                          UseDefaultName>,
                      PortDataType, int,
                      ::std::optional<drake::RandomDistribution>)>(
               &System_double_publicist::DeclareInputPort),
           py::arg("name"), py::arg("type"), py::arg("size"),
           py::arg("random_type") =
               ::std::optional<drake::RandomDistribution>(std::nullopt))
      .def("DeclareInputPort",
           static_cast<InputPort<PySystem_double_0> &(
               System<PySystem_double_0>::*)(PortDataType, int,
                                             ::std::optional<
                                                 drake::RandomDistribution>)>(
               &System_double_publicist::DeclareInputPort),
           py::arg("type"), py::arg("size"),
           py::arg("random_type") =
               ::std::optional<drake::RandomDistribution>(std::nullopt))
      .def("DispatchDiscreteVariableUpdateHandler",
           static_cast<void (System<PySystem_double_0>::*)(
               Context<PySystem_double_0> const &,
               EventCollection<DiscreteUpdateEvent<PySystem_double_0>> const &,
               DiscreteValues<PySystem_double_0> *) const>(
               &System_double_publicist::DispatchDiscreteVariableUpdateHandler),
           py::arg("context"), py::arg("events"), py::arg("discrete_state"))
      .def("DispatchPublishHandler",
           static_cast<void (System<PySystem_double_0>::*)(
               Context<PySystem_double_0> const &,
               EventCollection<PublishEvent<PySystem_double_0>> const &) const>(
               &System_double_publicist::DispatchPublishHandler),
           py::arg("context"), py::arg("events"))
      .def("DispatchUnrestrictedUpdateHandler",
           static_cast<void (System<PySystem_double_0>::*)(
               Context<PySystem_double_0> const &,
               EventCollection<UnrestrictedUpdateEvent<PySystem_double_0>> const
                   &,
               State<PySystem_double_0> *) const>(
               &System_double_publicist::DispatchUnrestrictedUpdateHandler),
           py::arg("context"), py::arg("events"), py::arg("state"))
      .def("DoApplyDiscreteVariableUpdate",
           static_cast<void (System<PySystem_double_0>::*)(
               EventCollection<DiscreteUpdateEvent<PySystem_double_0>> const &,
               DiscreteValues<PySystem_double_0> *,
               Context<PySystem_double_0> *) const>(
               &System_double_publicist::DoApplyDiscreteVariableUpdate),
           py::arg("events"), py::arg("discrete_state"), py::arg("context"))
      .def("DoApplyUnrestrictedUpdate",
           static_cast<void (System<PySystem_double_0>::*)(
               EventCollection<UnrestrictedUpdateEvent<PySystem_double_0>> const
                   &,
               State<PySystem_double_0> *, Context<PySystem_double_0> *) const>(
               &System_double_publicist::DoApplyUnrestrictedUpdate),
           py::arg("events"), py::arg("state"), py::arg("context"))
      .def("DoCalcConservativePower",
           static_cast<double (System<PySystem_double_0>::*)(
               Context<PySystem_double_0> const &) const>(
               &System_double_publicist::DoCalcConservativePower),
           py::arg("context"))
      .def("DoCalcImplicitTimeDerivativesResidual",
           [](System<PySystem_double_0> &self,
              Context<PySystem_double_0> const &context,
              ContinuousState<PySystem_double_0> const &proposed_derivatives,
              Eigen::Ref<
                  ::drake::EigenPtr<Eigen::Matrix<double, -1, 1, 0, -1, 1>>, 0,
                  Eigen::Stride<Eigen::Dynamic, Eigen::Dynamic>>
                  residual) {
             return self.DoCalcImplicitTimeDerivativesResidual(
                 context, proposed_derivatives, residual);
           })
      .def("DoCalcKineticEnergy",
           static_cast<double (System<PySystem_double_0>::*)(
               Context<PySystem_double_0> const &) const>(
               &System_double_publicist::DoCalcKineticEnergy),
           py::arg("context"))
      .def("DoCalcNextUpdateTime",
           static_cast<void (System<PySystem_double_0>::*)(
               Context<PySystem_double_0> const &,
               CompositeEventCollection<PySystem_double_0> *, double *) const>(
               &System_double_publicist::DoCalcNextUpdateTime),
           py::arg("context"), py::arg("events"), py::arg("time"))
      .def("DoCalcNonConservativePower",
           static_cast<double (System<PySystem_double_0>::*)(
               Context<PySystem_double_0> const &) const>(
               &System_double_publicist::DoCalcNonConservativePower),
           py::arg("context"))
      .def("DoCalcPotentialEnergy",
           static_cast<double (System<PySystem_double_0>::*)(
               Context<PySystem_double_0> const &) const>(
               &System_double_publicist::DoCalcPotentialEnergy),
           py::arg("context"))
      .def("DoCalcTimeDerivatives",
           static_cast<void (System<PySystem_double_0>::*)(
               Context<PySystem_double_0> const &,
               ContinuousState<PySystem_double_0> *) const>(
               &System_double_publicist::DoCalcTimeDerivatives),
           py::arg("context"), py::arg("derivatives"))
      .def("DoCalcWitnessValue",
           static_cast<double (System<PySystem_double_0>::*)(
               Context<PySystem_double_0> const &,
               WitnessFunction<PySystem_double_0> const &) const>(
               &System_double_publicist::DoCalcWitnessValue),
           py::arg("context"), py::arg("witness_func"))
      .def("DoGetInitializationEvents",
           static_cast<void (System<PySystem_double_0>::*)(
               Context<PySystem_double_0> const &,
               CompositeEventCollection<PySystem_double_0> *) const>(
               &System_double_publicist::DoGetInitializationEvents),
           py::arg("context"), py::arg("events"))
      .def("DoGetMutableTargetSystemCompositeEventCollection",
           static_cast<CompositeEventCollection<PySystem_double_0> *(
               System<PySystem_double_0>::*)(System<PySystem_double_0> const &,
                                             CompositeEventCollection<
                                                 PySystem_double_0> *)const>(
               &System<PySystem_double_0>::
                   DoGetMutableTargetSystemCompositeEventCollection),
           py::arg("target_system"), py::arg("events"))
      .def("DoGetMutableTargetSystemState",
           static_cast<State<PySystem_double_0> *(
               System<PySystem_double_0>::*)(System<PySystem_double_0> const &,
                                             State<PySystem_double_0> *)const>(
               &System<PySystem_double_0>::DoGetMutableTargetSystemState),
           py::arg("target_system"), py::arg("state"))
      .def("DoGetPerStepEvents",
           static_cast<void (System<PySystem_double_0>::*)(
               Context<PySystem_double_0> const &,
               CompositeEventCollection<PySystem_double_0> *) const>(
               &System_double_publicist::DoGetPerStepEvents),
           py::arg("context"), py::arg("events"))
      .def("DoGetPeriodicEvents",
           static_cast<::std::map<
               PeriodicEventData,
               std::vector<const Event<PySystem_double_0> *,
                           std::allocator<const Event<PySystem_double_0> *>>,
               PeriodicEventDataComparator,
               std::allocator<std::pair<
                   const PeriodicEventData,
                   std::vector<
                       const Event<PySystem_double_0> *,
                       std::allocator<const Event<PySystem_double_0> *>>>>> (
               System<PySystem_double_0>::*)() const>(
               &System_double_publicist::DoGetPeriodicEvents))
      .def(
          "DoGetTargetSystemCompositeEventCollection",
          static_cast<CompositeEventCollection<PySystem_double_0> const *(
              System<PySystem_double_0>::
                  *)(System<PySystem_double_0> const &,
                     CompositeEventCollection<PySystem_double_0> const *)const>(
              &System<PySystem_double_0>::
                  DoGetTargetSystemCompositeEventCollection),
          py::arg("target_system"), py::arg("events"))
      .def("DoGetTargetSystemContext",
           static_cast<Context<PySystem_double_0> const *(
               System<PySystem_double_0>::*)(System<PySystem_double_0> const &,
                                             Context<PySystem_double_0> const *)
                           const>(
               &System<PySystem_double_0>::DoGetTargetSystemContext),
           py::arg("target_system"), py::arg("context"))
      .def("DoGetTargetSystemContinuousState",
           static_cast<ContinuousState<PySystem_double_0> const *(
               System<PySystem_double_0>::
                   *)(System<PySystem_double_0> const &,
                      ContinuousState<PySystem_double_0> const *)const>(
               &System<PySystem_double_0>::DoGetTargetSystemContinuousState),
           py::arg("target_system"), py::arg("xc"))
      .def("DoGetTargetSystemState",
           static_cast<State<PySystem_double_0> const *(
               System<PySystem_double_0>::*)(System<PySystem_double_0> const &,
                                             State<PySystem_double_0> const *)
                           const>(
               &System<PySystem_double_0>::DoGetTargetSystemState),
           py::arg("target_system"), py::arg("state"))
      .def(
          "DoGetWitnessFunctions",
          static_cast<void (System<PySystem_double_0>::*)(
              Context<PySystem_double_0> const &,
              ::std::vector<
                  const WitnessFunction<PySystem_double_0> *,
                  std::allocator<const WitnessFunction<PySystem_double_0> *>> *)
                          const>(
              &System_double_publicist::DoGetWitnessFunctions),
          py::arg("arg0"), py::arg("arg1"))
      .def("DoMapQDotToVelocity",
           [](System<PySystem_double_0> &self,
              Context<PySystem_double_0> const &context,
              ::Eigen::Ref<const Eigen::Matrix<double, -1, 1, 0, -1, 1>, 0,
                           Eigen::InnerStride<1>> const &qdot,
              VectorBase<PySystem_double_0> *generalized_velocity) {
             return self.DoMapQDotToVelocity(context, qdot,
                                             generalized_velocity);
           })
      .def("DoMapVelocityToQDot",
           [](System<PySystem_double_0> &self,
              Context<PySystem_double_0> const &context,
              ::Eigen::Ref<const Eigen::Matrix<double, -1, 1, 0, -1, 1>, 0,
                           Eigen::InnerStride<1>> const &generalized_velocity,
              VectorBase<PySystem_double_0> *qdot) {
             return self.DoMapVelocityToQDot(context, generalized_velocity,
                                             qdot);
           })
      .def("EvalConservativePower",
           static_cast<double const &(
               System<PySystem_double_0>::*)(Context<PySystem_double_0> const &)
                           const>(
               &System<PySystem_double_0>::EvalConservativePower),
           py::arg("context"))
      .def("EvalEigenVectorInput",
           static_cast<::Eigen::VectorBlock<
               const Eigen::Matrix<double, -1, 1, 0, -1, 1>, -1> (
               System<PySystem_double_0>::*)(Context<PySystem_double_0> const &,
                                             int) const>(
               &System<PySystem_double_0>::EvalEigenVectorInput),
           py::arg("context"), py::arg("port_index"))
      .def(
          "EvalKineticEnergy",
          static_cast<double const &(
              System<PySystem_double_0>::*)(Context<PySystem_double_0> const &)
                          const>(&System<PySystem_double_0>::EvalKineticEnergy),
          py::arg("context"))
      .def("EvalNonConservativePower",
           static_cast<double const &(
               System<PySystem_double_0>::*)(Context<PySystem_double_0> const &)
                           const>(
               &System<PySystem_double_0>::EvalNonConservativePower),
           py::arg("context"))
      .def("EvalPotentialEnergy",
           static_cast<double const &(
               System<PySystem_double_0>::*)(Context<PySystem_double_0> const &)
                           const>(
               &System<PySystem_double_0>::EvalPotentialEnergy),
           py::arg("context"))
      .def("EvalTimeDerivatives",
           static_cast<ContinuousState<PySystem_double_0> const &(
               System<PySystem_double_0>::*)(Context<PySystem_double_0> const &)
                           const>(
               &System<PySystem_double_0>::EvalTimeDerivatives),
           py::arg("context"))
      .def(
          "FixInputPortsFrom",
          static_cast<void (System<PySystem_double_0>::*)(
              System<PySystem_double_0> const &,
              Context<PySystem_double_0> const &, Context<PySystem_double_0> *)
                          const>(&System<PySystem_double_0>::FixInputPortsFrom),
          py::arg("other_system"), py::arg("other_context"),
          py::arg("target_context"))
      .def("GetGraphvizFragment",
           static_cast<void (System<PySystem_double_0>::*)(
               int, ::std::stringstream *) const>(
               &System<PySystem_double_0>::GetGraphvizFragment),
           py::arg("max_depth"), py::arg("dot"))
      .def("GetGraphvizId",
           static_cast<::int64_t (System<PySystem_double_0>::*)() const>(
               &System<PySystem_double_0>::GetGraphvizId))
      .def("GetGraphvizInputPortToken",
           static_cast<void (System<PySystem_double_0>::*)(
               InputPort<PySystem_double_0> const &, int, ::std::stringstream *)
                           const>(
               &System<PySystem_double_0>::GetGraphvizInputPortToken),
           py::arg("port"), py::arg("max_depth"), py::arg("dot"))
      .def("GetGraphvizOutputPortToken",
           static_cast<void (System<PySystem_double_0>::*)(
               OutputPort<PySystem_double_0> const &, int,
               ::std::stringstream *) const>(
               &System<PySystem_double_0>::GetGraphvizOutputPortToken),
           py::arg("port"), py::arg("max_depth"), py::arg("dot"))
      .def("GetGraphvizString",
           static_cast<::std::string (System<PySystem_double_0>::*)(int) const>(
               &System<PySystem_double_0>::GetGraphvizString),
           py::arg("max_depth") = int(std::numeric_limits<int>::max()))
      .def("GetInitializationEvents",
           static_cast<void (System<PySystem_double_0>::*)(
               Context<PySystem_double_0> const &,
               CompositeEventCollection<PySystem_double_0> *) const>(
               &System<PySystem_double_0>::GetInitializationEvents),
           py::arg("context"), py::arg("events"))
      .def("GetInputPort",
           static_cast<InputPort<PySystem_double_0> const &(
               System<PySystem_double_0>::*)(::std::string const &)const>(
               &System<PySystem_double_0>::GetInputPort),
           py::arg("port_name"))
      .def("GetMemoryObjectName",
           static_cast<::std::string (System<PySystem_double_0>::*)() const>(
               &System<PySystem_double_0>::GetMemoryObjectName))
      .def(
          "GetMutableOutputVector",
          static_cast<
              ::Eigen::VectorBlock<Eigen::Matrix<double, -1, 1, 0, -1, 1>, -1> (
                  System<PySystem_double_0>::*)(
                  SystemOutput<PySystem_double_0> *, int) const>(
              &System_double_publicist::GetMutableOutputVector),
          py::arg("output"), py::arg("port_index"))
      .def(
          "GetMutableSubsystemContext",
          static_cast<Context<PySystem_double_0> &(
              System<PySystem_double_0>::*)(System<PySystem_double_0> const &,
                                            Context<PySystem_double_0> *)const>(
              &System<PySystem_double_0>::GetMutableSubsystemContext),
          py::arg("subsystem"), py::arg("context"))
      .def("GetMyContextFromRoot",
           static_cast<Context<PySystem_double_0> const &(
               System<PySystem_double_0>::*)(Context<PySystem_double_0> const &)
                           const>(
               &System<PySystem_double_0>::GetMyContextFromRoot),
           py::arg("root_context"))
      .def(
          "GetMyMutableContextFromRoot",
          static_cast<Context<PySystem_double_0> &(
              System<PySystem_double_0>::*)(Context<PySystem_double_0> *)const>(
              &System<PySystem_double_0>::GetMyMutableContextFromRoot),
          py::arg("root_context"))
      .def("GetOutputPort",
           static_cast<OutputPort<PySystem_double_0> const &(
               System<PySystem_double_0>::*)(::std::string const &)const>(
               &System<PySystem_double_0>::GetOutputPort),
           py::arg("port_name"))
      .def("GetPerStepEvents",
           static_cast<void (System<PySystem_double_0>::*)(
               Context<PySystem_double_0> const &,
               CompositeEventCollection<PySystem_double_0> *) const>(
               &System<PySystem_double_0>::GetPerStepEvents),
           py::arg("context"), py::arg("events"))
      .def("GetPeriodicEvents",
           static_cast<::std::map<
               PeriodicEventData,
               std::vector<const Event<PySystem_double_0> *,
                           std::allocator<const Event<PySystem_double_0> *>>,
               PeriodicEventDataComparator,
               std::allocator<std::pair<
                   const PeriodicEventData,
                   std::vector<
                       const Event<PySystem_double_0> *,
                       std::allocator<const Event<PySystem_double_0> *>>>>> (
               System<PySystem_double_0>::*)() const>(
               &System<PySystem_double_0>::GetPeriodicEvents))
      .def("GetSubsystemContext",
           static_cast<Context<PySystem_double_0> const &(
               System<PySystem_double_0>::*)(System<PySystem_double_0> const &,
                                             Context<PySystem_double_0> const &)
                           const>(
               &System<PySystem_double_0>::GetSubsystemContext),
           py::arg("subsystem"), py::arg("context"))
      .def(
          "GetUniquePeriodicDiscreteUpdateAttribute",
          static_cast<::std::optional<PeriodicEventData> (
              System<PySystem_double_0>::*)() const>(
              &System<
                  PySystem_double_0>::GetUniquePeriodicDiscreteUpdateAttribute))
      .def(
          "GetWitnessFunctions",
          static_cast<void (System<PySystem_double_0>::*)(
              Context<PySystem_double_0> const &,
              ::std::vector<
                  const WitnessFunction<PySystem_double_0> *,
                  std::allocator<const WitnessFunction<PySystem_double_0> *>> *)
                          const>(
              &System<PySystem_double_0>::GetWitnessFunctions),
          py::arg("context"), py::arg("w"))
      .def("HasAnyDirectFeedthrough",
           static_cast<bool (System<PySystem_double_0>::*)() const>(
               &System<PySystem_double_0>::HasAnyDirectFeedthrough))
      .def("HasDirectFeedthrough",
           static_cast<bool (System<PySystem_double_0>::*)(int) const>(
               &System<PySystem_double_0>::HasDirectFeedthrough),
           py::arg("output_port"))
      .def("HasDirectFeedthrough",
           static_cast<bool (System<PySystem_double_0>::*)(int, int) const>(
               &System<PySystem_double_0>::HasDirectFeedthrough),
           py::arg("input_port"), py::arg("output_port"))
      .def(
          "HasInputPort",
          static_cast<bool (System<PySystem_double_0>::*)(::std::string const &)
                          const>(&System<PySystem_double_0>::HasInputPort),
          py::arg("port_name"))
      .def(
          "HasOutputPort",
          static_cast<bool (System<PySystem_double_0>::*)(::std::string const &)
                          const>(&System<PySystem_double_0>::HasOutputPort),
          py::arg("port_name"))
      .def("IsDifferenceEquationSystem",
           static_cast<bool (System<PySystem_double_0>::*)(double *) const>(
               &System<PySystem_double_0>::IsDifferenceEquationSystem),
           py::arg("time_period") = (double *)nullptr)
      .def("MapQDotToVelocity",
           static_cast<void (System<PySystem_double_0>::*)(
               Context<PySystem_double_0> const &,
               VectorBase<PySystem_double_0> const &,
               VectorBase<PySystem_double_0> *) const>(
               &System<PySystem_double_0>::MapQDotToVelocity),
           py::arg("context"), py::arg("qdot"), py::arg("generalized_velocity"))
      .def("MapQDotToVelocity",
           [](System<PySystem_double_0> &self,
              Context<PySystem_double_0> const &context,
              ::Eigen::Ref<const Eigen::Matrix<double, -1, 1, 0, -1, 1>, 0,
                           Eigen::InnerStride<1>> const &qdot,
              VectorBase<PySystem_double_0> *generalized_velocity) {
             return self.MapQDotToVelocity(context, qdot, generalized_velocity);
           })
      .def("MapVelocityToQDot",
           static_cast<void (System<PySystem_double_0>::*)(
               Context<PySystem_double_0> const &,
               VectorBase<PySystem_double_0> const &,
               VectorBase<PySystem_double_0> *) const>(
               &System<PySystem_double_0>::MapVelocityToQDot),
           py::arg("context"), py::arg("generalized_velocity"), py::arg("qdot"))
      .def("MapVelocityToQDot",
           [](System<PySystem_double_0> &self,
              Context<PySystem_double_0> const &context,
              ::Eigen::Ref<const Eigen::Matrix<double, -1, 1, 0, -1, 1>, 0,
                           Eigen::InnerStride<1>> const &generalized_velocity,
              VectorBase<PySystem_double_0> *qdot) {
             return self.MapVelocityToQDot(context, generalized_velocity, qdot);
           })
      .def("Publish",
           static_cast<void (System<PySystem_double_0>::*)(
               Context<PySystem_double_0> const &,
               EventCollection<PublishEvent<PySystem_double_0>> const &) const>(
               &System<PySystem_double_0>::Publish),
           py::arg("context"), py::arg("events"))
      .def("Publish",
           static_cast<void (System<PySystem_double_0>::*)(
               Context<PySystem_double_0> const &) const>(
               &System<PySystem_double_0>::Publish),
           py::arg("context"))
      .def("SetDefaultContext",
           static_cast<void (System<PySystem_double_0>::*)(
               Context<PySystem_double_0> *) const>(
               &System<PySystem_double_0>::SetDefaultContext),
           py::arg("context"))
      .def("SetDefaultParameters",
           static_cast<void (System<PySystem_double_0>::*)(
               Context<PySystem_double_0> const &,
               Parameters<PySystem_double_0> *) const>(
               &System<PySystem_double_0>::SetDefaultParameters),
           py::arg("context"), py::arg("parameters"))
      .def("SetDefaultState",
           static_cast<void (System<PySystem_double_0>::*)(
               Context<PySystem_double_0> const &, State<PySystem_double_0> *)
                           const>(&System<PySystem_double_0>::SetDefaultState),
           py::arg("context"), py::arg("state"))
      .def("SetRandomContext",
           static_cast<void (System<PySystem_double_0>::*)(
               Context<PySystem_double_0> *, ::drake::RandomGenerator *) const>(
               &System<PySystem_double_0>::SetRandomContext),
           py::arg("context"), py::arg("generator"))
      .def("SetRandomParameters",
           static_cast<void (System<PySystem_double_0>::*)(
               Context<PySystem_double_0> const &,
               Parameters<PySystem_double_0> *, ::drake::RandomGenerator *)
                           const>(
               &System<PySystem_double_0>::SetRandomParameters),
           py::arg("context"), py::arg("parameters"), py::arg("generator"))
      .def("SetRandomState",
           static_cast<void (System<PySystem_double_0>::*)(
               Context<PySystem_double_0> const &, State<PySystem_double_0> *,
               ::drake::RandomGenerator *) const>(
               &System<PySystem_double_0>::SetRandomState),
           py::arg("context"), py::arg("state"), py::arg("generator"))
      .def("ToAutoDiffXd",
           static_cast<::std::unique_ptr<
               System<::drake::AutoDiffXd>,
               std::default_delete<System<::drake::AutoDiffXd>>> (
               System<PySystem_double_0>::*)() const>(
               &System<PySystem_double_0>::ToAutoDiffXd))
      .def("ToAutoDiffXdMaybe",
           static_cast<::std::unique_ptr<
               System<::drake::AutoDiffXd>,
               std::default_delete<System<::drake::AutoDiffXd>>> (
               System<PySystem_double_0>::*)() const>(
               &System<PySystem_double_0>::ToAutoDiffXdMaybe))
      .def("ToSymbolic",
           static_cast<::std::unique_ptr<
               System<::drake::symbolic::Expression>,
               std::default_delete<System<::drake::symbolic::Expression>>> (
               System<PySystem_double_0>::*)() const>(
               &System<PySystem_double_0>::ToSymbolic))
      .def("ToSymbolicMaybe",
           static_cast<::std::unique_ptr<
               System<::drake::symbolic::Expression>,
               std::default_delete<System<::drake::symbolic::Expression>>> (
               System<PySystem_double_0>::*)() const>(
               &System<PySystem_double_0>::ToSymbolicMaybe))
      .def("forced_discrete_update_events_exist",
           static_cast<bool (System<PySystem_double_0>::*)() const>(
               &System_double_publicist::forced_discrete_update_events_exist))
      .def("forced_publish_events_exist",
           static_cast<bool (System<PySystem_double_0>::*)() const>(
               &System_double_publicist::forced_publish_events_exist))
      .def("forced_unrestricted_update_events_exist",
           static_cast<bool (System<PySystem_double_0>::*)() const>(
               &System_double_publicist::
                   forced_unrestricted_update_events_exist))
      .def("get_constraint",
           static_cast<SystemConstraint<PySystem_double_0> const &(
               System<PySystem_double_0>::*)(SystemConstraintIndex) const>(
               &System<PySystem_double_0>::get_constraint),
           py::arg("constraint_index"))
      .def("get_forced_discrete_update_events",
           static_cast<
               EventCollection<DiscreteUpdateEvent<PySystem_double_0>> const &(
                   System<PySystem_double_0>::*)() const>(
               &System_double_publicist::get_forced_discrete_update_events))
      .def("get_forced_publish_events",
           static_cast<EventCollection<PublishEvent<PySystem_double_0>> const &(
               System<PySystem_double_0>::*)() const>(
               &System_double_publicist::get_forced_publish_events))
      .def("get_forced_unrestricted_update_events",
           static_cast<
               EventCollection<UnrestrictedUpdateEvent<PySystem_double_0>> const
                   &(System<PySystem_double_0>::*)() const>(
               &System_double_publicist::get_forced_unrestricted_update_events))
      .def("get_input_port",
           static_cast<InputPort<PySystem_double_0> const &(
               System<PySystem_double_0>::*)(int)const>(
               &System<PySystem_double_0>::get_input_port),
           py::arg("port_index"))
      .def("get_input_port", static_cast<InputPort<PySystem_double_0> const &(
                                 System<PySystem_double_0>::*)() const>(
                                 &System<PySystem_double_0>::get_input_port))
      .def("get_input_port_selection",
           static_cast<InputPort<PySystem_double_0> const *(
               System<PySystem_double_0>::
                   *)(::std::variant<InputPortSelection,
                                     drake::TypeSafeIndex<InputPortTag>>)const>(
               &System<PySystem_double_0>::get_input_port_selection),
           py::arg("port_index"))
      .def(
          "get_mutable_forced_discrete_update_events",
          static_cast<EventCollection<DiscreteUpdateEvent<PySystem_double_0>> &(
              System<PySystem_double_0>::*)()>(
              &System_double_publicist::
                  get_mutable_forced_discrete_update_events))
      .def("get_mutable_forced_publish_events",
           static_cast<EventCollection<PublishEvent<PySystem_double_0>> &(
               System<PySystem_double_0>::*)()>(
               &System_double_publicist::get_mutable_forced_publish_events))
      .def("get_mutable_forced_unrestricted_update_events",
           static_cast<
               EventCollection<UnrestrictedUpdateEvent<PySystem_double_0>> &(
                   System<PySystem_double_0>::*)()>(
               &System_double_publicist::
                   get_mutable_forced_unrestricted_update_events))
      .def("get_output_port",
           static_cast<OutputPort<PySystem_double_0> const &(
               System<PySystem_double_0>::*)(int)const>(
               &System<PySystem_double_0>::get_output_port),
           py::arg("port_index"))
      .def("get_output_port", static_cast<OutputPort<PySystem_double_0> const &(
                                  System<PySystem_double_0>::*)() const>(
                                  &System<PySystem_double_0>::get_output_port))
      .def(
          "get_output_port_selection",
          static_cast<OutputPort<PySystem_double_0> const *(
              System<PySystem_double_0>::
                  *)(::std::variant<OutputPortSelection,
                                    drake::TypeSafeIndex<OutputPortTag>>)const>(
              &System<PySystem_double_0>::get_output_port_selection),
          py::arg("port_index"))
      .def("get_system_scalar_converter",
           static_cast<SystemScalarConverter const &(
               System<PySystem_double_0>::*)() const>(
               &System<PySystem_double_0>::get_system_scalar_converter))
      .def(
          "get_time_derivatives_cache_entry",
          static_cast<CacheEntry const &(System<PySystem_double_0>::*)() const>(
              &System<PySystem_double_0>::get_time_derivatives_cache_entry))
      .def("num_constraints",
           static_cast<int (System<PySystem_double_0>::*)() const>(
               &System<PySystem_double_0>::num_constraints))
      .def("set_forced_discrete_update_events",
           [](System<PySystem_double_0> &self,
              EventCollection<DiscreteUpdateEvent<PySystem_double_0>> forced) {
             self.set_forced_discrete_update_events(
                 std::make_unique<
                     EventCollection<DiscreteUpdateEvent<PySystem_double_0>>>(
                     forced));
           })
      .def("set_forced_publish_events",
           [](System<PySystem_double_0> &self,
              EventCollection<PublishEvent<PySystem_double_0>> forced) {
             self.set_forced_publish_events(
                 std::make_unique<
                     EventCollection<PublishEvent<PySystem_double_0>>>(forced));
           })
      .def("set_forced_unrestricted_update_events",
           [](System<PySystem_double_0> &self,
              EventCollection<UnrestrictedUpdateEvent<PySystem_double_0>>
                  forced) {
             self.set_forced_unrestricted_update_events(
                 std::make_unique<EventCollection<
                     UnrestrictedUpdateEvent<PySystem_double_0>>>(forced));
           })

      ;

  using PySystem_Eigen_AutoDiffScalar_Eigen_VectorXd_0 = ::drake::AutoDiffXd;

  py::class_<System<Eigen::AutoDiffScalar<Eigen::VectorXd>>, SystemBase>
      PySystem_Eigen_AutoDiffScalar_Eigen_VectorXd(
          m, "System_Eigen_AutoDiffScalar_Eigen_VectorXd");

  PySystem_Eigen_AutoDiffScalar_Eigen_VectorXd
      .def("Accept",
           [](System<Eigen::AutoDiffScalar<Eigen::VectorXd>> &self,
              SystemVisitor<Eigen::AutoDiffScalar<Eigen::VectorXd>> *v) {
             return self.Accept(v);
           })
      .def("AddConstraint",
           [](System<Eigen::AutoDiffScalar<Eigen::VectorXd>> &self,
              SystemConstraint<::drake::AutoDiffXd> constraint) {
             return self.AddConstraint(
                 std::make_unique<SystemConstraint<::drake::AutoDiffXd>>(
                     constraint));
           })
      .def("AddExternalConstraint",
           static_cast<SystemConstraintIndex (
               System<Eigen::AutoDiffScalar<Eigen::VectorXd>>::*)(
               ExternalSystemConstraint)>(
               &System<Eigen::AutoDiffScalar<Eigen::VectorXd>>::
                   AddExternalConstraint),
           py::arg("constraint"))
      .def("AddTriggeredWitnessFunctionToCompositeEventCollection",
           [](System<Eigen::AutoDiffScalar<Eigen::VectorXd>> &self,
              Event<Eigen::AutoDiffScalar<Eigen::VectorXd>> *event,
              CompositeEventCollection<Eigen::AutoDiffScalar<Eigen::VectorXd>>
                  *events) {
             return self.AddTriggeredWitnessFunctionToCompositeEventCollection(
                 event, events);
           })
      .def("AllocateCompositeEventCollection",
           static_cast<
               ::std::unique_ptr<CompositeEventCollection<::drake::AutoDiffXd>,
                                 std::default_delete<CompositeEventCollection<
                                     ::drake::AutoDiffXd>>> (
                   System<Eigen::AutoDiffScalar<Eigen::VectorXd>>::*)() const>(
               &System<Eigen::AutoDiffScalar<Eigen::VectorXd>>::
                   AllocateCompositeEventCollection))
      .def(
          "AllocateContext",
          static_cast<::std::unique_ptr<
              Context<::drake::AutoDiffXd>,
              std::default_delete<Context<::drake::AutoDiffXd>>> (
              System<Eigen::AutoDiffScalar<Eigen::VectorXd>>::*)() const>(
              &System<Eigen::AutoDiffScalar<Eigen::VectorXd>>::AllocateContext))
      .def("AllocateDiscreteVariables",
           static_cast<::std::unique_ptr<
               DiscreteValues<::drake::AutoDiffXd>,
               std::default_delete<DiscreteValues<::drake::AutoDiffXd>>> (
               System<Eigen::AutoDiffScalar<Eigen::VectorXd>>::*)() const>(
               &System<Eigen::AutoDiffScalar<Eigen::VectorXd>>::
                   AllocateDiscreteVariables))
      .def("AllocateFixedInputs",
           [](System<Eigen::AutoDiffScalar<Eigen::VectorXd>> &self,
              Context<Eigen::AutoDiffScalar<Eigen::VectorXd>> *context) {
             return self.AllocateFixedInputs(context);
           })
      .def("AllocateForcedDiscreteUpdateEventCollection",
           static_cast<::std::unique_ptr<
               EventCollection<DiscreteUpdateEvent<::drake::AutoDiffXd>>,
               std::default_delete<
                   EventCollection<DiscreteUpdateEvent<::drake::AutoDiffXd>>>> (
               System<Eigen::AutoDiffScalar<Eigen::VectorXd>>::*)() const>(
               &System<Eigen::AutoDiffScalar<Eigen::VectorXd>>::
                   AllocateForcedDiscreteUpdateEventCollection))
      .def("AllocateForcedPublishEventCollection",
           static_cast<::std::unique_ptr<
               EventCollection<PublishEvent<::drake::AutoDiffXd>>,
               std::default_delete<
                   EventCollection<PublishEvent<::drake::AutoDiffXd>>>> (
               System<Eigen::AutoDiffScalar<Eigen::VectorXd>>::*)() const>(
               &System<Eigen::AutoDiffScalar<Eigen::VectorXd>>::
                   AllocateForcedPublishEventCollection))
      .def("AllocateForcedUnrestrictedUpdateEventCollection",
           static_cast<::std::unique_ptr<
               EventCollection<UnrestrictedUpdateEvent<::drake::AutoDiffXd>>,
               std::default_delete<EventCollection<
                   UnrestrictedUpdateEvent<::drake::AutoDiffXd>>>> (
               System<Eigen::AutoDiffScalar<Eigen::VectorXd>>::*)() const>(
               &System<Eigen::AutoDiffScalar<Eigen::VectorXd>>::
                   AllocateForcedUnrestrictedUpdateEventCollection))
      .def("AllocateImplicitTimeDerivativesResidual",
           static_cast<::Eigen::Matrix<Eigen::AutoDiffScalar<Eigen::VectorXd>,
                                       -1, 1, 0, -1, 1> (
               System<Eigen::AutoDiffScalar<Eigen::VectorXd>>::*)() const>(
               &System<Eigen::AutoDiffScalar<Eigen::VectorXd>>::
                   AllocateImplicitTimeDerivativesResidual))
      .def("AllocateInputAbstract",
           [](System<Eigen::AutoDiffScalar<Eigen::VectorXd>> &self,
              InputPort<Eigen::AutoDiffScalar<Eigen::VectorXd>> const
                  &input_port) {
             return self.AllocateInputAbstract(input_port);
           })
      .def("AllocateInputVector",
           [](System<Eigen::AutoDiffScalar<Eigen::VectorXd>> &self,
              InputPort<Eigen::AutoDiffScalar<Eigen::VectorXd>> const
                  &input_port) { return self.AllocateInputVector(input_port); })
      .def("AllocateOutput",
           static_cast<::std::unique_ptr<
               SystemOutput<::drake::AutoDiffXd>,
               std::default_delete<SystemOutput<::drake::AutoDiffXd>>> (
               System<Eigen::AutoDiffScalar<Eigen::VectorXd>>::*)() const>(
               &System<Eigen::AutoDiffScalar<Eigen::VectorXd>>::AllocateOutput))
      .def("AllocateTimeDerivatives",
           static_cast<::std::unique_ptr<
               ContinuousState<::drake::AutoDiffXd>,
               std::default_delete<ContinuousState<::drake::AutoDiffXd>>> (
               System<Eigen::AutoDiffScalar<Eigen::VectorXd>>::*)() const>(
               &System<Eigen::AutoDiffScalar<Eigen::VectorXd>>::
                   AllocateTimeDerivatives))
      .def("ApplyDiscreteVariableUpdate",
           [](System<Eigen::AutoDiffScalar<Eigen::VectorXd>> &self,
              EventCollection<DiscreteUpdateEvent<::drake::AutoDiffXd>> const
                  &events,
              DiscreteValues<Eigen::AutoDiffScalar<Eigen::VectorXd>>
                  *discrete_state,
              Context<Eigen::AutoDiffScalar<Eigen::VectorXd>> *context) {
             return self.ApplyDiscreteVariableUpdate(events, discrete_state,
                                                     context);
           })
      .def(
          "ApplyUnrestrictedUpdate",
          [](System<Eigen::AutoDiffScalar<Eigen::VectorXd>> &self,
             EventCollection<UnrestrictedUpdateEvent<::drake::AutoDiffXd>> const
                 &events,
             State<Eigen::AutoDiffScalar<Eigen::VectorXd>> *state,
             Context<Eigen::AutoDiffScalar<Eigen::VectorXd>> *context) {
            return self.ApplyUnrestrictedUpdate(events, state, context);
          })
      .def("CalcConservativePower",
           [](System<Eigen::AutoDiffScalar<Eigen::VectorXd>> &self,
              Context<Eigen::AutoDiffScalar<Eigen::VectorXd>> const &context) {
             return self.CalcConservativePower(context);
           })
      .def("CalcDiscreteVariableUpdates",
           [](System<Eigen::AutoDiffScalar<Eigen::VectorXd>> &self,
              Context<Eigen::AutoDiffScalar<Eigen::VectorXd>> const &context,
              EventCollection<DiscreteUpdateEvent<::drake::AutoDiffXd>> const
                  &events,
              DiscreteValues<Eigen::AutoDiffScalar<Eigen::VectorXd>>
                  *discrete_state) {
             return self.CalcDiscreteVariableUpdates(context, events,
                                                     discrete_state);
           })
      .def("CalcDiscreteVariableUpdates",
           [](System<Eigen::AutoDiffScalar<Eigen::VectorXd>> &self,
              Context<Eigen::AutoDiffScalar<Eigen::VectorXd>> const &context,
              DiscreteValues<Eigen::AutoDiffScalar<Eigen::VectorXd>>
                  *discrete_state) {
             return self.CalcDiscreteVariableUpdates(context, discrete_state);
           })
      .def("CalcImplicitTimeDerivativesResidual",
           [](System<Eigen::AutoDiffScalar<Eigen::VectorXd>> &self,
              Context<Eigen::AutoDiffScalar<Eigen::VectorXd>> const &context,
              ContinuousState<Eigen::AutoDiffScalar<Eigen::VectorXd>> const
                  &proposed_derivatives,
              Eigen::Ref<
                  ::drake::EigenPtr<Eigen::Matrix<
                      Eigen::AutoDiffScalar<Eigen::VectorXd>, -1, 1, 0, -1, 1>>,
                  0, Eigen::Stride<Eigen::Dynamic, Eigen::Dynamic>>
                  residual) {
             return self.CalcImplicitTimeDerivativesResidual(
                 context, proposed_derivatives, residual);
           })
      .def("CalcKineticEnergy",
           [](System<Eigen::AutoDiffScalar<Eigen::VectorXd>> &self,
              Context<Eigen::AutoDiffScalar<Eigen::VectorXd>> const &context) {
             return self.CalcKineticEnergy(context);
           })
      .def("CalcNextUpdateTime",
           [](System<Eigen::AutoDiffScalar<Eigen::VectorXd>> &self,
              Context<Eigen::AutoDiffScalar<Eigen::VectorXd>> const &context,
              CompositeEventCollection<Eigen::AutoDiffScalar<Eigen::VectorXd>>
                  *events) { return self.CalcNextUpdateTime(context, events); })
      .def("CalcNonConservativePower",
           [](System<Eigen::AutoDiffScalar<Eigen::VectorXd>> &self,
              Context<Eigen::AutoDiffScalar<Eigen::VectorXd>> const &context) {
             return self.CalcNonConservativePower(context);
           })
      .def("CalcOutput",
           [](System<Eigen::AutoDiffScalar<Eigen::VectorXd>> &self,
              Context<Eigen::AutoDiffScalar<Eigen::VectorXd>> const &context,
              SystemOutput<Eigen::AutoDiffScalar<Eigen::VectorXd>> *outputs) {
             return self.CalcOutput(context, outputs);
           })
      .def("CalcPotentialEnergy",
           [](System<Eigen::AutoDiffScalar<Eigen::VectorXd>> &self,
              Context<Eigen::AutoDiffScalar<Eigen::VectorXd>> const &context) {
             return self.CalcPotentialEnergy(context);
           })
      .def("CalcTimeDerivatives",
           [](System<Eigen::AutoDiffScalar<Eigen::VectorXd>> &self,
              Context<Eigen::AutoDiffScalar<Eigen::VectorXd>> const &context,
              ContinuousState<Eigen::AutoDiffScalar<Eigen::VectorXd>>
                  *derivatives) {
             return self.CalcTimeDerivatives(context, derivatives);
           })
      .def(
          "CalcUnrestrictedUpdate",
          [](System<Eigen::AutoDiffScalar<Eigen::VectorXd>> &self,
             Context<Eigen::AutoDiffScalar<Eigen::VectorXd>> const &context,
             EventCollection<UnrestrictedUpdateEvent<::drake::AutoDiffXd>> const
                 &events,
             State<Eigen::AutoDiffScalar<Eigen::VectorXd>> *state) {
            return self.CalcUnrestrictedUpdate(context, events, state);
          })
      .def("CalcUnrestrictedUpdate",
           [](System<Eigen::AutoDiffScalar<Eigen::VectorXd>> &self,
              Context<Eigen::AutoDiffScalar<Eigen::VectorXd>> const &context,
              State<Eigen::AutoDiffScalar<Eigen::VectorXd>> *state) {
             return self.CalcUnrestrictedUpdate(context, state);
           })
      .def("CalcWitnessValue",
           [](System<Eigen::AutoDiffScalar<Eigen::VectorXd>> &self,
              Context<Eigen::AutoDiffScalar<Eigen::VectorXd>> const &context,
              WitnessFunction<Eigen::AutoDiffScalar<Eigen::VectorXd>> const
                  &witness_func) {
             return self.CalcWitnessValue(context, witness_func);
           })
      .def("CheckSystemConstraintsSatisfied",
           [](System<Eigen::AutoDiffScalar<Eigen::VectorXd>> &self,
              Context<Eigen::AutoDiffScalar<Eigen::VectorXd>> const &context,
              double tol) {
             return self.CheckSystemConstraintsSatisfied(context, tol);
           })
      .def("CheckValidOutput",
           [](System<Eigen::AutoDiffScalar<Eigen::VectorXd>> &self,
              SystemOutput<Eigen::AutoDiffScalar<Eigen::VectorXd>> const
                  *output) { return self.CheckValidOutput(output); })
      .def("CopyContinuousStateVector",
           [](System<Eigen::AutoDiffScalar<Eigen::VectorXd>> &self,
              Context<Eigen::AutoDiffScalar<Eigen::VectorXd>> const &context) {
             return self.CopyContinuousStateVector(context);
           })
      .def("CreateDefaultContext",
           static_cast<::std::unique_ptr<
               Context<::drake::AutoDiffXd>,
               std::default_delete<Context<::drake::AutoDiffXd>>> (
               System<Eigen::AutoDiffScalar<Eigen::VectorXd>>::*)() const>(
               &System<Eigen::AutoDiffScalar<Eigen::VectorXd>>::
                   CreateDefaultContext))
      .def("DeclareInputPort",
           static_cast<InputPort<Eigen::AutoDiffScalar<Eigen::VectorXd>> &(
               System<Eigen::AutoDiffScalar<Eigen::VectorXd>>::
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
               ::std::optional<drake::RandomDistribution>(std::nullopt),
           py::return_value_policy::reference_internal)
      .def("DeclareInputPort",
           static_cast<InputPort<Eigen::AutoDiffScalar<Eigen::VectorXd>> &(
               System<Eigen::AutoDiffScalar<Eigen::VectorXd>>::
                   *)(PortDataType, int,
                      ::std::optional<drake::RandomDistribution>)>(
               &System_Eigen_AutoDiffScalar_Eigen_VectorXd_publicist::
                   DeclareInputPort),
           py::arg("type"), py::arg("size"),
           py::arg("random_type") =
               ::std::optional<drake::RandomDistribution>(std::nullopt),
           py::return_value_policy::reference_internal)
      .def("DispatchDiscreteVariableUpdateHandler",
           [](System<Eigen::AutoDiffScalar<Eigen::VectorXd>> &self,
              Context<Eigen::AutoDiffScalar<Eigen::VectorXd>> const &context,
              EventCollection<DiscreteUpdateEvent<::drake::AutoDiffXd>> const
                  &events,
              DiscreteValues<Eigen::AutoDiffScalar<Eigen::VectorXd>>
                  *discrete_state) {
             return self.DispatchDiscreteVariableUpdateHandler(context, events,
                                                               discrete_state);
           })
      .def(
          "DispatchPublishHandler",
          [](System<Eigen::AutoDiffScalar<Eigen::VectorXd>> &self,
             Context<Eigen::AutoDiffScalar<Eigen::VectorXd>> const &context,
             EventCollection<PublishEvent<::drake::AutoDiffXd>> const &events) {
            return self.DispatchPublishHandler(context, events);
          })
      .def(
          "DispatchUnrestrictedUpdateHandler",
          [](System<Eigen::AutoDiffScalar<Eigen::VectorXd>> &self,
             Context<Eigen::AutoDiffScalar<Eigen::VectorXd>> const &context,
             EventCollection<UnrestrictedUpdateEvent<::drake::AutoDiffXd>> const
                 &events,
             State<Eigen::AutoDiffScalar<Eigen::VectorXd>> *state) {
            return self.DispatchUnrestrictedUpdateHandler(context, events,
                                                          state);
          })
      .def("DoApplyDiscreteVariableUpdate",
           [](System<Eigen::AutoDiffScalar<Eigen::VectorXd>> &self,
              EventCollection<DiscreteUpdateEvent<::drake::AutoDiffXd>> const
                  &events,
              DiscreteValues<Eigen::AutoDiffScalar<Eigen::VectorXd>>
                  *discrete_state,
              Context<Eigen::AutoDiffScalar<Eigen::VectorXd>> *context) {
             return self.DoApplyDiscreteVariableUpdate(events, discrete_state,
                                                       context);
           })
      .def(
          "DoApplyUnrestrictedUpdate",
          [](System<Eigen::AutoDiffScalar<Eigen::VectorXd>> &self,
             EventCollection<UnrestrictedUpdateEvent<::drake::AutoDiffXd>> const
                 &events,
             State<Eigen::AutoDiffScalar<Eigen::VectorXd>> *state,
             Context<Eigen::AutoDiffScalar<Eigen::VectorXd>> *context) {
            return self.DoApplyUnrestrictedUpdate(events, state, context);
          })
      .def("DoCalcConservativePower",
           [](System<Eigen::AutoDiffScalar<Eigen::VectorXd>> &self,
              Context<Eigen::AutoDiffScalar<Eigen::VectorXd>> const &context) {
             return self.DoCalcConservativePower(context);
           })
      .def("DoCalcImplicitTimeDerivativesResidual",
           [](System<Eigen::AutoDiffScalar<Eigen::VectorXd>> &self,
              Context<Eigen::AutoDiffScalar<Eigen::VectorXd>> const &context,
              ContinuousState<Eigen::AutoDiffScalar<Eigen::VectorXd>> const
                  &proposed_derivatives,
              Eigen::Ref<
                  ::drake::EigenPtr<Eigen::Matrix<
                      Eigen::AutoDiffScalar<Eigen::VectorXd>, -1, 1, 0, -1, 1>>,
                  0, Eigen::Stride<Eigen::Dynamic, Eigen::Dynamic>>
                  residual) {
             return self.DoCalcImplicitTimeDerivativesResidual(
                 context, proposed_derivatives, residual);
           })
      .def("DoCalcKineticEnergy",
           [](System<Eigen::AutoDiffScalar<Eigen::VectorXd>> &self,
              Context<Eigen::AutoDiffScalar<Eigen::VectorXd>> const &context) {
             return self.DoCalcKineticEnergy(context);
           })
      .def("DoCalcNextUpdateTime",
           [](System<Eigen::AutoDiffScalar<Eigen::VectorXd>> &self,
              Context<Eigen::AutoDiffScalar<Eigen::VectorXd>> const &context,
              CompositeEventCollection<Eigen::AutoDiffScalar<Eigen::VectorXd>>
                  *events,
              Eigen::Ref<::Eigen::AutoDiffScalar<
                             Eigen::Matrix<double, -1, 1, 0, -1, 1>> *,
                         0, Eigen::Stride<Eigen::Dynamic, Eigen::Dynamic>>
                  time) {
             return self.DoCalcNextUpdateTime(context, events, time);
           })
      .def("DoCalcNonConservativePower",
           [](System<Eigen::AutoDiffScalar<Eigen::VectorXd>> &self,
              Context<Eigen::AutoDiffScalar<Eigen::VectorXd>> const &context) {
             return self.DoCalcNonConservativePower(context);
           })
      .def("DoCalcPotentialEnergy",
           [](System<Eigen::AutoDiffScalar<Eigen::VectorXd>> &self,
              Context<Eigen::AutoDiffScalar<Eigen::VectorXd>> const &context) {
             return self.DoCalcPotentialEnergy(context);
           })
      .def("DoCalcTimeDerivatives",
           [](System<Eigen::AutoDiffScalar<Eigen::VectorXd>> &self,
              Context<Eigen::AutoDiffScalar<Eigen::VectorXd>> const &context,
              ContinuousState<Eigen::AutoDiffScalar<Eigen::VectorXd>>
                  *derivatives) {
             return self.DoCalcTimeDerivatives(context, derivatives);
           })
      .def("DoCalcWitnessValue",
           [](System<Eigen::AutoDiffScalar<Eigen::VectorXd>> &self,
              Context<Eigen::AutoDiffScalar<Eigen::VectorXd>> const &context,
              WitnessFunction<Eigen::AutoDiffScalar<Eigen::VectorXd>> const
                  &witness_func) {
             return self.DoCalcWitnessValue(context, witness_func);
           })
      .def("DoGetInitializationEvents",
           [](System<Eigen::AutoDiffScalar<Eigen::VectorXd>> &self,
              Context<Eigen::AutoDiffScalar<Eigen::VectorXd>> const &context,
              CompositeEventCollection<Eigen::AutoDiffScalar<Eigen::VectorXd>>
                  *events) {
             return self.DoGetInitializationEvents(context, events);
           })
      .def(
          "DoGetMutableTargetSystemCompositeEventCollection",
          [](System<Eigen::AutoDiffScalar<Eigen::VectorXd>> &self,
             System<Eigen::AutoDiffScalar<Eigen::VectorXd>> const
                 &target_system,
             CompositeEventCollection<Eigen::AutoDiffScalar<Eigen::VectorXd>>
                 *events) {
            return self.DoGetMutableTargetSystemCompositeEventCollection(
                target_system, events);
          },
          py::return_value_policy::reference_internal)
      .def(
          "DoGetMutableTargetSystemState",
          [](System<Eigen::AutoDiffScalar<Eigen::VectorXd>> &self,
             System<Eigen::AutoDiffScalar<Eigen::VectorXd>> const
                 &target_system,
             State<Eigen::AutoDiffScalar<Eigen::VectorXd>> *state) {
            return self.DoGetMutableTargetSystemState(target_system, state);
          },
          py::return_value_policy::reference_internal)
      .def("DoGetPerStepEvents",
           [](System<Eigen::AutoDiffScalar<Eigen::VectorXd>> &self,
              Context<Eigen::AutoDiffScalar<Eigen::VectorXd>> const &context,
              CompositeEventCollection<Eigen::AutoDiffScalar<Eigen::VectorXd>>
                  *events) { return self.DoGetPerStepEvents(context, events); })
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
               System<Eigen::AutoDiffScalar<Eigen::VectorXd>>::*)() const>(
               &System_Eigen_AutoDiffScalar_Eigen_VectorXd_publicist::
                   DoGetPeriodicEvents))
      .def(
          "DoGetTargetSystemCompositeEventCollection",
          [](System<Eigen::AutoDiffScalar<Eigen::VectorXd>> &self,
             System<Eigen::AutoDiffScalar<Eigen::VectorXd>> const
                 &target_system,
             CompositeEventCollection<
                 Eigen::AutoDiffScalar<Eigen::VectorXd>> const *events) {
            return self.DoGetTargetSystemCompositeEventCollection(target_system,
                                                                  events);
          },
          py::return_value_policy::reference_internal)
      .def(
          "DoGetTargetSystemContext",
          [](System<Eigen::AutoDiffScalar<Eigen::VectorXd>> &self,
             System<Eigen::AutoDiffScalar<Eigen::VectorXd>> const
                 &target_system,
             Context<Eigen::AutoDiffScalar<Eigen::VectorXd>> const *context) {
            return self.DoGetTargetSystemContext(target_system, context);
          },
          py::return_value_policy::reference_internal)
      .def(
          "DoGetTargetSystemContinuousState",
          [](System<Eigen::AutoDiffScalar<Eigen::VectorXd>> &self,
             System<Eigen::AutoDiffScalar<Eigen::VectorXd>> const
                 &target_system,
             ContinuousState<Eigen::AutoDiffScalar<Eigen::VectorXd>> const
                 *xc) {
            return self.DoGetTargetSystemContinuousState(target_system, xc);
          },
          py::return_value_policy::reference_internal)
      .def(
          "DoGetTargetSystemState",
          [](System<Eigen::AutoDiffScalar<Eigen::VectorXd>> &self,
             System<Eigen::AutoDiffScalar<Eigen::VectorXd>> const
                 &target_system,
             State<Eigen::AutoDiffScalar<Eigen::VectorXd>> const *state) {
            return self.DoGetTargetSystemState(target_system, state);
          },
          py::return_value_policy::reference_internal)
      .def("DoGetWitnessFunctions",
           [](System<Eigen::AutoDiffScalar<Eigen::VectorXd>> &self,
              Context<Eigen::AutoDiffScalar<Eigen::VectorXd>> const &arg0,
              ::std::vector<
                  const WitnessFunction<::drake::AutoDiffXd> *,
                  std::allocator<const WitnessFunction<::drake::AutoDiffXd> *>>
                  *arg1) { return self.DoGetWitnessFunctions(arg0, arg1); })
      .def("DoMapQDotToVelocity",
           [](System<Eigen::AutoDiffScalar<Eigen::VectorXd>> &self,
              Context<Eigen::AutoDiffScalar<Eigen::VectorXd>> const &context,
              ::Eigen::Ref<
                  const Eigen::Matrix<Eigen::AutoDiffScalar<Eigen::VectorXd>,
                                      -1, 1, 0, -1, 1>,
                  0, Eigen::InnerStride<1>> const &qdot,
              VectorBase<Eigen::AutoDiffScalar<Eigen::VectorXd>>
                  *generalized_velocity) {
             return self.DoMapQDotToVelocity(context, qdot,
                                             generalized_velocity);
           })
      .def("DoMapVelocityToQDot",
           [](System<Eigen::AutoDiffScalar<Eigen::VectorXd>> &self,
              Context<Eigen::AutoDiffScalar<Eigen::VectorXd>> const &context,
              ::Eigen::Ref<
                  const Eigen::Matrix<Eigen::AutoDiffScalar<Eigen::VectorXd>,
                                      -1, 1, 0, -1, 1>,
                  0, Eigen::InnerStride<1>> const &generalized_velocity,
              VectorBase<Eigen::AutoDiffScalar<Eigen::VectorXd>> *qdot) {
             return self.DoMapVelocityToQDot(context, generalized_velocity,
                                             qdot);
           })
      .def(
          "EvalConservativePower",
          [](System<Eigen::AutoDiffScalar<Eigen::VectorXd>> &self,
             Context<Eigen::AutoDiffScalar<Eigen::VectorXd>> const &context) {
            return self.EvalConservativePower(context);
          },
          py::return_value_policy::reference_internal)
      .def("EvalEigenVectorInput",
           [](System<Eigen::AutoDiffScalar<Eigen::VectorXd>> &self,
              Context<Eigen::AutoDiffScalar<Eigen::VectorXd>> const &context,
              int port_index) {
             return self.EvalEigenVectorInput(context, port_index);
           })
      .def(
          "EvalKineticEnergy",
          [](System<Eigen::AutoDiffScalar<Eigen::VectorXd>> &self,
             Context<Eigen::AutoDiffScalar<Eigen::VectorXd>> const &context) {
            return self.EvalKineticEnergy(context);
          },
          py::return_value_policy::reference_internal)
      .def(
          "EvalNonConservativePower",
          [](System<Eigen::AutoDiffScalar<Eigen::VectorXd>> &self,
             Context<Eigen::AutoDiffScalar<Eigen::VectorXd>> const &context) {
            return self.EvalNonConservativePower(context);
          },
          py::return_value_policy::reference_internal)
      .def(
          "EvalPotentialEnergy",
          [](System<Eigen::AutoDiffScalar<Eigen::VectorXd>> &self,
             Context<Eigen::AutoDiffScalar<Eigen::VectorXd>> const &context) {
            return self.EvalPotentialEnergy(context);
          },
          py::return_value_policy::reference_internal)
      .def(
          "EvalTimeDerivatives",
          [](System<Eigen::AutoDiffScalar<Eigen::VectorXd>> &self,
             Context<Eigen::AutoDiffScalar<Eigen::VectorXd>> const &context) {
            return self.EvalTimeDerivatives(context);
          },
          py::return_value_policy::reference_internal)
      .def("FixInputPortsFrom",
           [](System<Eigen::AutoDiffScalar<Eigen::VectorXd>> &self,
              System<double> const &other_system,
              Context<double> const &other_context,
              Context<Eigen::AutoDiffScalar<Eigen::VectorXd>> *target_context) {
             return self.FixInputPortsFrom(other_system, other_context,
                                           target_context);
           })
      .def(
          "GetGraphvizFragment",
          static_cast<void (System<Eigen::AutoDiffScalar<Eigen::VectorXd>>::*)(
              int, ::std::stringstream *) const>(
              &System<
                  Eigen::AutoDiffScalar<Eigen::VectorXd>>::GetGraphvizFragment),
          py::arg("max_depth"), py::arg("dot"))
      .def("GetGraphvizId",
           static_cast<::int64_t (
               System<Eigen::AutoDiffScalar<Eigen::VectorXd>>::*)() const>(
               &System<Eigen::AutoDiffScalar<Eigen::VectorXd>>::GetGraphvizId))
      .def("GetGraphvizInputPortToken",
           [](System<Eigen::AutoDiffScalar<Eigen::VectorXd>> &self,
              InputPort<Eigen::AutoDiffScalar<Eigen::VectorXd>> const &port,
              int max_depth, ::std::stringstream *dot) {
             return self.GetGraphvizInputPortToken(port, max_depth, dot);
           })
      .def("GetGraphvizOutputPortToken",
           [](System<Eigen::AutoDiffScalar<Eigen::VectorXd>> &self,
              OutputPort<Eigen::AutoDiffScalar<Eigen::VectorXd>> const &port,
              int max_depth, ::std::stringstream *dot) {
             return self.GetGraphvizOutputPortToken(port, max_depth, dot);
           })
      .def("GetGraphvizString",
           static_cast<::std::string (
               System<Eigen::AutoDiffScalar<Eigen::VectorXd>>::*)(int) const>(
               &System<
                   Eigen::AutoDiffScalar<Eigen::VectorXd>>::GetGraphvizString),
           py::arg("max_depth") = int(std::numeric_limits<int>::max()))
      .def("GetInitializationEvents",
           [](System<Eigen::AutoDiffScalar<Eigen::VectorXd>> &self,
              Context<Eigen::AutoDiffScalar<Eigen::VectorXd>> const &context,
              CompositeEventCollection<Eigen::AutoDiffScalar<Eigen::VectorXd>>
                  *events) {
             return self.GetInitializationEvents(context, events);
           })
      .def(
          "GetInputPort",
          static_cast<InputPort<Eigen::AutoDiffScalar<Eigen::VectorXd>> const &(
              System<Eigen::AutoDiffScalar<Eigen::VectorXd>>::
                  *)(::std::string const &)const>(
              &System<Eigen::AutoDiffScalar<Eigen::VectorXd>>::GetInputPort),
          py::arg("port_name"), py::return_value_policy::reference_internal)
      .def(
          "GetMemoryObjectName",
          static_cast<::std::string (
              System<Eigen::AutoDiffScalar<Eigen::VectorXd>>::*)() const>(
              &System<
                  Eigen::AutoDiffScalar<Eigen::VectorXd>>::GetMemoryObjectName))
      .def("GetMutableOutputVector",
           [](System<Eigen::AutoDiffScalar<Eigen::VectorXd>> &self,
              SystemOutput<Eigen::AutoDiffScalar<Eigen::VectorXd>> *output,
              int port_index) {
             return self.GetMutableOutputVector(output, port_index);
           })
      .def(
          "GetMutableSubsystemContext",
          [](System<Eigen::AutoDiffScalar<Eigen::VectorXd>> &self,
             System<Eigen::AutoDiffScalar<Eigen::VectorXd>> const &subsystem,
             Context<Eigen::AutoDiffScalar<Eigen::VectorXd>> *context) {
            return self.GetMutableSubsystemContext(subsystem, context);
          },
          py::return_value_policy::reference_internal)
      .def(
          "GetMyContextFromRoot",
          [](System<Eigen::AutoDiffScalar<Eigen::VectorXd>> &self,
             Context<Eigen::AutoDiffScalar<Eigen::VectorXd>> const
                 &root_context) {
            return self.GetMyContextFromRoot(root_context);
          },
          py::return_value_policy::reference_internal)
      .def(
          "GetMyMutableContextFromRoot",
          [](System<Eigen::AutoDiffScalar<Eigen::VectorXd>> &self,
             Context<Eigen::AutoDiffScalar<Eigen::VectorXd>> *root_context) {
            return self.GetMyMutableContextFromRoot(root_context);
          },
          py::return_value_policy::reference_internal)
      .def("GetOutputPort",
           static_cast<OutputPort<Eigen::AutoDiffScalar<Eigen::VectorXd>> const
                           &(System<Eigen::AutoDiffScalar<Eigen::VectorXd>>::
                                 *)(::std::string const &)const>(
               &System<Eigen::AutoDiffScalar<Eigen::VectorXd>>::GetOutputPort),
           py::arg("port_name"), py::return_value_policy::reference_internal)
      .def("GetPerStepEvents",
           [](System<Eigen::AutoDiffScalar<Eigen::VectorXd>> &self,
              Context<Eigen::AutoDiffScalar<Eigen::VectorXd>> const &context,
              CompositeEventCollection<Eigen::AutoDiffScalar<Eigen::VectorXd>>
                  *events) { return self.GetPerStepEvents(context, events); })
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
               System<Eigen::AutoDiffScalar<Eigen::VectorXd>>::*)() const>(
               &System<
                   Eigen::AutoDiffScalar<Eigen::VectorXd>>::GetPeriodicEvents))
      .def(
          "GetSubsystemContext",
          [](System<Eigen::AutoDiffScalar<Eigen::VectorXd>> &self,
             System<Eigen::AutoDiffScalar<Eigen::VectorXd>> const &subsystem,
             Context<Eigen::AutoDiffScalar<Eigen::VectorXd>> const &context) {
            return self.GetSubsystemContext(subsystem, context);
          },
          py::return_value_policy::reference_internal)
      .def("GetUniquePeriodicDiscreteUpdateAttribute",
           static_cast<::std::optional<PeriodicEventData> (
               System<Eigen::AutoDiffScalar<Eigen::VectorXd>>::*)() const>(
               &System<Eigen::AutoDiffScalar<Eigen::VectorXd>>::
                   GetUniquePeriodicDiscreteUpdateAttribute))
      .def("GetWitnessFunctions",
           [](System<Eigen::AutoDiffScalar<Eigen::VectorXd>> &self,
              Context<Eigen::AutoDiffScalar<Eigen::VectorXd>> const &context,
              ::std::vector<
                  const WitnessFunction<::drake::AutoDiffXd> *,
                  std::allocator<const WitnessFunction<::drake::AutoDiffXd> *>>
                  *w) { return self.GetWitnessFunctions(context, w); })
      .def("HasAnyDirectFeedthrough",
           static_cast<bool (
               System<Eigen::AutoDiffScalar<Eigen::VectorXd>>::*)() const>(
               &System<Eigen::AutoDiffScalar<Eigen::VectorXd>>::
                   HasAnyDirectFeedthrough))
      .def("HasDirectFeedthrough",
           static_cast<bool (System<Eigen::AutoDiffScalar<Eigen::VectorXd>>::*)(
               int) const>(&System<Eigen::AutoDiffScalar<Eigen::VectorXd>>::
                               HasDirectFeedthrough),
           py::arg("output_port"))
      .def(
          "HasDirectFeedthrough",
          static_cast<bool (System<Eigen::AutoDiffScalar<Eigen::VectorXd>>::*)(
              int, int) const>(&System<Eigen::AutoDiffScalar<Eigen::VectorXd>>::
                                   HasDirectFeedthrough),
          py::arg("input_port"), py::arg("output_port"))
      .def("HasInputPort",
           static_cast<bool (System<Eigen::AutoDiffScalar<Eigen::VectorXd>>::*)(
               ::std::string const &) const>(
               &System<Eigen::AutoDiffScalar<Eigen::VectorXd>>::HasInputPort),
           py::arg("port_name"))
      .def("HasOutputPort",
           static_cast<bool (System<Eigen::AutoDiffScalar<Eigen::VectorXd>>::*)(
               ::std::string const &) const>(
               &System<Eigen::AutoDiffScalar<Eigen::VectorXd>>::HasOutputPort),
           py::arg("port_name"))
      .def(
          "IsDifferenceEquationSystem",
          static_cast<bool (System<Eigen::AutoDiffScalar<Eigen::VectorXd>>::*)(
              double *) const>(&System<Eigen::AutoDiffScalar<Eigen::VectorXd>>::
                                   IsDifferenceEquationSystem),
          py::arg("time_period") = (double *)nullptr)
      .def("MapQDotToVelocity",
           [](System<Eigen::AutoDiffScalar<Eigen::VectorXd>> &self,
              Context<Eigen::AutoDiffScalar<Eigen::VectorXd>> const &context,
              VectorBase<Eigen::AutoDiffScalar<Eigen::VectorXd>> const &qdot,
              VectorBase<Eigen::AutoDiffScalar<Eigen::VectorXd>>
                  *generalized_velocity) {
             return self.MapQDotToVelocity(context, qdot, generalized_velocity);
           })
      .def("MapQDotToVelocity",
           [](System<Eigen::AutoDiffScalar<Eigen::VectorXd>> &self,
              Context<Eigen::AutoDiffScalar<Eigen::VectorXd>> const &context,
              ::Eigen::Ref<
                  const Eigen::Matrix<Eigen::AutoDiffScalar<Eigen::VectorXd>,
                                      -1, 1, 0, -1, 1>,
                  0, Eigen::InnerStride<1>> const &qdot,
              VectorBase<Eigen::AutoDiffScalar<Eigen::VectorXd>>
                  *generalized_velocity) {
             return self.MapQDotToVelocity(context, qdot, generalized_velocity);
           })
      .def("MapVelocityToQDot",
           [](System<Eigen::AutoDiffScalar<Eigen::VectorXd>> &self,
              Context<Eigen::AutoDiffScalar<Eigen::VectorXd>> const &context,
              VectorBase<Eigen::AutoDiffScalar<Eigen::VectorXd>> const
                  &generalized_velocity,
              VectorBase<Eigen::AutoDiffScalar<Eigen::VectorXd>> *qdot) {
             return self.MapVelocityToQDot(context, generalized_velocity, qdot);
           })
      .def("MapVelocityToQDot",
           [](System<Eigen::AutoDiffScalar<Eigen::VectorXd>> &self,
              Context<Eigen::AutoDiffScalar<Eigen::VectorXd>> const &context,
              ::Eigen::Ref<
                  const Eigen::Matrix<Eigen::AutoDiffScalar<Eigen::VectorXd>,
                                      -1, 1, 0, -1, 1>,
                  0, Eigen::InnerStride<1>> const &generalized_velocity,
              VectorBase<Eigen::AutoDiffScalar<Eigen::VectorXd>> *qdot) {
             return self.MapVelocityToQDot(context, generalized_velocity, qdot);
           })
      .def(
          "Publish",
          [](System<Eigen::AutoDiffScalar<Eigen::VectorXd>> &self,
             Context<Eigen::AutoDiffScalar<Eigen::VectorXd>> const &context,
             EventCollection<PublishEvent<::drake::AutoDiffXd>> const &events) {
            return self.Publish(context, events);
          })
      .def("Publish",
           [](System<Eigen::AutoDiffScalar<Eigen::VectorXd>> &self,
              Context<Eigen::AutoDiffScalar<Eigen::VectorXd>> const &context) {
             return self.Publish(context);
           })
      .def("SetDefaultContext",
           [](System<Eigen::AutoDiffScalar<Eigen::VectorXd>> &self,
              Context<Eigen::AutoDiffScalar<Eigen::VectorXd>> *context) {
             return self.SetDefaultContext(context);
           })
      .def("SetDefaultParameters",
           [](System<Eigen::AutoDiffScalar<Eigen::VectorXd>> &self,
              Context<Eigen::AutoDiffScalar<Eigen::VectorXd>> const &context,
              Parameters<Eigen::AutoDiffScalar<Eigen::VectorXd>> *parameters) {
             return self.SetDefaultParameters(context, parameters);
           })
      .def("SetDefaultState",
           [](System<Eigen::AutoDiffScalar<Eigen::VectorXd>> &self,
              Context<Eigen::AutoDiffScalar<Eigen::VectorXd>> const &context,
              State<Eigen::AutoDiffScalar<Eigen::VectorXd>> *state) {
             return self.SetDefaultState(context, state);
           })
      .def("SetRandomContext",
           [](System<Eigen::AutoDiffScalar<Eigen::VectorXd>> &self,
              Context<Eigen::AutoDiffScalar<Eigen::VectorXd>> *context,
              ::drake::RandomGenerator *generator) {
             return self.SetRandomContext(context, generator);
           })
      .def("SetRandomParameters",
           [](System<Eigen::AutoDiffScalar<Eigen::VectorXd>> &self,
              Context<Eigen::AutoDiffScalar<Eigen::VectorXd>> const &context,
              Parameters<Eigen::AutoDiffScalar<Eigen::VectorXd>> *parameters,
              ::drake::RandomGenerator *generator) {
             return self.SetRandomParameters(context, parameters, generator);
           })
      .def("SetRandomState",
           [](System<Eigen::AutoDiffScalar<Eigen::VectorXd>> &self,
              Context<Eigen::AutoDiffScalar<Eigen::VectorXd>> const &context,
              State<Eigen::AutoDiffScalar<Eigen::VectorXd>> *state,
              ::drake::RandomGenerator *generator) {
             return self.SetRandomState(context, state, generator);
           })
      .def("ToAutoDiffXd",
           static_cast<::std::unique_ptr<
               System<::drake::AutoDiffXd>,
               std::default_delete<System<::drake::AutoDiffXd>>> (
               System<Eigen::AutoDiffScalar<Eigen::VectorXd>>::*)() const>(
               &System<Eigen::AutoDiffScalar<Eigen::VectorXd>>::ToAutoDiffXd))
      .def("ToAutoDiffXdMaybe",
           static_cast<::std::unique_ptr<
               System<::drake::AutoDiffXd>,
               std::default_delete<System<::drake::AutoDiffXd>>> (
               System<Eigen::AutoDiffScalar<Eigen::VectorXd>>::*)() const>(
               &System<
                   Eigen::AutoDiffScalar<Eigen::VectorXd>>::ToAutoDiffXdMaybe))
      .def("ToSymbolic",
           static_cast<::std::unique_ptr<
               System<::drake::symbolic::Expression>,
               std::default_delete<System<::drake::symbolic::Expression>>> (
               System<Eigen::AutoDiffScalar<Eigen::VectorXd>>::*)() const>(
               &System<Eigen::AutoDiffScalar<Eigen::VectorXd>>::ToSymbolic))
      .def(
          "ToSymbolicMaybe",
          static_cast<::std::unique_ptr<
              System<::drake::symbolic::Expression>,
              std::default_delete<System<::drake::symbolic::Expression>>> (
              System<Eigen::AutoDiffScalar<Eigen::VectorXd>>::*)() const>(
              &System<Eigen::AutoDiffScalar<Eigen::VectorXd>>::ToSymbolicMaybe))
      .def("forced_discrete_update_events_exist",
           static_cast<bool (
               System<Eigen::AutoDiffScalar<Eigen::VectorXd>>::*)() const>(
               &System_Eigen_AutoDiffScalar_Eigen_VectorXd_publicist::
                   forced_discrete_update_events_exist))
      .def("forced_publish_events_exist",
           static_cast<bool (
               System<Eigen::AutoDiffScalar<Eigen::VectorXd>>::*)() const>(
               &System_Eigen_AutoDiffScalar_Eigen_VectorXd_publicist::
                   forced_publish_events_exist))
      .def("forced_unrestricted_update_events_exist",
           static_cast<bool (
               System<Eigen::AutoDiffScalar<Eigen::VectorXd>>::*)() const>(
               &System_Eigen_AutoDiffScalar_Eigen_VectorXd_publicist::
                   forced_unrestricted_update_events_exist))
      .def("get_constraint",
           static_cast<
               SystemConstraint<Eigen::AutoDiffScalar<Eigen::VectorXd>> const &(
                   System<Eigen::AutoDiffScalar<Eigen::VectorXd>>::
                       *)(SystemConstraintIndex) const>(
               &System<Eigen::AutoDiffScalar<Eigen::VectorXd>>::get_constraint),
           py::arg("constraint_index"),
           py::return_value_policy::reference_internal)
      .def(
          "get_forced_discrete_update_events",
          static_cast<
              EventCollection<DiscreteUpdateEvent<::drake::AutoDiffXd>> const &(
                  System<Eigen::AutoDiffScalar<Eigen::VectorXd>>::*)() const>(
              &System_Eigen_AutoDiffScalar_Eigen_VectorXd_publicist::
                  get_forced_discrete_update_events))
      .def("get_forced_publish_events",
           static_cast<
               EventCollection<PublishEvent<::drake::AutoDiffXd>> const &(
                   System<Eigen::AutoDiffScalar<Eigen::VectorXd>>::*)() const>(
               &System_Eigen_AutoDiffScalar_Eigen_VectorXd_publicist::
                   get_forced_publish_events))
      .def(
          "get_forced_unrestricted_update_events",
          static_cast<EventCollection<
              UnrestrictedUpdateEvent<::drake::AutoDiffXd>> const
                          &(System<Eigen::AutoDiffScalar<Eigen::VectorXd>>::*)()
                              const>(
              &System_Eigen_AutoDiffScalar_Eigen_VectorXd_publicist::
                  get_forced_unrestricted_update_events))
      .def(
          "get_input_port",
          static_cast<InputPort<Eigen::AutoDiffScalar<Eigen::VectorXd>> const &(
              System<Eigen::AutoDiffScalar<Eigen::VectorXd>>::*)(int)const>(
              &System<Eigen::AutoDiffScalar<Eigen::VectorXd>>::get_input_port),
          py::arg("port_index"), py::return_value_policy::reference_internal)
      .def(
          "get_input_port",
          static_cast<InputPort<Eigen::AutoDiffScalar<Eigen::VectorXd>> const &(
              System<Eigen::AutoDiffScalar<Eigen::VectorXd>>::*)() const>(
              &System<Eigen::AutoDiffScalar<Eigen::VectorXd>>::get_input_port),
          py::return_value_policy::reference_internal)
      .def(
          "get_input_port_selection",
          static_cast<InputPort<Eigen::AutoDiffScalar<Eigen::VectorXd>> const *(
              System<Eigen::AutoDiffScalar<Eigen::VectorXd>>::
                  *)(::std::variant<InputPortSelection,
                                    drake::TypeSafeIndex<InputPortTag>>)const>(
              &System<Eigen::AutoDiffScalar<Eigen::VectorXd>>::
                  get_input_port_selection),
          py::arg("port_index"), py::return_value_policy::reference_internal)
      .def("get_mutable_forced_discrete_update_events",
           static_cast<
               EventCollection<DiscreteUpdateEvent<::drake::AutoDiffXd>> &(
                   System<Eigen::AutoDiffScalar<Eigen::VectorXd>>::*)()>(
               &System_Eigen_AutoDiffScalar_Eigen_VectorXd_publicist::
                   get_mutable_forced_discrete_update_events))
      .def("get_mutable_forced_publish_events",
           static_cast<EventCollection<PublishEvent<::drake::AutoDiffXd>> &(
               System<Eigen::AutoDiffScalar<Eigen::VectorXd>>::*)()>(
               &System_Eigen_AutoDiffScalar_Eigen_VectorXd_publicist::
                   get_mutable_forced_publish_events))
      .def("get_mutable_forced_unrestricted_update_events",
           static_cast<
               EventCollection<UnrestrictedUpdateEvent<::drake::AutoDiffXd>> &(
                   System<Eigen::AutoDiffScalar<Eigen::VectorXd>>::*)()>(
               &System_Eigen_AutoDiffScalar_Eigen_VectorXd_publicist::
                   get_mutable_forced_unrestricted_update_events))
      .def(
          "get_output_port",
          static_cast<
              OutputPort<Eigen::AutoDiffScalar<Eigen::VectorXd>> const &(
                  System<Eigen::AutoDiffScalar<Eigen::VectorXd>>::*)(int)const>(
              &System<Eigen::AutoDiffScalar<Eigen::VectorXd>>::get_output_port),
          py::arg("port_index"), py::return_value_policy::reference_internal)
      .def(
          "get_output_port",
          static_cast<
              OutputPort<Eigen::AutoDiffScalar<Eigen::VectorXd>> const &(
                  System<Eigen::AutoDiffScalar<Eigen::VectorXd>>::*)() const>(
              &System<Eigen::AutoDiffScalar<Eigen::VectorXd>>::get_output_port),
          py::return_value_policy::reference_internal)
      .def("get_output_port_selection",
           static_cast<
               OutputPort<Eigen::AutoDiffScalar<Eigen::VectorXd>> const *(
                   System<Eigen::AutoDiffScalar<Eigen::VectorXd>>::
                       *)(::std::variant<OutputPortSelection,
                                         drake::TypeSafeIndex<OutputPortTag>>)
                   const>(&System<Eigen::AutoDiffScalar<Eigen::VectorXd>>::
                              get_output_port_selection),
           py::arg("port_index"), py::return_value_policy::reference_internal)
      .def("get_system_scalar_converter",
           static_cast<SystemScalarConverter const &(
               System<Eigen::AutoDiffScalar<Eigen::VectorXd>>::*)() const>(
               &System<Eigen::AutoDiffScalar<Eigen::VectorXd>>::
                   get_system_scalar_converter))
      .def("get_time_derivatives_cache_entry",
           static_cast<CacheEntry const &(
               System<Eigen::AutoDiffScalar<Eigen::VectorXd>>::*)() const>(
               &System<Eigen::AutoDiffScalar<Eigen::VectorXd>>::
                   get_time_derivatives_cache_entry))
      .def(
          "num_constraints",
          static_cast<int (System<Eigen::AutoDiffScalar<Eigen::VectorXd>>::*)()
                          const>(
              &System<Eigen::AutoDiffScalar<Eigen::VectorXd>>::num_constraints))
      .def(
          "set_forced_discrete_update_events",
          [](System<Eigen::AutoDiffScalar<Eigen::VectorXd>> &self,
             EventCollection<DiscreteUpdateEvent<::drake::AutoDiffXd>> forced) {
            self.set_forced_discrete_update_events(
                std::make_unique<
                    EventCollection<DiscreteUpdateEvent<::drake::AutoDiffXd>>>(
                    forced));
          })
      .def("set_forced_publish_events",
           [](System<Eigen::AutoDiffScalar<Eigen::VectorXd>> &self,
              EventCollection<PublishEvent<::drake::AutoDiffXd>> forced) {
             self.set_forced_publish_events(
                 std::make_unique<
                     EventCollection<PublishEvent<::drake::AutoDiffXd>>>(
                     forced));
           })
      .def("set_forced_unrestricted_update_events",
           [](System<Eigen::AutoDiffScalar<Eigen::VectorXd>> &self,
              EventCollection<UnrestrictedUpdateEvent<::drake::AutoDiffXd>>
                  forced) {
             self.set_forced_unrestricted_update_events(
                 std::make_unique<EventCollection<
                     UnrestrictedUpdateEvent<::drake::AutoDiffXd>>>(forced));
           })

      ;
}
