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
      .def(
          "AddConstraint",
          [](System<PySystem_double_0> &self,
             SystemConstraint<PySystem_double_0> constraint) {
            return self.AddConstraint(
                std::make_unique<SystemConstraint<PySystem_double_0>>(
                    constraint));
          },
          R"""(/** Adds an already-created constraint to the list of constraints for this 
System.  Ownership of the SystemConstraint is transferred to this system. */)""")
      .def("AddExternalConstraint",
           static_cast<SystemConstraintIndex (System<PySystem_double_0>::*)(
               ExternalSystemConstraint)>(
               &System<PySystem_double_0>::AddExternalConstraint),
           py::arg("constraint"),
           R"""(/** Adds an "external" constraint to this System. 
 
This method is intended for use by applications that are examining this 
System to add additional constraints based on their particular situation 
(e.g., that a velocity state element has an upper bound); it is not 
intended for declaring intrinsic constraints that some particular System 
subclass might always impose on itself (e.g., that a mass parameter is 
non-negative).  To that end, this method should not be called by 
subclasses of `this` during their constructor. 
 
The `constraint` will automatically persist across system scalar 
conversion. */)""")
      .def(
          "AddTriggeredWitnessFunctionToCompositeEventCollection",
          static_cast<void (System<PySystem_double_0>::*)(
              Event<PySystem_double_0> *,
              CompositeEventCollection<PySystem_double_0> *) const>(
              &System<PySystem_double_0>::
                  AddTriggeredWitnessFunctionToCompositeEventCollection),
          py::arg("event"), py::arg("events"),
          R"""(/** Add `event` to `events` due to a witness function triggering. `events` 
should be allocated with this system's AllocateCompositeEventCollection. 
Neither `event` nor `events` can be nullptr. Additionally, `event` must 
contain event data (event->get_event_data() must not be nullptr) and 
the type of that data must be WitnessTriggeredEventData. */)""")
      .def(
          "AllocateCompositeEventCollection",
          static_cast<::std::unique_ptr<
              CompositeEventCollection<PySystem_double_0>,
              std::default_delete<CompositeEventCollection<
                  PySystem_double_0>>> (System<PySystem_double_0>::*)() const>(
              &System<PySystem_double_0>::AllocateCompositeEventCollection),
          R"""(/** Allocates a CompositeEventCollection for this system. The allocated 
instance is used for populating collections of triggered events; for 
example, Simulator passes this object to System::CalcNextUpdateTime() to 
allow the system to identify and handle upcoming events. */)""")
      .def(
          "AllocateContext",
          static_cast<::std::unique_ptr<
              Context<PySystem_double_0>,
              std::default_delete<Context<PySystem_double_0>>> (
              System<PySystem_double_0>::*)() const>(
              &System<PySystem_double_0>::AllocateContext),
          R"""(/** Returns a Context<T> suitable for use with this System<T>. */)""")
      .def(
          "AllocateDiscreteVariables",
          static_cast<::std::unique_ptr<
              DiscreteValues<PySystem_double_0>,
              std::default_delete<DiscreteValues<PySystem_double_0>>> (
              System<PySystem_double_0>::*)() const>(
              &System<PySystem_double_0>::AllocateDiscreteVariables),
          R"""(/** Returns a DiscreteValues of the same dimensions as the discrete_state 
allocated in CreateDefaultContext. The simulator will provide this state 
as the output argument to Update. */)""")
      .def(
          "AllocateFixedInputs",
          static_cast<void (System<PySystem_double_0>::*)(
              Context<PySystem_double_0> *) const>(
              &System<PySystem_double_0>::AllocateFixedInputs),
          py::arg("context"),
          R"""(/** For each input port, allocates a fixed input of the concrete type 
that this System requires, and binds it to the port, disconnecting any 
prior input. Does not assign any values to the fixed inputs. */)""")
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
      .def(
          "AllocateImplicitTimeDerivativesResidual",
          static_cast<::Eigen::Matrix<double, -1, 1, 0, -1, 1> (
              System<PySystem_double_0>::*)() const>(
              &System<
                  PySystem_double_0>::AllocateImplicitTimeDerivativesResidual),
          R"""(/** Returns an Eigen VectorX suitable for use as the output argument to 
the CalcImplicitTimeDerivativesResidual() method. The returned VectorX 
will have size implicit_time_derivatives_residual_size() with the 
elements uninitialized. This is just a convenience method -- you are free 
to use any properly-sized mutable Eigen object as the residual vector. */)""")
      .def(
          "AllocateInputAbstract",
          static_cast<::std::unique_ptr<
              drake::AbstractValue, std::default_delete<drake::AbstractValue>> (
              System<PySystem_double_0>::*)(
              InputPort<PySystem_double_0> const &) const>(
              &System<PySystem_double_0>::AllocateInputAbstract),
          py::arg("input_port"),
          R"""(/** Given an input port, allocates the abstract storage.  The @p input_port 
must match a port declared via DeclareInputPort. */)""")
      .def(
          "AllocateInputVector",
          static_cast<::std::unique_ptr<
              BasicVector<PySystem_double_0>,
              std::default_delete<BasicVector<PySystem_double_0>>> (
              System<PySystem_double_0>::*)(
              InputPort<PySystem_double_0> const &) const>(
              &System<PySystem_double_0>::AllocateInputVector),
          py::arg("input_port"),
          R"""(/** Given an input port, allocates the vector storage.  The @p input_port 
must match a port declared via DeclareInputPort. */)""")
      .def(
          "AllocateOutput",
          static_cast<::std::unique_ptr<
              SystemOutput<PySystem_double_0>,
              std::default_delete<SystemOutput<PySystem_double_0>>> (
              System<PySystem_double_0>::*)() const>(
              &System<PySystem_double_0>::AllocateOutput),
          R"""(/** Returns a container that can hold the values of all of this System's 
output ports. It is sized with the number of output ports and uses each 
output port's allocation method to provide an object of the right type 
for that port. */)""")
      .def(
          "AllocateTimeDerivatives",
          static_cast<::std::unique_ptr<
              ContinuousState<PySystem_double_0>,
              std::default_delete<ContinuousState<PySystem_double_0>>> (
              System<PySystem_double_0>::*)() const>(
              &System<PySystem_double_0>::AllocateTimeDerivatives),
          R"""(/** Returns a ContinuousState of the same size as the continuous_state 
allocated in CreateDefaultContext. The simulator will provide this state 
as the output argument to EvalTimeDerivatives. */)""")
      .def("ApplyDiscreteVariableUpdate",
           static_cast<void (System<PySystem_double_0>::*)(
               EventCollection<DiscreteUpdateEvent<PySystem_double_0>> const &,
               DiscreteValues<PySystem_double_0> *,
               Context<PySystem_double_0> *) const>(
               &System<PySystem_double_0>::ApplyDiscreteVariableUpdate),
           py::arg("events"), py::arg("discrete_state"), py::arg("context"),
           R"""(/** Given the @p discrete_state results of a previous call to 
CalcDiscreteVariableUpdates() that dispatched the given collection of 
events, modifies the @p context to reflect the updated @p discrete_state. 
@param[in] events 
    The Event collection that resulted in the given @p discrete_state. 
@param[in,out] discrete_state 
    The updated discrete state from a CalcDiscreteVariableUpdates() 
    call. This is mutable to permit its contents to be swapped with the 
    corresponding @p context contents (rather than copied). 
@param[in,out] context 
    The Context whose discrete state is modified to match 
    @p discrete_state. Note that swapping contents with @p discrete_state 
    may cause addresses of individual discrete state group vectors in 
    @p context to be different on return than they were on entry. 
@pre @p discrete_state is the result of a previous 
     CalcDiscreteVariableUpdates() call that dispatched this @p events 
     collection. */)""")
      .def(
          "ApplyUnrestrictedUpdate",
          static_cast<void (System<PySystem_double_0>::*)(
              EventCollection<UnrestrictedUpdateEvent<PySystem_double_0>> const
                  &,
              State<PySystem_double_0> *, Context<PySystem_double_0> *) const>(
              &System<PySystem_double_0>::ApplyUnrestrictedUpdate),
          py::arg("events"), py::arg("state"), py::arg("context"),
          R"""(/** Given the @p state results of a previous call to CalcUnrestrictedUpdate() 
that dispatched the given collection of events, modifies the @p context to 
reflect the updated @p state. 
@param[in] events 
    The Event collection that resulted in the given @p state. 
@param[in,out] state 
    The updated State from a CalcUnrestrictedUpdate() call. This is 
    mutable to permit its contents to be swapped with the corresponding 
    @p context contents (rather than copied). 
@param[in,out] context 
    The Context whose State is modified to match @p state. Note that 
    swapping contents with the @p state may cause addresses of 
    continuous, discrete, and abstract state containers in @p context 
    to be different on return than they were on entry. 
@pre @p state is the result of a previous CalcUnrestrictedUpdate() call 
     that dispatched this @p events collection. */)""")
      .def(
          "CalcConservativePower",
          static_cast<double (System<PySystem_double_0>::*)(
              Context<PySystem_double_0> const &) const>(
              &System<PySystem_double_0>::CalcConservativePower),
          py::arg("context"),
          R"""(/** Calculates and returns the conservative power represented by the current 
contents of the given `context`. Prefer EvalConservativePower() to avoid 
unnecessary recalculation. 
 
@see EvalConservativePower() for more information. */)""")
      .def(
          "CalcDiscreteVariableUpdates",
          static_cast<void (System<PySystem_double_0>::*)(
              Context<PySystem_double_0> const &,
              EventCollection<DiscreteUpdateEvent<PySystem_double_0>> const &,
              DiscreteValues<PySystem_double_0> *) const>(
              &System<PySystem_double_0>::CalcDiscreteVariableUpdates),
          py::arg("context"), py::arg("events"), py::arg("discrete_state"),
          R"""(/** This method is the public entry point for dispatching all discrete 
variable update event handlers. Using all the discrete update handlers in 
@p events, the method calculates the update `xd(n+1)` to discrete 
variables `xd(n)` in @p context and outputs the results to @p 
discrete_state. See documentation for 
DispatchDiscreteVariableUpdateHandler() for more details. */)""")
      .def(
          "CalcDiscreteVariableUpdates",
          static_cast<void (System<PySystem_double_0>::*)(
              Context<PySystem_double_0> const &,
              DiscreteValues<PySystem_double_0> *) const>(
              &System<PySystem_double_0>::CalcDiscreteVariableUpdates),
          py::arg("context"), py::arg("discrete_state"),
          R"""(/** This method forces a discrete update on the system given a @p context, 
and the updated discrete state is stored in @p discrete_state. The 
discrete update event will have a trigger type of kForced, with no 
attribute or custom callback. */)""")
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
      .def(
          "CalcKineticEnergy",
          static_cast<double (System<PySystem_double_0>::*)(
              Context<PySystem_double_0> const &) const>(
              &System<PySystem_double_0>::CalcKineticEnergy),
          py::arg("context"),
          R"""(/** Calculates and returns the kinetic energy represented by the current 
configuration and velocity provided in `context`. Prefer 
EvalKineticEnergy() to avoid unnecessary recalculation. 
 
@see EvalKineticEnergy() for more information. */)""")
      .def(
          "CalcNextUpdateTime",
          static_cast<double (System<PySystem_double_0>::*)(
              Context<PySystem_double_0> const &,
              CompositeEventCollection<PySystem_double_0> *) const>(
              &System<PySystem_double_0>::CalcNextUpdateTime),
          py::arg("context"), py::arg("events"),
          R"""(/** This method is called by a Simulator during its calculation of the size of 
the next continuous step to attempt. The System returns the next time at 
which some discrete action must be taken, and records what those actions 
ought to be in @p events. Upon reaching that time, the simulator will 
merge @p events with the other CompositeEventCollection instances 
triggered through other mechanisms (e.g. GetPerStepEvents()), and the 
merged CompositeEventCollection will be passed to all event handling 
mechanisms. 
 
If there is no timed event coming, the return value is Infinity. If 
a finite update time is returned, there will be at least one Event object 
in the returned event collection. 
 
@p events cannot be null. @p events will be cleared on entry. */)""")
      .def(
          "CalcNonConservativePower",
          static_cast<double (System<PySystem_double_0>::*)(
              Context<PySystem_double_0> const &) const>(
              &System<PySystem_double_0>::CalcNonConservativePower),
          py::arg("context"),
          R"""(/** Calculates and returns the non-conservative power represented by the 
current contents of the given `context`. Prefer EvalNonConservativePower() 
to avoid unnecessary recalculation. 
 
@see EvalNonConservativePower() for more information. */)""")
      .def(
          "CalcOutput",
          static_cast<void (System<PySystem_double_0>::*)(
              Context<PySystem_double_0> const &,
              SystemOutput<PySystem_double_0> *) const>(
              &System<PySystem_double_0>::CalcOutput),
          py::arg("context"), py::arg("outputs"),
          R"""(/** Utility method that computes for _every_ output port i the value y(i) that 
should result from the current contents of the given Context. Note that 
individual output port values can be calculated using 
`get_output_port(i).Calc()`; this method invokes that for each output port 
in index order. The result may depend on time and the current values of 
input ports, parameters, and state variables. The result is written to 
`outputs` which must already have been allocated to have the right number 
of entries of the right types. */)""")
      .def(
          "CalcPotentialEnergy",
          static_cast<double (System<PySystem_double_0>::*)(
              Context<PySystem_double_0> const &) const>(
              &System<PySystem_double_0>::CalcPotentialEnergy),
          py::arg("context"),
          R"""(/** Calculates and returns the potential energy represented by the current 
configuration provided in `context`. Prefer EvalPotentialEnergy() to 
avoid unnecessary recalculation. 
 
@see EvalPotentialEnergy() for more information. */)""")
      .def(
          "CalcTimeDerivatives",
          static_cast<void (System<PySystem_double_0>::*)(
              Context<PySystem_double_0> const &,
              ContinuousState<PySystem_double_0> *) const>(
              &System<PySystem_double_0>::CalcTimeDerivatives),
          py::arg("context"), py::arg("derivatives"),
          R"""(/** Calculates the time derivatives ẋ꜀ of the continuous state x꜀ into 
a given output argument. Prefer EvalTimeDerivatives() instead to avoid 
unnecessary recomputation. 
 
This method solves the %System equations in explicit form: 
 
    ẋ꜀ = fₑ(𝓒) 
 
where `𝓒 = {a, p, t, x, u}` is the current value of the given Context from 
which accuracy a, parameters p, time t, state x (`={x꜀ xd xₐ}`) and 
input values u are obtained. 
 
@param[in] context The source for time, state, inputs, etc. defining the 
    point at which the derivatives should be calculated. 
@param[out] derivatives The time derivatives ẋ꜀. Must be the same size as 
    the continuous state vector in `context`. 
 
@see EvalTimeDerivatives() for more information. 
@see CalcImplicitTimeDerivativesResidual() for the implicit form of these 
     equations.*/)""")
      .def(
          "CalcUnrestrictedUpdate",
          static_cast<void (System<PySystem_double_0>::*)(
              Context<PySystem_double_0> const &,
              EventCollection<UnrestrictedUpdateEvent<PySystem_double_0>> const
                  &,
              State<PySystem_double_0> *) const>(
              &System<PySystem_double_0>::CalcUnrestrictedUpdate),
          py::arg("context"), py::arg("events"), py::arg("state"),
          R"""(/** This method is the public entry point for dispatching all unrestricted 
update event handlers. Using all the unrestricted update handers in 
@p events, it updates *any* state variables in the @p context, and 
outputs the results to @p state. It does not allow the dimensionality 
of the state variables to change. See the documentation for 
DispatchUnrestrictedUpdateHandler() for more details. 
 
@throws std::logic_error if the dimensionality of the state variables 
        changes in the callback. */)""")
      .def(
          "CalcUnrestrictedUpdate",
          static_cast<void (System<PySystem_double_0>::*)(
              Context<PySystem_double_0> const &, State<PySystem_double_0> *)
                          const>(
              &System<PySystem_double_0>::CalcUnrestrictedUpdate),
          py::arg("context"), py::arg("state"),
          R"""(/** This method forces an unrestricted update on the system given a 
@p context, and the updated state is stored in @p state. The 
unrestricted update event will have a trigger type of kForced, with no 
additional data, attribute or custom callback. 
 
@sa CalcUnrestrictedUpdate(const Context<T>&, const 
EventCollection<UnrestrictedUpdateEvent<T>>*, State<T>* state) 
    for more information. */)""")
      .def("CalcWitnessValue",
           static_cast<double (System<PySystem_double_0>::*)(
               Context<PySystem_double_0> const &,
               WitnessFunction<PySystem_double_0> const &) const>(
               &System<PySystem_double_0>::CalcWitnessValue),
           py::arg("context"), py::arg("witness_func"),
           R"""(/** Evaluates a witness function at the given context. */)""")
      .def("CheckSystemConstraintsSatisfied",
           static_cast<::drake::scalar_predicate<PySystem_double_0>::type (
               System<PySystem_double_0>::*)(Context<PySystem_double_0> const &,
                                             double) const>(
               &System<PySystem_double_0>::CheckSystemConstraintsSatisfied),
           py::arg("context"), py::arg("tol"),
           R"""(/** Returns true if @p context satisfies all of the registered 
SystemConstraints with tolerance @p tol.  @see 
SystemConstraint::CheckSatisfied. */)""")
      .def(
          "CheckValidOutput",
          static_cast<void (System<PySystem_double_0>::*)(
              SystemOutput<PySystem_double_0> const *) const>(
              &System<PySystem_double_0>::CheckValidOutput),
          py::arg("output"),
          R"""(/** Checks that @p output is consistent with the number and size of output 
ports declared by the system. 
@throws std::exception unless `output` is non-null and valid for this 
system. */)""")
      .def(
          "CopyContinuousStateVector",
          static_cast<::Eigen::Matrix<double, -1, 1, 0, -1, 1> (
              System<PySystem_double_0>::*)(Context<PySystem_double_0> const &)
                          const>(
              &System<PySystem_double_0>::CopyContinuousStateVector),
          py::arg("context"),
          R"""(/** Returns a copy of the continuous state vector x꜀ into an Eigen 
vector. */)""")
      .def(
          "CreateDefaultContext",
          static_cast<::std::unique_ptr<
              Context<PySystem_double_0>,
              std::default_delete<Context<PySystem_double_0>>> (
              System<PySystem_double_0>::*)() const>(
              &System<PySystem_double_0>::CreateDefaultContext),
          R"""(/** This convenience method allocates a context using AllocateContext() and 
sets its default values using SetDefaultContext(). */)""")
      .def(
          "DeclareInputPort",
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
              ::std::optional<drake::RandomDistribution>(std::nullopt),
          R"""(/** Adds a port with the specified @p type and @p size to the input topology. 
 
Input port names must be unique for this system (passing in a duplicate 
@p name will throw std::logic_error). If @p name is given as 
kUseDefaultName, then a default value of e.g. "u2", where 2 
is the input number will be provided. An empty @p name is not permitted. 
 
If the port is intended to model a random noise or disturbance input, 
@p random_type can (optionally) be used to label it as such; doing so 
enables algorithms for design and analysis (e.g. state estimation) to 
reason explicitly about randomness at the system level.  All random input 
ports are assumed to be statistically independent. 
@pre @p name must not be empty. 
@throws std::logic_error for a duplicate port name. 
@returns the declared port. */)""")
      .def(
          "DeclareInputPort",
          static_cast<InputPort<PySystem_double_0> &(
              System<PySystem_double_0>::*)(PortDataType, int,
                                            ::std::optional<
                                                drake::RandomDistribution>)>(
              &System_double_publicist::DeclareInputPort),
          py::arg("type"), py::arg("size"),
          py::arg("random_type") =
              ::std::optional<drake::RandomDistribution>(std::nullopt),
          R"""(/** See the nearly identical signature with an additional (first) argument 
specifying the port name.  This version will be deprecated as discussed 
in #9447. */)""")
      .def(
          "DispatchDiscreteVariableUpdateHandler",
          static_cast<void (System<PySystem_double_0>::*)(
              Context<PySystem_double_0> const &,
              EventCollection<DiscreteUpdateEvent<PySystem_double_0>> const &,
              DiscreteValues<PySystem_double_0> *) const>(
              &System_double_publicist::DispatchDiscreteVariableUpdateHandler),
          py::arg("context"), py::arg("events"), py::arg("discrete_state"),
          R"""(/** This function dispatches all discrete update events to the appropriate 
handlers. @p discrete_state cannot be null. */)""")
      .def(
          "DispatchPublishHandler",
          static_cast<void (System<PySystem_double_0>::*)(
              Context<PySystem_double_0> const &,
              EventCollection<PublishEvent<PySystem_double_0>> const &) const>(
              &System_double_publicist::DispatchPublishHandler),
          py::arg("context"), py::arg("events"),
          R"""(/** This function dispatches all publish events to the appropriate 
handlers. */)""")
      .def(
          "DispatchUnrestrictedUpdateHandler",
          static_cast<void (System<PySystem_double_0>::*)(
              Context<PySystem_double_0> const &,
              EventCollection<UnrestrictedUpdateEvent<PySystem_double_0>> const
                  &,
              State<PySystem_double_0> *) const>(
              &System_double_publicist::DispatchUnrestrictedUpdateHandler),
          py::arg("context"), py::arg("events"), py::arg("state"),
          R"""(/** This function dispatches all unrestricted update events to the appropriate 
handlers. @p state cannot be null. */)""")
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
      .def(
          "DoCalcConservativePower",
          static_cast<double (System<PySystem_double_0>::*)(
              Context<PySystem_double_0> const &) const>(
              &System_double_publicist::DoCalcConservativePower),
          py::arg("context"),
          R"""(/** Override this method to return the rate Pc at which mechanical energy is 
being converted _from_ potential energy _to_ kinetic energy by this system 
in the given Context. By default, returns zero. Physical systems should 
override. You may assume that `context` has already been validated before 
it is passed to you here. 
 
See EvalConservativePower() for details on what you must compute here. In 
particular, this quantity must be _positive_ when potential energy 
is _decreasing_, and your conservative power method must _not_ depend 
explicitly on time or any input port values. */)""")
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
      .def(
          "DoCalcKineticEnergy",
          static_cast<double (System<PySystem_double_0>::*)(
              Context<PySystem_double_0> const &) const>(
              &System_double_publicist::DoCalcKineticEnergy),
          py::arg("context"),
          R"""(/** Override this method for physical systems to calculate the kinetic 
energy KE currently present in the motion provided in the given 
Context. The default implementation returns 0 which is correct for 
non-physical systems. You may assume that `context` has already 
been validated before it is passed to you here. 
 
See EvalKineticEnergy() for details on what you must compute here. In 
particular, your kinetic energy method must _not_ depend explicitly on 
time or any input port values. */)""")
      .def(
          "DoCalcNextUpdateTime",
          static_cast<void (System<PySystem_double_0>::*)(
              Context<PySystem_double_0> const &,
              CompositeEventCollection<PySystem_double_0> *, double *) const>(
              &System_double_publicist::DoCalcNextUpdateTime),
          py::arg("context"), py::arg("events"), py::arg("time"),
          R"""(/** Computes the next time at which this System must perform a discrete 
action. 
 
Override this method if your System has any discrete actions which must 
interrupt the continuous simulation. This method is called only from the 
public non-virtual CalcNextUpdateTime() which will already have 
error-checked the parameters so you don't have to. You may assume that 
@p context has already been validated and @p events pointer is not 
null. 
 
If you override this method, you _must_ set the returned @p time. Set it to 
Infinity if there are no upcoming timed events. If you return a finite update 
time, you _must_ put at least one Event object in the @p events collection. 
These requirements are enforced by the public CalcNextUpdateTime() method. 
 
The default implementation returns with the next sample time being 
Infinity and no events added to @p events. */)""")
      .def(
          "DoCalcNonConservativePower",
          static_cast<double (System<PySystem_double_0>::*)(
              Context<PySystem_double_0> const &) const>(
              &System_double_publicist::DoCalcNonConservativePower),
          py::arg("context"),
          R"""(/** Override this method to return the rate Pnc at which work W is done on the 
system by non-conservative forces. By default, returns zero. Physical 
systems should override. You may assume that `context` has already been 
validated before it is passed to you here. 
 
See EvalNonConservativePower() for details on what you must compute here. 
In particular, this quantity must be _negative_ if the non-conservative 
forces are _dissipative_, positive otherwise. Your non-conservative power 
method can depend on anything you find in the given Context, including 
time and input ports. */)""")
      .def(
          "DoCalcPotentialEnergy",
          static_cast<double (System<PySystem_double_0>::*)(
              Context<PySystem_double_0> const &) const>(
              &System_double_publicist::DoCalcPotentialEnergy),
          py::arg("context"),
          R"""(/** Override this method for physical systems to calculate the potential 
energy PE currently stored in the configuration provided in the given 
Context. The default implementation returns 0 which is correct for 
non-physical systems. You may assume that `context` has already 
been validated before it is passed to you here. 
 
See EvalPotentialEnergy() for details on what you must compute here. In 
particular, your potential energy method must _not_ depend explicitly on 
time, velocities, or any input port values. */)""")
      .def(
          "DoCalcTimeDerivatives",
          static_cast<void (System<PySystem_double_0>::*)(
              Context<PySystem_double_0> const &,
              ContinuousState<PySystem_double_0> *) const>(
              &System_double_publicist::DoCalcTimeDerivatives),
          py::arg("context"), py::arg("derivatives"),
          R"""(/** Override this if you have any continuous state variables x꜀ in your 
concrete %System to calculate their time derivatives. The `derivatives` 
vector will correspond elementwise with the state vector 
`Context.state.continuous_state.get_state()`. Thus, if the state in the 
Context has second-order structure `x꜀=[q v z]`, that same structure 
applies to the derivatives. 
 
This method is called only from the public non-virtual 
CalcTimeDerivatives() which will already have error-checked the parameters 
so you don't have to. In particular, implementations may assume that the 
given Context is valid for this %System; that the `derivatives` pointer is 
non-null, and that the referenced object has the same constituent 
structure as was produced by AllocateTimeDerivatives(). 
 
The default implementation does nothing if the `derivatives` vector is 
size zero and aborts otherwise. */)""")
      .def(
          "DoCalcWitnessValue",
          static_cast<double (System<PySystem_double_0>::*)(
              Context<PySystem_double_0> const &,
              WitnessFunction<PySystem_double_0> const &) const>(
              &System_double_publicist::DoCalcWitnessValue),
          py::arg("context"), py::arg("witness_func"),
          R"""(/** Derived classes will implement this method to evaluate a witness function 
at the given context. */)""")
      .def(
          "DoGetInitializationEvents",
          static_cast<void (System<PySystem_double_0>::*)(
              Context<PySystem_double_0> const &,
              CompositeEventCollection<PySystem_double_0> *) const>(
              &System_double_publicist::DoGetInitializationEvents),
          py::arg("context"), py::arg("events"),
          R"""(/** Implement this method to return any events to be handled at the 
simulator's initialization step. @p events is cleared in the public 
non-virtual GetInitializationEvents(). You may assume that @p context has 
already been validated and that @p events is not null. @p events can be 
changed freely by the overriding implementation. 
 
The default implementation returns without changing @p events. 
@sa GetInitializationEvents() */)""")
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
      .def(
          "DoGetPerStepEvents",
          static_cast<void (System<PySystem_double_0>::*)(
              Context<PySystem_double_0> const &,
              CompositeEventCollection<PySystem_double_0> *) const>(
              &System_double_publicist::DoGetPerStepEvents),
          py::arg("context"), py::arg("events"),
          R"""(/** Implement this method to return any events to be handled before the 
simulator integrates the system's continuous state at each time step. 
@p events is cleared in the public non-virtual GetPerStepEvents() 
before that method calls this function. You may assume that @p context 
has already been validated and that @p events is not null. @p events 
can be changed freely by the overriding implementation. 
 
The default implementation returns without changing @p events. 
@sa GetPerStepEvents() */)""")
      .def(
          "DoGetPeriodicEvents",
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
              &System_double_publicist::DoGetPeriodicEvents),
          R"""(/** Implement this method to return all periodic triggered events. 
@see GetPeriodicEvents() for a detailed description of the returned 
     variable. 
@note The default implementation returns an empty map. */)""")
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
          py::arg("arg0"), py::arg("arg1"),
          R"""(/** Derived classes can override this method to provide witness functions 
active for the given state. The default implementation does nothing. On 
entry to this function, the context will have already been validated and 
the vector of witness functions will have been validated to be both empty 
and non-null. */)""")
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
      .def(
          "EvalConservativePower",
          static_cast<double const &(
              System<PySystem_double_0>::*)(Context<PySystem_double_0> const &)
                          const>(
              &System<PySystem_double_0>::EvalConservativePower),
          py::arg("context"),
          R"""(/** Returns a reference to the cached value of the conservative power (Pc), 
evaluating first if necessary using CalcConservativePower(). 
 
The returned Pc represents the rate at which mechanical energy is being 
converted _from_ potential energy (PE) _to_ kinetic energy (KE) by this 
system in the given Context. This quantity will be _positive_ when PE 
is _decreasing_. By definition here, conservative power may depend only 
on quantities that explicitly contribute to PE and KE. See 
EvalPotentialEnergy() and EvalKineticEnergy() for details. 
 
Power due to non-conservative forces (e.g. dampers) can contribute to the 
rate of change of KE. Therefore this method alone cannot be used to 
determine whether KE is increasing or decreasing, only whether the 
conservative power is adding or removing kinetic energy. 
EvalNonConservativePower() can be used in conjunction with this method to 
find the total rate of change of KE. 
 
Non-physical systems where Pc is not meaningful will return Pc = 0. 
 
@param context The Context whose contents may be used to evaluate 
               conservative power. 
@retval Pc The conservative power in watts (W or J/s) represented by the 
           contents of the given `context`. 
@see CalcConservativePower(), EvalNonConservativePower(), 
     EvalPotentialEnergy(), EvalKineticEnergy() */)""")
      .def(
          "EvalEigenVectorInput",
          static_cast<
              ::Eigen::VectorBlock<const Eigen::Matrix<double, -1, 1, 0, -1, 1>,
                                   -1> (System<PySystem_double_0>::*)(
                  Context<PySystem_double_0> const &, int) const>(
              &System<PySystem_double_0>::EvalEigenVectorInput),
          py::arg("context"), py::arg("port_index"),
          R"""(/** Returns the value of the vector-valued input port with the given 
`port_index` as an %Eigen vector. Causes the value to become up to date 
first if necessary. See EvalAbstractInput() for more information. 
 
@pre `port_index` selects an existing input port of this System. 
@pre the port must have been declared to be vector-valued. 
@pre the port must be evaluable (connected or fixed). 
 
@see EvalVectorInput() */)""")
      .def(
          "EvalKineticEnergy",
          static_cast<double const &(
              System<PySystem_double_0>::*)(Context<PySystem_double_0> const &)
                          const>(&System<PySystem_double_0>::EvalKineticEnergy),
          py::arg("context"),
          R"""(/** Returns a reference to the cached value of the kinetic energy (KE), 
evaluating first if necessary using CalcKineticEnergy(). 
 
By definition here, kinetic energy depends only on "configuration" and 
"velocity" (e.g. angular and translational velocity) of moving masses 
which includes a subset of the state variables, and parameters that affect 
configuration, velocities, or mass properties. The calculated value may 
also be affected by the accuracy value supplied in the Context. KE cannot 
depend explicitly on time (∂KE/∂t = 0) or input port values (∂KE/∂u = 0). 
 
Non-physical systems where KE is not meaningful will return KE = 0. 
 
@param context The Context whose configuration and velocity variables may 
               be used to evaluate kinetic energy. 
@retval KE The kinetic energy in joules (J) represented by the 
           configuration and velocity given in `context`. 
@see CalcKineticEnergy() */)""")
      .def(
          "EvalNonConservativePower",
          static_cast<double const &(
              System<PySystem_double_0>::*)(Context<PySystem_double_0> const &)
                          const>(
              &System<PySystem_double_0>::EvalNonConservativePower),
          py::arg("context"),
          R"""(/** Returns a reference to the cached value of the non-conservative power 
(Pnc), evaluating first if necessary using CalcNonConservativePower(). 
 
The returned Pnc represents the rate at which work W is done on the system 
by non-conservative forces. Pnc is _negative_ if the non-conservative 
forces are _dissipative_, positive otherwise. Time integration of Pnc 
yields work W, and the total mechanical energy `E = PE + KE − W` should be 
conserved by any physically-correct model, to within integration accuracy 
of W. Power is in watts (J/s). (Watts are abbreviated W but not to be 
confused with work!) Any values in the supplied Context (including time 
and input ports) may contribute to the computation of non-conservative 
power. 
 
Non-physical systems where Pnc is not meaningful will return Pnc = 0. 
 
@param context The Context whose contents may be used to evaluate 
               non-conservative power. 
@retval Pnc The non-conservative power in watts (W or J/s) represented by 
            the contents of the given `context`. 
@see CalcNonConservativePower(), EvalConservativePower() */)""")
      .def(
          "EvalPotentialEnergy",
          static_cast<double const &(
              System<PySystem_double_0>::*)(Context<PySystem_double_0> const &)
                          const>(
              &System<PySystem_double_0>::EvalPotentialEnergy),
          py::arg("context"),
          R"""(/** Returns a reference to the cached value of the potential energy (PE), 
evaluating first if necessary using CalcPotentialEnergy(). 
 
By definition here, potential energy depends only on "configuration" 
(e.g. orientation and position), which includes a subset of the state 
variables, and parameters that affect configuration or conservative 
forces (such as lengths and masses). The calculated value may also be 
affected by the accuracy value supplied in the Context. PE cannot depend 
explicitly on time (∂PE/∂t = 0), velocities (∂PE/∂v = 0), or input port 
values (∂PE/∂u = 0). 
 
Non-physical systems where PE is not meaningful will return PE = 0. 
 
@param context The Context whose configuration variables may be used to 
               evaluate potential energy. 
@retval PE The potential energy in joules (J) represented by the 
           configuration given in `context`. 
@see CalcPotentialEnergy() */)""")
      .def(
          "EvalTimeDerivatives",
          static_cast<ContinuousState<PySystem_double_0> const &(
              System<PySystem_double_0>::*)(Context<PySystem_double_0> const &)
                          const>(
              &System<PySystem_double_0>::EvalTimeDerivatives),
          py::arg("context"),
          R"""(/** Returns a reference to the cached value of the continuous state variable 
time derivatives, evaluating first if necessary using CalcTimeDerivatives(). 
 
This method returns the time derivatives ẋ꜀ of the continuous state 
x꜀. The referenced return object will correspond elementwise with the 
continuous state in the given Context. Thus, if the state in the Context 
has second-order structure `x꜀ = [q v z]`, that same structure applies to 
the derivatives so we will have `ẋ꜀ = [q̇ ̇v̇ ż]`. 
 
@param context The Context whose time, input port, parameter, state, and 
accuracy values may be used to evaluate the derivatives. 
 
@retval xcdot Time derivatives ẋ꜀ of x꜀ returned as a reference to an object 
              of the same type and size as `context`'s continuous state. 
@see CalcTimeDerivatives(), CalcImplicitTimeDerivativesResidual(), 
     get_time_derivatives_cache_entry() */)""")
      .def(
          "FixInputPortsFrom",
          static_cast<void (System<PySystem_double_0>::*)(
              System<PySystem_double_0> const &,
              Context<PySystem_double_0> const &, Context<PySystem_double_0> *)
                          const>(&System<PySystem_double_0>::FixInputPortsFrom),
          py::arg("other_system"), py::arg("other_context"),
          py::arg("target_context"),
          R"""(/** Fixes all of the input ports in @p target_context to their current values 
in @p other_context, as evaluated by @p other_system. 
@throws std::exception unless `other_context` and `target_context` both 
have the same shape as this System, and the `other_system`. Ignores 
disconnected inputs. */)""")
      .def(
          "GetGraphvizFragment",
          static_cast<void (System<PySystem_double_0>::*)(
              int, ::std::stringstream *) const>(
              &System<PySystem_double_0>::GetGraphvizFragment),
          py::arg("max_depth"), py::arg("dot"),
          R"""(/** Appends a Graphviz fragment to the @p dot stream.  The fragment must be 
valid Graphviz when wrapped in a `digraph` or `subgraph` stanza.  Does 
nothing by default. 
 
@param max_depth Sets a limit to the depth of nested diagrams to 
visualize.  Set to zero to render a diagram as a single system block. */)""")
      .def(
          "GetGraphvizId",
          static_cast<::int64_t (System<PySystem_double_0>::*)() const>(
              &System<PySystem_double_0>::GetGraphvizId),
          R"""(/** Returns an opaque integer that uniquely identifies this system in the 
Graphviz output. */)""")
      .def(
          "GetGraphvizInputPortToken",
          static_cast<void (System<PySystem_double_0>::*)(
              InputPort<PySystem_double_0> const &, int, ::std::stringstream *)
                          const>(
              &System<PySystem_double_0>::GetGraphvizInputPortToken),
          py::arg("port"), py::arg("max_depth"), py::arg("dot"),
          R"""(/** Appends a fragment to the @p dot stream identifying the graphviz node 
representing @p port. Does nothing by default. */)""")
      .def(
          "GetGraphvizOutputPortToken",
          static_cast<void (System<PySystem_double_0>::*)(
              OutputPort<PySystem_double_0> const &, int, ::std::stringstream *)
                          const>(
              &System<PySystem_double_0>::GetGraphvizOutputPortToken),
          py::arg("port"), py::arg("max_depth"), py::arg("dot"),
          R"""(/** Appends a fragment to the @p dot stream identifying the graphviz node 
representing @p port. Does nothing by default. */)""")
      .def(
          "GetGraphvizString",
          static_cast<::std::string (System<PySystem_double_0>::*)(int) const>(
              &System<PySystem_double_0>::GetGraphvizString),
          py::arg("max_depth") = int(std::numeric_limits<int>::max()),
          R"""(/** Returns a Graphviz string describing this System.  To render the string, 
use the Graphviz tool, ``dot``. http://www.graphviz.org/ 
 
@param max_depth Sets a limit to the depth of nested diagrams to 
visualize.  Set to zero to render a diagram as a single system block. 
 
@see GenerateHtml 
*/)""")
      .def(
          "GetInitializationEvents",
          static_cast<void (System<PySystem_double_0>::*)(
              Context<PySystem_double_0> const &,
              CompositeEventCollection<PySystem_double_0> *) const>(
              &System<PySystem_double_0>::GetInitializationEvents),
          py::arg("context"), py::arg("events"),
          R"""(/** This method is called by Simulator::Initialize() to gather all 
update and publish events that need to be handled at initialization 
before the simulator starts integration. 
 
@p events cannot be null. @p events will be cleared on entry. */)""")
      .def(
          "GetInputPort",
          static_cast<InputPort<PySystem_double_0> const &(
              System<PySystem_double_0>::*)(::std::string const &)const>(
              &System<PySystem_double_0>::GetInputPort),
          py::arg("port_name"),
          R"""(/** Returns the typed input port with the unique name @p port_name. 
The current implementation performs a linear search over strings; prefer 
get_input_port() when performance is a concern. 
@throws std::logic_error if port_name is not found. */)""")
      .def(
          "GetMemoryObjectName",
          static_cast<::std::string (System<PySystem_double_0>::*)() const>(
              &System<PySystem_double_0>::GetMemoryObjectName),
          R"""(/** Returns a name for this %System based on a stringification of its type 
name and memory address.  This is intended for use in diagnostic output 
and should not be used for behavioral logic, because the stringification 
of the type name may produce differing results across platforms and 
because the address can vary from run to run. */)""")
      .def(
          "GetMutableOutputVector",
          static_cast<
              ::Eigen::VectorBlock<Eigen::Matrix<double, -1, 1, 0, -1, 1>, -1> (
                  System<PySystem_double_0>::*)(
                  SystemOutput<PySystem_double_0> *, int) const>(
              &System_double_publicist::GetMutableOutputVector),
          py::arg("output"), py::arg("port_index"),
          R"""(/** Returns a mutable Eigen expression for a vector valued output port with 
index @p port_index in this system. All input ports that directly depend 
on this output port will be notified that upstream data has changed, and 
may invalidate cache entries as a result. */)""")
      .def(
          "GetMutableSubsystemContext",
          static_cast<Context<PySystem_double_0> &(
              System<PySystem_double_0>::*)(System<PySystem_double_0> const &,
                                            Context<PySystem_double_0> *)const>(
              &System<PySystem_double_0>::GetMutableSubsystemContext),
          py::arg("subsystem"), py::arg("context"),
          R"""(/** Returns a mutable reference to the subcontext that corresponds to the 
contained %System `subsystem`. 
@throws std::logic_error if `subsystem` not contained in `this` %System. 
@pre The given `context` is valid for use with `this` %System. */)""")
      .def(
          "GetMyContextFromRoot",
          static_cast<Context<PySystem_double_0> const &(
              System<PySystem_double_0>::*)(Context<PySystem_double_0> const &)
                          const>(
              &System<PySystem_double_0>::GetMyContextFromRoot),
          py::arg("root_context"),
          R"""(/** Returns the const Context for `this` subsystem, given a root context. If 
`this` %System is already the top level (root) %System, just returns 
`root_context`. (A root Context is one that does not have a parent 
Context.) 
@throws std::logic_error if the given `root_context` is not actually 
    a root context. 
@see GetSubsystemContext() */)""")
      .def(
          "GetMyMutableContextFromRoot",
          static_cast<Context<PySystem_double_0> &(
              System<PySystem_double_0>::*)(Context<PySystem_double_0> *)const>(
              &System<PySystem_double_0>::GetMyMutableContextFromRoot),
          py::arg("root_context"),
          R"""(/** Returns the mutable subsystem context for `this` system, given a root 
context. 
@see GetMyContextFromRoot() */)""")
      .def(
          "GetOutputPort",
          static_cast<OutputPort<PySystem_double_0> const &(
              System<PySystem_double_0>::*)(::std::string const &)const>(
              &System<PySystem_double_0>::GetOutputPort),
          py::arg("port_name"),
          R"""(/** Returns the typed output port with the unique name @p port_name. 
The current implementation performs a linear search over strings; prefer 
get_output_port() when performance is a concern. 
@throws std::logic_error if port_name is not found. */)""")
      .def(
          "GetPerStepEvents",
          static_cast<void (System<PySystem_double_0>::*)(
              Context<PySystem_double_0> const &,
              CompositeEventCollection<PySystem_double_0> *) const>(
              &System<PySystem_double_0>::GetPerStepEvents),
          py::arg("context"), py::arg("events"),
          R"""(/** This method is called by Simulator::Initialize() to gather all update 
and publish events that are to be handled in AdvanceTo() at the point 
before Simulator integrates continuous state. It is assumed that these 
events remain constant throughout the simulation. The "step" here refers 
to the major time step taken by the Simulator. During every simulation 
step, the simulator will merge @p events with the event collections 
populated by other types of event triggering mechanism (e.g., 
CalcNextUpdateTime()), and the merged CompositeEventCollection objects 
will be passed to the appropriate handlers before Simulator integrates the 
continuous state. 
 
@p events cannot be null. @p events will be cleared on entry. */)""")
      .def(
          "GetPeriodicEvents",
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
              &System<PySystem_double_0>::GetPeriodicEvents),
          R"""(/** Gets all periodic triggered events for a system. Each periodic attribute 
(offset and period, in seconds) is mapped to one or more update events 
that are to be triggered at the proper times. */)""")
      .def(
          "GetSubsystemContext",
          static_cast<Context<PySystem_double_0> const &(
              System<PySystem_double_0>::*)(System<PySystem_double_0> const &,
                                            Context<PySystem_double_0> const &)
                          const>(
              &System<PySystem_double_0>::GetSubsystemContext),
          py::arg("subsystem"), py::arg("context"),
          R"""(/** Returns a const reference to the subcontext that corresponds to the 
contained %System `subsystem`. 
@throws std::logic_error if `subsystem` not contained in `this` %System. 
@pre The given `context` is valid for use with `this` %System. */)""")
      .def(
          "GetUniquePeriodicDiscreteUpdateAttribute",
          static_cast<::std::optional<PeriodicEventData> (
              System<PySystem_double_0>::*)() const>(
              &System<
                  PySystem_double_0>::GetUniquePeriodicDiscreteUpdateAttribute),
          R"""(/** Gets whether there exists a unique periodic attribute that triggers 
one or more discrete update events (and, if so, returns that unique 
periodic attribute). Thus, this method can be used (1) as a test to 
determine whether a system's dynamics are at least partially governed by 
difference equations and (2) to obtain the difference equation update 
times. 
@returns optional<PeriodicEventData> Contains the periodic trigger 
attributes if the unique periodic attribute exists, otherwise `nullopt`. */)""")
      .def(
          "GetWitnessFunctions",
          static_cast<void (System<PySystem_double_0>::*)(
              Context<PySystem_double_0> const &,
              ::std::vector<
                  const WitnessFunction<PySystem_double_0> *,
                  std::allocator<const WitnessFunction<PySystem_double_0> *>> *)
                          const>(
              &System<PySystem_double_0>::GetWitnessFunctions),
          py::arg("context"), py::arg("w"),
          R"""(/** Gets the witness functions active for the given state. 
DoGetWitnessFunctions() does the actual work. The vector of active witness 
functions are expected to change only upon an unrestricted update. 
@param context a valid context for the System (aborts if not true). 
@param[out] w a valid pointer to an empty vector that will store 
            pointers to the witness functions active for the current 
            state. The method aborts if witnesses is null or non-empty. */)""")
      .def(
          "HasAnyDirectFeedthrough",
          static_cast<bool (System<PySystem_double_0>::*)() const>(
              &System<PySystem_double_0>::HasAnyDirectFeedthrough),
          R"""(/** Returns `true` if any of the inputs to the system might be directly 
fed through to any of its outputs and `false` otherwise. */)""")
      .def(
          "HasDirectFeedthrough",
          static_cast<bool (System<PySystem_double_0>::*)(int) const>(
              &System<PySystem_double_0>::HasDirectFeedthrough),
          py::arg("output_port"),
          R"""(/** Returns true if there might be direct-feedthrough from any input port to 
the given @p output_port, and false otherwise. */)""")
      .def(
          "HasDirectFeedthrough",
          static_cast<bool (System<PySystem_double_0>::*)(int, int) const>(
              &System<PySystem_double_0>::HasDirectFeedthrough),
          py::arg("input_port"), py::arg("output_port"),
          R"""(/** Returns true if there might be direct-feedthrough from the given 
@p input_port to the given @p output_port, and false otherwise. */)""")
      .def(
          "HasInputPort",
          static_cast<bool (System<PySystem_double_0>::*)(::std::string const &)
                          const>(&System<PySystem_double_0>::HasInputPort),
          py::arg("port_name"),
          R"""(/** Returns true iff the system has an InputPort of the given @p 
 port_name. */)""")
      .def(
          "HasOutputPort",
          static_cast<bool (System<PySystem_double_0>::*)(::std::string const &)
                          const>(&System<PySystem_double_0>::HasOutputPort),
          py::arg("port_name"),
          R"""(/** Returns true iff the system has an OutputPort of the given @p 
 port_name. */)""")
      .def(
          "IsDifferenceEquationSystem",
          static_cast<bool (System<PySystem_double_0>::*)(double *) const>(
              &System<PySystem_double_0>::IsDifferenceEquationSystem),
          py::arg("time_period") = (double *)nullptr,
          R"""(/** Returns true iff the state dynamics of this system are governed 
exclusively by a difference equation on a single discrete state group 
and with a unique periodic update (having zero offset).  E.g., it is 
amenable to analysis of the form: 
  x[n+1] = f(x[n], u[n]) 
Note that we do NOT consider the number of input ports here, because 
in practice many systems of interest (e.g. MultibodyPlant) have input 
ports that are safely treated as constant during the analysis. 
Consider using get_input_port_selection() to choose one. 
 
@param[out] time_period if non-null, then iff the function 
returns `true`, then time_period is set to the period data 
returned from GetUniquePeriodicDiscreteUpdateAttribute().  If the 
function returns `false` (the system is not a difference equation 
system), then `time_period` does not receive a value. */)""")
      .def(
          "MapQDotToVelocity",
          static_cast<void (System<PySystem_double_0>::*)(
              Context<PySystem_double_0> const &,
              VectorBase<PySystem_double_0> const &,
              VectorBase<PySystem_double_0> *) const>(
              &System<PySystem_double_0>::MapQDotToVelocity),
          py::arg("context"), py::arg("qdot"), py::arg("generalized_velocity"),
          R"""(/** Transforms the time derivative `qdot` of the generalized configuration `q` 
to generalized velocities `v`. `v` and `qdot` are related linearly by 
`qdot = N(q) * v`, where `N` is a block diagonal matrix. For example, in a 
multibody system there will be one block of `N` per tree joint. Although 
`N` is not necessarily square, its left pseudo-inverse `N+` can be used to 
invert that relationship without residual error, provided that `qdot` is 
in the range space of `N` (that is, if it *could* have been produced as 
`qdot=N*v` for some `v`). Using the configuration `q` from the given 
Context this method calculates `v = N+ * qdot` (where `N+=N+(q)`) for 
a given `qdot`. This computation requires only `O(nq)` time where `nq` is 
the size of `qdot`. Note that this method does not take `qdot` from the 
Context. 
 
See the alternate signature if you already have `qdot` in an %Eigen 
VectorX object; this signature will copy the VectorBase into an %Eigen 
object before performing the computation. 
@see MapVelocityToQDot() */)""")
      .def("MapQDotToVelocity",
           [](System<PySystem_double_0> &self,
              Context<PySystem_double_0> const &context,
              ::Eigen::Ref<const Eigen::Matrix<double, -1, 1, 0, -1, 1>, 0,
                           Eigen::InnerStride<1>> const &qdot,
              VectorBase<PySystem_double_0> *generalized_velocity) {
             return self.MapQDotToVelocity(context, qdot, generalized_velocity);
           })
      .def(
          "MapVelocityToQDot",
          static_cast<void (System<PySystem_double_0>::*)(
              Context<PySystem_double_0> const &,
              VectorBase<PySystem_double_0> const &,
              VectorBase<PySystem_double_0> *) const>(
              &System<PySystem_double_0>::MapVelocityToQDot),
          py::arg("context"), py::arg("generalized_velocity"), py::arg("qdot"),
          R"""(/** Transforms a given generalized velocity `v` to the time derivative `qdot` 
of the generalized configuration `q` taken from the supplied Context. 
`v` and `qdot` are related linearly by `qdot = N(q) * v`, where `N` is a 
block diagonal matrix. For example, in a multibody system there will be 
one block of `N` per tree joint. This computation requires only `O(nq)` 
time where `nq` is the size of `qdot`. Note that `v` is *not* taken from 
the Context; it is given as an argument here. 
 
See the alternate signature if you already have the generalized 
velocity in an Eigen VectorX object; this signature will copy the 
VectorBase into an Eigen object before performing the computation. 
@see MapQDotToVelocity() */)""")
      .def("MapVelocityToQDot",
           [](System<PySystem_double_0> &self,
              Context<PySystem_double_0> const &context,
              ::Eigen::Ref<const Eigen::Matrix<double, -1, 1, 0, -1, 1>, 0,
                           Eigen::InnerStride<1>> const &generalized_velocity,
              VectorBase<PySystem_double_0> *qdot) {
             return self.MapVelocityToQDot(context, generalized_velocity, qdot);
           })
      .def(
          "Publish",
          static_cast<void (System<PySystem_double_0>::*)(
              Context<PySystem_double_0> const &,
              EventCollection<PublishEvent<PySystem_double_0>> const &) const>(
              &System<PySystem_double_0>::Publish),
          py::arg("context"), py::arg("events"),
          R"""(/** This method is the public entry point for dispatching all publish event 
handlers. It checks the validity of @p context, and directly calls 
DispatchPublishHandler. @p events is a homogeneous collection of publish 
events. 
 
@note When publishing is triggered at particular times, those times likely 
will not coincide with integrator step times. A Simulator may interpolate 
to generate a suitable Context, or it may adjust the integrator step size 
so that a step begins exactly at the next publication time. In the latter 
case the change in step size may affect the numerical result somewhat 
since a smaller integrator step produces a more accurate solution. */)""")
      .def(
          "Publish",
          static_cast<void (System<PySystem_double_0>::*)(
              Context<PySystem_double_0> const &) const>(
              &System<PySystem_double_0>::Publish),
          py::arg("context"),
          R"""(/** Forces a publish on the system, given a @p context. The publish event will 
have a trigger type of kForced, with no additional data, attribute or 
custom callback. The Simulator can be configured to call this in 
Simulator::Initialize() and at the start of each continuous integration 
step. See the Simulator API for more details. */)""")
      .def(
          "SetDefaultContext",
          static_cast<void (System<PySystem_double_0>::*)(
              Context<PySystem_double_0> *) const>(
              &System<PySystem_double_0>::SetDefaultContext),
          py::arg("context"),
          R"""(/** Sets Context fields to their default values.  User code should not 
override. */)""")
      .def(
          "SetDefaultParameters",
          static_cast<void (System<PySystem_double_0>::*)(
              Context<PySystem_double_0> const &,
              Parameters<PySystem_double_0> *) const>(
              &System<PySystem_double_0>::SetDefaultParameters),
          py::arg("context"), py::arg("parameters"),
          R"""(/** Assigns default values to all parameters. Overrides must not 
change the number of parameters. */)""")
      .def(
          "SetDefaultState",
          static_cast<void (System<PySystem_double_0>::*)(
              Context<PySystem_double_0> const &, State<PySystem_double_0> *)
                          const>(&System<PySystem_double_0>::SetDefaultState),
          py::arg("context"), py::arg("state"),
          R"""(/** Assigns default values to all elements of the state. Overrides must not 
change the number of state variables. */)""")
      .def("SetRandomContext",
           static_cast<void (System<PySystem_double_0>::*)(
               Context<PySystem_double_0> *, ::drake::RandomGenerator *) const>(
               &System<PySystem_double_0>::SetRandomContext),
           py::arg("context"), py::arg("generator"),
           R"""(/** Sets Context fields to random values.  User code should not 
override. */)""")
      .def("SetRandomParameters",
           static_cast<void (System<PySystem_double_0>::*)(
               Context<PySystem_double_0> const &,
               Parameters<PySystem_double_0> *, ::drake::RandomGenerator *)
                           const>(
               &System<PySystem_double_0>::SetRandomParameters),
           py::arg("context"), py::arg("parameters"), py::arg("generator"),
           R"""(/** Assigns random values to all parameters. 
This default implementation calls SetDefaultParameters; override this 
method to provide random parameters using the stdc++ random library, e.g.: 
@code 
  std::uniform_real_distribution<T> uniform(); 
  parameters->get_mutable_numeric_parameter(0) 
            ->SetAtIndex(0, uniform(*generator)); 
@endcode 
Overrides must not change the number of state variables. 
 
@see @ref stochastic_systems */)""")
      .def("SetRandomState",
           static_cast<void (System<PySystem_double_0>::*)(
               Context<PySystem_double_0> const &, State<PySystem_double_0> *,
               ::drake::RandomGenerator *) const>(
               &System<PySystem_double_0>::SetRandomState),
           py::arg("context"), py::arg("state"), py::arg("generator"),
           R"""(/** Assigns random values to all elements of the state. 
This default implementation calls SetDefaultState; override this method to 
provide random initial conditions using the stdc++ random library, e.g.: 
@code 
  std::normal_distribution<T> gaussian(); 
  state->get_mutable_continuous_state()->get_mutable_vector() 
       ->SetAtIndex(0, gaussian(*generator)); 
@endcode 
Overrides must not change the number of state variables. 
 
@see @ref stochastic_systems */)""")
      .def(
          "ToAutoDiffXd",
          static_cast<::std::unique_ptr<
              System<::drake::AutoDiffXd>,
              std::default_delete<System<::drake::AutoDiffXd>>> (
              System<PySystem_double_0>::*)() const>(
              &System<PySystem_double_0>::ToAutoDiffXd),
          R"""(/** Creates a deep copy of this System, transmogrified to use the autodiff 
scalar type, with a dynamic-sized vector of partial derivatives.  The 
result is never nullptr. 
@throws std::exception if this System does not support autodiff 
 
See @ref system_scalar_conversion for detailed background and examples 
related to scalar-type conversion support. */)""")
      .def(
          "ToAutoDiffXdMaybe",
          static_cast<::std::unique_ptr<
              System<::drake::AutoDiffXd>,
              std::default_delete<System<::drake::AutoDiffXd>>> (
              System<PySystem_double_0>::*)() const>(
              &System<PySystem_double_0>::ToAutoDiffXdMaybe),
          R"""(/** Creates a deep copy of this system exactly like ToAutoDiffXd(), but 
returns nullptr if this System does not support autodiff, instead of 
throwing an exception. */)""")
      .def(
          "ToSymbolic",
          static_cast<::std::unique_ptr<
              System<::drake::symbolic::Expression>,
              std::default_delete<System<::drake::symbolic::Expression>>> (
              System<PySystem_double_0>::*)() const>(
              &System<PySystem_double_0>::ToSymbolic),
          R"""(/** Creates a deep copy of this System, transmogrified to use the symbolic 
scalar type. The result is never nullptr. 
@throws std::exception if this System does not support symbolic 
 
See @ref system_scalar_conversion for detailed background and examples 
related to scalar-type conversion support. */)""")
      .def(
          "ToSymbolicMaybe",
          static_cast<::std::unique_ptr<
              System<::drake::symbolic::Expression>,
              std::default_delete<System<::drake::symbolic::Expression>>> (
              System<PySystem_double_0>::*)() const>(
              &System<PySystem_double_0>::ToSymbolicMaybe),
          R"""(/** Creates a deep copy of this system exactly like ToSymbolic(), but returns 
nullptr if this System does not support symbolic, instead of throwing an 
exception. */)""")
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
           py::arg("constraint_index"),
           R"""(/** Returns the constraint at index @p constraint_index. 
@throws std::out_of_range for an invalid constraint_index. */)""")
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
           py::arg("port_index"),
           R"""(/** Returns the typed input port at index @p port_index. */)""")
      .def(
          "get_input_port",
          static_cast<InputPort<PySystem_double_0> const &(
              System<PySystem_double_0>::*)() const>(
              &System<PySystem_double_0>::get_input_port),
          R"""(/** Convenience method for the case of exactly one input port. */)""")
      .def(
          "get_input_port_selection",
          static_cast<InputPort<PySystem_double_0> const *(
              System<PySystem_double_0>::
                  *)(::std::variant<InputPortSelection,
                                    drake::TypeSafeIndex<InputPortTag>>)const>(
              &System<PySystem_double_0>::get_input_port_selection),
          py::arg("port_index"),
          R"""(/** Returns the typed input port specified by the InputPortSelection or by 
the InputPortIndex.  Returns nullptr if no port is selected.  This is 
provided as a convenience method since many algorithms provide the same 
common default or optional port semantics. */)""")
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
      .def(
          "get_output_port",
          static_cast<OutputPort<PySystem_double_0> const &(
              System<PySystem_double_0>::*)(int)const>(
              &System<PySystem_double_0>::get_output_port),
          py::arg("port_index"),
          R"""(/** Returns the typed output port at index @p port_index. */)""")
      .def(
          "get_output_port",
          static_cast<OutputPort<PySystem_double_0> const &(
              System<PySystem_double_0>::*)() const>(
              &System<PySystem_double_0>::get_output_port),
          R"""(/** Convenience method for the case of exactly one output port. */)""")
      .def(
          "get_output_port_selection",
          static_cast<OutputPort<PySystem_double_0> const *(
              System<PySystem_double_0>::
                  *)(::std::variant<OutputPortSelection,
                                    drake::TypeSafeIndex<OutputPortTag>>)const>(
              &System<PySystem_double_0>::get_output_port_selection),
          py::arg("port_index"),
          R"""(/** Returns the typed output port specified by the OutputPortSelection or by 
the OutputPortIndex.  Returns nullptr if no port is selected. This is 
provided as a convenience method since many algorithms provide the same 
common default or optional port semantics. */)""")
      .def(
          "get_system_scalar_converter",
          static_cast<SystemScalarConverter const &(
              System<PySystem_double_0>::*)() const>(
              &System<PySystem_double_0>::get_system_scalar_converter),
          R"""(/** (Advanced) Returns the SystemScalarConverter for this object.  This is an 
expert-level API intended for framework authors.  Most users should 
prefer the convenience helpers such as System::ToAutoDiffXd. */)""")
      .def(
          "get_time_derivatives_cache_entry",
          static_cast<CacheEntry const &(System<PySystem_double_0>::*)() const>(
              &System<PySystem_double_0>::get_time_derivatives_cache_entry),
          R"""(/** (Advanced) Returns the CacheEntry used to cache time derivatives for 
EvalTimeDerivatives(). */)""")
      .def(
          "num_constraints",
          static_cast<int (System<PySystem_double_0>::*)() const>(
              &System<PySystem_double_0>::num_constraints),
          R"""(/** Returns the number of constraints specified for the system. */)""")
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
      .def(
          "AddConstraint",
          [](System<Eigen::AutoDiffScalar<Eigen::VectorXd>> &self,
             SystemConstraint<::drake::AutoDiffXd> constraint) {
            return self.AddConstraint(
                std::make_unique<SystemConstraint<::drake::AutoDiffXd>>(
                    constraint));
          },
          R"""(/** Adds an already-created constraint to the list of constraints for this 
System.  Ownership of the SystemConstraint is transferred to this system. */)""")
      .def("AddExternalConstraint",
           static_cast<SystemConstraintIndex (
               System<Eigen::AutoDiffScalar<Eigen::VectorXd>>::*)(
               ExternalSystemConstraint)>(
               &System<Eigen::AutoDiffScalar<Eigen::VectorXd>>::
                   AddExternalConstraint),
           py::arg("constraint"),
           R"""(/** Adds an "external" constraint to this System. 
 
This method is intended for use by applications that are examining this 
System to add additional constraints based on their particular situation 
(e.g., that a velocity state element has an upper bound); it is not 
intended for declaring intrinsic constraints that some particular System 
subclass might always impose on itself (e.g., that a mass parameter is 
non-negative).  To that end, this method should not be called by 
subclasses of `this` during their constructor. 
 
The `constraint` will automatically persist across system scalar 
conversion. */)""")
      .def("AddTriggeredWitnessFunctionToCompositeEventCollection",
           [](System<Eigen::AutoDiffScalar<Eigen::VectorXd>> &self,
              Event<Eigen::AutoDiffScalar<Eigen::VectorXd>> *event,
              CompositeEventCollection<Eigen::AutoDiffScalar<Eigen::VectorXd>>
                  *events) {
             return self.AddTriggeredWitnessFunctionToCompositeEventCollection(
                 event, events);
           })
      .def(
          "AllocateCompositeEventCollection",
          static_cast<
              ::std::unique_ptr<CompositeEventCollection<::drake::AutoDiffXd>,
                                std::default_delete<CompositeEventCollection<
                                    ::drake::AutoDiffXd>>> (
                  System<Eigen::AutoDiffScalar<Eigen::VectorXd>>::*)() const>(
              &System<Eigen::AutoDiffScalar<Eigen::VectorXd>>::
                  AllocateCompositeEventCollection),
          R"""(/** Allocates a CompositeEventCollection for this system. The allocated 
instance is used for populating collections of triggered events; for 
example, Simulator passes this object to System::CalcNextUpdateTime() to 
allow the system to identify and handle upcoming events. */)""")
      .def(
          "AllocateContext",
          static_cast<::std::unique_ptr<
              Context<::drake::AutoDiffXd>,
              std::default_delete<Context<::drake::AutoDiffXd>>> (
              System<Eigen::AutoDiffScalar<Eigen::VectorXd>>::*)() const>(
              &System<Eigen::AutoDiffScalar<Eigen::VectorXd>>::AllocateContext),
          R"""(/** Returns a Context<T> suitable for use with this System<T>. */)""")
      .def(
          "AllocateDiscreteVariables",
          static_cast<::std::unique_ptr<
              DiscreteValues<::drake::AutoDiffXd>,
              std::default_delete<DiscreteValues<::drake::AutoDiffXd>>> (
              System<Eigen::AutoDiffScalar<Eigen::VectorXd>>::*)() const>(
              &System<Eigen::AutoDiffScalar<Eigen::VectorXd>>::
                  AllocateDiscreteVariables),
          R"""(/** Returns a DiscreteValues of the same dimensions as the discrete_state 
allocated in CreateDefaultContext. The simulator will provide this state 
as the output argument to Update. */)""")
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
      .def(
          "AllocateImplicitTimeDerivativesResidual",
          static_cast<::Eigen::Matrix<Eigen::AutoDiffScalar<Eigen::VectorXd>,
                                      -1, 1, 0, -1, 1> (
              System<Eigen::AutoDiffScalar<Eigen::VectorXd>>::*)() const>(
              &System<Eigen::AutoDiffScalar<Eigen::VectorXd>>::
                  AllocateImplicitTimeDerivativesResidual),
          R"""(/** Returns an Eigen VectorX suitable for use as the output argument to 
the CalcImplicitTimeDerivativesResidual() method. The returned VectorX 
will have size implicit_time_derivatives_residual_size() with the 
elements uninitialized. This is just a convenience method -- you are free 
to use any properly-sized mutable Eigen object as the residual vector. */)""")
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
      .def(
          "AllocateOutput",
          static_cast<::std::unique_ptr<
              SystemOutput<::drake::AutoDiffXd>,
              std::default_delete<SystemOutput<::drake::AutoDiffXd>>> (
              System<Eigen::AutoDiffScalar<Eigen::VectorXd>>::*)() const>(
              &System<Eigen::AutoDiffScalar<Eigen::VectorXd>>::AllocateOutput),
          R"""(/** Returns a container that can hold the values of all of this System's 
output ports. It is sized with the number of output ports and uses each 
output port's allocation method to provide an object of the right type 
for that port. */)""")
      .def(
          "AllocateTimeDerivatives",
          static_cast<::std::unique_ptr<
              ContinuousState<::drake::AutoDiffXd>,
              std::default_delete<ContinuousState<::drake::AutoDiffXd>>> (
              System<Eigen::AutoDiffScalar<Eigen::VectorXd>>::*)() const>(
              &System<Eigen::AutoDiffScalar<Eigen::VectorXd>>::
                  AllocateTimeDerivatives),
          R"""(/** Returns a ContinuousState of the same size as the continuous_state 
allocated in CreateDefaultContext. The simulator will provide this state 
as the output argument to EvalTimeDerivatives. */)""")
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
      .def(
          "CreateDefaultContext",
          static_cast<::std::unique_ptr<
              Context<::drake::AutoDiffXd>,
              std::default_delete<Context<::drake::AutoDiffXd>>> (
              System<Eigen::AutoDiffScalar<Eigen::VectorXd>>::*)() const>(
              &System<Eigen::AutoDiffScalar<Eigen::VectorXd>>::
                  CreateDefaultContext),
          R"""(/** This convenience method allocates a context using AllocateContext() and 
sets its default values using SetDefaultContext(). */)""")
      .def(
          "DeclareInputPort",
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
          R"""(/** Adds a port with the specified @p type and @p size to the input topology. 
 
Input port names must be unique for this system (passing in a duplicate 
@p name will throw std::logic_error). If @p name is given as 
kUseDefaultName, then a default value of e.g. "u2", where 2 
is the input number will be provided. An empty @p name is not permitted. 
 
If the port is intended to model a random noise or disturbance input, 
@p random_type can (optionally) be used to label it as such; doing so 
enables algorithms for design and analysis (e.g. state estimation) to 
reason explicitly about randomness at the system level.  All random input 
ports are assumed to be statistically independent. 
@pre @p name must not be empty. 
@throws std::logic_error for a duplicate port name. 
@returns the declared port. */)""",
          py::return_value_policy::reference_internal)
      .def(
          "DeclareInputPort",
          static_cast<InputPort<Eigen::AutoDiffScalar<Eigen::VectorXd>> &(
              System<Eigen::AutoDiffScalar<Eigen::VectorXd>>::
                  *)(PortDataType, int,
                     ::std::optional<drake::RandomDistribution>)>(
              &System_Eigen_AutoDiffScalar_Eigen_VectorXd_publicist::
                  DeclareInputPort),
          py::arg("type"), py::arg("size"),
          py::arg("random_type") =
              ::std::optional<drake::RandomDistribution>(std::nullopt),
          R"""(/** See the nearly identical signature with an additional (first) argument 
specifying the port name.  This version will be deprecated as discussed 
in #9447. */)""",
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
      .def(
          "DoGetPeriodicEvents",
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
                  DoGetPeriodicEvents),
          R"""(/** Implement this method to return all periodic triggered events. 
@see GetPeriodicEvents() for a detailed description of the returned 
     variable. 
@note The default implementation returns an empty map. */)""")
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
          py::arg("max_depth"), py::arg("dot"),
          R"""(/** Appends a Graphviz fragment to the @p dot stream.  The fragment must be 
valid Graphviz when wrapped in a `digraph` or `subgraph` stanza.  Does 
nothing by default. 
 
@param max_depth Sets a limit to the depth of nested diagrams to 
visualize.  Set to zero to render a diagram as a single system block. */)""")
      .def(
          "GetGraphvizId",
          static_cast<::int64_t (
              System<Eigen::AutoDiffScalar<Eigen::VectorXd>>::*)() const>(
              &System<Eigen::AutoDiffScalar<Eigen::VectorXd>>::GetGraphvizId),
          R"""(/** Returns an opaque integer that uniquely identifies this system in the 
Graphviz output. */)""")
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
      .def(
          "GetGraphvizString",
          static_cast<::std::string (
              System<Eigen::AutoDiffScalar<Eigen::VectorXd>>::*)(int) const>(
              &System<
                  Eigen::AutoDiffScalar<Eigen::VectorXd>>::GetGraphvizString),
          py::arg("max_depth") = int(std::numeric_limits<int>::max()),
          R"""(/** Returns a Graphviz string describing this System.  To render the string, 
use the Graphviz tool, ``dot``. http://www.graphviz.org/ 
 
@param max_depth Sets a limit to the depth of nested diagrams to 
visualize.  Set to zero to render a diagram as a single system block. 
 
@see GenerateHtml 
*/)""")
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
          py::arg("port_name"),
          R"""(/** Returns the typed input port with the unique name @p port_name. 
The current implementation performs a linear search over strings; prefer 
get_input_port() when performance is a concern. 
@throws std::logic_error if port_name is not found. */)""",
          py::return_value_policy::reference_internal)
      .def(
          "GetMemoryObjectName",
          static_cast<::std::string (
              System<Eigen::AutoDiffScalar<Eigen::VectorXd>>::*)() const>(
              &System<
                  Eigen::AutoDiffScalar<Eigen::VectorXd>>::GetMemoryObjectName),
          R"""(/** Returns a name for this %System based on a stringification of its type 
name and memory address.  This is intended for use in diagnostic output 
and should not be used for behavioral logic, because the stringification 
of the type name may produce differing results across platforms and 
because the address can vary from run to run. */)""")
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
      .def(
          "GetOutputPort",
          static_cast<OutputPort<Eigen::AutoDiffScalar<Eigen::VectorXd>> const
                          &(System<Eigen::AutoDiffScalar<Eigen::VectorXd>>::
                                *)(::std::string const &)const>(
              &System<Eigen::AutoDiffScalar<Eigen::VectorXd>>::GetOutputPort),
          py::arg("port_name"),
          R"""(/** Returns the typed output port with the unique name @p port_name. 
The current implementation performs a linear search over strings; prefer 
get_output_port() when performance is a concern. 
@throws std::logic_error if port_name is not found. */)""",
          py::return_value_policy::reference_internal)
      .def("GetPerStepEvents",
           [](System<Eigen::AutoDiffScalar<Eigen::VectorXd>> &self,
              Context<Eigen::AutoDiffScalar<Eigen::VectorXd>> const &context,
              CompositeEventCollection<Eigen::AutoDiffScalar<Eigen::VectorXd>>
                  *events) { return self.GetPerStepEvents(context, events); })
      .def(
          "GetPeriodicEvents",
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
                  Eigen::AutoDiffScalar<Eigen::VectorXd>>::GetPeriodicEvents),
          R"""(/** Gets all periodic triggered events for a system. Each periodic attribute 
(offset and period, in seconds) is mapped to one or more update events 
that are to be triggered at the proper times. */)""")
      .def(
          "GetSubsystemContext",
          [](System<Eigen::AutoDiffScalar<Eigen::VectorXd>> &self,
             System<Eigen::AutoDiffScalar<Eigen::VectorXd>> const &subsystem,
             Context<Eigen::AutoDiffScalar<Eigen::VectorXd>> const &context) {
            return self.GetSubsystemContext(subsystem, context);
          },
          py::return_value_policy::reference_internal)
      .def(
          "GetUniquePeriodicDiscreteUpdateAttribute",
          static_cast<::std::optional<PeriodicEventData> (
              System<Eigen::AutoDiffScalar<Eigen::VectorXd>>::*)() const>(
              &System<Eigen::AutoDiffScalar<Eigen::VectorXd>>::
                  GetUniquePeriodicDiscreteUpdateAttribute),
          R"""(/** Gets whether there exists a unique periodic attribute that triggers 
one or more discrete update events (and, if so, returns that unique 
periodic attribute). Thus, this method can be used (1) as a test to 
determine whether a system's dynamics are at least partially governed by 
difference equations and (2) to obtain the difference equation update 
times. 
@returns optional<PeriodicEventData> Contains the periodic trigger 
attributes if the unique periodic attribute exists, otherwise `nullopt`. */)""")
      .def("GetWitnessFunctions",
           [](System<Eigen::AutoDiffScalar<Eigen::VectorXd>> &self,
              Context<Eigen::AutoDiffScalar<Eigen::VectorXd>> const &context,
              ::std::vector<
                  const WitnessFunction<::drake::AutoDiffXd> *,
                  std::allocator<const WitnessFunction<::drake::AutoDiffXd> *>>
                  *w) { return self.GetWitnessFunctions(context, w); })
      .def(
          "HasAnyDirectFeedthrough",
          static_cast<bool (System<Eigen::AutoDiffScalar<Eigen::VectorXd>>::*)()
                          const>(
              &System<Eigen::AutoDiffScalar<Eigen::VectorXd>>::
                  HasAnyDirectFeedthrough),
          R"""(/** Returns `true` if any of the inputs to the system might be directly 
fed through to any of its outputs and `false` otherwise. */)""")
      .def(
          "HasDirectFeedthrough",
          static_cast<bool (System<Eigen::AutoDiffScalar<Eigen::VectorXd>>::*)(
              int) const>(&System<Eigen::AutoDiffScalar<Eigen::VectorXd>>::
                              HasDirectFeedthrough),
          py::arg("output_port"),
          R"""(/** Returns true if there might be direct-feedthrough from any input port to 
the given @p output_port, and false otherwise. */)""")
      .def(
          "HasDirectFeedthrough",
          static_cast<bool (System<Eigen::AutoDiffScalar<Eigen::VectorXd>>::*)(
              int, int) const>(&System<Eigen::AutoDiffScalar<Eigen::VectorXd>>::
                                   HasDirectFeedthrough),
          py::arg("input_port"), py::arg("output_port"),
          R"""(/** Returns true if there might be direct-feedthrough from the given 
@p input_port to the given @p output_port, and false otherwise. */)""")
      .def(
          "HasInputPort",
          static_cast<bool (System<Eigen::AutoDiffScalar<Eigen::VectorXd>>::*)(
              ::std::string const &) const>(
              &System<Eigen::AutoDiffScalar<Eigen::VectorXd>>::HasInputPort),
          py::arg("port_name"),
          R"""(/** Returns true iff the system has an InputPort of the given @p 
 port_name. */)""")
      .def(
          "HasOutputPort",
          static_cast<bool (System<Eigen::AutoDiffScalar<Eigen::VectorXd>>::*)(
              ::std::string const &) const>(
              &System<Eigen::AutoDiffScalar<Eigen::VectorXd>>::HasOutputPort),
          py::arg("port_name"),
          R"""(/** Returns true iff the system has an OutputPort of the given @p 
 port_name. */)""")
      .def(
          "IsDifferenceEquationSystem",
          static_cast<bool (System<Eigen::AutoDiffScalar<Eigen::VectorXd>>::*)(
              double *) const>(&System<Eigen::AutoDiffScalar<Eigen::VectorXd>>::
                                   IsDifferenceEquationSystem),
          py::arg("time_period") = (double *)nullptr,
          R"""(/** Returns true iff the state dynamics of this system are governed 
exclusively by a difference equation on a single discrete state group 
and with a unique periodic update (having zero offset).  E.g., it is 
amenable to analysis of the form: 
  x[n+1] = f(x[n], u[n]) 
Note that we do NOT consider the number of input ports here, because 
in practice many systems of interest (e.g. MultibodyPlant) have input 
ports that are safely treated as constant during the analysis. 
Consider using get_input_port_selection() to choose one. 
 
@param[out] time_period if non-null, then iff the function 
returns `true`, then time_period is set to the period data 
returned from GetUniquePeriodicDiscreteUpdateAttribute().  If the 
function returns `false` (the system is not a difference equation 
system), then `time_period` does not receive a value. */)""")
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
      .def(
          "ToAutoDiffXd",
          static_cast<::std::unique_ptr<
              System<::drake::AutoDiffXd>,
              std::default_delete<System<::drake::AutoDiffXd>>> (
              System<Eigen::AutoDiffScalar<Eigen::VectorXd>>::*)() const>(
              &System<Eigen::AutoDiffScalar<Eigen::VectorXd>>::ToAutoDiffXd),
          R"""(/** Creates a deep copy of this System, transmogrified to use the autodiff 
scalar type, with a dynamic-sized vector of partial derivatives.  The 
result is never nullptr. 
@throws std::exception if this System does not support autodiff 
 
See @ref system_scalar_conversion for detailed background and examples 
related to scalar-type conversion support. */)""")
      .def(
          "ToAutoDiffXdMaybe",
          static_cast<::std::unique_ptr<
              System<::drake::AutoDiffXd>,
              std::default_delete<System<::drake::AutoDiffXd>>> (
              System<Eigen::AutoDiffScalar<Eigen::VectorXd>>::*)() const>(
              &System<
                  Eigen::AutoDiffScalar<Eigen::VectorXd>>::ToAutoDiffXdMaybe),
          R"""(/** Creates a deep copy of this system exactly like ToAutoDiffXd(), but 
returns nullptr if this System does not support autodiff, instead of 
throwing an exception. */)""")
      .def(
          "ToSymbolic",
          static_cast<::std::unique_ptr<
              System<::drake::symbolic::Expression>,
              std::default_delete<System<::drake::symbolic::Expression>>> (
              System<Eigen::AutoDiffScalar<Eigen::VectorXd>>::*)() const>(
              &System<Eigen::AutoDiffScalar<Eigen::VectorXd>>::ToSymbolic),
          R"""(/** Creates a deep copy of this System, transmogrified to use the symbolic 
scalar type. The result is never nullptr. 
@throws std::exception if this System does not support symbolic 
 
See @ref system_scalar_conversion for detailed background and examples 
related to scalar-type conversion support. */)""")
      .def(
          "ToSymbolicMaybe",
          static_cast<::std::unique_ptr<
              System<::drake::symbolic::Expression>,
              std::default_delete<System<::drake::symbolic::Expression>>> (
              System<Eigen::AutoDiffScalar<Eigen::VectorXd>>::*)() const>(
              &System<Eigen::AutoDiffScalar<Eigen::VectorXd>>::ToSymbolicMaybe),
          R"""(/** Creates a deep copy of this system exactly like ToSymbolic(), but returns 
nullptr if this System does not support symbolic, instead of throwing an 
exception. */)""")
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
           R"""(/** Returns the constraint at index @p constraint_index. 
@throws std::out_of_range for an invalid constraint_index. */)""",
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
          py::arg("port_index"),
          R"""(/** Returns the typed input port at index @p port_index. */)""",
          py::return_value_policy::reference_internal)
      .def(
          "get_input_port",
          static_cast<InputPort<Eigen::AutoDiffScalar<Eigen::VectorXd>> const &(
              System<Eigen::AutoDiffScalar<Eigen::VectorXd>>::*)() const>(
              &System<Eigen::AutoDiffScalar<Eigen::VectorXd>>::get_input_port),
          R"""(/** Convenience method for the case of exactly one input port. */)""",
          py::return_value_policy::reference_internal)
      .def(
          "get_input_port_selection",
          static_cast<InputPort<Eigen::AutoDiffScalar<Eigen::VectorXd>> const *(
              System<Eigen::AutoDiffScalar<Eigen::VectorXd>>::
                  *)(::std::variant<InputPortSelection,
                                    drake::TypeSafeIndex<InputPortTag>>)const>(
              &System<Eigen::AutoDiffScalar<Eigen::VectorXd>>::
                  get_input_port_selection),
          py::arg("port_index"),
          R"""(/** Returns the typed input port specified by the InputPortSelection or by 
the InputPortIndex.  Returns nullptr if no port is selected.  This is 
provided as a convenience method since many algorithms provide the same 
common default or optional port semantics. */)""",
          py::return_value_policy::reference_internal)
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
          py::arg("port_index"),
          R"""(/** Returns the typed output port at index @p port_index. */)""",
          py::return_value_policy::reference_internal)
      .def(
          "get_output_port",
          static_cast<
              OutputPort<Eigen::AutoDiffScalar<Eigen::VectorXd>> const &(
                  System<Eigen::AutoDiffScalar<Eigen::VectorXd>>::*)() const>(
              &System<Eigen::AutoDiffScalar<Eigen::VectorXd>>::get_output_port),
          R"""(/** Convenience method for the case of exactly one output port. */)""",
          py::return_value_policy::reference_internal)
      .def(
          "get_output_port_selection",
          static_cast<
              OutputPort<Eigen::AutoDiffScalar<Eigen::VectorXd>> const *(
                  System<Eigen::AutoDiffScalar<Eigen::VectorXd>>::
                      *)(::std::variant<OutputPortSelection,
                                        drake::TypeSafeIndex<OutputPortTag>>)
                  const>(&System<Eigen::AutoDiffScalar<Eigen::VectorXd>>::
                             get_output_port_selection),
          py::arg("port_index"),
          R"""(/** Returns the typed output port specified by the OutputPortSelection or by 
the OutputPortIndex.  Returns nullptr if no port is selected. This is 
provided as a convenience method since many algorithms provide the same 
common default or optional port semantics. */)""",
          py::return_value_policy::reference_internal)
      .def(
          "get_system_scalar_converter",
          static_cast<SystemScalarConverter const &(
              System<Eigen::AutoDiffScalar<Eigen::VectorXd>>::*)() const>(
              &System<Eigen::AutoDiffScalar<Eigen::VectorXd>>::
                  get_system_scalar_converter),
          R"""(/** (Advanced) Returns the SystemScalarConverter for this object.  This is an 
expert-level API intended for framework authors.  Most users should 
prefer the convenience helpers such as System::ToAutoDiffXd. */)""")
      .def(
          "get_time_derivatives_cache_entry",
          static_cast<CacheEntry const &(
              System<Eigen::AutoDiffScalar<Eigen::VectorXd>>::*)() const>(
              &System<Eigen::AutoDiffScalar<Eigen::VectorXd>>::
                  get_time_derivatives_cache_entry),
          R"""(/** (Advanced) Returns the CacheEntry used to cache time derivatives for 
EvalTimeDerivatives(). */)""")
      .def(
          "num_constraints",
          static_cast<int (System<Eigen::AutoDiffScalar<Eigen::VectorXd>>::*)()
                          const>(
              &System<Eigen::AutoDiffScalar<Eigen::VectorXd>>::num_constraints),
          R"""(/** Returns the number of constraints specified for the system. */)""")
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
