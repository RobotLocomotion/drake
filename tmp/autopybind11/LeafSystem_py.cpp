#include "drake/systems/framework/leaf_system.h"
#include <pybind11/eigen.h>
#include <pybind11/iostream.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

class LeafSystem_double_publicist
    : public ::drake::systems::LeafSystem<double> {
public:
  using ::drake::systems::LeafSystem<
      double>::AddTriggeredWitnessFunctionToCompositeEventCollection;
  using ::drake::systems::LeafSystem<double>::AllocateAbstractState;
  using ::drake::systems::LeafSystem<double>::AllocateContinuousState;
  using ::drake::systems::LeafSystem<double>::AllocateDiscreteState;
  using ::drake::systems::LeafSystem<double>::AllocateParameters;
  using ::drake::systems::LeafSystem<double>::DeclareAbstractInputPort;
  using ::drake::systems::LeafSystem<double>::DeclareAbstractInputPort;
  using ::drake::systems::LeafSystem<double>::DeclareAbstractOutputPort;
  using ::drake::systems::LeafSystem<double>::DeclareAbstractOutputPort;
  using ::drake::systems::LeafSystem<double>::DeclareAbstractParameter;
  using ::drake::systems::LeafSystem<double>::DeclareAbstractState;
  using ::drake::systems::LeafSystem<double>::DeclareAbstractState;
  using ::drake::systems::LeafSystem<double>::DeclareContinuousState;
  using ::drake::systems::LeafSystem<double>::DeclareContinuousState;
  using ::drake::systems::LeafSystem<double>::DeclareContinuousState;
  using ::drake::systems::LeafSystem<double>::DeclareContinuousState;
  using ::drake::systems::LeafSystem<double>::DeclareDiscreteState;
  using ::drake::systems::LeafSystem<double>::DeclareDiscreteState;
  using ::drake::systems::LeafSystem<double>::DeclareDiscreteState;
  using ::drake::systems::LeafSystem<double>::DeclareEqualityConstraint;
  using ::drake::systems::LeafSystem<
      double>::DeclareImplicitTimeDerivativesResidualSize;
  using ::drake::systems::LeafSystem<double>::DeclareInequalityConstraint;
  using ::drake::systems::LeafSystem<double>::DeclareNumericParameter;
  using ::drake::systems::LeafSystem<double>::DeclarePeriodicDiscreteUpdate;
  using ::drake::systems::LeafSystem<double>::DeclarePeriodicPublish;
  using ::drake::systems::LeafSystem<double>::DeclarePeriodicUnrestrictedUpdate;
  using ::drake::systems::LeafSystem<double>::DeclareVectorInputPort;
  using ::drake::systems::LeafSystem<double>::DeclareVectorInputPort;
  using ::drake::systems::LeafSystem<double>::DeclareVectorOutputPort;
  using ::drake::systems::LeafSystem<double>::DeclareVectorOutputPort;
  using ::drake::systems::LeafSystem<double>::DoCalcDiscreteVariableUpdates;
  using ::drake::systems::LeafSystem<double>::DoCalcNextUpdateTime;
  using ::drake::systems::LeafSystem<double>::DoCalcUnrestrictedUpdate;
  using ::drake::systems::LeafSystem<double>::DoCalcWitnessValue;
  using ::drake::systems::LeafSystem<double>::DoMakeLeafContext;
  using ::drake::systems::LeafSystem<double>::DoPublish;
  using ::drake::systems::LeafSystem<double>::DoValidateAllocatedLeafContext;
  using ::drake::systems::LeafSystem<double>::GetGraphvizFragment;
  using ::drake::systems::LeafSystem<double>::GetGraphvizInputPortToken;
  using ::drake::systems::LeafSystem<double>::GetGraphvizOutputPortToken;
  using ::drake::systems::LeafSystem<double>::MakeWitnessFunction;
  using ::drake::systems::LeafSystem<double>::MakeWitnessFunction;
};

class LeafSystem_Eigen_AutoDiffScalar_Eigen_VectorXd_publicist
    : public ::drake::systems::LeafSystem<
          Eigen::AutoDiffScalar<Eigen::VectorXd>> {
public:
  using ::drake::systems::LeafSystem<Eigen::AutoDiffScalar<Eigen::VectorXd>>::
      AddTriggeredWitnessFunctionToCompositeEventCollection;
  using ::drake::systems::LeafSystem<
      Eigen::AutoDiffScalar<Eigen::VectorXd>>::AllocateAbstractState;
  using ::drake::systems::LeafSystem<
      Eigen::AutoDiffScalar<Eigen::VectorXd>>::AllocateContinuousState;
  using ::drake::systems::LeafSystem<
      Eigen::AutoDiffScalar<Eigen::VectorXd>>::AllocateDiscreteState;
  using ::drake::systems::LeafSystem<
      Eigen::AutoDiffScalar<Eigen::VectorXd>>::AllocateParameters;
  using ::drake::systems::LeafSystem<
      Eigen::AutoDiffScalar<Eigen::VectorXd>>::DeclareAbstractInputPort;
  using ::drake::systems::LeafSystem<
      Eigen::AutoDiffScalar<Eigen::VectorXd>>::DeclareAbstractInputPort;
  using ::drake::systems::LeafSystem<
      Eigen::AutoDiffScalar<Eigen::VectorXd>>::DeclareAbstractOutputPort;
  using ::drake::systems::LeafSystem<
      Eigen::AutoDiffScalar<Eigen::VectorXd>>::DeclareAbstractOutputPort;
  using ::drake::systems::LeafSystem<
      Eigen::AutoDiffScalar<Eigen::VectorXd>>::DeclareAbstractParameter;
  using ::drake::systems::LeafSystem<
      Eigen::AutoDiffScalar<Eigen::VectorXd>>::DeclareAbstractState;
  using ::drake::systems::LeafSystem<
      Eigen::AutoDiffScalar<Eigen::VectorXd>>::DeclareAbstractState;
  using ::drake::systems::LeafSystem<
      Eigen::AutoDiffScalar<Eigen::VectorXd>>::DeclareContinuousState;
  using ::drake::systems::LeafSystem<
      Eigen::AutoDiffScalar<Eigen::VectorXd>>::DeclareContinuousState;
  using ::drake::systems::LeafSystem<
      Eigen::AutoDiffScalar<Eigen::VectorXd>>::DeclareContinuousState;
  using ::drake::systems::LeafSystem<
      Eigen::AutoDiffScalar<Eigen::VectorXd>>::DeclareContinuousState;
  using ::drake::systems::LeafSystem<
      Eigen::AutoDiffScalar<Eigen::VectorXd>>::DeclareDiscreteState;
  using ::drake::systems::LeafSystem<
      Eigen::AutoDiffScalar<Eigen::VectorXd>>::DeclareDiscreteState;
  using ::drake::systems::LeafSystem<
      Eigen::AutoDiffScalar<Eigen::VectorXd>>::DeclareDiscreteState;
  using ::drake::systems::LeafSystem<
      Eigen::AutoDiffScalar<Eigen::VectorXd>>::DeclareEqualityConstraint;
  using ::drake::systems::LeafSystem<Eigen::AutoDiffScalar<Eigen::VectorXd>>::
      DeclareImplicitTimeDerivativesResidualSize;
  using ::drake::systems::LeafSystem<
      Eigen::AutoDiffScalar<Eigen::VectorXd>>::DeclareInequalityConstraint;
  using ::drake::systems::LeafSystem<
      Eigen::AutoDiffScalar<Eigen::VectorXd>>::DeclareNumericParameter;
  using ::drake::systems::LeafSystem<
      Eigen::AutoDiffScalar<Eigen::VectorXd>>::DeclarePeriodicDiscreteUpdate;
  using ::drake::systems::LeafSystem<
      Eigen::AutoDiffScalar<Eigen::VectorXd>>::DeclarePeriodicPublish;
  using ::drake::systems::LeafSystem<Eigen::AutoDiffScalar<Eigen::VectorXd>>::
      DeclarePeriodicUnrestrictedUpdate;
  using ::drake::systems::LeafSystem<
      Eigen::AutoDiffScalar<Eigen::VectorXd>>::DeclareVectorInputPort;
  using ::drake::systems::LeafSystem<
      Eigen::AutoDiffScalar<Eigen::VectorXd>>::DeclareVectorInputPort;
  using ::drake::systems::LeafSystem<
      Eigen::AutoDiffScalar<Eigen::VectorXd>>::DeclareVectorOutputPort;
  using ::drake::systems::LeafSystem<
      Eigen::AutoDiffScalar<Eigen::VectorXd>>::DeclareVectorOutputPort;
  using ::drake::systems::LeafSystem<
      Eigen::AutoDiffScalar<Eigen::VectorXd>>::DoCalcDiscreteVariableUpdates;
  using ::drake::systems::LeafSystem<
      Eigen::AutoDiffScalar<Eigen::VectorXd>>::DoCalcNextUpdateTime;
  using ::drake::systems::LeafSystem<
      Eigen::AutoDiffScalar<Eigen::VectorXd>>::DoCalcUnrestrictedUpdate;
  using ::drake::systems::LeafSystem<
      Eigen::AutoDiffScalar<Eigen::VectorXd>>::DoCalcWitnessValue;
  using ::drake::systems::LeafSystem<
      Eigen::AutoDiffScalar<Eigen::VectorXd>>::DoMakeLeafContext;
  using ::drake::systems::LeafSystem<
      Eigen::AutoDiffScalar<Eigen::VectorXd>>::DoPublish;
  using ::drake::systems::LeafSystem<
      Eigen::AutoDiffScalar<Eigen::VectorXd>>::DoValidateAllocatedLeafContext;
  using ::drake::systems::LeafSystem<
      Eigen::AutoDiffScalar<Eigen::VectorXd>>::GetGraphvizFragment;
  using ::drake::systems::LeafSystem<
      Eigen::AutoDiffScalar<Eigen::VectorXd>>::GetGraphvizInputPortToken;
  using ::drake::systems::LeafSystem<
      Eigen::AutoDiffScalar<Eigen::VectorXd>>::GetGraphvizOutputPortToken;
  using ::drake::systems::LeafSystem<
      Eigen::AutoDiffScalar<Eigen::VectorXd>>::MakeWitnessFunction;
  using ::drake::systems::LeafSystem<
      Eigen::AutoDiffScalar<Eigen::VectorXd>>::MakeWitnessFunction;
};

class LeafSystem_float_publicist : public ::drake::systems::LeafSystem<float> {
public:
  using ::drake::systems::LeafSystem<
      float>::AddTriggeredWitnessFunctionToCompositeEventCollection;
  using ::drake::systems::LeafSystem<float>::AllocateAbstractState;
  using ::drake::systems::LeafSystem<float>::AllocateContinuousState;
  using ::drake::systems::LeafSystem<float>::AllocateDiscreteState;
  using ::drake::systems::LeafSystem<float>::AllocateParameters;
  using ::drake::systems::LeafSystem<float>::DeclareAbstractOutputPort;
  using ::drake::systems::LeafSystem<float>::DeclareAbstractOutputPort;
  using ::drake::systems::LeafSystem<float>::DeclareAbstractParameter;
  using ::drake::systems::LeafSystem<float>::DeclareAbstractState;
  using ::drake::systems::LeafSystem<float>::DeclareAbstractState;
  using ::drake::systems::LeafSystem<float>::DeclareContinuousState;
  using ::drake::systems::LeafSystem<float>::DeclareContinuousState;
  using ::drake::systems::LeafSystem<float>::DeclareContinuousState;
  using ::drake::systems::LeafSystem<float>::DeclareContinuousState;
  using ::drake::systems::LeafSystem<float>::DeclareDiscreteState;
  using ::drake::systems::LeafSystem<float>::DeclareDiscreteState;
  using ::drake::systems::LeafSystem<float>::DeclareDiscreteState;
  using ::drake::systems::LeafSystem<float>::DeclareEqualityConstraint;
  using ::drake::systems::LeafSystem<
      float>::DeclareImplicitTimeDerivativesResidualSize;
  using ::drake::systems::LeafSystem<float>::DeclareInequalityConstraint;
  using ::drake::systems::LeafSystem<float>::DeclareNumericParameter;
  using ::drake::systems::LeafSystem<float>::DeclarePeriodicDiscreteUpdate;
  using ::drake::systems::LeafSystem<float>::DeclarePeriodicPublish;
  using ::drake::systems::LeafSystem<float>::DeclarePeriodicUnrestrictedUpdate;
  using ::drake::systems::LeafSystem<float>::DeclareVectorOutputPort;
  using ::drake::systems::LeafSystem<float>::DeclareVectorOutputPort;
  using ::drake::systems::LeafSystem<float>::DoCalcDiscreteVariableUpdates;
  using ::drake::systems::LeafSystem<float>::DoCalcNextUpdateTime;
  using ::drake::systems::LeafSystem<float>::DoCalcUnrestrictedUpdate;
  using ::drake::systems::LeafSystem<float>::DoMakeLeafContext;
  using ::drake::systems::LeafSystem<float>::DoPublish;
  using ::drake::systems::LeafSystem<float>::DoValidateAllocatedLeafContext;
  using ::drake::systems::LeafSystem<float>::GetGraphvizFragment;
  using ::drake::systems::LeafSystem<float>::GetGraphvizOutputPortToken;
  using ::drake::systems::LeafSystem<float>::MakeWitnessFunction;
  using ::drake::systems::LeafSystem<float>::MakeWitnessFunction;
};

namespace py = pybind11;
void apb11_pydrake_LeafSystem_py_register(py::module &m) {
  static bool called = false;
  if (called) {
    return;
  }
  called = true;
  using namespace drake::systems;

  py::class_<LeafSystem<double>, System<double>> PyLeafSystem_double(
      m, "LeafSystem_double");

  PyLeafSystem_double
      .def("AddTriggeredWitnessFunctionToCompositeEventCollection",
           static_cast<void (LeafSystem<double>::*)(
               Event<double> *, CompositeEventCollection<double> *) const>(
               &LeafSystem_double_publicist::
                   AddTriggeredWitnessFunctionToCompositeEventCollection),
           py::arg("event"), py::arg("events"))
      .def(
          "AllocateAbstractState",
          static_cast<::std::unique_ptr<
              drake::systems::AbstractValues,
              std::default_delete<drake::systems::AbstractValues>> (
              LeafSystem<double>::*)() const>(
              &LeafSystem_double_publicist::AllocateAbstractState),
          R"""(/** Returns a copy of the states declared in DeclareAbstractState() calls. */)""")
      .def(
          "AllocateCompositeEventCollection",
          static_cast<::std::unique_ptr<
              drake::systems::CompositeEventCollection<double>,
              std::default_delete<drake::systems::CompositeEventCollection<
                  double>>> (LeafSystem<double>::*)() const>(
              &LeafSystem<double>::AllocateCompositeEventCollection),
          R"""(/** Allocates a CompositeEventCollection object for this system. 
@sa System::AllocateCompositeEventCollection(). */)""")
      .def(
          "AllocateContext",
          static_cast<::std::unique_ptr<
              drake::systems::LeafContext<double>,
              std::default_delete<drake::systems::LeafContext<double>>> (
              LeafSystem<double>::*)() const>(
              &LeafSystem<double>::AllocateContext),
          R"""(/** Shadows System<T>::AllocateContext to provide a more concrete return 
type LeafContext<T>. */)""")
      .def("AllocateContinuousState",
           static_cast<::std::unique_ptr<
               drake::systems::ContinuousState<double>,
               std::default_delete<drake::systems::ContinuousState<double>>> (
               LeafSystem<double>::*)() const>(
               &LeafSystem_double_publicist::AllocateContinuousState),
           R"""(/** Returns a copy of the state declared in the most recent 
DeclareContinuousState() call, or else a zero-sized state if that method 
has never been called. */)""")
      .def(
          "AllocateDiscreteState",
          static_cast<::std::unique_ptr<
              drake::systems::DiscreteValues<double>,
              std::default_delete<drake::systems::DiscreteValues<double>>> (
              LeafSystem<double>::*)() const>(
              &LeafSystem_double_publicist::AllocateDiscreteState),
          R"""(/** Returns a copy of the states declared in DeclareDiscreteState() calls. */)""")
      .def("AllocateDiscreteVariables",
           static_cast<::std::unique_ptr<
               drake::systems::DiscreteValues<double>,
               std::default_delete<drake::systems::DiscreteValues<double>>> (
               LeafSystem<double>::*)() const>(
               &LeafSystem<double>::AllocateDiscreteVariables))
      .def(
          "AllocateForcedDiscreteUpdateEventCollection",
          static_cast<::std::unique_ptr<
              drake::systems::EventCollection<
                  drake::systems::DiscreteUpdateEvent<double>>,
              std::default_delete<drake::systems::EventCollection<
                  drake::systems::DiscreteUpdateEvent<double>>>> (
              LeafSystem<double>::*)() const>(
              &LeafSystem<double>::AllocateForcedDiscreteUpdateEventCollection))
      .def("AllocateForcedPublishEventCollection",
           static_cast<::std::unique_ptr<
               drake::systems::EventCollection<
                   drake::systems::PublishEvent<double>>,
               std::default_delete<drake::systems::EventCollection<
                   drake::systems::PublishEvent<double>>>> (
               LeafSystem<double>::*)() const>(
               &LeafSystem<double>::AllocateForcedPublishEventCollection))
      .def("AllocateForcedUnrestrictedUpdateEventCollection",
           static_cast<::std::unique_ptr<
               drake::systems::EventCollection<
                   drake::systems::UnrestrictedUpdateEvent<double>>,
               std::default_delete<drake::systems::EventCollection<
                   drake::systems::UnrestrictedUpdateEvent<double>>>> (
               LeafSystem<double>::*)() const>(
               &LeafSystem<
                   double>::AllocateForcedUnrestrictedUpdateEventCollection))
      .def(
          "AllocateParameters",
          static_cast<::std::unique_ptr<
              drake::systems::Parameters<double>,
              std::default_delete<drake::systems::Parameters<double>>> (
              LeafSystem<double>::*)() const>(
              &LeafSystem_double_publicist::AllocateParameters),
          R"""(/** Returns a copy of the parameters declared in DeclareNumericParameter() 
and DeclareAbstractParameter() calls. */)""")
      .def("AllocateTimeDerivatives",
           static_cast<::std::unique_ptr<
               drake::systems::ContinuousState<double>,
               std::default_delete<drake::systems::ContinuousState<double>>> (
               LeafSystem<double>::*)() const>(
               &LeafSystem<double>::AllocateTimeDerivatives))
      .def(
          "DeclareAbstractInputPort",
          static_cast<InputPort<double> &(
              LeafSystem<double>::
                  *)(::std::variant<
                         std::basic_string<char, std::char_traits<char>,
                                           std::allocator<char>>,
                         drake::systems::UseDefaultName>,
                     ::drake::AbstractValue const &)>(
              &LeafSystem_double_publicist::DeclareAbstractInputPort),
          py::arg("name"), py::arg("model_value"),
          R"""(/** Declares an abstract-valued input port using the given @p model_value. 
This is the best way to declare LeafSystem abstract input ports. 
 
Any port connected to this input, and any call to FixValue for this 
input, must provide for values whose type matches this @p model_value. 
 
@see System::DeclareInputPort() for more information. */)""")
      .def(
          "DeclareAbstractInputPort",
          static_cast<InputPort<double> &(
              LeafSystem<double>::*)(::drake::AbstractValue const &)>(
              &LeafSystem_double_publicist::DeclareAbstractInputPort),
          py::arg("model_value"),
          R"""(/** See the nearly identical signature with an additional (first) argument 
specifying the port name.  This version will be deprecated as discussed 
in #9447. */)""")
      .def(
          "DeclareAbstractOutputPort",
          [](LeafSystem<double> &self,
             ::std::variant<std::basic_string<char, std::char_traits<char>,
                                              std::allocator<char>>,
                            drake::systems::UseDefaultName>
                 name,
             LeafOutputPort<double>::AllocCallback alloc_function,
             LeafOutputPort<double>::CalcCallback calc_function,
             ::std::set<
                 drake::TypeSafeIndex<drake::systems::DependencyTag>,
                 std::less<drake::TypeSafeIndex<drake::systems::DependencyTag>>,
                 std::allocator<
                     drake::TypeSafeIndex<drake::systems::DependencyTag>>>
                 prerequisites_of_calc) {
            return self.DeclareAbstractOutputPort(
                name, alloc_function, calc_function, prerequisites_of_calc);
          },
          R"""(/** (Advanced) Declares an abstract-valued output port using the given 
allocator and calculator functions provided in their most generic forms. 
If you have a member function available use one of the other signatures. 
@see LeafOutputPort::AllocCallback, LeafOutputPort::CalcCallback */)""")
      .def(
          "DeclareAbstractOutputPort",
          [](LeafSystem<double> &self,
             LeafOutputPort<double>::AllocCallback alloc_function,
             LeafOutputPort<double>::CalcCallback calc_function,
             ::std::set<
                 drake::TypeSafeIndex<drake::systems::DependencyTag>,
                 std::less<drake::TypeSafeIndex<drake::systems::DependencyTag>>,
                 std::allocator<
                     drake::TypeSafeIndex<drake::systems::DependencyTag>>>
                 prerequisites_of_calc) {
            return self.DeclareAbstractOutputPort(alloc_function, calc_function,
                                                  prerequisites_of_calc);
          },
          R"""(/** See the nearly identical signature with an additional (first) argument 
specifying the port name.  This version will be deprecated as discussed 
in #9447. */)""")
      .def(
          "DeclareAbstractParameter",
          static_cast<int (LeafSystem<double>::*)(
              ::drake::AbstractValue const &)>(
              &LeafSystem_double_publicist::DeclareAbstractParameter),
          py::arg("model_value"),
          R"""(/** Declares an abstract parameter using the given @p model_value. 
LeafSystem's default implementation of SetDefaultParameters() will reset 
parameters to their model values.  Returns the index of the new 
parameter. */)""")
      .def("DeclareAbstractState",
           static_cast<AbstractStateIndex (LeafSystem<double>::*)(
               ::drake::AbstractValue const &)>(
               &LeafSystem_double_publicist::DeclareAbstractState),
           py::arg("abstract_state"),
           R"""(/** Declares an abstract state. 
@param abstract_state The abstract state model value. 
@return index of the declared abstract state. */)""")
      .def(
          "DeclareAbstractState",
          [](LeafSystem<double> &self, drake::AbstractValue abstract_state) {
            return self.DeclareAbstractState(
                std::make_unique<drake::AbstractValue>(abstract_state));
          },
          R"""(/** Declares an abstract state. 
@param abstract_state The abstract state model value.  The internal model 
value will contain a copy of `value` (not retain a pointer to `value`). 
@return index of the declared abstract state. */)""")
      .def(
          "DeclareContinuousState",
          static_cast<void (LeafSystem<double>::*)(int)>(
              &LeafSystem_double_publicist::DeclareContinuousState),
          py::arg("num_state_variables"),
          R"""(/** Declares that this System should reserve continuous state with 
@p num_state_variables state variables, which have no second-order 
structure. */)""")
      .def(
          "DeclareContinuousState",
          static_cast<void (LeafSystem<double>::*)(int, int, int)>(
              &LeafSystem_double_publicist::DeclareContinuousState),
          py::arg("num_q"), py::arg("num_v"), py::arg("num_z"),
          R"""(/** Declares that this System should reserve continuous state with @p num_q 
generalized positions, @p num_v generalized velocities, and @p num_z 
miscellaneous state variables. */)""")
      .def(
          "DeclareContinuousState",
          static_cast<void (LeafSystem<double>::*)(
              BasicVector<double> const &)>(
              &LeafSystem_double_publicist::DeclareContinuousState),
          py::arg("model_vector"),
          R"""(/** Declares that this System should reserve continuous state with 
@p model_vector.size() miscellaneous state variables, stored in a 
vector cloned from @p model_vector. */)""")
      .def(
          "DeclareContinuousState",
          static_cast<void (LeafSystem<double>::*)(BasicVector<double> const &,
                                                   int, int, int)>(
              &LeafSystem_double_publicist::DeclareContinuousState),
          py::arg("model_vector"), py::arg("num_q"), py::arg("num_v"),
          py::arg("num_z"),
          R"""(/** Declares that this System should reserve continuous state with @p num_q 
generalized positions, @p num_v generalized velocities, and @p num_z 
miscellaneous state variables, stored in a vector cloned from 
@p model_vector. Aborts if @p model_vector has the wrong size. If the 
@p model_vector declares any VectorBase::GetElementBounds() 
constraints, they will be re-declared as inequality constraints on this 
system (see DeclareInequalityConstraint()). */)""")
      .def(
          "DeclareDiscreteState",
          static_cast<DiscreteStateIndex (LeafSystem<double>::*)(
              BasicVector<double> const &)>(
              &LeafSystem_double_publicist::DeclareDiscreteState),
          py::arg("model_vector"),
          R"""(/** Declares a discrete state group with @p model_vector.size() state 
variables, stored in a vector cloned from @p model_vector (preserving the 
concrete type and value). */)""")
      .def("DeclareDiscreteState",
           [](LeafSystem<double> &self,
              ::Eigen::Ref<const Eigen::Matrix<double, -1, 1, 0, -1, 1>, 0,
                           Eigen::InnerStride<1>> const &vector) {
             return self.DeclareDiscreteState(vector);
           })
      .def(
          "DeclareDiscreteState",
          static_cast<DiscreteStateIndex (LeafSystem<double>::*)(int)>(
              &LeafSystem_double_publicist::DeclareDiscreteState),
          py::arg("num_state_variables"),
          R"""(/** Declares a discrete state group with @p num_state_variables state 
variables, stored in a BasicVector initialized to be all-zero. If you want 
non-zero initial values, use an alternate DeclareDiscreteState() signature 
that accepts a `model_vector` parameter. 
@pre `num_state_variables` must be non-negative. */)""")
      .def("DeclareEqualityConstraint",
           [](LeafSystem<double> &self,
              Eigen::Ref<::std::function<void(
                             const drake::systems::Context<double> &,
                             Eigen::Matrix<double, -1, 1, 0, -1, 1> *)>,
                         0, Eigen::Stride<Eigen::Dynamic, Eigen::Dynamic>>
                  calc,
              int count, ::std::string description) {
             return self.DeclareEqualityConstraint(calc, count, description);
           })
      .def("DeclareImplicitTimeDerivativesResidualSize",
           static_cast<void (LeafSystem<double>::*)(int)>(
               &LeafSystem_double_publicist::
                   DeclareImplicitTimeDerivativesResidualSize),
           py::arg("n"),
           R"""(/** (Advanced) Overrides the default size for the implicit time 
derivatives residual. If no value is set, the default size is 
n=num_continuous_states(). 
 
@param[in] n The size of the residual vector output argument of 
             System::CalcImplicitTimeDerivativesResidual(). If n <= 0 
             restore to the default, num_continuous_states(). 
 
@see implicit_time_derivatives_residual_size() 
@see System::CalcImplicitTimeDerivativesResidual() */)""")
      .def("DeclareInequalityConstraint",
           [](LeafSystem<double> &self,
              Eigen::Ref<::std::function<void(
                             const drake::systems::Context<double> &,
                             Eigen::Matrix<double, -1, 1, 0, -1, 1> *)>,
                         0, Eigen::Stride<Eigen::Dynamic, Eigen::Dynamic>>
                  calc,
              SystemConstraintBounds bounds, ::std::string description) {
             return self.DeclareInequalityConstraint(calc, bounds, description);
           })
      .def(
          "DeclareNumericParameter",
          static_cast<int (LeafSystem<double>::*)(BasicVector<double> const &)>(
              &LeafSystem_double_publicist::DeclareNumericParameter),
          py::arg("model_vector"),
          R"""(/** Declares a numeric parameter using the given @p model_vector. 
LeafSystem's default implementation of SetDefaultParameters() will reset 
parameters to their model vectors.  If the @p model_vector declares any 
VectorBase::GetElementBounds() constraints, they will be re-declared as 
inequality constraints on this system (see 
DeclareInequalityConstraint()).  Returns the index of the new parameter. */)""")
      .def(
          "DeclarePeriodicDiscreteUpdate",
          static_cast<void (LeafSystem<double>::*)(double, double)>(
              &LeafSystem_double_publicist::DeclarePeriodicDiscreteUpdate),
          py::arg("period_sec"), py::arg("offset_sec") = double(0),
          R"""(/** (To be deprecated) Declares a periodic discrete update event that invokes 
the DiscreteUpdate() dispatcher but does not provide a handler 
function. This does guarantee that a Simulator step will end exactly at 
the update time, but otherwise has no effect unless the 
DoDiscreteUpdate() dispatcher has been overloaded (not recommended). */)""")
      .def(
          "DeclarePeriodicPublish",
          static_cast<void (LeafSystem<double>::*)(double, double)>(
              &LeafSystem_double_publicist::DeclarePeriodicPublish),
          py::arg("period_sec"), py::arg("offset_sec") = double(0),
          R"""(/** (To be deprecated) Declares a periodic publish event that invokes the 
Publish() dispatcher but does not provide a handler function. This does 
guarantee that a Simulator step will end exactly at the publish time, 
but otherwise has no effect unless the DoPublish() dispatcher has been 
overloaded (not recommended). */)""")
      .def(
          "DeclarePeriodicUnrestrictedUpdate",
          static_cast<void (LeafSystem<double>::*)(double, double)>(
              &LeafSystem_double_publicist::DeclarePeriodicUnrestrictedUpdate),
          py::arg("period_sec"), py::arg("offset_sec") = double(0),
          R"""(/** (To be deprecated) Declares a periodic unrestricted update event that 
invokes the UnrestrictedUpdate() dispatcher but does not provide a handler 
function. This does guarantee that a Simulator step will end exactly at 
the update time, but otherwise has no effect unless the 
DoUnrestrictedUpdate() dispatcher has been overloaded (not recommended). */)""")
      .def(
          "DeclareVectorInputPort",
          static_cast<InputPort<double> &(
              LeafSystem<double>::
                  *)(::std::variant<
                         std::basic_string<char, std::char_traits<char>,
                                           std::allocator<char>>,
                         drake::systems::UseDefaultName>,
                     BasicVector<double> const &,
                     ::std::optional<drake::RandomDistribution>)>(
              &LeafSystem_double_publicist::DeclareVectorInputPort),
          py::arg("name"), py::arg("model_vector"),
          py::arg("random_type") =
              ::std::optional<drake::RandomDistribution>(std::nullopt),
          R"""(/** Declares a vector-valued input port using the given @p model_vector. 
This is the best way to declare LeafSystem input ports that require 
subclasses of BasicVector.  The port's size and type will be the same as 
model_vector. If the port is intended to model a random noise or 
disturbance input, @p random_type can (optionally) be used to label it 
as such.  If the @p model_vector declares any 
VectorBase::GetElementBounds() constraints, they will be 
re-declared as inequality constraints on this system (see 
DeclareInequalityConstraint()). 
 
@see System::DeclareInputPort() for more information. */)""")
      .def(
          "DeclareVectorInputPort",
          static_cast<InputPort<double> &(
              LeafSystem<double>::*)(BasicVector<double> const &,
                                     ::std::optional<
                                         drake::RandomDistribution>)>(
              &LeafSystem_double_publicist::DeclareVectorInputPort),
          py::arg("model_vector"),
          py::arg("random_type") =
              ::std::optional<drake::RandomDistribution>(std::nullopt),
          R"""(/** See the nearly identical signature with an additional (first) argument 
specifying the port name.  This version will be deprecated as discussed 
in #9447. */)""")
      .def(
          "DeclareVectorOutputPort",
          static_cast<LeafOutputPort<double> &(
              LeafSystem<double>::
                  *)(::std::variant<
                         std::basic_string<char, std::char_traits<char>,
                                           std::allocator<char>>,
                         drake::systems::UseDefaultName>,
                     BasicVector<double> const &,
                     LeafOutputPort<double>::CalcVectorCallback,
                     ::std::set<
                         drake::TypeSafeIndex<drake::systems::DependencyTag>,
                         std::less<drake::TypeSafeIndex<
                             drake::systems::DependencyTag>>,
                         std::allocator<drake::TypeSafeIndex<
                             drake::systems::DependencyTag>>>)>(
              &LeafSystem_double_publicist::DeclareVectorOutputPort),
          py::arg("name"), py::arg("model_vector"),
          py::arg("vector_calc_function"),
          py::arg("prerequisites_of_calc") = ::std::set<
              drake::TypeSafeIndex<drake::systems::DependencyTag>,
              std::less<drake::TypeSafeIndex<drake::systems::DependencyTag>>,
              std::allocator<
                  drake::TypeSafeIndex<drake::systems::DependencyTag>>>(
              {drake::systems::SystemBase::all_sources_ticket()}),
          R"""(/** (Advanced) Declares a vector-valued output port using the given 
`model_vector` and a function for calculating the port's value at runtime. 
The port's size will be model_vector.size(), and the default allocator for 
the port will be model_vector.Clone(). Note that this takes the calculator 
function in its most generic form; if you have a member function available 
use one of the other signatures. 
@see LeafOutputPort::CalcVectorCallback */)""")
      .def(
          "DeclareVectorOutputPort",
          static_cast<LeafOutputPort<double> &(
              LeafSystem<double>::
                  *)(BasicVector<double> const &,
                     LeafOutputPort<double>::CalcVectorCallback,
                     ::std::set<
                         drake::TypeSafeIndex<drake::systems::DependencyTag>,
                         std::less<drake::TypeSafeIndex<
                             drake::systems::DependencyTag>>,
                         std::allocator<drake::TypeSafeIndex<
                             drake::systems::DependencyTag>>>)>(
              &LeafSystem_double_publicist::DeclareVectorOutputPort),
          py::arg("model_vector"), py::arg("vector_calc_function"),
          py::arg("prerequisites_of_calc") = ::std::set<
              drake::TypeSafeIndex<drake::systems::DependencyTag>,
              std::less<drake::TypeSafeIndex<drake::systems::DependencyTag>>,
              std::allocator<
                  drake::TypeSafeIndex<drake::systems::DependencyTag>>>(
              {drake::systems::SystemBase::all_sources_ticket()}),
          R"""(/** See the nearly identical signature with an additional (first) argument 
specifying the port name.  This version will be deprecated as discussed 
in #9447. */)""")
      .def("DoAllocateContext",
           static_cast<::std::unique_ptr<
               drake::systems::ContextBase,
               std::default_delete<drake::systems::ContextBase>> (
               LeafSystem<double>::*)() const>(
               &LeafSystem<double>::DoAllocateContext))
      .def(
          "DoCalcDiscreteVariableUpdates",
          static_cast<void (LeafSystem<double>::*)(
              Context<double> const &,
              ::std::vector<
                  const drake::systems::DiscreteUpdateEvent<double> *,
                  std::allocator<const drake::systems::DiscreteUpdateEvent<
                      double> *>> const &,
              DiscreteValues<double> *) const>(
              &LeafSystem_double_publicist::DoCalcDiscreteVariableUpdates),
          py::arg("context"), py::arg("events"), py::arg("discrete_state"),
          R"""(/** Derived-class event dispatcher for all simultaneous discrete update 
events. Override this in your derived LeafSystem only if you require 
behavior other than the default dispatch behavior (not common). 
The default behavior is to traverse events in the arbitrary order they 
appear in @p events, and for each event that has a callback function, 
to invoke the callback with @p context, that event, and @p discrete_state. 
Note that the same (possibly modified) @p discrete_state is passed to 
subsequent callbacks. 
 
Do not override this just to handle an event -- instead declare the event 
and a handler callback for it using one of the 
`Declare...DiscreteUpdateEvent()` methods. 
 
This method is called only from the virtual 
DispatchDiscreteVariableUpdateHandler(), which is only called from 
the public non-virtual CalcDiscreteVariableUpdates(), which will already 
have error-checked the parameters so you don't have to. In particular, 
implementations may assume that @p context is valid; that 
@p discrete_state is non-null, and that the referenced object has the 
same constituent structure as was produced by AllocateDiscreteVariables(). 
 
@param[in] context The "before" state. 
@param[in] events All the discrete update events that need handling. 
@param[in,out] discrete_state The current state of the system on input; 
the desired state of the system on return. */)""")
      .def(
          "DoCalcNextUpdateTime",
          static_cast<void (LeafSystem<double>::*)(
              Context<double> const &, CompositeEventCollection<double> *,
              double *) const>(
              &LeafSystem_double_publicist::DoCalcNextUpdateTime),
          py::arg("context"), py::arg("events"), py::arg("time"),
          R"""(/** Computes the next update time based on the configured periodic events, for 
scalar types that are arithmetic, or aborts for scalar types that are not 
arithmetic. Subclasses that require aperiodic events should override, but 
be sure to invoke the parent class implementation at the start of the 
override if you want periodic events to continue to be handled. 
 
@post `time` is set to a value greater than or equal to 
      `context.get_time()` on return. 
@warning If you override this method, think carefully before setting 
         `time` to `context.get_time()` on return, which can inadvertently 
         cause simulations of systems derived from %LeafSystem to loop 
         interminably. Such a loop will occur if, for example, the 
         event(s) does not modify the state. */)""")
      .def(
          "DoCalcUnrestrictedUpdate",
          static_cast<void (LeafSystem<double>::*)(
              Context<double> const &,
              ::std::vector<
                  const drake::systems::UnrestrictedUpdateEvent<double> *,
                  std::allocator<const drake::systems::UnrestrictedUpdateEvent<
                      double> *>> const &,
              State<double> *) const>(
              &LeafSystem_double_publicist::DoCalcUnrestrictedUpdate),
          py::arg("context"), py::arg("events"), py::arg("state"),
          R"""(/** Derived-class event dispatcher for all simultaneous unrestricted update 
events. Override this in your derived LeafSystem only if you require 
behavior other than the default dispatch behavior (not common). 
The default behavior is to traverse events in the arbitrary order they 
appear in @p events, and for each event that has a callback function, 
to invoke the callback with @p context, that event, and @p state. 
Note that the same (possibly modified) @p state is passed to subsequent 
callbacks. 
 
Do not override this just to handle an event -- instead declare the event 
and a handler callback for it using one of the 
`Declare...UnrestrictedUpdateEvent()` methods. 
 
This method is called only from the virtual 
DispatchUnrestrictedUpdateHandler(), which is only called from the 
non-virtual public CalcUnrestrictedUpdate(), which will already have 
error-checked the parameters so you don't have to. In particular, 
implementations may assume that the @p context is valid; that @p state 
is non-null, and that the referenced object has the same constituent 
structure as the state in @p context. 
 
@param[in]     context The "before" state that is to be used to calculate 
                       the returned state update. 
@param[in]     events All the unrestricted update events that need 
                      handling. 
@param[in,out] state   The current state of the system on input; the 
                       desired state of the system on return. */)""")
      .def("DoCalcWitnessValue",
           static_cast<double (LeafSystem<double>::*)(
               Context<double> const &, WitnessFunction<double> const &) const>(
               &LeafSystem_double_publicist::DoCalcWitnessValue),
           py::arg("context"), py::arg("witness_func"))
      .def(
          "DoMakeLeafContext",
          static_cast<::std::unique_ptr<
              drake::systems::LeafContext<double>,
              std::default_delete<drake::systems::LeafContext<double>>> (
              LeafSystem<double>::*)() const>(
              &LeafSystem_double_publicist::DoMakeLeafContext),
          R"""(/** Provides a new instance of the leaf context for this system. Derived 
leaf systems with custom derived leaf system contexts should override this 
to provide a context of the appropriate type. The returned context should 
be "empty"; invoked by AllocateContext(), the caller will take the 
responsibility to initialize the core LeafContext data. The default 
implementation provides a default-constructed `LeafContext<T>`. */)""")
      .def(
          "DoPublish",
          static_cast<void (LeafSystem<double>::*)(
              Context<double> const &,
              ::std::vector<const drake::systems::PublishEvent<double> *,
                            std::allocator<const drake::systems::PublishEvent<
                                double> *>> const &) const>(
              &LeafSystem_double_publicist::DoPublish),
          py::arg("context"), py::arg("events"),
          R"""(/** Derived-class event dispatcher for all simultaneous publish events 
in @p events. Override this in your derived LeafSystem only if you require 
behavior other than the default dispatch behavior (not common). 
The default behavior is to traverse events in the arbitrary order they 
appear in @p events, and for each event that has a callback function, 
to invoke the callback with @p context and that event. 
 
Do not override this just to handle an event -- instead declare the event 
and a handler callback for it using one of the `Declare...PublishEvent()` 
methods. 
 
This method is called only from the virtual DispatchPublishHandler, which 
is only called from the public non-virtual Publish(), which will have 
already error-checked @p context so you may assume that it is valid. 
 
@param[in] context Const current context. 
@param[in] events All the publish events that need handling. */)""")
      .def(
          "DoValidateAllocatedLeafContext",
          static_cast<void (LeafSystem<double>::*)(LeafContext<double> const &)
                          const>(
              &LeafSystem_double_publicist::DoValidateAllocatedLeafContext),
          py::arg("context"),
          R"""(/** Derived classes that impose restrictions on what resources are permitted 
should check those restrictions by implementing this. For example, a 
derived class might require a single input and single output. Note that 
the supplied Context will be complete except that input and output 
dependencies on peer and parent subcontexts will not yet have been set up, 
so you may not consider them for validation. 
The default implementation does nothing. */)""")
      .def("GetDirectFeedthroughs",
           static_cast<
               ::std::multimap<int, int, std::less<int>,
                               std::allocator<std::pair<const int, int>>> (
                   LeafSystem<double>::*)() const>(
               &LeafSystem<double>::GetDirectFeedthroughs))
      .def(
          "GetGraphvizFragment",
          static_cast<void (LeafSystem<double>::*)(int, ::std::stringstream *)
                          const>(
              &LeafSystem_double_publicist::GetGraphvizFragment),
          py::arg("max_depth"), py::arg("dot"),
          R"""(/** Emits a graphviz fragment for this System. Leaf systems are visualized as 
records. For instance, a leaf system with 2 inputs and 1 output is: 
 
@verbatim 
123456 [shape= record, label="name | {<u0> 0 |<y0> 0} | {<u1> 1 | }"]; 
@endverbatim 
 
which looks like: 
 
@verbatim 
+------------+----+ 
| name  | u0 | u1 | 
|       | y0 |    | 
+-------+----+----+ 
@endverbatim */)""")
      .def("GetGraphvizInputPortToken",
           static_cast<void (LeafSystem<double>::*)(
               InputPort<double> const &, int, ::std::stringstream *) const>(
               &LeafSystem_double_publicist::GetGraphvizInputPortToken),
           py::arg("port"), py::arg("max_depth"), py::arg("dot"))
      .def("GetGraphvizOutputPortToken",
           static_cast<void (LeafSystem<double>::*)(
               OutputPort<double> const &, int, ::std::stringstream *) const>(
               &LeafSystem_double_publicist::GetGraphvizOutputPortToken),
           py::arg("port"), py::arg("max_depth"), py::arg("dot"))
      .def(
          "MakeWitnessFunction",
          static_cast<::std::unique_ptr<
              drake::systems::WitnessFunction<double>,
              std::default_delete<drake::systems::WitnessFunction<double>>> (
              LeafSystem<double>::*)(
              ::std::string const &,
              WitnessFunctionDirection const &,
              ::std::function<double(const drake::systems::Context<double> &)>)
                          const>(
              &LeafSystem_double_publicist::MakeWitnessFunction),
          py::arg("description"), py::arg("direction_type"), py::arg("calc"),
          R"""(/** Constructs the witness function with the given description (used primarily 
for debugging and logging), direction type, and calculator function; and 
with no event object. 
 
@note In order for the witness function to be used, you MUST 
overload System::DoGetWitnessFunctions(). */)""")
      .def(
          "MakeWitnessFunction",
          static_cast<::std::unique_ptr<
              drake::systems::WitnessFunction<double>,
              std::default_delete<drake::systems::WitnessFunction<double>>> (
              LeafSystem<double>::*)(
              ::std::string const &,
              WitnessFunctionDirection const &,
              ::std::function<double(const drake::systems::Context<double> &)>,
              Event<double> const &) const>(
              &LeafSystem_double_publicist::MakeWitnessFunction),
          py::arg("description"), py::arg("direction_type"), py::arg("calc"),
          py::arg("e"),
          R"""(/** Constructs the witness function with the given description (used primarily 
for debugging and logging), direction type, and calculator 
function, and with an object corresponding to the event that is to be 
dispatched when this witness function triggers. Example types of event 
objects are publish, discrete variable update, unrestricted update events. 
A clone of the event will be owned by the newly constructed 
WitnessFunction. 
 
@note In order for the witness function to be used, you MUST 
overload System::DoGetWitnessFunctions(). */)""")
      .def(
          "SetDefaultParameters",
          static_cast<void (LeafSystem<double>::*)(Context<double> const &,
                                                   Parameters<double> *) const>(
              &LeafSystem<double>::SetDefaultParameters),
          py::arg("context"), py::arg("parameters"),
          R"""(/** Default implementation: sets all numeric parameters to the model vector 
given to DeclareNumericParameter, or else if no model was provided sets 
the numeric parameter to one.  It sets all abstract parameters to the 
model value given to DeclareAbstractParameter.  Overrides must not change 
the number of parameters. */)""")
      .def(
          "SetDefaultState",
          static_cast<void (LeafSystem<double>::*)(Context<double> const &,
                                                   State<double> *) const>(
              &LeafSystem<double>::SetDefaultState),
          py::arg("context"), py::arg("state"),
          R"""(/** Default implementation: sets all continuous state to the model vector 
given in DeclareContinuousState (or zero if no model vector was given) and 
discrete states to zero. Overrides must not change the number of state 
variables. */)""")

      ;

  using T_0 = ::drake::AutoDiffXd;

  py::class_<LeafSystem<T_0>, System<T_0>>
      PyLeafSystem_Eigen_AutoDiffScalar_Eigen_VectorXd(
          m, "LeafSystem_Eigen_AutoDiffScalar_Eigen_VectorXd");

  PyLeafSystem_Eigen_AutoDiffScalar_Eigen_VectorXd
      .def("AddTriggeredWitnessFunctionToCompositeEventCollection",
           [](LeafSystem<T_0> &self, Event<T_0> *event,
              CompositeEventCollection<T_0> *events) {
             return self.AddTriggeredWitnessFunctionToCompositeEventCollection(
                 event, events);
           })
      .def(
          "AllocateAbstractState",
          static_cast<::std::unique_ptr<
              drake::systems::AbstractValues,
              std::default_delete<drake::systems::AbstractValues>> (
              LeafSystem<T_0>::*)() const>(
              &LeafSystem_Eigen_AutoDiffScalar_Eigen_VectorXd_publicist::
                  AllocateAbstractState),
          R"""(/** Returns a copy of the states declared in DeclareAbstractState() calls. */)""")
      .def(
          "AllocateCompositeEventCollection",
          static_cast<::std::unique_ptr<
              drake::systems::CompositeEventCollection<::drake::AutoDiffXd>,
              std::default_delete<drake::systems::CompositeEventCollection<
                  ::drake::AutoDiffXd>>> (LeafSystem<T_0>::*)() const>(
              &LeafSystem<T_0>::AllocateCompositeEventCollection),
          R"""(/** Allocates a CompositeEventCollection object for this system. 
@sa System::AllocateCompositeEventCollection(). */)""")
      .def(
          "AllocateContext",
          static_cast<::std::unique_ptr<
              drake::systems::LeafContext<::drake::AutoDiffXd>,
              std::default_delete<drake::systems::LeafContext<
                  ::drake::AutoDiffXd>>> (LeafSystem<T_0>::*)() const>(
              &LeafSystem<T_0>::AllocateContext),
          R"""(/** Shadows System<T>::AllocateContext to provide a more concrete return 
type LeafContext<T>. */)""")
      .def("AllocateContinuousState",
           static_cast<::std::unique_ptr<
               drake::systems::ContinuousState<::drake::AutoDiffXd>,
               std::default_delete<drake::systems::ContinuousState<
                   ::drake::AutoDiffXd>>> (LeafSystem<T_0>::*)() const>(
               &LeafSystem_Eigen_AutoDiffScalar_Eigen_VectorXd_publicist::
                   AllocateContinuousState),
           R"""(/** Returns a copy of the state declared in the most recent 
DeclareContinuousState() call, or else a zero-sized state if that method 
has never been called. */)""")
      .def(
          "AllocateDiscreteState",
          static_cast<::std::unique_ptr<
              drake::systems::DiscreteValues<::drake::AutoDiffXd>,
              std::default_delete<drake::systems::DiscreteValues<
                  ::drake::AutoDiffXd>>> (LeafSystem<T_0>::*)() const>(
              &LeafSystem_Eigen_AutoDiffScalar_Eigen_VectorXd_publicist::
                  AllocateDiscreteState),
          R"""(/** Returns a copy of the states declared in DeclareDiscreteState() calls. */)""")
      .def("AllocateDiscreteVariables",
           static_cast<::std::unique_ptr<
               drake::systems::DiscreteValues<::drake::AutoDiffXd>,
               std::default_delete<drake::systems::DiscreteValues<
                   ::drake::AutoDiffXd>>> (LeafSystem<T_0>::*)() const>(
               &LeafSystem<T_0>::AllocateDiscreteVariables))
      .def("AllocateForcedDiscreteUpdateEventCollection",
           static_cast<::std::unique_ptr<
               drake::systems::EventCollection<
                   drake::systems::DiscreteUpdateEvent<::drake::AutoDiffXd>>,
               std::default_delete<drake::systems::EventCollection<
                   drake::systems::DiscreteUpdateEvent<::drake::AutoDiffXd>>>> (
               LeafSystem<T_0>::*)() const>(
               &LeafSystem<T_0>::AllocateForcedDiscreteUpdateEventCollection))
      .def("AllocateForcedPublishEventCollection",
           static_cast<::std::unique_ptr<
               drake::systems::EventCollection<
                   drake::systems::PublishEvent<::drake::AutoDiffXd>>,
               std::default_delete<drake::systems::EventCollection<
                   drake::systems::PublishEvent<::drake::AutoDiffXd>>>> (
               LeafSystem<T_0>::*)() const>(
               &LeafSystem<T_0>::AllocateForcedPublishEventCollection))
      .def(
          "AllocateForcedUnrestrictedUpdateEventCollection",
          static_cast<::std::unique_ptr<
              drake::systems::EventCollection<
                  drake::systems::UnrestrictedUpdateEvent<::drake::AutoDiffXd>>,
              std::default_delete<drake::systems::EventCollection<
                  drake::systems::UnrestrictedUpdateEvent<
                      ::drake::AutoDiffXd>>>> (LeafSystem<T_0>::*)() const>(
              &LeafSystem<
                  T_0>::AllocateForcedUnrestrictedUpdateEventCollection))
      .def(
          "AllocateParameters",
          static_cast<::std::unique_ptr<
              drake::systems::Parameters<::drake::AutoDiffXd>,
              std::default_delete<drake::systems::Parameters<
                  ::drake::AutoDiffXd>>> (LeafSystem<T_0>::*)() const>(
              &LeafSystem_Eigen_AutoDiffScalar_Eigen_VectorXd_publicist::
                  AllocateParameters),
          R"""(/** Returns a copy of the parameters declared in DeclareNumericParameter() 
and DeclareAbstractParameter() calls. */)""")
      .def("AllocateTimeDerivatives",
           static_cast<::std::unique_ptr<
               drake::systems::ContinuousState<::drake::AutoDiffXd>,
               std::default_delete<drake::systems::ContinuousState<
                   ::drake::AutoDiffXd>>> (LeafSystem<T_0>::*)() const>(
               &LeafSystem<T_0>::AllocateTimeDerivatives))
      .def(
          "DeclareAbstractInputPort",
          static_cast<InputPort<T_0> &(
              LeafSystem<T_0>::*)(::std::variant<
                                      std::basic_string<char,
                                                        std::char_traits<char>,
                                                        std::allocator<char>>,
                                      drake::systems::UseDefaultName>,
                                  ::drake::AbstractValue const &)>(
              &LeafSystem_Eigen_AutoDiffScalar_Eigen_VectorXd_publicist::
                  DeclareAbstractInputPort),
          py::arg("name"), py::arg("model_value"),
          R"""(/** Declares an abstract-valued input port using the given @p model_value. 
This is the best way to declare LeafSystem abstract input ports. 
 
Any port connected to this input, and any call to FixValue for this 
input, must provide for values whose type matches this @p model_value. 
 
@see System::DeclareInputPort() for more information. */)""",
          py::return_value_policy::reference_internal)
      .def(
          "DeclareAbstractInputPort",
          static_cast<InputPort<T_0> &(
              LeafSystem<T_0>::*)(::drake::AbstractValue const &)>(
              &LeafSystem_Eigen_AutoDiffScalar_Eigen_VectorXd_publicist::
                  DeclareAbstractInputPort),
          py::arg("model_value"),
          R"""(/** See the nearly identical signature with an additional (first) argument 
specifying the port name.  This version will be deprecated as discussed 
in #9447. */)""",
          py::return_value_policy::reference_internal)
      .def(
          "DeclareAbstractOutputPort",
          [](LeafSystem<T_0> &self,
             ::std::variant<std::basic_string<char, std::char_traits<char>,
                                              std::allocator<char>>,
                            drake::systems::UseDefaultName>
                 name,
             LeafOutputPort<T_0>::AllocCallback alloc_function,
             LeafOutputPort<T_0>::CalcCallback calc_function,
             ::std::set<
                 drake::TypeSafeIndex<drake::systems::DependencyTag>,
                 std::less<drake::TypeSafeIndex<drake::systems::DependencyTag>>,
                 std::allocator<
                     drake::TypeSafeIndex<drake::systems::DependencyTag>>>
                 prerequisites_of_calc) {
            return self.DeclareAbstractOutputPort(
                name, alloc_function, calc_function, prerequisites_of_calc);
          },
          R"""(/** (Advanced) Declares an abstract-valued output port using the given 
allocator and calculator functions provided in their most generic forms. 
If you have a member function available use one of the other signatures. 
@see LeafOutputPort::AllocCallback, LeafOutputPort::CalcCallback */)""")
      .def(
          "DeclareAbstractOutputPort",
          [](LeafSystem<T_0> &self,
             LeafOutputPort<T_0>::AllocCallback alloc_function,
             LeafOutputPort<T_0>::CalcCallback calc_function,
             ::std::set<
                 drake::TypeSafeIndex<drake::systems::DependencyTag>,
                 std::less<drake::TypeSafeIndex<drake::systems::DependencyTag>>,
                 std::allocator<
                     drake::TypeSafeIndex<drake::systems::DependencyTag>>>
                 prerequisites_of_calc) {
            return self.DeclareAbstractOutputPort(alloc_function, calc_function,
                                                  prerequisites_of_calc);
          },
          R"""(/** See the nearly identical signature with an additional (first) argument 
specifying the port name.  This version will be deprecated as discussed 
in #9447. */)""")
      .def(
          "DeclareAbstractParameter",
          static_cast<int (LeafSystem<T_0>::*)(::drake::AbstractValue const &)>(
              &LeafSystem_Eigen_AutoDiffScalar_Eigen_VectorXd_publicist::
                  DeclareAbstractParameter),
          py::arg("model_value"),
          R"""(/** Declares an abstract parameter using the given @p model_value. 
LeafSystem's default implementation of SetDefaultParameters() will reset 
parameters to their model values.  Returns the index of the new 
parameter. */)""")
      .def("DeclareAbstractState",
           static_cast<AbstractStateIndex (LeafSystem<T_0>::*)(
               ::drake::AbstractValue const &)>(
               &LeafSystem_Eigen_AutoDiffScalar_Eigen_VectorXd_publicist::
                   DeclareAbstractState),
           py::arg("abstract_state"),
           R"""(/** Declares an abstract state. 
@param abstract_state The abstract state model value. 
@return index of the declared abstract state. */)""")
      .def(
          "DeclareAbstractState",
          [](LeafSystem<T_0> &self, drake::AbstractValue abstract_state) {
            return self.DeclareAbstractState(
                std::make_unique<drake::AbstractValue>(abstract_state));
          },
          R"""(/** Declares an abstract state. 
@param abstract_state The abstract state model value.  The internal model 
value will contain a copy of `value` (not retain a pointer to `value`). 
@return index of the declared abstract state. */)""")
      .def(
          "DeclareContinuousState",
          static_cast<void (LeafSystem<T_0>::*)(int)>(
              &LeafSystem_Eigen_AutoDiffScalar_Eigen_VectorXd_publicist::
                  DeclareContinuousState),
          py::arg("num_state_variables"),
          R"""(/** Declares that this System should reserve continuous state with 
@p num_state_variables state variables, which have no second-order 
structure. */)""")
      .def(
          "DeclareContinuousState",
          static_cast<void (LeafSystem<T_0>::*)(int, int, int)>(
              &LeafSystem_Eigen_AutoDiffScalar_Eigen_VectorXd_publicist::
                  DeclareContinuousState),
          py::arg("num_q"), py::arg("num_v"), py::arg("num_z"),
          R"""(/** Declares that this System should reserve continuous state with @p num_q 
generalized positions, @p num_v generalized velocities, and @p num_z 
miscellaneous state variables. */)""")
      .def("DeclareContinuousState",
           [](LeafSystem<T_0> &self, BasicVector<T_0> const &model_vector) {
             return self.DeclareContinuousState(model_vector);
           })
      .def("DeclareContinuousState",
           [](LeafSystem<T_0> &self, BasicVector<T_0> const &model_vector,
              int num_q, int num_v, int num_z) {
             return self.DeclareContinuousState(model_vector, num_q, num_v,
                                                num_z);
           })
      .def("DeclareDiscreteState",
           [](LeafSystem<T_0> &self, BasicVector<T_0> const &model_vector) {
             return self.DeclareDiscreteState(model_vector);
           })
      .def("DeclareDiscreteState",
           [](LeafSystem<T_0> &self,
              ::Eigen::Ref<
                  const Eigen::Matrix<Eigen::AutoDiffScalar<Eigen::VectorXd>,
                                      -1, 1, 0, -1, 1>,
                  0, Eigen::InnerStride<1>> const &vector) {
             return self.DeclareDiscreteState(vector);
           })
      .def(
          "DeclareDiscreteState",
          static_cast<DiscreteStateIndex (LeafSystem<T_0>::*)(int)>(
              &LeafSystem_Eigen_AutoDiffScalar_Eigen_VectorXd_publicist::
                  DeclareDiscreteState),
          py::arg("num_state_variables"),
          R"""(/** Declares a discrete state group with @p num_state_variables state 
variables, stored in a BasicVector initialized to be all-zero. If you want 
non-zero initial values, use an alternate DeclareDiscreteState() signature 
that accepts a `model_vector` parameter. 
@pre `num_state_variables` must be non-negative. */)""")
      .def("DeclareEqualityConstraint",
           [](LeafSystem<T_0> &self,
              Eigen::Ref<
                  ::std::function<void(
                      const drake::systems::Context<::drake::AutoDiffXd> &,
                      Eigen::Matrix<Eigen::AutoDiffScalar<Eigen::VectorXd>, -1,
                                    1, 0, -1, 1> *)>,
                  0, Eigen::Stride<Eigen::Dynamic, Eigen::Dynamic>>
                  calc,
              int count, ::std::string description) {
             return self.DeclareEqualityConstraint(calc, count, description);
           })
      .def("DeclareImplicitTimeDerivativesResidualSize",
           static_cast<void (LeafSystem<T_0>::*)(int)>(
               &LeafSystem_Eigen_AutoDiffScalar_Eigen_VectorXd_publicist::
                   DeclareImplicitTimeDerivativesResidualSize),
           py::arg("n"),
           R"""(/** (Advanced) Overrides the default size for the implicit time 
derivatives residual. If no value is set, the default size is 
n=num_continuous_states(). 
 
@param[in] n The size of the residual vector output argument of 
             System::CalcImplicitTimeDerivativesResidual(). If n <= 0 
             restore to the default, num_continuous_states(). 
 
@see implicit_time_derivatives_residual_size() 
@see System::CalcImplicitTimeDerivativesResidual() */)""")
      .def("DeclareInequalityConstraint",
           [](LeafSystem<T_0> &self,
              Eigen::Ref<
                  ::std::function<void(
                      const drake::systems::Context<::drake::AutoDiffXd> &,
                      Eigen::Matrix<Eigen::AutoDiffScalar<Eigen::VectorXd>, -1,
                                    1, 0, -1, 1> *)>,
                  0, Eigen::Stride<Eigen::Dynamic, Eigen::Dynamic>>
                  calc,
              SystemConstraintBounds bounds, ::std::string description) {
             return self.DeclareInequalityConstraint(calc, bounds, description);
           })
      .def("DeclareNumericParameter",
           [](LeafSystem<T_0> &self, BasicVector<T_0> const &model_vector) {
             return self.DeclareNumericParameter(model_vector);
           })
      .def(
          "DeclarePeriodicDiscreteUpdate",
          static_cast<void (LeafSystem<T_0>::*)(double, double)>(
              &LeafSystem_Eigen_AutoDiffScalar_Eigen_VectorXd_publicist::
                  DeclarePeriodicDiscreteUpdate),
          py::arg("period_sec"), py::arg("offset_sec") = double(0),
          R"""(/** (To be deprecated) Declares a periodic discrete update event that invokes 
the DiscreteUpdate() dispatcher but does not provide a handler 
function. This does guarantee that a Simulator step will end exactly at 
the update time, but otherwise has no effect unless the 
DoDiscreteUpdate() dispatcher has been overloaded (not recommended). */)""")
      .def(
          "DeclarePeriodicPublish",
          static_cast<void (LeafSystem<T_0>::*)(double, double)>(
              &LeafSystem_Eigen_AutoDiffScalar_Eigen_VectorXd_publicist::
                  DeclarePeriodicPublish),
          py::arg("period_sec"), py::arg("offset_sec") = double(0),
          R"""(/** (To be deprecated) Declares a periodic publish event that invokes the 
Publish() dispatcher but does not provide a handler function. This does 
guarantee that a Simulator step will end exactly at the publish time, 
but otherwise has no effect unless the DoPublish() dispatcher has been 
overloaded (not recommended). */)""")
      .def(
          "DeclarePeriodicUnrestrictedUpdate",
          static_cast<void (LeafSystem<T_0>::*)(double, double)>(
              &LeafSystem_Eigen_AutoDiffScalar_Eigen_VectorXd_publicist::
                  DeclarePeriodicUnrestrictedUpdate),
          py::arg("period_sec"), py::arg("offset_sec") = double(0),
          R"""(/** (To be deprecated) Declares a periodic unrestricted update event that 
invokes the UnrestrictedUpdate() dispatcher but does not provide a handler 
function. This does guarantee that a Simulator step will end exactly at 
the update time, but otherwise has no effect unless the 
DoUnrestrictedUpdate() dispatcher has been overloaded (not recommended). */)""")
      .def(
          "DeclareVectorInputPort",
          [](LeafSystem<T_0> &self,
             ::std::variant<std::basic_string<char, std::char_traits<char>,
                                              std::allocator<char>>,
                            drake::systems::UseDefaultName>
                 name,
             BasicVector<T_0> const &model_vector,
             ::std::optional<drake::RandomDistribution> random_type) {
            return self.DeclareVectorInputPort(name, model_vector, random_type);
          },
          py::return_value_policy::reference_internal)
      .def(
          "DeclareVectorInputPort",
          [](LeafSystem<T_0> &self, BasicVector<T_0> const &model_vector,
             ::std::optional<drake::RandomDistribution> random_type) {
            return self.DeclareVectorInputPort(model_vector, random_type);
          },
          py::return_value_policy::reference_internal)
      .def(
          "DeclareVectorOutputPort",
          [](LeafSystem<T_0> &self,
             ::std::variant<std::basic_string<char, std::char_traits<char>,
                                              std::allocator<char>>,
                            drake::systems::UseDefaultName>
                 name,
             BasicVector<T_0> const &model_vector,
             LeafOutputPort<T_0>::CalcVectorCallback vector_calc_function,
             ::std::set<
                 drake::TypeSafeIndex<drake::systems::DependencyTag>,
                 std::less<drake::TypeSafeIndex<drake::systems::DependencyTag>>,
                 std::allocator<
                     drake::TypeSafeIndex<drake::systems::DependencyTag>>>
                 prerequisites_of_calc) {
            return self.DeclareVectorOutputPort(name, model_vector,
                                                vector_calc_function,
                                                prerequisites_of_calc);
          },
          py::return_value_policy::reference_internal)
      .def(
          "DeclareVectorOutputPort",
          [](LeafSystem<T_0> &self, BasicVector<T_0> const &model_vector,
             LeafOutputPort<T_0>::CalcVectorCallback vector_calc_function,
             ::std::set<
                 drake::TypeSafeIndex<drake::systems::DependencyTag>,
                 std::less<drake::TypeSafeIndex<drake::systems::DependencyTag>>,
                 std::allocator<
                     drake::TypeSafeIndex<drake::systems::DependencyTag>>>
                 prerequisites_of_calc) {
            return self.DeclareVectorOutputPort(
                model_vector, vector_calc_function, prerequisites_of_calc);
          },
          py::return_value_policy::reference_internal)
      .def(
          "DoAllocateContext",
          static_cast<::std::unique_ptr<
              drake::systems::ContextBase,
              std::default_delete<drake::systems::ContextBase>> (
              LeafSystem<T_0>::*)() const>(&LeafSystem<T_0>::DoAllocateContext))
      .def("DoCalcDiscreteVariableUpdates",
           [](LeafSystem<T_0> &self, Context<T_0> const &context,
              ::std::vector<
                  const drake::systems::DiscreteUpdateEvent<::drake::AutoDiffXd>
                      *,
                  std::allocator<const drake::systems::DiscreteUpdateEvent<
                      ::drake::AutoDiffXd> *>> const &events,
              DiscreteValues<T_0> *discrete_state) {
             return self.DoCalcDiscreteVariableUpdates(context, events,
                                                       discrete_state);
           })
      .def("DoCalcNextUpdateTime",
           [](LeafSystem<T_0> &self, Context<T_0> const &context,
              CompositeEventCollection<T_0> *events,
              Eigen::Ref<::Eigen::AutoDiffScalar<
                             Eigen::Matrix<double, -1, 1, 0, -1, 1>> *,
                         0, Eigen::Stride<Eigen::Dynamic, Eigen::Dynamic>>
                  time) {
             return self.DoCalcNextUpdateTime(context, events, time);
           })
      .def("DoCalcUnrestrictedUpdate",
           [](LeafSystem<T_0> &self, Context<T_0> const &context,
              ::std::vector<
                  const drake::systems::UnrestrictedUpdateEvent<
                      ::drake::AutoDiffXd> *,
                  std::allocator<const drake::systems::UnrestrictedUpdateEvent<
                      ::drake::AutoDiffXd> *>> const &events,
              State<T_0> *state) {
             return self.DoCalcUnrestrictedUpdate(context, events, state);
           })
      .def("DoCalcWitnessValue",
           [](LeafSystem<T_0> &self, Context<T_0> const &context,
              WitnessFunction<T_0> const &witness_func) {
             return self.DoCalcWitnessValue(context, witness_func);
           })
      .def(
          "DoMakeLeafContext",
          static_cast<::std::unique_ptr<
              drake::systems::LeafContext<::drake::AutoDiffXd>,
              std::default_delete<drake::systems::LeafContext<
                  ::drake::AutoDiffXd>>> (LeafSystem<T_0>::*)() const>(
              &LeafSystem_Eigen_AutoDiffScalar_Eigen_VectorXd_publicist::
                  DoMakeLeafContext),
          R"""(/** Provides a new instance of the leaf context for this system. Derived 
leaf systems with custom derived leaf system contexts should override this 
to provide a context of the appropriate type. The returned context should 
be "empty"; invoked by AllocateContext(), the caller will take the 
responsibility to initialize the core LeafContext data. The default 
implementation provides a default-constructed `LeafContext<T>`. */)""")
      .def("DoPublish",
           [](LeafSystem<T_0> &self, Context<T_0> const &context,
              ::std::vector<
                  const drake::systems::PublishEvent<::drake::AutoDiffXd> *,
                  std::allocator<const drake::systems::PublishEvent<
                      ::drake::AutoDiffXd> *>> const &events) {
             return self.DoPublish(context, events);
           })
      .def("DoValidateAllocatedLeafContext",
           [](LeafSystem<T_0> &self, LeafContext<T_0> const &context) {
             return self.DoValidateAllocatedLeafContext(context);
           })
      .def("GetDirectFeedthroughs",
           static_cast<
               ::std::multimap<int, int, std::less<int>,
                               std::allocator<std::pair<const int, int>>> (
                   LeafSystem<T_0>::*)() const>(
               &LeafSystem<T_0>::GetDirectFeedthroughs))
      .def(
          "GetGraphvizFragment",
          static_cast<void (LeafSystem<T_0>::*)(int, ::std::stringstream *)
                          const>(
              &LeafSystem_Eigen_AutoDiffScalar_Eigen_VectorXd_publicist::
                  GetGraphvizFragment),
          py::arg("max_depth"), py::arg("dot"),
          R"""(/** Emits a graphviz fragment for this System. Leaf systems are visualized as 
records. For instance, a leaf system with 2 inputs and 1 output is: 
 
@verbatim 
123456 [shape= record, label="name | {<u0> 0 |<y0> 0} | {<u1> 1 | }"]; 
@endverbatim 
 
which looks like: 
 
@verbatim 
+------------+----+ 
| name  | u0 | u1 | 
|       | y0 |    | 
+-------+----+----+ 
@endverbatim */)""")
      .def("GetGraphvizInputPortToken",
           [](LeafSystem<T_0> &self, InputPort<T_0> const &port, int max_depth,
              ::std::stringstream *dot) {
             return self.GetGraphvizInputPortToken(port, max_depth, dot);
           })
      .def("GetGraphvizOutputPortToken",
           [](LeafSystem<T_0> &self, OutputPort<T_0> const &port, int max_depth,
              ::std::stringstream *dot) {
             return self.GetGraphvizOutputPortToken(port, max_depth, dot);
           })
      .def("MakeWitnessFunction",
           [](LeafSystem<T_0> &self, ::std::string const &description,
              WitnessFunctionDirection const &direction_type,
              ::std::function<Eigen::AutoDiffScalar<Eigen::VectorXd>(
                  const drake::systems::Context<::drake::AutoDiffXd> &)>
                  calc) {
             return self.MakeWitnessFunction(description, direction_type, calc);
           })
      .def("MakeWitnessFunction",
           [](LeafSystem<T_0> &self, ::std::string const &description,
              WitnessFunctionDirection const &direction_type,
              ::std::function<Eigen::AutoDiffScalar<Eigen::VectorXd>(
                  const drake::systems::Context<::drake::AutoDiffXd> &)>
                  calc,
              Event<T_0> const &e) {
             return self.MakeWitnessFunction(description, direction_type, calc,
                                             e);
           })
      .def("SetDefaultParameters",
           [](LeafSystem<T_0> &self, Context<T_0> const &context,
              Parameters<T_0> *parameters) {
             return self.SetDefaultParameters(context, parameters);
           })
      .def("SetDefaultState",
           [](LeafSystem<T_0> &self, Context<T_0> const &context,
              State<T_0> *state) {
             return self.SetDefaultState(context, state);
           })

      ;

  py::class_<LeafSystem<float>> PyLeafSystem_float(m, "LeafSystem_float");

  PyLeafSystem_float
      .def("AddTriggeredWitnessFunctionToCompositeEventCollection",
           static_cast<void (LeafSystem<float>::*)(
               Event<float> *, CompositeEventCollection<float> *) const>(
               &LeafSystem_float_publicist::
                   AddTriggeredWitnessFunctionToCompositeEventCollection),
           py::arg("event"), py::arg("events"))
      .def("AllocateAbstractState",
           static_cast<::std::unique_ptr<
               drake::systems::AbstractValues,
               std::default_delete<drake::systems::AbstractValues>> (
               LeafSystem<float>::*)() const>(
               &LeafSystem_float_publicist::AllocateAbstractState))
      .def("AllocateCompositeEventCollection",
           static_cast<::std::unique_ptr<
               drake::systems::CompositeEventCollection<float>,
               std::default_delete<drake::systems::CompositeEventCollection<
                   float>>> (LeafSystem<float>::*)() const>(
               &LeafSystem<float>::AllocateCompositeEventCollection))
      .def("AllocateContext",
           static_cast<::std::unique_ptr<
               drake::systems::LeafContext<float>,
               std::default_delete<drake::systems::LeafContext<float>>> (
               LeafSystem<float>::*)() const>(
               &LeafSystem<float>::AllocateContext))
      .def("AllocateContinuousState",
           static_cast<::std::unique_ptr<
               drake::systems::ContinuousState<float>,
               std::default_delete<drake::systems::ContinuousState<float>>> (
               LeafSystem<float>::*)() const>(
               &LeafSystem_float_publicist::AllocateContinuousState))
      .def("AllocateDiscreteState",
           static_cast<::std::unique_ptr<
               drake::systems::DiscreteValues<float>,
               std::default_delete<drake::systems::DiscreteValues<float>>> (
               LeafSystem<float>::*)() const>(
               &LeafSystem_float_publicist::AllocateDiscreteState))
      .def("AllocateDiscreteVariables",
           static_cast<::std::unique_ptr<
               drake::systems::DiscreteValues<float>,
               std::default_delete<drake::systems::DiscreteValues<float>>> (
               LeafSystem<float>::*)() const>(
               &LeafSystem<float>::AllocateDiscreteVariables))
      .def("AllocateForcedDiscreteUpdateEventCollection",
           static_cast<::std::unique_ptr<
               drake::systems::EventCollection<
                   drake::systems::DiscreteUpdateEvent<float>>,
               std::default_delete<drake::systems::EventCollection<
                   drake::systems::DiscreteUpdateEvent<float>>>> (
               LeafSystem<float>::*)() const>(
               &LeafSystem<float>::AllocateForcedDiscreteUpdateEventCollection))
      .def("AllocateForcedPublishEventCollection",
           static_cast<::std::unique_ptr<
               drake::systems::EventCollection<
                   drake::systems::PublishEvent<float>>,
               std::default_delete<drake::systems::EventCollection<
                   drake::systems::PublishEvent<float>>>> (
               LeafSystem<float>::*)() const>(
               &LeafSystem<float>::AllocateForcedPublishEventCollection))
      .def("AllocateForcedUnrestrictedUpdateEventCollection",
           static_cast<::std::unique_ptr<
               drake::systems::EventCollection<
                   drake::systems::UnrestrictedUpdateEvent<float>>,
               std::default_delete<drake::systems::EventCollection<
                   drake::systems::UnrestrictedUpdateEvent<float>>>> (
               LeafSystem<float>::*)() const>(
               &LeafSystem<
                   float>::AllocateForcedUnrestrictedUpdateEventCollection))
      .def("AllocateParameters",
           static_cast<::std::unique_ptr<
               drake::systems::Parameters<float>,
               std::default_delete<drake::systems::Parameters<float>>> (
               LeafSystem<float>::*)() const>(
               &LeafSystem_float_publicist::AllocateParameters))
      .def("AllocateTimeDerivatives",
           static_cast<::std::unique_ptr<
               drake::systems::ContinuousState<float>,
               std::default_delete<drake::systems::ContinuousState<float>>> (
               LeafSystem<float>::*)() const>(
               &LeafSystem<float>::AllocateTimeDerivatives))
      .def(
          "DeclareAbstractOutputPort",
          [](LeafSystem<float> &self,
             ::std::variant<std::basic_string<char, std::char_traits<char>,
                                              std::allocator<char>>,
                            drake::systems::UseDefaultName>
                 name,
             LeafOutputPort<float>::AllocCallback alloc_function,
             LeafOutputPort<float>::CalcCallback calc_function,
             ::std::set<
                 drake::TypeSafeIndex<drake::systems::DependencyTag>,
                 std::less<drake::TypeSafeIndex<drake::systems::DependencyTag>>,
                 std::allocator<
                     drake::TypeSafeIndex<drake::systems::DependencyTag>>>
                 prerequisites_of_calc) {
            return self.DeclareAbstractOutputPort(
                name, alloc_function, calc_function, prerequisites_of_calc);
          })
      .def(
          "DeclareAbstractOutputPort",
          [](LeafSystem<float> &self,
             LeafOutputPort<float>::AllocCallback alloc_function,
             LeafOutputPort<float>::CalcCallback calc_function,
             ::std::set<
                 drake::TypeSafeIndex<drake::systems::DependencyTag>,
                 std::less<drake::TypeSafeIndex<drake::systems::DependencyTag>>,
                 std::allocator<
                     drake::TypeSafeIndex<drake::systems::DependencyTag>>>
                 prerequisites_of_calc) {
            return self.DeclareAbstractOutputPort(alloc_function, calc_function,
                                                  prerequisites_of_calc);
          })
      .def("DeclareAbstractParameter",
           static_cast<int (LeafSystem<float>::*)(
               ::drake::AbstractValue const &)>(
               &LeafSystem_float_publicist::DeclareAbstractParameter),
           py::arg("model_value"))
      .def("DeclareAbstractState",
           static_cast<AbstractStateIndex (LeafSystem<float>::*)(
               ::drake::AbstractValue const &)>(
               &LeafSystem_float_publicist::DeclareAbstractState),
           py::arg("abstract_state"))
      .def("DeclareAbstractState",
           [](LeafSystem<float> &self, drake::AbstractValue abstract_state) {
             return self.DeclareAbstractState(
                 std::make_unique<drake::AbstractValue>(abstract_state));
           })
      .def("DeclareContinuousState",
           static_cast<void (LeafSystem<float>::*)(int)>(
               &LeafSystem_float_publicist::DeclareContinuousState),
           py::arg("num_state_variables"))
      .def("DeclareContinuousState",
           static_cast<void (LeafSystem<float>::*)(int, int, int)>(
               &LeafSystem_float_publicist::DeclareContinuousState),
           py::arg("num_q"), py::arg("num_v"), py::arg("num_z"))
      .def("DeclareContinuousState",
           static_cast<void (LeafSystem<float>::*)(BasicVector<float> const &)>(
               &LeafSystem_float_publicist::DeclareContinuousState),
           py::arg("model_vector"))
      .def("DeclareContinuousState",
           static_cast<void (LeafSystem<float>::*)(BasicVector<float> const &,
                                                   int, int, int)>(
               &LeafSystem_float_publicist::DeclareContinuousState),
           py::arg("model_vector"), py::arg("num_q"), py::arg("num_v"),
           py::arg("num_z"))
      .def("DeclareDiscreteState",
           static_cast<DiscreteStateIndex (LeafSystem<float>::*)(
               BasicVector<float> const &)>(
               &LeafSystem_float_publicist::DeclareDiscreteState),
           py::arg("model_vector"))
      .def("DeclareDiscreteState",
           [](LeafSystem<float> &self,
              ::Eigen::Ref<const Eigen::Matrix<float, -1, 1, 0, -1, 1>, 0,
                           Eigen::InnerStride<1>> const &vector) {
             return self.DeclareDiscreteState(vector);
           })
      .def("DeclareDiscreteState",
           static_cast<DiscreteStateIndex (LeafSystem<float>::*)(int)>(
               &LeafSystem_float_publicist::DeclareDiscreteState),
           py::arg("num_state_variables"))
      .def(
          "DeclareEqualityConstraint",
          [](LeafSystem<float> &self,
             Eigen::Ref<
                 ::std::function<void(const drake::systems::Context<float> &,
                                      Eigen::Matrix<float, -1, 1, 0, -1, 1> *)>,
                 0, Eigen::Stride<Eigen::Dynamic, Eigen::Dynamic>>
                 calc,
             int count, ::std::string description) {
            return self.DeclareEqualityConstraint(calc, count, description);
          })
      .def("DeclareImplicitTimeDerivativesResidualSize",
           static_cast<void (LeafSystem<float>::*)(int)>(
               &LeafSystem_float_publicist::
                   DeclareImplicitTimeDerivativesResidualSize),
           py::arg("n"),
           R"""(/** (Advanced) Overrides the default size for the implicit time 
derivatives residual. If no value is set, the default size is 
n=num_continuous_states(). 
 
@param[in] n The size of the residual vector output argument of 
             System::CalcImplicitTimeDerivativesResidual(). If n <= 0 
             restore to the default, num_continuous_states(). 
 
@see implicit_time_derivatives_residual_size() 
@see System::CalcImplicitTimeDerivativesResidual() */)""")
      .def(
          "DeclareInequalityConstraint",
          [](LeafSystem<float> &self,
             Eigen::Ref<
                 ::std::function<void(const drake::systems::Context<float> &,
                                      Eigen::Matrix<float, -1, 1, 0, -1, 1> *)>,
                 0, Eigen::Stride<Eigen::Dynamic, Eigen::Dynamic>>
                 calc,
             SystemConstraintBounds bounds, ::std::string description) {
            return self.DeclareInequalityConstraint(calc, bounds, description);
          })
      .def("DeclareNumericParameter",
           static_cast<int (LeafSystem<float>::*)(BasicVector<float> const &)>(
               &LeafSystem_float_publicist::DeclareNumericParameter),
           py::arg("model_vector"))
      .def("DeclarePeriodicDiscreteUpdate",
           static_cast<void (LeafSystem<float>::*)(double, double)>(
               &LeafSystem_float_publicist::DeclarePeriodicDiscreteUpdate),
           py::arg("period_sec"), py::arg("offset_sec") = double(0))
      .def("DeclarePeriodicPublish",
           static_cast<void (LeafSystem<float>::*)(double, double)>(
               &LeafSystem_float_publicist::DeclarePeriodicPublish),
           py::arg("period_sec"), py::arg("offset_sec") = double(0))
      .def("DeclarePeriodicUnrestrictedUpdate",
           static_cast<void (LeafSystem<float>::*)(double, double)>(
               &LeafSystem_float_publicist::DeclarePeriodicUnrestrictedUpdate),
           py::arg("period_sec"), py::arg("offset_sec") = double(0))
      .def("DeclareVectorOutputPort",
           static_cast<LeafOutputPort<float> &(
               LeafSystem<float>::
                   *)(::std::variant<
                          std::basic_string<char, std::char_traits<char>,
                                            std::allocator<char>>,
                          drake::systems::UseDefaultName>,
                      BasicVector<float> const &,
                      LeafOutputPort<float>::CalcVectorCallback,
                      ::std::set<
                          drake::TypeSafeIndex<drake::systems::DependencyTag>,
                          std::less<drake::TypeSafeIndex<
                              drake::systems::DependencyTag>>,
                          std::allocator<drake::TypeSafeIndex<
                              drake::systems::DependencyTag>>>)>(
               &LeafSystem_float_publicist::DeclareVectorOutputPort),
           py::arg("name"), py::arg("model_vector"),
           py::arg("vector_calc_function"),
           py::arg("prerequisites_of_calc") = ::std::set<
               drake::TypeSafeIndex<drake::systems::DependencyTag>,
               std::less<drake::TypeSafeIndex<drake::systems::DependencyTag>>,
               std::allocator<
                   drake::TypeSafeIndex<drake::systems::DependencyTag>>>(
               {drake::systems::SystemBase::all_sources_ticket()}))
      .def("DeclareVectorOutputPort",
           static_cast<LeafOutputPort<float> &(
               LeafSystem<float>::
                   *)(BasicVector<float> const &,
                      LeafOutputPort<float>::CalcVectorCallback,
                      ::std::set<
                          drake::TypeSafeIndex<drake::systems::DependencyTag>,
                          std::less<drake::TypeSafeIndex<
                              drake::systems::DependencyTag>>,
                          std::allocator<drake::TypeSafeIndex<
                              drake::systems::DependencyTag>>>)>(
               &LeafSystem_float_publicist::DeclareVectorOutputPort),
           py::arg("model_vector"), py::arg("vector_calc_function"),
           py::arg("prerequisites_of_calc") = ::std::set<
               drake::TypeSafeIndex<drake::systems::DependencyTag>,
               std::less<drake::TypeSafeIndex<drake::systems::DependencyTag>>,
               std::allocator<
                   drake::TypeSafeIndex<drake::systems::DependencyTag>>>(
               {drake::systems::SystemBase::all_sources_ticket()}))
      .def("DoAllocateContext",
           static_cast<::std::unique_ptr<
               drake::systems::ContextBase,
               std::default_delete<drake::systems::ContextBase>> (
               LeafSystem<float>::*)() const>(
               &LeafSystem<float>::DoAllocateContext))
      .def("DoCalcDiscreteVariableUpdates",
           static_cast<void (LeafSystem<float>::*)(
               Context<float> const &,
               ::std::vector<
                   const drake::systems::DiscreteUpdateEvent<float> *,
                   std::allocator<const drake::systems::DiscreteUpdateEvent<
                       float> *>> const &,
               DiscreteValues<float> *) const>(
               &LeafSystem_float_publicist::DoCalcDiscreteVariableUpdates),
           py::arg("context"), py::arg("events"), py::arg("discrete_state"))
      .def("DoCalcNextUpdateTime",
           static_cast<void (LeafSystem<float>::*)(
               Context<float> const &, CompositeEventCollection<float> *,
               float *) const>(
               &LeafSystem_float_publicist::DoCalcNextUpdateTime),
           py::arg("context"), py::arg("events"), py::arg("time"))
      .def("DoCalcUnrestrictedUpdate",
           static_cast<void (LeafSystem<float>::*)(
               Context<float> const &,
               ::std::vector<
                   const drake::systems::UnrestrictedUpdateEvent<float> *,
                   std::allocator<const drake::systems::UnrestrictedUpdateEvent<
                       float> *>> const &,
               State<float> *) const>(
               &LeafSystem_float_publicist::DoCalcUnrestrictedUpdate),
           py::arg("context"), py::arg("events"), py::arg("state"))
      .def("DoMakeLeafContext",
           static_cast<::std::unique_ptr<
               drake::systems::LeafContext<float>,
               std::default_delete<drake::systems::LeafContext<float>>> (
               LeafSystem<float>::*)() const>(
               &LeafSystem_float_publicist::DoMakeLeafContext))
      .def("DoPublish",
           static_cast<void (LeafSystem<float>::*)(
               Context<float> const &,
               ::std::vector<const drake::systems::PublishEvent<float> *,
                             std::allocator<const drake::systems::PublishEvent<
                                 float> *>> const &) const>(
               &LeafSystem_float_publicist::DoPublish),
           py::arg("context"), py::arg("events"))
      .def(
          "DoValidateAllocatedLeafContext",
          static_cast<void (LeafSystem<float>::*)(LeafContext<float> const &)
                          const>(
              &LeafSystem_float_publicist::DoValidateAllocatedLeafContext),
          py::arg("context"),
          R"""(/** Derived classes that impose restrictions on what resources are permitted 
should check those restrictions by implementing this. For example, a 
derived class might require a single input and single output. Note that 
the supplied Context will be complete except that input and output 
dependencies on peer and parent subcontexts will not yet have been set up, 
so you may not consider them for validation. 
The default implementation does nothing. */)""")
      .def("GetDirectFeedthroughs",
           static_cast<
               ::std::multimap<int, int, std::less<int>,
                               std::allocator<std::pair<const int, int>>> (
                   LeafSystem<float>::*)() const>(
               &LeafSystem<float>::GetDirectFeedthroughs))
      .def("GetGraphvizFragment",
           static_cast<void (LeafSystem<float>::*)(int, ::std::stringstream *)
                           const>(
               &LeafSystem_float_publicist::GetGraphvizFragment),
           py::arg("max_depth"), py::arg("dot"))
      .def("GetGraphvizOutputPortToken",
           static_cast<void (LeafSystem<float>::*)(
               OutputPort<float> const &, int, ::std::stringstream *) const>(
               &LeafSystem_float_publicist::GetGraphvizOutputPortToken),
           py::arg("port"), py::arg("max_depth"), py::arg("dot"))
      .def("MakeWitnessFunction",
           static_cast<::std::unique_ptr<
               drake::systems::WitnessFunction<float>,
               std::default_delete<drake::systems::WitnessFunction<float>>> (
               LeafSystem<float>::*)(
               ::std::string const &, WitnessFunctionDirection const &,
               ::std::function<float(const drake::systems::Context<float> &)>)
                           const>(
               &LeafSystem_float_publicist::MakeWitnessFunction),
           py::arg("description"), py::arg("direction_type"), py::arg("calc"))
      .def("MakeWitnessFunction",
           static_cast<::std::unique_ptr<
               drake::systems::WitnessFunction<float>,
               std::default_delete<drake::systems::WitnessFunction<float>>> (
               LeafSystem<float>::*)(
               ::std::string const &, WitnessFunctionDirection const &,
               ::std::function<float(const drake::systems::Context<float> &)>,
               Event<float> const &) const>(
               &LeafSystem_float_publicist::MakeWitnessFunction),
           py::arg("description"), py::arg("direction_type"), py::arg("calc"),
           py::arg("e"))
      .def("SetDefaultParameters",
           static_cast<void (LeafSystem<float>::*)(Context<float> const &,
                                                   Parameters<float> *) const>(
               &LeafSystem<float>::SetDefaultParameters),
           py::arg("context"), py::arg("parameters"))
      .def("SetDefaultState",
           static_cast<void (LeafSystem<float>::*)(Context<float> const &,
                                                   State<float> *) const>(
               &LeafSystem<float>::SetDefaultState),
           py::arg("context"), py::arg("state"))

      ;
}
