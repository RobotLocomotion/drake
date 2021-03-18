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

  using PyLeafSystem_double_0 = double;

  py::class_<LeafSystem<PyLeafSystem_double_0>, System<PyLeafSystem_double_0>>
      PyLeafSystem_double(m, "LeafSystem_double");

  PyLeafSystem_double
      .def("AddTriggeredWitnessFunctionToCompositeEventCollection",
           static_cast<void (LeafSystem<PyLeafSystem_double_0>::*)(
               Event<PyLeafSystem_double_0> *,
               CompositeEventCollection<PyLeafSystem_double_0> *) const>(
               &LeafSystem_double_publicist::
                   AddTriggeredWitnessFunctionToCompositeEventCollection),
           py::arg("event"), py::arg("events"))
      .def("AllocateAbstractState",
           static_cast<::std::unique_ptr<AbstractValues,
                                         std::default_delete<AbstractValues>> (
               LeafSystem<PyLeafSystem_double_0>::*)() const>(
               &LeafSystem_double_publicist::AllocateAbstractState))
      .def("AllocateCompositeEventCollection",
           static_cast<::std::unique_ptr<
               CompositeEventCollection<PyLeafSystem_double_0>,
               std::default_delete<
                   CompositeEventCollection<PyLeafSystem_double_0>>> (
               LeafSystem<PyLeafSystem_double_0>::*)() const>(
               &LeafSystem<
                   PyLeafSystem_double_0>::AllocateCompositeEventCollection))
      .def("AllocateContext",
           static_cast<::std::unique_ptr<
               LeafContext<PyLeafSystem_double_0>,
               std::default_delete<LeafContext<PyLeafSystem_double_0>>> (
               LeafSystem<PyLeafSystem_double_0>::*)() const>(
               &LeafSystem<PyLeafSystem_double_0>::AllocateContext))
      .def("AllocateContinuousState",
           static_cast<::std::unique_ptr<
               ContinuousState<PyLeafSystem_double_0>,
               std::default_delete<ContinuousState<PyLeafSystem_double_0>>> (
               LeafSystem<PyLeafSystem_double_0>::*)() const>(
               &LeafSystem_double_publicist::AllocateContinuousState))
      .def("AllocateDiscreteState",
           static_cast<::std::unique_ptr<
               DiscreteValues<PyLeafSystem_double_0>,
               std::default_delete<DiscreteValues<PyLeafSystem_double_0>>> (
               LeafSystem<PyLeafSystem_double_0>::*)() const>(
               &LeafSystem_double_publicist::AllocateDiscreteState))
      .def("AllocateDiscreteVariables",
           static_cast<::std::unique_ptr<
               DiscreteValues<PyLeafSystem_double_0>,
               std::default_delete<DiscreteValues<PyLeafSystem_double_0>>> (
               LeafSystem<PyLeafSystem_double_0>::*)() const>(
               &LeafSystem<PyLeafSystem_double_0>::AllocateDiscreteVariables))
      .def("AllocateForcedDiscreteUpdateEventCollection",
           static_cast<::std::unique_ptr<
               EventCollection<DiscreteUpdateEvent<PyLeafSystem_double_0>>,
               std::default_delete<EventCollection<
                   DiscreteUpdateEvent<PyLeafSystem_double_0>>>> (
               LeafSystem<PyLeafSystem_double_0>::*)() const>(
               &LeafSystem<PyLeafSystem_double_0>::
                   AllocateForcedDiscreteUpdateEventCollection))
      .def(
          "AllocateForcedPublishEventCollection",
          static_cast<::std::unique_ptr<
              EventCollection<PublishEvent<PyLeafSystem_double_0>>,
              std::default_delete<
                  EventCollection<PublishEvent<PyLeafSystem_double_0>>>> (
              LeafSystem<PyLeafSystem_double_0>::*)() const>(
              &LeafSystem<
                  PyLeafSystem_double_0>::AllocateForcedPublishEventCollection))
      .def("AllocateForcedUnrestrictedUpdateEventCollection",
           static_cast<::std::unique_ptr<
               EventCollection<UnrestrictedUpdateEvent<PyLeafSystem_double_0>>,
               std::default_delete<EventCollection<
                   UnrestrictedUpdateEvent<PyLeafSystem_double_0>>>> (
               LeafSystem<PyLeafSystem_double_0>::*)() const>(
               &LeafSystem<PyLeafSystem_double_0>::
                   AllocateForcedUnrestrictedUpdateEventCollection))
      .def("AllocateParameters",
           static_cast<::std::unique_ptr<
               Parameters<PyLeafSystem_double_0>,
               std::default_delete<Parameters<PyLeafSystem_double_0>>> (
               LeafSystem<PyLeafSystem_double_0>::*)() const>(
               &LeafSystem_double_publicist::AllocateParameters))
      .def("AllocateTimeDerivatives",
           static_cast<::std::unique_ptr<
               ContinuousState<PyLeafSystem_double_0>,
               std::default_delete<ContinuousState<PyLeafSystem_double_0>>> (
               LeafSystem<PyLeafSystem_double_0>::*)() const>(
               &LeafSystem<PyLeafSystem_double_0>::AllocateTimeDerivatives))
      .def("DeclareAbstractInputPort",
           static_cast<InputPort<PyLeafSystem_double_0> &(
               LeafSystem<PyLeafSystem_double_0>::
                   *)(::std::variant<
                          std::basic_string<char, std::char_traits<char>,
                                            std::allocator<char>>,
                          UseDefaultName>,
                      ::drake::AbstractValue const &)>(
               &LeafSystem_double_publicist::DeclareAbstractInputPort),
           py::arg("name"), py::arg("model_value"))
      .def(
          "DeclareAbstractInputPort",
          static_cast<InputPort<PyLeafSystem_double_0> &(
              LeafSystem<PyLeafSystem_double_0>::*)(::drake::AbstractValue const
                                                        &)>(
              &LeafSystem_double_publicist::DeclareAbstractInputPort),
          py::arg("model_value"))
      .def("DeclareAbstractOutputPort",
           [](LeafSystem<PyLeafSystem_double_0> &self,
              ::std::variant<std::basic_string<char, std::char_traits<char>,
                                               std::allocator<char>>,
                             UseDefaultName>
                  name,
              LeafOutputPort<PyLeafSystem_double_0>::AllocCallback
                  alloc_function,
              LeafOutputPort<PyLeafSystem_double_0>::CalcCallback calc_function,
              ::std::set<drake::TypeSafeIndex<DependencyTag>,
                         std::less<drake::TypeSafeIndex<DependencyTag>>,
                         std::allocator<drake::TypeSafeIndex<DependencyTag>>>
                  prerequisites_of_calc) {
             return self.DeclareAbstractOutputPort(
                 name, alloc_function, calc_function, prerequisites_of_calc);
           })
      .def("DeclareAbstractOutputPort",
           [](LeafSystem<PyLeafSystem_double_0> &self,
              LeafOutputPort<PyLeafSystem_double_0>::AllocCallback
                  alloc_function,
              LeafOutputPort<PyLeafSystem_double_0>::CalcCallback calc_function,
              ::std::set<drake::TypeSafeIndex<DependencyTag>,
                         std::less<drake::TypeSafeIndex<DependencyTag>>,
                         std::allocator<drake::TypeSafeIndex<DependencyTag>>>
                  prerequisites_of_calc) {
             return self.DeclareAbstractOutputPort(
                 alloc_function, calc_function, prerequisites_of_calc);
           })
      .def("DeclareAbstractParameter",
           static_cast<int (LeafSystem<PyLeafSystem_double_0>::*)(
               ::drake::AbstractValue const &)>(
               &LeafSystem_double_publicist::DeclareAbstractParameter),
           py::arg("model_value"))
      .def(
          "DeclareAbstractState",
          static_cast<AbstractStateIndex (LeafSystem<PyLeafSystem_double_0>::*)(
              ::drake::AbstractValue const &)>(
              &LeafSystem_double_publicist::DeclareAbstractState),
          py::arg("abstract_state"))
      .def("DeclareAbstractState",
           [](LeafSystem<PyLeafSystem_double_0> &self,
              drake::AbstractValue abstract_state) {
             return self.DeclareAbstractState(
                 std::make_unique<drake::AbstractValue>(abstract_state));
           })
      .def("DeclareContinuousState",
           static_cast<void (LeafSystem<PyLeafSystem_double_0>::*)(int)>(
               &LeafSystem_double_publicist::DeclareContinuousState),
           py::arg("num_state_variables"))
      .def("DeclareContinuousState",
           static_cast<void (LeafSystem<PyLeafSystem_double_0>::*)(int, int,
                                                                   int)>(
               &LeafSystem_double_publicist::DeclareContinuousState),
           py::arg("num_q"), py::arg("num_v"), py::arg("num_z"))
      .def("DeclareContinuousState",
           static_cast<void (LeafSystem<PyLeafSystem_double_0>::*)(
               BasicVector<PyLeafSystem_double_0> const &)>(
               &LeafSystem_double_publicist::DeclareContinuousState),
           py::arg("model_vector"))
      .def("DeclareContinuousState",
           static_cast<void (LeafSystem<PyLeafSystem_double_0>::*)(
               BasicVector<PyLeafSystem_double_0> const &, int, int, int)>(
               &LeafSystem_double_publicist::DeclareContinuousState),
           py::arg("model_vector"), py::arg("num_q"), py::arg("num_v"),
           py::arg("num_z"))
      .def(
          "DeclareDiscreteState",
          static_cast<DiscreteStateIndex (LeafSystem<PyLeafSystem_double_0>::*)(
              BasicVector<PyLeafSystem_double_0> const &)>(
              &LeafSystem_double_publicist::DeclareDiscreteState),
          py::arg("model_vector"))
      .def("DeclareDiscreteState",
           [](LeafSystem<PyLeafSystem_double_0> &self,
              ::Eigen::Ref<const Eigen::Matrix<double, -1, 1, 0, -1, 1>, 0,
                           Eigen::InnerStride<1>> const &vector) {
             return self.DeclareDiscreteState(vector);
           })
      .def(
          "DeclareDiscreteState",
          static_cast<DiscreteStateIndex (LeafSystem<PyLeafSystem_double_0>::*)(
              int)>(&LeafSystem_double_publicist::DeclareDiscreteState),
          py::arg("num_state_variables"))
      .def("DeclareEqualityConstraint",
           [](LeafSystem<PyLeafSystem_double_0> &self,
              Eigen::Ref<::std::function<void(
                             const Context<PyLeafSystem_double_0> &,
                             Eigen::Matrix<double, -1, 1, 0, -1, 1> *)>,
                         0, Eigen::Stride<Eigen::Dynamic, Eigen::Dynamic>>
                  calc,
              int count, ::std::string description) {
             return self.DeclareEqualityConstraint(calc, count, description);
           })
      .def("DeclareImplicitTimeDerivativesResidualSize",
           static_cast<void (LeafSystem<PyLeafSystem_double_0>::*)(int)>(
               &LeafSystem_double_publicist::
                   DeclareImplicitTimeDerivativesResidualSize),
           py::arg("n"))
      .def("DeclareInequalityConstraint",
           [](LeafSystem<PyLeafSystem_double_0> &self,
              Eigen::Ref<::std::function<void(
                             const Context<PyLeafSystem_double_0> &,
                             Eigen::Matrix<double, -1, 1, 0, -1, 1> *)>,
                         0, Eigen::Stride<Eigen::Dynamic, Eigen::Dynamic>>
                  calc,
              SystemConstraintBounds bounds, ::std::string description) {
             return self.DeclareInequalityConstraint(calc, bounds, description);
           })
      .def("DeclareNumericParameter",
           static_cast<int (LeafSystem<PyLeafSystem_double_0>::*)(
               BasicVector<PyLeafSystem_double_0> const &)>(
               &LeafSystem_double_publicist::DeclareNumericParameter),
           py::arg("model_vector"))
      .def("DeclarePeriodicDiscreteUpdate",
           static_cast<void (LeafSystem<PyLeafSystem_double_0>::*)(double,
                                                                   double)>(
               &LeafSystem_double_publicist::DeclarePeriodicDiscreteUpdate),
           py::arg("period_sec"), py::arg("offset_sec") = double(0))
      .def("DeclarePeriodicPublish",
           static_cast<void (LeafSystem<PyLeafSystem_double_0>::*)(double,
                                                                   double)>(
               &LeafSystem_double_publicist::DeclarePeriodicPublish),
           py::arg("period_sec"), py::arg("offset_sec") = double(0))
      .def("DeclarePeriodicUnrestrictedUpdate",
           static_cast<void (LeafSystem<PyLeafSystem_double_0>::*)(double,
                                                                   double)>(
               &LeafSystem_double_publicist::DeclarePeriodicUnrestrictedUpdate),
           py::arg("period_sec"), py::arg("offset_sec") = double(0))
      .def("DeclareVectorInputPort",
           static_cast<InputPort<PyLeafSystem_double_0> &(
               LeafSystem<PyLeafSystem_double_0>::
                   *)(::std::variant<
                          std::basic_string<char, std::char_traits<char>,
                                            std::allocator<char>>,
                          UseDefaultName>,
                      BasicVector<PyLeafSystem_double_0> const &,
                      ::std::optional<drake::RandomDistribution>)>(
               &LeafSystem_double_publicist::DeclareVectorInputPort),
           py::arg("name"), py::arg("model_vector"),
           py::arg("random_type") =
               ::std::optional<drake::RandomDistribution>(std::nullopt))
      .def("DeclareVectorInputPort",
           static_cast<InputPort<PyLeafSystem_double_0> &(
               LeafSystem<PyLeafSystem_double_0>::
                   *)(BasicVector<PyLeafSystem_double_0> const &,
                      ::std::optional<drake::RandomDistribution>)>(
               &LeafSystem_double_publicist::DeclareVectorInputPort),
           py::arg("model_vector"),
           py::arg("random_type") =
               ::std::optional<drake::RandomDistribution>(std::nullopt))
      .def(
          "DeclareVectorOutputPort",
          static_cast<LeafOutputPort<PyLeafSystem_double_0> &(
              LeafSystem<PyLeafSystem_double_0>::
                  *)(::std::variant<
                         std::basic_string<char, std::char_traits<char>,
                                           std::allocator<char>>,
                         UseDefaultName>,
                     BasicVector<PyLeafSystem_double_0> const &,
                     LeafOutputPort<PyLeafSystem_double_0>::CalcVectorCallback,
                     ::std::set<
                         drake::TypeSafeIndex<DependencyTag>,
                         std::less<drake::TypeSafeIndex<DependencyTag>>,
                         std::allocator<drake::TypeSafeIndex<DependencyTag>>>)>(
              &LeafSystem_double_publicist::DeclareVectorOutputPort),
          py::arg("name"), py::arg("model_vector"),
          py::arg("vector_calc_function"),
          py::arg("prerequisites_of_calc") =
              ::std::set<drake::TypeSafeIndex<DependencyTag>,
                         std::less<drake::TypeSafeIndex<DependencyTag>>,
                         std::allocator<drake::TypeSafeIndex<DependencyTag>>>(
                  {SystemBase::all_sources_ticket()}))
      .def(
          "DeclareVectorOutputPort",
          static_cast<LeafOutputPort<PyLeafSystem_double_0> &(
              LeafSystem<PyLeafSystem_double_0>::
                  *)(BasicVector<PyLeafSystem_double_0> const &,
                     LeafOutputPort<PyLeafSystem_double_0>::CalcVectorCallback,
                     ::std::set<
                         drake::TypeSafeIndex<DependencyTag>,
                         std::less<drake::TypeSafeIndex<DependencyTag>>,
                         std::allocator<drake::TypeSafeIndex<DependencyTag>>>)>(
              &LeafSystem_double_publicist::DeclareVectorOutputPort),
          py::arg("model_vector"), py::arg("vector_calc_function"),
          py::arg("prerequisites_of_calc") =
              ::std::set<drake::TypeSafeIndex<DependencyTag>,
                         std::less<drake::TypeSafeIndex<DependencyTag>>,
                         std::allocator<drake::TypeSafeIndex<DependencyTag>>>(
                  {SystemBase::all_sources_ticket()}))
      .def(
          "DoAllocateContext",
          static_cast<
              ::std::unique_ptr<ContextBase, std::default_delete<ContextBase>> (
                  LeafSystem<PyLeafSystem_double_0>::*)() const>(
              &LeafSystem<PyLeafSystem_double_0>::DoAllocateContext))
      .def("DoCalcDiscreteVariableUpdates",
           static_cast<void (LeafSystem<PyLeafSystem_double_0>::*)(
               Context<PyLeafSystem_double_0> const &,
               ::std::vector<const DiscreteUpdateEvent<PyLeafSystem_double_0> *,
                             std::allocator<const DiscreteUpdateEvent<
                                 PyLeafSystem_double_0> *>> const &,
               DiscreteValues<PyLeafSystem_double_0> *) const>(
               &LeafSystem_double_publicist::DoCalcDiscreteVariableUpdates),
           py::arg("context"), py::arg("events"), py::arg("discrete_state"))
      .def("DoCalcNextUpdateTime",
           static_cast<void (LeafSystem<PyLeafSystem_double_0>::*)(
               Context<PyLeafSystem_double_0> const &,
               CompositeEventCollection<PyLeafSystem_double_0> *, double *)
                           const>(
               &LeafSystem_double_publicist::DoCalcNextUpdateTime),
           py::arg("context"), py::arg("events"), py::arg("time"))
      .def("DoCalcUnrestrictedUpdate",
           static_cast<void (LeafSystem<PyLeafSystem_double_0>::*)(
               Context<PyLeafSystem_double_0> const &,
               ::std::vector<
                   const UnrestrictedUpdateEvent<PyLeafSystem_double_0> *,
                   std::allocator<const UnrestrictedUpdateEvent<
                       PyLeafSystem_double_0> *>> const &,
               State<PyLeafSystem_double_0> *) const>(
               &LeafSystem_double_publicist::DoCalcUnrestrictedUpdate),
           py::arg("context"), py::arg("events"), py::arg("state"))
      .def("DoCalcWitnessValue",
           static_cast<double (LeafSystem<PyLeafSystem_double_0>::*)(
               Context<PyLeafSystem_double_0> const &,
               WitnessFunction<PyLeafSystem_double_0> const &) const>(
               &LeafSystem_double_publicist::DoCalcWitnessValue),
           py::arg("context"), py::arg("witness_func"))
      .def("DoMakeLeafContext",
           static_cast<::std::unique_ptr<
               LeafContext<PyLeafSystem_double_0>,
               std::default_delete<LeafContext<PyLeafSystem_double_0>>> (
               LeafSystem<PyLeafSystem_double_0>::*)() const>(
               &LeafSystem_double_publicist::DoMakeLeafContext))
      .def("DoPublish",
           static_cast<void (LeafSystem<PyLeafSystem_double_0>::*)(
               Context<PyLeafSystem_double_0> const &,
               ::std::vector<const PublishEvent<PyLeafSystem_double_0> *,
                             std::allocator<const PublishEvent<
                                 PyLeafSystem_double_0> *>> const &) const>(
               &LeafSystem_double_publicist::DoPublish),
           py::arg("context"), py::arg("events"))
      .def("DoValidateAllocatedLeafContext",
           static_cast<void (LeafSystem<PyLeafSystem_double_0>::*)(
               LeafContext<PyLeafSystem_double_0> const &) const>(
               &LeafSystem_double_publicist::DoValidateAllocatedLeafContext),
           py::arg("context"))
      .def("GetDirectFeedthroughs",
           static_cast<
               ::std::multimap<int, int, std::less<int>,
                               std::allocator<std::pair<const int, int>>> (
                   LeafSystem<PyLeafSystem_double_0>::*)() const>(
               &LeafSystem<PyLeafSystem_double_0>::GetDirectFeedthroughs))
      .def("GetGraphvizFragment",
           static_cast<void (LeafSystem<PyLeafSystem_double_0>::*)(
               int, ::std::stringstream *) const>(
               &LeafSystem_double_publicist::GetGraphvizFragment),
           py::arg("max_depth"), py::arg("dot"))
      .def("GetGraphvizInputPortToken",
           static_cast<void (LeafSystem<PyLeafSystem_double_0>::*)(
               InputPort<PyLeafSystem_double_0> const &, int,
               ::std::stringstream *) const>(
               &LeafSystem_double_publicist::GetGraphvizInputPortToken),
           py::arg("port"), py::arg("max_depth"), py::arg("dot"))
      .def("GetGraphvizOutputPortToken",
           static_cast<void (LeafSystem<PyLeafSystem_double_0>::*)(
               OutputPort<PyLeafSystem_double_0> const &, int,
               ::std::stringstream *) const>(
               &LeafSystem_double_publicist::GetGraphvizOutputPortToken),
           py::arg("port"), py::arg("max_depth"), py::arg("dot"))
      .def("MakeWitnessFunction",
           static_cast<::std::unique_ptr<
               WitnessFunction<PyLeafSystem_double_0>,
               std::default_delete<WitnessFunction<PyLeafSystem_double_0>>> (
               LeafSystem<PyLeafSystem_double_0>::*)(
               ::std::string const &, WitnessFunctionDirection const &,
               ::std::function<double(const Context<PyLeafSystem_double_0> &)>)
                           const>(
               &LeafSystem_double_publicist::MakeWitnessFunction),
           py::arg("description"), py::arg("direction_type"), py::arg("calc"))
      .def("MakeWitnessFunction",
           static_cast<::std::unique_ptr<
               WitnessFunction<PyLeafSystem_double_0>,
               std::default_delete<WitnessFunction<PyLeafSystem_double_0>>> (
               LeafSystem<PyLeafSystem_double_0>::*)(
               ::std::string const &, WitnessFunctionDirection const &,
               ::std::function<double(const Context<PyLeafSystem_double_0> &)>,
               Event<PyLeafSystem_double_0> const &) const>(
               &LeafSystem_double_publicist::MakeWitnessFunction),
           py::arg("description"), py::arg("direction_type"), py::arg("calc"),
           py::arg("e"))
      .def("SetDefaultParameters",
           static_cast<void (LeafSystem<PyLeafSystem_double_0>::*)(
               Context<PyLeafSystem_double_0> const &,
               Parameters<PyLeafSystem_double_0> *) const>(
               &LeafSystem<PyLeafSystem_double_0>::SetDefaultParameters),
           py::arg("context"), py::arg("parameters"))
      .def("SetDefaultState",
           static_cast<void (LeafSystem<PyLeafSystem_double_0>::*)(
               Context<PyLeafSystem_double_0> const &,
               State<PyLeafSystem_double_0> *) const>(
               &LeafSystem<PyLeafSystem_double_0>::SetDefaultState),
           py::arg("context"), py::arg("state"))

      ;

  using PyLeafSystem_Eigen_AutoDiffScalar_Eigen_VectorXd_0 =
      ::drake::AutoDiffXd;

  py::class_<LeafSystem<Eigen::AutoDiffScalar<Eigen::VectorXd>>,
             System<Eigen::AutoDiffScalar<Eigen::VectorXd>>>
      PyLeafSystem_Eigen_AutoDiffScalar_Eigen_VectorXd(
          m, "LeafSystem_Eigen_AutoDiffScalar_Eigen_VectorXd");

  PyLeafSystem_Eigen_AutoDiffScalar_Eigen_VectorXd
      .def("AddTriggeredWitnessFunctionToCompositeEventCollection",
           [](LeafSystem<Eigen::AutoDiffScalar<Eigen::VectorXd>> &self,
              Event<Eigen::AutoDiffScalar<Eigen::VectorXd>> *event,
              CompositeEventCollection<Eigen::AutoDiffScalar<Eigen::VectorXd>>
                  *events) {
             return self.AddTriggeredWitnessFunctionToCompositeEventCollection(
                 event, events);
           })
      .def("AllocateAbstractState",
           static_cast<::std::unique_ptr<AbstractValues,
                                         std::default_delete<AbstractValues>> (
               LeafSystem<Eigen::AutoDiffScalar<Eigen::VectorXd>>::*)() const>(
               &LeafSystem_Eigen_AutoDiffScalar_Eigen_VectorXd_publicist::
                   AllocateAbstractState))
      .def("AllocateCompositeEventCollection",
           static_cast<::std::unique_ptr<
               CompositeEventCollection<::drake::AutoDiffXd>,
               std::default_delete<
                   CompositeEventCollection<::drake::AutoDiffXd>>> (
               LeafSystem<Eigen::AutoDiffScalar<Eigen::VectorXd>>::*)() const>(
               &LeafSystem<Eigen::AutoDiffScalar<Eigen::VectorXd>>::
                   AllocateCompositeEventCollection))
      .def("AllocateContext",
           static_cast<::std::unique_ptr<
               LeafContext<::drake::AutoDiffXd>,
               std::default_delete<LeafContext<::drake::AutoDiffXd>>> (
               LeafSystem<Eigen::AutoDiffScalar<Eigen::VectorXd>>::*)() const>(
               &LeafSystem<
                   Eigen::AutoDiffScalar<Eigen::VectorXd>>::AllocateContext))
      .def("AllocateContinuousState",
           static_cast<::std::unique_ptr<
               ContinuousState<::drake::AutoDiffXd>,
               std::default_delete<ContinuousState<::drake::AutoDiffXd>>> (
               LeafSystem<Eigen::AutoDiffScalar<Eigen::VectorXd>>::*)() const>(
               &LeafSystem_Eigen_AutoDiffScalar_Eigen_VectorXd_publicist::
                   AllocateContinuousState))
      .def("AllocateDiscreteState",
           static_cast<::std::unique_ptr<
               DiscreteValues<::drake::AutoDiffXd>,
               std::default_delete<DiscreteValues<::drake::AutoDiffXd>>> (
               LeafSystem<Eigen::AutoDiffScalar<Eigen::VectorXd>>::*)() const>(
               &LeafSystem_Eigen_AutoDiffScalar_Eigen_VectorXd_publicist::
                   AllocateDiscreteState))
      .def("AllocateDiscreteVariables",
           static_cast<::std::unique_ptr<
               DiscreteValues<::drake::AutoDiffXd>,
               std::default_delete<DiscreteValues<::drake::AutoDiffXd>>> (
               LeafSystem<Eigen::AutoDiffScalar<Eigen::VectorXd>>::*)() const>(
               &LeafSystem<Eigen::AutoDiffScalar<Eigen::VectorXd>>::
                   AllocateDiscreteVariables))
      .def("AllocateForcedDiscreteUpdateEventCollection",
           static_cast<::std::unique_ptr<
               EventCollection<DiscreteUpdateEvent<::drake::AutoDiffXd>>,
               std::default_delete<
                   EventCollection<DiscreteUpdateEvent<::drake::AutoDiffXd>>>> (
               LeafSystem<Eigen::AutoDiffScalar<Eigen::VectorXd>>::*)() const>(
               &LeafSystem<Eigen::AutoDiffScalar<Eigen::VectorXd>>::
                   AllocateForcedDiscreteUpdateEventCollection))
      .def("AllocateForcedPublishEventCollection",
           static_cast<::std::unique_ptr<
               EventCollection<PublishEvent<::drake::AutoDiffXd>>,
               std::default_delete<
                   EventCollection<PublishEvent<::drake::AutoDiffXd>>>> (
               LeafSystem<Eigen::AutoDiffScalar<Eigen::VectorXd>>::*)() const>(
               &LeafSystem<Eigen::AutoDiffScalar<Eigen::VectorXd>>::
                   AllocateForcedPublishEventCollection))
      .def("AllocateForcedUnrestrictedUpdateEventCollection",
           static_cast<::std::unique_ptr<
               EventCollection<UnrestrictedUpdateEvent<::drake::AutoDiffXd>>,
               std::default_delete<EventCollection<
                   UnrestrictedUpdateEvent<::drake::AutoDiffXd>>>> (
               LeafSystem<Eigen::AutoDiffScalar<Eigen::VectorXd>>::*)() const>(
               &LeafSystem<Eigen::AutoDiffScalar<Eigen::VectorXd>>::
                   AllocateForcedUnrestrictedUpdateEventCollection))
      .def("AllocateParameters",
           static_cast<::std::unique_ptr<
               Parameters<::drake::AutoDiffXd>,
               std::default_delete<Parameters<::drake::AutoDiffXd>>> (
               LeafSystem<Eigen::AutoDiffScalar<Eigen::VectorXd>>::*)() const>(
               &LeafSystem_Eigen_AutoDiffScalar_Eigen_VectorXd_publicist::
                   AllocateParameters))
      .def("AllocateTimeDerivatives",
           static_cast<::std::unique_ptr<
               ContinuousState<::drake::AutoDiffXd>,
               std::default_delete<ContinuousState<::drake::AutoDiffXd>>> (
               LeafSystem<Eigen::AutoDiffScalar<Eigen::VectorXd>>::*)() const>(
               &LeafSystem<Eigen::AutoDiffScalar<Eigen::VectorXd>>::
                   AllocateTimeDerivatives))
      .def("DeclareAbstractInputPort",
           static_cast<InputPort<Eigen::AutoDiffScalar<Eigen::VectorXd>> &(
               LeafSystem<Eigen::AutoDiffScalar<Eigen::VectorXd>>::
                   *)(::std::variant<
                          std::basic_string<char, std::char_traits<char>,
                                            std::allocator<char>>,
                          UseDefaultName>,
                      ::drake::AbstractValue const &)>(
               &LeafSystem_Eigen_AutoDiffScalar_Eigen_VectorXd_publicist::
                   DeclareAbstractInputPort),
           py::arg("name"), py::arg("model_value"),
           py::return_value_policy::reference_internal)
      .def("DeclareAbstractInputPort",
           static_cast<InputPort<Eigen::AutoDiffScalar<Eigen::VectorXd>> &(
               LeafSystem<Eigen::AutoDiffScalar<Eigen::VectorXd>>::
                   *)(::drake::AbstractValue const &)>(
               &LeafSystem_Eigen_AutoDiffScalar_Eigen_VectorXd_publicist::
                   DeclareAbstractInputPort),
           py::arg("model_value"), py::return_value_policy::reference_internal)
      .def("DeclareAbstractOutputPort",
           [](LeafSystem<Eigen::AutoDiffScalar<Eigen::VectorXd>> &self,
              ::std::variant<std::basic_string<char, std::char_traits<char>,
                                               std::allocator<char>>,
                             UseDefaultName>
                  name,
              LeafOutputPort<Eigen::AutoDiffScalar<Eigen::VectorXd>>::
                  AllocCallback alloc_function,
              LeafOutputPort<Eigen::AutoDiffScalar<Eigen::VectorXd>>::
                  CalcCallback calc_function,
              ::std::set<drake::TypeSafeIndex<DependencyTag>,
                         std::less<drake::TypeSafeIndex<DependencyTag>>,
                         std::allocator<drake::TypeSafeIndex<DependencyTag>>>
                  prerequisites_of_calc) {
             return self.DeclareAbstractOutputPort(
                 name, alloc_function, calc_function, prerequisites_of_calc);
           })
      .def("DeclareAbstractOutputPort",
           [](LeafSystem<Eigen::AutoDiffScalar<Eigen::VectorXd>> &self,
              LeafOutputPort<Eigen::AutoDiffScalar<Eigen::VectorXd>>::
                  AllocCallback alloc_function,
              LeafOutputPort<Eigen::AutoDiffScalar<Eigen::VectorXd>>::
                  CalcCallback calc_function,
              ::std::set<drake::TypeSafeIndex<DependencyTag>,
                         std::less<drake::TypeSafeIndex<DependencyTag>>,
                         std::allocator<drake::TypeSafeIndex<DependencyTag>>>
                  prerequisites_of_calc) {
             return self.DeclareAbstractOutputPort(
                 alloc_function, calc_function, prerequisites_of_calc);
           })
      .def("DeclareAbstractParameter",
           static_cast<int (
               LeafSystem<Eigen::AutoDiffScalar<Eigen::VectorXd>>::*)(
               ::drake::AbstractValue const &)>(
               &LeafSystem_Eigen_AutoDiffScalar_Eigen_VectorXd_publicist::
                   DeclareAbstractParameter),
           py::arg("model_value"))
      .def("DeclareAbstractState",
           static_cast<AbstractStateIndex (
               LeafSystem<Eigen::AutoDiffScalar<Eigen::VectorXd>>::*)(
               ::drake::AbstractValue const &)>(
               &LeafSystem_Eigen_AutoDiffScalar_Eigen_VectorXd_publicist::
                   DeclareAbstractState),
           py::arg("abstract_state"))
      .def("DeclareAbstractState",
           [](LeafSystem<Eigen::AutoDiffScalar<Eigen::VectorXd>> &self,
              drake::AbstractValue abstract_state) {
             return self.DeclareAbstractState(
                 std::make_unique<drake::AbstractValue>(abstract_state));
           })
      .def("DeclareContinuousState",
           static_cast<void (
               LeafSystem<Eigen::AutoDiffScalar<Eigen::VectorXd>>::*)(int)>(
               &LeafSystem_Eigen_AutoDiffScalar_Eigen_VectorXd_publicist::
                   DeclareContinuousState),
           py::arg("num_state_variables"))
      .def("DeclareContinuousState",
           static_cast<void (
               LeafSystem<Eigen::AutoDiffScalar<Eigen::VectorXd>>::*)(int, int,
                                                                      int)>(
               &LeafSystem_Eigen_AutoDiffScalar_Eigen_VectorXd_publicist::
                   DeclareContinuousState),
           py::arg("num_q"), py::arg("num_v"), py::arg("num_z"))
      .def("DeclareContinuousState",
           [](LeafSystem<Eigen::AutoDiffScalar<Eigen::VectorXd>> &self,
              BasicVector<Eigen::AutoDiffScalar<Eigen::VectorXd>> const
                  &model_vector) {
             return self.DeclareContinuousState(model_vector);
           })
      .def("DeclareContinuousState",
           [](LeafSystem<Eigen::AutoDiffScalar<Eigen::VectorXd>> &self,
              BasicVector<Eigen::AutoDiffScalar<Eigen::VectorXd>> const
                  &model_vector,
              int num_q, int num_v, int num_z) {
             return self.DeclareContinuousState(model_vector, num_q, num_v,
                                                num_z);
           })
      .def("DeclareDiscreteState",
           [](LeafSystem<Eigen::AutoDiffScalar<Eigen::VectorXd>> &self,
              BasicVector<Eigen::AutoDiffScalar<Eigen::VectorXd>> const
                  &model_vector) {
             return self.DeclareDiscreteState(model_vector);
           })
      .def("DeclareDiscreteState",
           [](LeafSystem<Eigen::AutoDiffScalar<Eigen::VectorXd>> &self,
              ::Eigen::Ref<
                  const Eigen::Matrix<Eigen::AutoDiffScalar<Eigen::VectorXd>,
                                      -1, 1, 0, -1, 1>,
                  0, Eigen::InnerStride<1>> const &vector) {
             return self.DeclareDiscreteState(vector);
           })
      .def("DeclareDiscreteState",
           static_cast<DiscreteStateIndex (
               LeafSystem<Eigen::AutoDiffScalar<Eigen::VectorXd>>::*)(int)>(
               &LeafSystem_Eigen_AutoDiffScalar_Eigen_VectorXd_publicist::
                   DeclareDiscreteState),
           py::arg("num_state_variables"))
      .def("DeclareEqualityConstraint",
           [](LeafSystem<Eigen::AutoDiffScalar<Eigen::VectorXd>> &self,
              Eigen::Ref<
                  ::std::function<void(
                      const Context<::drake::AutoDiffXd> &,
                      Eigen::Matrix<Eigen::AutoDiffScalar<Eigen::VectorXd>, -1,
                                    1, 0, -1, 1> *)>,
                  0, Eigen::Stride<Eigen::Dynamic, Eigen::Dynamic>>
                  calc,
              int count, ::std::string description) {
             return self.DeclareEqualityConstraint(calc, count, description);
           })
      .def("DeclareImplicitTimeDerivativesResidualSize",
           static_cast<void (
               LeafSystem<Eigen::AutoDiffScalar<Eigen::VectorXd>>::*)(int)>(
               &LeafSystem_Eigen_AutoDiffScalar_Eigen_VectorXd_publicist::
                   DeclareImplicitTimeDerivativesResidualSize),
           py::arg("n"))
      .def("DeclareInequalityConstraint",
           [](LeafSystem<Eigen::AutoDiffScalar<Eigen::VectorXd>> &self,
              Eigen::Ref<
                  ::std::function<void(
                      const Context<::drake::AutoDiffXd> &,
                      Eigen::Matrix<Eigen::AutoDiffScalar<Eigen::VectorXd>, -1,
                                    1, 0, -1, 1> *)>,
                  0, Eigen::Stride<Eigen::Dynamic, Eigen::Dynamic>>
                  calc,
              SystemConstraintBounds bounds, ::std::string description) {
             return self.DeclareInequalityConstraint(calc, bounds, description);
           })
      .def("DeclareNumericParameter",
           [](LeafSystem<Eigen::AutoDiffScalar<Eigen::VectorXd>> &self,
              BasicVector<Eigen::AutoDiffScalar<Eigen::VectorXd>> const
                  &model_vector) {
             return self.DeclareNumericParameter(model_vector);
           })
      .def("DeclarePeriodicDiscreteUpdate",
           static_cast<void (
               LeafSystem<Eigen::AutoDiffScalar<Eigen::VectorXd>>::*)(double,
                                                                      double)>(
               &LeafSystem_Eigen_AutoDiffScalar_Eigen_VectorXd_publicist::
                   DeclarePeriodicDiscreteUpdate),
           py::arg("period_sec"), py::arg("offset_sec") = double(0))
      .def("DeclarePeriodicPublish",
           static_cast<void (
               LeafSystem<Eigen::AutoDiffScalar<Eigen::VectorXd>>::*)(double,
                                                                      double)>(
               &LeafSystem_Eigen_AutoDiffScalar_Eigen_VectorXd_publicist::
                   DeclarePeriodicPublish),
           py::arg("period_sec"), py::arg("offset_sec") = double(0))
      .def("DeclarePeriodicUnrestrictedUpdate",
           static_cast<void (
               LeafSystem<Eigen::AutoDiffScalar<Eigen::VectorXd>>::*)(double,
                                                                      double)>(
               &LeafSystem_Eigen_AutoDiffScalar_Eigen_VectorXd_publicist::
                   DeclarePeriodicUnrestrictedUpdate),
           py::arg("period_sec"), py::arg("offset_sec") = double(0))
      .def(
          "DeclareVectorInputPort",
          [](LeafSystem<Eigen::AutoDiffScalar<Eigen::VectorXd>> &self,
             ::std::variant<std::basic_string<char, std::char_traits<char>,
                                              std::allocator<char>>,
                            UseDefaultName>
                 name,
             BasicVector<Eigen::AutoDiffScalar<Eigen::VectorXd>> const
                 &model_vector,
             ::std::optional<drake::RandomDistribution> random_type) {
            return self.DeclareVectorInputPort(name, model_vector, random_type);
          },
          py::return_value_policy::reference_internal)
      .def(
          "DeclareVectorInputPort",
          [](LeafSystem<Eigen::AutoDiffScalar<Eigen::VectorXd>> &self,
             BasicVector<Eigen::AutoDiffScalar<Eigen::VectorXd>> const
                 &model_vector,
             ::std::optional<drake::RandomDistribution> random_type) {
            return self.DeclareVectorInputPort(model_vector, random_type);
          },
          py::return_value_policy::reference_internal)
      .def(
          "DeclareVectorOutputPort",
          [](LeafSystem<Eigen::AutoDiffScalar<Eigen::VectorXd>> &self,
             ::std::variant<std::basic_string<char, std::char_traits<char>,
                                              std::allocator<char>>,
                            UseDefaultName>
                 name,
             BasicVector<Eigen::AutoDiffScalar<Eigen::VectorXd>> const
                 &model_vector,
             LeafOutputPort<Eigen::AutoDiffScalar<Eigen::VectorXd>>::
                 CalcVectorCallback vector_calc_function,
             ::std::set<drake::TypeSafeIndex<DependencyTag>,
                        std::less<drake::TypeSafeIndex<DependencyTag>>,
                        std::allocator<drake::TypeSafeIndex<DependencyTag>>>
                 prerequisites_of_calc) {
            return self.DeclareVectorOutputPort(name, model_vector,
                                                vector_calc_function,
                                                prerequisites_of_calc);
          },
          py::return_value_policy::reference_internal)
      .def(
          "DeclareVectorOutputPort",
          [](LeafSystem<Eigen::AutoDiffScalar<Eigen::VectorXd>> &self,
             BasicVector<Eigen::AutoDiffScalar<Eigen::VectorXd>> const
                 &model_vector,
             LeafOutputPort<Eigen::AutoDiffScalar<Eigen::VectorXd>>::
                 CalcVectorCallback vector_calc_function,
             ::std::set<drake::TypeSafeIndex<DependencyTag>,
                        std::less<drake::TypeSafeIndex<DependencyTag>>,
                        std::allocator<drake::TypeSafeIndex<DependencyTag>>>
                 prerequisites_of_calc) {
            return self.DeclareVectorOutputPort(
                model_vector, vector_calc_function, prerequisites_of_calc);
          },
          py::return_value_policy::reference_internal)
      .def("DoAllocateContext",
           static_cast<::std::unique_ptr<ContextBase,
                                         std::default_delete<ContextBase>> (
               LeafSystem<Eigen::AutoDiffScalar<Eigen::VectorXd>>::*)() const>(
               &LeafSystem<
                   Eigen::AutoDiffScalar<Eigen::VectorXd>>::DoAllocateContext))
      .def("DoCalcDiscreteVariableUpdates",
           [](LeafSystem<Eigen::AutoDiffScalar<Eigen::VectorXd>> &self,
              Context<Eigen::AutoDiffScalar<Eigen::VectorXd>> const &context,
              ::std::vector<const DiscreteUpdateEvent<::drake::AutoDiffXd> *,
                            std::allocator<const DiscreteUpdateEvent<
                                ::drake::AutoDiffXd> *>> const &events,
              DiscreteValues<Eigen::AutoDiffScalar<Eigen::VectorXd>>
                  *discrete_state) {
             return self.DoCalcDiscreteVariableUpdates(context, events,
                                                       discrete_state);
           })
      .def("DoCalcNextUpdateTime",
           [](LeafSystem<Eigen::AutoDiffScalar<Eigen::VectorXd>> &self,
              Context<Eigen::AutoDiffScalar<Eigen::VectorXd>> const &context,
              CompositeEventCollection<Eigen::AutoDiffScalar<Eigen::VectorXd>>
                  *events,
              Eigen::Ref<::Eigen::AutoDiffScalar<
                             Eigen::Matrix<double, -1, 1, 0, -1, 1>> *,
                         0, Eigen::Stride<Eigen::Dynamic, Eigen::Dynamic>>
                  time) {
             return self.DoCalcNextUpdateTime(context, events, time);
           })
      .def(
          "DoCalcUnrestrictedUpdate",
          [](LeafSystem<Eigen::AutoDiffScalar<Eigen::VectorXd>> &self,
             Context<Eigen::AutoDiffScalar<Eigen::VectorXd>> const &context,
             ::std::vector<const UnrestrictedUpdateEvent<::drake::AutoDiffXd> *,
                           std::allocator<const UnrestrictedUpdateEvent<
                               ::drake::AutoDiffXd> *>> const &events,
             State<Eigen::AutoDiffScalar<Eigen::VectorXd>> *state) {
            return self.DoCalcUnrestrictedUpdate(context, events, state);
          })
      .def("DoCalcWitnessValue",
           [](LeafSystem<Eigen::AutoDiffScalar<Eigen::VectorXd>> &self,
              Context<Eigen::AutoDiffScalar<Eigen::VectorXd>> const &context,
              WitnessFunction<Eigen::AutoDiffScalar<Eigen::VectorXd>> const
                  &witness_func) {
             return self.DoCalcWitnessValue(context, witness_func);
           })
      .def("DoMakeLeafContext",
           static_cast<::std::unique_ptr<
               LeafContext<::drake::AutoDiffXd>,
               std::default_delete<LeafContext<::drake::AutoDiffXd>>> (
               LeafSystem<Eigen::AutoDiffScalar<Eigen::VectorXd>>::*)() const>(
               &LeafSystem_Eigen_AutoDiffScalar_Eigen_VectorXd_publicist::
                   DoMakeLeafContext))
      .def("DoPublish",
           [](LeafSystem<Eigen::AutoDiffScalar<Eigen::VectorXd>> &self,
              Context<Eigen::AutoDiffScalar<Eigen::VectorXd>> const &context,
              ::std::vector<const PublishEvent<::drake::AutoDiffXd> *,
                            std::allocator<const PublishEvent<
                                ::drake::AutoDiffXd> *>> const &events) {
             return self.DoPublish(context, events);
           })
      .def("DoValidateAllocatedLeafContext",
           [](LeafSystem<Eigen::AutoDiffScalar<Eigen::VectorXd>> &self,
              LeafContext<Eigen::AutoDiffScalar<Eigen::VectorXd>> const
                  &context) {
             return self.DoValidateAllocatedLeafContext(context);
           })
      .def("GetDirectFeedthroughs",
           static_cast<
               ::std::multimap<int, int, std::less<int>,
                               std::allocator<std::pair<const int, int>>> (
                   LeafSystem<Eigen::AutoDiffScalar<Eigen::VectorXd>>::*)()
                   const>(&LeafSystem<Eigen::AutoDiffScalar<Eigen::VectorXd>>::
                              GetDirectFeedthroughs))
      .def("GetGraphvizFragment",
           static_cast<void (
               LeafSystem<Eigen::AutoDiffScalar<Eigen::VectorXd>>::*)(
               int, ::std::stringstream *) const>(
               &LeafSystem_Eigen_AutoDiffScalar_Eigen_VectorXd_publicist::
                   GetGraphvizFragment),
           py::arg("max_depth"), py::arg("dot"))
      .def("GetGraphvizInputPortToken",
           [](LeafSystem<Eigen::AutoDiffScalar<Eigen::VectorXd>> &self,
              InputPort<Eigen::AutoDiffScalar<Eigen::VectorXd>> const &port,
              int max_depth, ::std::stringstream *dot) {
             return self.GetGraphvizInputPortToken(port, max_depth, dot);
           })
      .def("GetGraphvizOutputPortToken",
           [](LeafSystem<Eigen::AutoDiffScalar<Eigen::VectorXd>> &self,
              OutputPort<Eigen::AutoDiffScalar<Eigen::VectorXd>> const &port,
              int max_depth, ::std::stringstream *dot) {
             return self.GetGraphvizOutputPortToken(port, max_depth, dot);
           })
      .def("MakeWitnessFunction",
           [](LeafSystem<Eigen::AutoDiffScalar<Eigen::VectorXd>> &self,
              ::std::string const &description,
              WitnessFunctionDirection const &direction_type,
              ::std::function<Eigen::AutoDiffScalar<Eigen::VectorXd>(
                  const Context<::drake::AutoDiffXd> &)>
                  calc) {
             return self.MakeWitnessFunction(description, direction_type, calc);
           })
      .def("MakeWitnessFunction",
           [](LeafSystem<Eigen::AutoDiffScalar<Eigen::VectorXd>> &self,
              ::std::string const &description,
              WitnessFunctionDirection const &direction_type,
              ::std::function<Eigen::AutoDiffScalar<Eigen::VectorXd>(
                  const Context<::drake::AutoDiffXd> &)>
                  calc,
              Event<Eigen::AutoDiffScalar<Eigen::VectorXd>> const &e) {
             return self.MakeWitnessFunction(description, direction_type, calc,
                                             e);
           })
      .def("SetDefaultParameters",
           [](LeafSystem<Eigen::AutoDiffScalar<Eigen::VectorXd>> &self,
              Context<Eigen::AutoDiffScalar<Eigen::VectorXd>> const &context,
              Parameters<Eigen::AutoDiffScalar<Eigen::VectorXd>> *parameters) {
             return self.SetDefaultParameters(context, parameters);
           })
      .def("SetDefaultState",
           [](LeafSystem<Eigen::AutoDiffScalar<Eigen::VectorXd>> &self,
              Context<Eigen::AutoDiffScalar<Eigen::VectorXd>> const &context,
              State<Eigen::AutoDiffScalar<Eigen::VectorXd>> *state) {
             return self.SetDefaultState(context, state);
           })

      ;

  using PyLeafSystem_float_0 = float;

  py::class_<LeafSystem<PyLeafSystem_float_0>> PyLeafSystem_float(
      m, "LeafSystem_float");

  PyLeafSystem_float
      .def("AddTriggeredWitnessFunctionToCompositeEventCollection",
           static_cast<void (LeafSystem<PyLeafSystem_float_0>::*)(
               Event<PyLeafSystem_float_0> *,
               CompositeEventCollection<PyLeafSystem_float_0> *) const>(
               &LeafSystem_float_publicist::
                   AddTriggeredWitnessFunctionToCompositeEventCollection),
           py::arg("event"), py::arg("events"))
      .def("AllocateAbstractState",
           static_cast<::std::unique_ptr<AbstractValues,
                                         std::default_delete<AbstractValues>> (
               LeafSystem<PyLeafSystem_float_0>::*)() const>(
               &LeafSystem_float_publicist::AllocateAbstractState))
      .def("AllocateCompositeEventCollection",
           static_cast<
               ::std::unique_ptr<CompositeEventCollection<PyLeafSystem_float_0>,
                                 std::default_delete<CompositeEventCollection<
                                     PyLeafSystem_float_0>>> (
                   LeafSystem<PyLeafSystem_float_0>::*)() const>(
               &LeafSystem<
                   PyLeafSystem_float_0>::AllocateCompositeEventCollection))
      .def("AllocateContext",
           static_cast<::std::unique_ptr<
               LeafContext<PyLeafSystem_float_0>,
               std::default_delete<LeafContext<PyLeafSystem_float_0>>> (
               LeafSystem<PyLeafSystem_float_0>::*)() const>(
               &LeafSystem<PyLeafSystem_float_0>::AllocateContext))
      .def("AllocateContinuousState",
           static_cast<::std::unique_ptr<
               ContinuousState<PyLeafSystem_float_0>,
               std::default_delete<ContinuousState<PyLeafSystem_float_0>>> (
               LeafSystem<PyLeafSystem_float_0>::*)() const>(
               &LeafSystem_float_publicist::AllocateContinuousState))
      .def("AllocateDiscreteState",
           static_cast<::std::unique_ptr<
               DiscreteValues<PyLeafSystem_float_0>,
               std::default_delete<DiscreteValues<PyLeafSystem_float_0>>> (
               LeafSystem<PyLeafSystem_float_0>::*)() const>(
               &LeafSystem_float_publicist::AllocateDiscreteState))
      .def("AllocateDiscreteVariables",
           static_cast<::std::unique_ptr<
               DiscreteValues<PyLeafSystem_float_0>,
               std::default_delete<DiscreteValues<PyLeafSystem_float_0>>> (
               LeafSystem<PyLeafSystem_float_0>::*)() const>(
               &LeafSystem<PyLeafSystem_float_0>::AllocateDiscreteVariables))
      .def(
          "AllocateForcedDiscreteUpdateEventCollection",
          static_cast<::std::unique_ptr<
              EventCollection<DiscreteUpdateEvent<PyLeafSystem_float_0>>,
              std::default_delete<
                  EventCollection<DiscreteUpdateEvent<PyLeafSystem_float_0>>>> (
              LeafSystem<PyLeafSystem_float_0>::*)() const>(
              &LeafSystem<PyLeafSystem_float_0>::
                  AllocateForcedDiscreteUpdateEventCollection))
      .def("AllocateForcedPublishEventCollection",
           static_cast<::std::unique_ptr<
               EventCollection<PublishEvent<PyLeafSystem_float_0>>,
               std::default_delete<
                   EventCollection<PublishEvent<PyLeafSystem_float_0>>>> (
               LeafSystem<PyLeafSystem_float_0>::*)() const>(
               &LeafSystem<
                   PyLeafSystem_float_0>::AllocateForcedPublishEventCollection))
      .def("AllocateForcedUnrestrictedUpdateEventCollection",
           static_cast<::std::unique_ptr<
               EventCollection<UnrestrictedUpdateEvent<PyLeafSystem_float_0>>,
               std::default_delete<EventCollection<
                   UnrestrictedUpdateEvent<PyLeafSystem_float_0>>>> (
               LeafSystem<PyLeafSystem_float_0>::*)() const>(
               &LeafSystem<PyLeafSystem_float_0>::
                   AllocateForcedUnrestrictedUpdateEventCollection))
      .def("AllocateParameters",
           static_cast<::std::unique_ptr<
               Parameters<PyLeafSystem_float_0>,
               std::default_delete<Parameters<PyLeafSystem_float_0>>> (
               LeafSystem<PyLeafSystem_float_0>::*)() const>(
               &LeafSystem_float_publicist::AllocateParameters))
      .def("AllocateTimeDerivatives",
           static_cast<::std::unique_ptr<
               ContinuousState<PyLeafSystem_float_0>,
               std::default_delete<ContinuousState<PyLeafSystem_float_0>>> (
               LeafSystem<PyLeafSystem_float_0>::*)() const>(
               &LeafSystem<PyLeafSystem_float_0>::AllocateTimeDerivatives))
      .def(
          "DeclareAbstractOutputPort",
          [](LeafSystem<PyLeafSystem_float_0> &self,
             ::std::variant<std::basic_string<char, std::char_traits<char>,
                                              std::allocator<char>>,
                            UseDefaultName>
                 name,
             LeafOutputPort<PyLeafSystem_float_0>::AllocCallback alloc_function,
             LeafOutputPort<PyLeafSystem_float_0>::CalcCallback calc_function,
             ::std::set<drake::TypeSafeIndex<DependencyTag>,
                        std::less<drake::TypeSafeIndex<DependencyTag>>,
                        std::allocator<drake::TypeSafeIndex<DependencyTag>>>
                 prerequisites_of_calc) {
            return self.DeclareAbstractOutputPort(
                name, alloc_function, calc_function, prerequisites_of_calc);
          })
      .def(
          "DeclareAbstractOutputPort",
          [](LeafSystem<PyLeafSystem_float_0> &self,
             LeafOutputPort<PyLeafSystem_float_0>::AllocCallback alloc_function,
             LeafOutputPort<PyLeafSystem_float_0>::CalcCallback calc_function,
             ::std::set<drake::TypeSafeIndex<DependencyTag>,
                        std::less<drake::TypeSafeIndex<DependencyTag>>,
                        std::allocator<drake::TypeSafeIndex<DependencyTag>>>
                 prerequisites_of_calc) {
            return self.DeclareAbstractOutputPort(alloc_function, calc_function,
                                                  prerequisites_of_calc);
          })
      .def("DeclareAbstractParameter",
           static_cast<int (LeafSystem<PyLeafSystem_float_0>::*)(
               ::drake::AbstractValue const &)>(
               &LeafSystem_float_publicist::DeclareAbstractParameter),
           py::arg("model_value"))
      .def("DeclareAbstractState",
           static_cast<AbstractStateIndex (LeafSystem<PyLeafSystem_float_0>::*)(
               ::drake::AbstractValue const &)>(
               &LeafSystem_float_publicist::DeclareAbstractState),
           py::arg("abstract_state"))
      .def("DeclareAbstractState",
           [](LeafSystem<PyLeafSystem_float_0> &self,
              drake::AbstractValue abstract_state) {
             return self.DeclareAbstractState(
                 std::make_unique<drake::AbstractValue>(abstract_state));
           })
      .def("DeclareContinuousState",
           static_cast<void (LeafSystem<PyLeafSystem_float_0>::*)(int)>(
               &LeafSystem_float_publicist::DeclareContinuousState),
           py::arg("num_state_variables"))
      .def("DeclareContinuousState",
           static_cast<void (LeafSystem<PyLeafSystem_float_0>::*)(int, int,
                                                                  int)>(
               &LeafSystem_float_publicist::DeclareContinuousState),
           py::arg("num_q"), py::arg("num_v"), py::arg("num_z"))
      .def("DeclareContinuousState",
           static_cast<void (LeafSystem<PyLeafSystem_float_0>::*)(
               BasicVector<PyLeafSystem_float_0> const &)>(
               &LeafSystem_float_publicist::DeclareContinuousState),
           py::arg("model_vector"))
      .def("DeclareContinuousState",
           static_cast<void (LeafSystem<PyLeafSystem_float_0>::*)(
               BasicVector<PyLeafSystem_float_0> const &, int, int, int)>(
               &LeafSystem_float_publicist::DeclareContinuousState),
           py::arg("model_vector"), py::arg("num_q"), py::arg("num_v"),
           py::arg("num_z"))
      .def("DeclareDiscreteState",
           static_cast<DiscreteStateIndex (LeafSystem<PyLeafSystem_float_0>::*)(
               BasicVector<PyLeafSystem_float_0> const &)>(
               &LeafSystem_float_publicist::DeclareDiscreteState),
           py::arg("model_vector"))
      .def("DeclareDiscreteState",
           [](LeafSystem<PyLeafSystem_float_0> &self,
              ::Eigen::Ref<const Eigen::Matrix<float, -1, 1, 0, -1, 1>, 0,
                           Eigen::InnerStride<1>> const &vector) {
             return self.DeclareDiscreteState(vector);
           })
      .def("DeclareDiscreteState",
           static_cast<DiscreteStateIndex (LeafSystem<PyLeafSystem_float_0>::*)(
               int)>(&LeafSystem_float_publicist::DeclareDiscreteState),
           py::arg("num_state_variables"))
      .def(
          "DeclareEqualityConstraint",
          [](LeafSystem<PyLeafSystem_float_0> &self,
             Eigen::Ref<
                 ::std::function<void(const Context<PyLeafSystem_float_0> &,
                                      Eigen::Matrix<float, -1, 1, 0, -1, 1> *)>,
                 0, Eigen::Stride<Eigen::Dynamic, Eigen::Dynamic>>
                 calc,
             int count, ::std::string description) {
            return self.DeclareEqualityConstraint(calc, count, description);
          })
      .def("DeclareImplicitTimeDerivativesResidualSize",
           static_cast<void (LeafSystem<PyLeafSystem_float_0>::*)(int)>(
               &LeafSystem_float_publicist::
                   DeclareImplicitTimeDerivativesResidualSize),
           py::arg("n"))
      .def(
          "DeclareInequalityConstraint",
          [](LeafSystem<PyLeafSystem_float_0> &self,
             Eigen::Ref<
                 ::std::function<void(const Context<PyLeafSystem_float_0> &,
                                      Eigen::Matrix<float, -1, 1, 0, -1, 1> *)>,
                 0, Eigen::Stride<Eigen::Dynamic, Eigen::Dynamic>>
                 calc,
             SystemConstraintBounds bounds, ::std::string description) {
            return self.DeclareInequalityConstraint(calc, bounds, description);
          })
      .def("DeclareNumericParameter",
           static_cast<int (LeafSystem<PyLeafSystem_float_0>::*)(
               BasicVector<PyLeafSystem_float_0> const &)>(
               &LeafSystem_float_publicist::DeclareNumericParameter),
           py::arg("model_vector"))
      .def("DeclarePeriodicDiscreteUpdate",
           static_cast<void (LeafSystem<PyLeafSystem_float_0>::*)(double,
                                                                  double)>(
               &LeafSystem_float_publicist::DeclarePeriodicDiscreteUpdate),
           py::arg("period_sec"), py::arg("offset_sec") = double(0))
      .def("DeclarePeriodicPublish",
           static_cast<void (LeafSystem<PyLeafSystem_float_0>::*)(double,
                                                                  double)>(
               &LeafSystem_float_publicist::DeclarePeriodicPublish),
           py::arg("period_sec"), py::arg("offset_sec") = double(0))
      .def("DeclarePeriodicUnrestrictedUpdate",
           static_cast<void (LeafSystem<PyLeafSystem_float_0>::*)(double,
                                                                  double)>(
               &LeafSystem_float_publicist::DeclarePeriodicUnrestrictedUpdate),
           py::arg("period_sec"), py::arg("offset_sec") = double(0))
      .def(
          "DeclareVectorOutputPort",
          static_cast<LeafOutputPort<PyLeafSystem_float_0> &(
              LeafSystem<PyLeafSystem_float_0>::
                  *)(::std::variant<
                         std::basic_string<char, std::char_traits<char>,
                                           std::allocator<char>>,
                         UseDefaultName>,
                     BasicVector<PyLeafSystem_float_0> const &,
                     LeafOutputPort<PyLeafSystem_float_0>::CalcVectorCallback,
                     ::std::set<
                         drake::TypeSafeIndex<DependencyTag>,
                         std::less<drake::TypeSafeIndex<DependencyTag>>,
                         std::allocator<drake::TypeSafeIndex<DependencyTag>>>)>(
              &LeafSystem_float_publicist::DeclareVectorOutputPort),
          py::arg("name"), py::arg("model_vector"),
          py::arg("vector_calc_function"),
          py::arg("prerequisites_of_calc") =
              ::std::set<drake::TypeSafeIndex<DependencyTag>,
                         std::less<drake::TypeSafeIndex<DependencyTag>>,
                         std::allocator<drake::TypeSafeIndex<DependencyTag>>>(
                  {SystemBase::all_sources_ticket()}))
      .def(
          "DeclareVectorOutputPort",
          static_cast<LeafOutputPort<PyLeafSystem_float_0> &(
              LeafSystem<PyLeafSystem_float_0>::
                  *)(BasicVector<PyLeafSystem_float_0> const &,
                     LeafOutputPort<PyLeafSystem_float_0>::CalcVectorCallback,
                     ::std::set<
                         drake::TypeSafeIndex<DependencyTag>,
                         std::less<drake::TypeSafeIndex<DependencyTag>>,
                         std::allocator<drake::TypeSafeIndex<DependencyTag>>>)>(
              &LeafSystem_float_publicist::DeclareVectorOutputPort),
          py::arg("model_vector"), py::arg("vector_calc_function"),
          py::arg("prerequisites_of_calc") =
              ::std::set<drake::TypeSafeIndex<DependencyTag>,
                         std::less<drake::TypeSafeIndex<DependencyTag>>,
                         std::allocator<drake::TypeSafeIndex<DependencyTag>>>(
                  {SystemBase::all_sources_ticket()}))
      .def(
          "DoAllocateContext",
          static_cast<
              ::std::unique_ptr<ContextBase, std::default_delete<ContextBase>> (
                  LeafSystem<PyLeafSystem_float_0>::*)() const>(
              &LeafSystem<PyLeafSystem_float_0>::DoAllocateContext))
      .def("DoCalcDiscreteVariableUpdates",
           static_cast<void (LeafSystem<PyLeafSystem_float_0>::*)(
               Context<PyLeafSystem_float_0> const &,
               ::std::vector<const DiscreteUpdateEvent<PyLeafSystem_float_0> *,
                             std::allocator<const DiscreteUpdateEvent<
                                 PyLeafSystem_float_0> *>> const &,
               DiscreteValues<PyLeafSystem_float_0> *) const>(
               &LeafSystem_float_publicist::DoCalcDiscreteVariableUpdates),
           py::arg("context"), py::arg("events"), py::arg("discrete_state"))
      .def(
          "DoCalcNextUpdateTime",
          static_cast<void (LeafSystem<PyLeafSystem_float_0>::*)(
              Context<PyLeafSystem_float_0> const &,
              CompositeEventCollection<PyLeafSystem_float_0> *, float *) const>(
              &LeafSystem_float_publicist::DoCalcNextUpdateTime),
          py::arg("context"), py::arg("events"), py::arg("time"))
      .def("DoCalcUnrestrictedUpdate",
           static_cast<void (LeafSystem<PyLeafSystem_float_0>::*)(
               Context<PyLeafSystem_float_0> const &,
               ::std::vector<
                   const UnrestrictedUpdateEvent<PyLeafSystem_float_0> *,
                   std::allocator<const UnrestrictedUpdateEvent<
                       PyLeafSystem_float_0> *>> const &,
               State<PyLeafSystem_float_0> *) const>(
               &LeafSystem_float_publicist::DoCalcUnrestrictedUpdate),
           py::arg("context"), py::arg("events"), py::arg("state"))
      .def("DoMakeLeafContext",
           static_cast<::std::unique_ptr<
               LeafContext<PyLeafSystem_float_0>,
               std::default_delete<LeafContext<PyLeafSystem_float_0>>> (
               LeafSystem<PyLeafSystem_float_0>::*)() const>(
               &LeafSystem_float_publicist::DoMakeLeafContext))
      .def("DoPublish",
           static_cast<void (LeafSystem<PyLeafSystem_float_0>::*)(
               Context<PyLeafSystem_float_0> const &,
               ::std::vector<const PublishEvent<PyLeafSystem_float_0> *,
                             std::allocator<const PublishEvent<
                                 PyLeafSystem_float_0> *>> const &) const>(
               &LeafSystem_float_publicist::DoPublish),
           py::arg("context"), py::arg("events"))
      .def("DoValidateAllocatedLeafContext",
           static_cast<void (LeafSystem<PyLeafSystem_float_0>::*)(
               LeafContext<PyLeafSystem_float_0> const &) const>(
               &LeafSystem_float_publicist::DoValidateAllocatedLeafContext),
           py::arg("context"))
      .def("GetDirectFeedthroughs",
           static_cast<
               ::std::multimap<int, int, std::less<int>,
                               std::allocator<std::pair<const int, int>>> (
                   LeafSystem<PyLeafSystem_float_0>::*)() const>(
               &LeafSystem<PyLeafSystem_float_0>::GetDirectFeedthroughs))
      .def("GetGraphvizFragment",
           static_cast<void (LeafSystem<PyLeafSystem_float_0>::*)(
               int, ::std::stringstream *) const>(
               &LeafSystem_float_publicist::GetGraphvizFragment),
           py::arg("max_depth"), py::arg("dot"))
      .def("GetGraphvizOutputPortToken",
           static_cast<void (LeafSystem<PyLeafSystem_float_0>::*)(
               OutputPort<PyLeafSystem_float_0> const &, int,
               ::std::stringstream *) const>(
               &LeafSystem_float_publicist::GetGraphvizOutputPortToken),
           py::arg("port"), py::arg("max_depth"), py::arg("dot"))
      .def("MakeWitnessFunction",
           static_cast<::std::unique_ptr<
               WitnessFunction<PyLeafSystem_float_0>,
               std::default_delete<WitnessFunction<PyLeafSystem_float_0>>> (
               LeafSystem<PyLeafSystem_float_0>::*)(
               ::std::string const &, WitnessFunctionDirection const &,
               ::std::function<float(const Context<PyLeafSystem_float_0> &)>)
                           const>(
               &LeafSystem_float_publicist::MakeWitnessFunction),
           py::arg("description"), py::arg("direction_type"), py::arg("calc"))
      .def("MakeWitnessFunction",
           static_cast<::std::unique_ptr<
               WitnessFunction<PyLeafSystem_float_0>,
               std::default_delete<WitnessFunction<PyLeafSystem_float_0>>> (
               LeafSystem<PyLeafSystem_float_0>::*)(
               ::std::string const &, WitnessFunctionDirection const &,
               ::std::function<float(const Context<PyLeafSystem_float_0> &)>,
               Event<PyLeafSystem_float_0> const &) const>(
               &LeafSystem_float_publicist::MakeWitnessFunction),
           py::arg("description"), py::arg("direction_type"), py::arg("calc"),
           py::arg("e"))
      .def("SetDefaultParameters",
           static_cast<void (LeafSystem<PyLeafSystem_float_0>::*)(
               Context<PyLeafSystem_float_0> const &,
               Parameters<PyLeafSystem_float_0> *) const>(
               &LeafSystem<PyLeafSystem_float_0>::SetDefaultParameters),
           py::arg("context"), py::arg("parameters"))
      .def("SetDefaultState",
           static_cast<void (LeafSystem<PyLeafSystem_float_0>::*)(
               Context<PyLeafSystem_float_0> const &,
               State<PyLeafSystem_float_0> *) const>(
               &LeafSystem<PyLeafSystem_float_0>::SetDefaultState),
           py::arg("context"), py::arg("state"))

      ;
}
