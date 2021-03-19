#include "drake/systems/framework/leaf_system.h"
#include <pybind11/eigen.h>
#include <pybind11/iostream.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

using namespace drake::systems;

class LeafSystem_double_publicist : public LeafSystem<double> {
public:
  using LeafSystem<
      double>::AddTriggeredWitnessFunctionToCompositeEventCollection;
  using LeafSystem<double>::AllocateAbstractState;
  using LeafSystem<double>::AllocateContinuousState;
  using LeafSystem<double>::AllocateDiscreteState;
  using LeafSystem<double>::AllocateParameters;
  using LeafSystem<double>::DeclareAbstractInputPort;
  using LeafSystem<double>::DeclareAbstractInputPort;
  using LeafSystem<double>::DeclareAbstractOutputPort;
  using LeafSystem<double>::DeclareAbstractOutputPort;
  using LeafSystem<double>::DeclareAbstractParameter;
  using LeafSystem<double>::DeclareAbstractState;
  using LeafSystem<double>::DeclareAbstractState;
  using LeafSystem<double>::DeclareContinuousState;
  using LeafSystem<double>::DeclareContinuousState;
  using LeafSystem<double>::DeclareContinuousState;
  using LeafSystem<double>::DeclareContinuousState;
  using LeafSystem<double>::DeclareDiscreteState;
  using LeafSystem<double>::DeclareDiscreteState;
  using LeafSystem<double>::DeclareDiscreteState;
  using LeafSystem<double>::DeclareEqualityConstraint;
  using LeafSystem<double>::DeclareImplicitTimeDerivativesResidualSize;
  using LeafSystem<double>::DeclareInequalityConstraint;
  using LeafSystem<double>::DeclareNumericParameter;
  using LeafSystem<double>::DeclarePeriodicDiscreteUpdate;
  using LeafSystem<double>::DeclarePeriodicPublish;
  using LeafSystem<double>::DeclarePeriodicUnrestrictedUpdate;
  using LeafSystem<double>::DeclareVectorInputPort;
  using LeafSystem<double>::DeclareVectorInputPort;
  using LeafSystem<double>::DeclareVectorOutputPort;
  using LeafSystem<double>::DeclareVectorOutputPort;
  using LeafSystem<double>::DoCalcDiscreteVariableUpdates;
  using LeafSystem<double>::DoCalcNextUpdateTime;
  using LeafSystem<double>::DoCalcUnrestrictedUpdate;
  using LeafSystem<double>::DoCalcWitnessValue;
  using LeafSystem<double>::DoMakeLeafContext;
  using LeafSystem<double>::DoPublish;
  using LeafSystem<double>::DoValidateAllocatedLeafContext;
  using LeafSystem<double>::GetGraphvizFragment;
  using LeafSystem<double>::GetGraphvizInputPortToken;
  using LeafSystem<double>::GetGraphvizOutputPortToken;
  using LeafSystem<double>::MakeWitnessFunction;
  using LeafSystem<double>::MakeWitnessFunction;
};

class LeafSystem_Eigen_AutoDiffScalar_Eigen_VectorXd_publicist
    : public LeafSystem<::drake::AutoDiffXd> {
public:
  using LeafSystem<::drake::AutoDiffXd>::
      AddTriggeredWitnessFunctionToCompositeEventCollection;
  using LeafSystem<::drake::AutoDiffXd>::AllocateAbstractState;
  using LeafSystem<::drake::AutoDiffXd>::AllocateContinuousState;
  using LeafSystem<::drake::AutoDiffXd>::AllocateDiscreteState;
  using LeafSystem<::drake::AutoDiffXd>::AllocateParameters;
  using LeafSystem<::drake::AutoDiffXd>::DeclareAbstractInputPort;
  using LeafSystem<::drake::AutoDiffXd>::DeclareAbstractInputPort;
  using LeafSystem<::drake::AutoDiffXd>::DeclareAbstractOutputPort;
  using LeafSystem<::drake::AutoDiffXd>::DeclareAbstractOutputPort;
  using LeafSystem<::drake::AutoDiffXd>::DeclareAbstractParameter;
  using LeafSystem<::drake::AutoDiffXd>::DeclareAbstractState;
  using LeafSystem<::drake::AutoDiffXd>::DeclareAbstractState;
  using LeafSystem<::drake::AutoDiffXd>::DeclareContinuousState;
  using LeafSystem<::drake::AutoDiffXd>::DeclareContinuousState;
  using LeafSystem<::drake::AutoDiffXd>::DeclareContinuousState;
  using LeafSystem<::drake::AutoDiffXd>::DeclareContinuousState;
  using LeafSystem<::drake::AutoDiffXd>::DeclareDiscreteState;
  using LeafSystem<::drake::AutoDiffXd>::DeclareDiscreteState;
  using LeafSystem<::drake::AutoDiffXd>::DeclareDiscreteState;
  using LeafSystem<::drake::AutoDiffXd>::DeclareEqualityConstraint;
  using LeafSystem<
      ::drake::AutoDiffXd>::DeclareImplicitTimeDerivativesResidualSize;
  using LeafSystem<::drake::AutoDiffXd>::DeclareInequalityConstraint;
  using LeafSystem<::drake::AutoDiffXd>::DeclareNumericParameter;
  using LeafSystem<::drake::AutoDiffXd>::DeclarePeriodicDiscreteUpdate;
  using LeafSystem<::drake::AutoDiffXd>::DeclarePeriodicPublish;
  using LeafSystem<::drake::AutoDiffXd>::DeclarePeriodicUnrestrictedUpdate;
  using LeafSystem<::drake::AutoDiffXd>::DeclareVectorInputPort;
  using LeafSystem<::drake::AutoDiffXd>::DeclareVectorInputPort;
  using LeafSystem<::drake::AutoDiffXd>::DeclareVectorOutputPort;
  using LeafSystem<::drake::AutoDiffXd>::DeclareVectorOutputPort;
  using LeafSystem<::drake::AutoDiffXd>::DoCalcDiscreteVariableUpdates;
  using LeafSystem<::drake::AutoDiffXd>::DoCalcNextUpdateTime;
  using LeafSystem<::drake::AutoDiffXd>::DoCalcUnrestrictedUpdate;
  using LeafSystem<::drake::AutoDiffXd>::DoCalcWitnessValue;
  using LeafSystem<::drake::AutoDiffXd>::DoMakeLeafContext;
  using LeafSystem<::drake::AutoDiffXd>::DoPublish;
  using LeafSystem<::drake::AutoDiffXd>::DoValidateAllocatedLeafContext;
  using LeafSystem<::drake::AutoDiffXd>::GetGraphvizFragment;
  using LeafSystem<::drake::AutoDiffXd>::GetGraphvizInputPortToken;
  using LeafSystem<::drake::AutoDiffXd>::GetGraphvizOutputPortToken;
  using LeafSystem<::drake::AutoDiffXd>::MakeWitnessFunction;
  using LeafSystem<::drake::AutoDiffXd>::MakeWitnessFunction;
};

class LeafSystem_float_publicist : public LeafSystem<float> {
public:
  using LeafSystem<
      float>::AddTriggeredWitnessFunctionToCompositeEventCollection;
  using LeafSystem<float>::AllocateAbstractState;
  using LeafSystem<float>::AllocateContinuousState;
  using LeafSystem<float>::AllocateDiscreteState;
  using LeafSystem<float>::AllocateParameters;
  using LeafSystem<float>::DeclareAbstractOutputPort;
  using LeafSystem<float>::DeclareAbstractOutputPort;
  using LeafSystem<float>::DeclareAbstractParameter;
  using LeafSystem<float>::DeclareAbstractState;
  using LeafSystem<float>::DeclareAbstractState;
  using LeafSystem<float>::DeclareContinuousState;
  using LeafSystem<float>::DeclareContinuousState;
  using LeafSystem<float>::DeclareContinuousState;
  using LeafSystem<float>::DeclareContinuousState;
  using LeafSystem<float>::DeclareDiscreteState;
  using LeafSystem<float>::DeclareDiscreteState;
  using LeafSystem<float>::DeclareDiscreteState;
  using LeafSystem<float>::DeclareEqualityConstraint;
  using LeafSystem<float>::DeclareImplicitTimeDerivativesResidualSize;
  using LeafSystem<float>::DeclareInequalityConstraint;
  using LeafSystem<float>::DeclareNumericParameter;
  using LeafSystem<float>::DeclarePeriodicDiscreteUpdate;
  using LeafSystem<float>::DeclarePeriodicPublish;
  using LeafSystem<float>::DeclarePeriodicUnrestrictedUpdate;
  using LeafSystem<float>::DeclareVectorOutputPort;
  using LeafSystem<float>::DeclareVectorOutputPort;
  using LeafSystem<float>::DoCalcDiscreteVariableUpdates;
  using LeafSystem<float>::DoCalcNextUpdateTime;
  using LeafSystem<float>::DoCalcUnrestrictedUpdate;
  using LeafSystem<float>::DoMakeLeafContext;
  using LeafSystem<float>::DoPublish;
  using LeafSystem<float>::DoValidateAllocatedLeafContext;
  using LeafSystem<float>::GetGraphvizFragment;
  using LeafSystem<float>::GetGraphvizOutputPortToken;
  using LeafSystem<float>::MakeWitnessFunction;
  using LeafSystem<float>::MakeWitnessFunction;
};

namespace py = pybind11;
void apb11_pydrake_LeafSystem_py_register(py::module &m) {
  static bool called = false;
  if (called) {
    return;
  }
  called = true;
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

  py::class_<LeafSystem<::drake::AutoDiffXd>, System<::drake::AutoDiffXd>>
      PyLeafSystem_Eigen_AutoDiffScalar_Eigen_VectorXd(
          m, "LeafSystem_Eigen_AutoDiffScalar_Eigen_VectorXd");

  PyLeafSystem_Eigen_AutoDiffScalar_Eigen_VectorXd
      .def("AddTriggeredWitnessFunctionToCompositeEventCollection",
           [](LeafSystem<::drake::AutoDiffXd> &self,
              Event<::drake::AutoDiffXd> *event,
              CompositeEventCollection<::drake::AutoDiffXd> *events) {
             return self.AddTriggeredWitnessFunctionToCompositeEventCollection(
                 event, events);
           })
      .def("AllocateAbstractState",
           static_cast<::std::unique_ptr<AbstractValues,
                                         std::default_delete<AbstractValues>> (
               LeafSystem<::drake::AutoDiffXd>::*)() const>(
               &LeafSystem_Eigen_AutoDiffScalar_Eigen_VectorXd_publicist::
                   AllocateAbstractState))
      .def("AllocateCompositeEventCollection",
           static_cast<
               ::std::unique_ptr<CompositeEventCollection<::drake::AutoDiffXd>,
                                 std::default_delete<CompositeEventCollection<
                                     ::drake::AutoDiffXd>>> (
                   LeafSystem<::drake::AutoDiffXd>::*)() const>(
               &LeafSystem<
                   ::drake::AutoDiffXd>::AllocateCompositeEventCollection))
      .def("AllocateContext",
           static_cast<::std::unique_ptr<
               LeafContext<::drake::AutoDiffXd>,
               std::default_delete<LeafContext<::drake::AutoDiffXd>>> (
               LeafSystem<::drake::AutoDiffXd>::*)() const>(
               &LeafSystem<::drake::AutoDiffXd>::AllocateContext))
      .def("AllocateContinuousState",
           static_cast<::std::unique_ptr<
               ContinuousState<::drake::AutoDiffXd>,
               std::default_delete<ContinuousState<::drake::AutoDiffXd>>> (
               LeafSystem<::drake::AutoDiffXd>::*)() const>(
               &LeafSystem_Eigen_AutoDiffScalar_Eigen_VectorXd_publicist::
                   AllocateContinuousState))
      .def("AllocateDiscreteState",
           static_cast<::std::unique_ptr<
               DiscreteValues<::drake::AutoDiffXd>,
               std::default_delete<DiscreteValues<::drake::AutoDiffXd>>> (
               LeafSystem<::drake::AutoDiffXd>::*)() const>(
               &LeafSystem_Eigen_AutoDiffScalar_Eigen_VectorXd_publicist::
                   AllocateDiscreteState))
      .def("AllocateDiscreteVariables",
           static_cast<::std::unique_ptr<
               DiscreteValues<::drake::AutoDiffXd>,
               std::default_delete<DiscreteValues<::drake::AutoDiffXd>>> (
               LeafSystem<::drake::AutoDiffXd>::*)() const>(
               &LeafSystem<::drake::AutoDiffXd>::AllocateDiscreteVariables))
      .def("AllocateForcedDiscreteUpdateEventCollection",
           static_cast<::std::unique_ptr<
               EventCollection<DiscreteUpdateEvent<::drake::AutoDiffXd>>,
               std::default_delete<
                   EventCollection<DiscreteUpdateEvent<::drake::AutoDiffXd>>>> (
               LeafSystem<::drake::AutoDiffXd>::*)() const>(
               &LeafSystem<::drake::AutoDiffXd>::
                   AllocateForcedDiscreteUpdateEventCollection))
      .def("AllocateForcedPublishEventCollection",
           static_cast<::std::unique_ptr<
               EventCollection<PublishEvent<::drake::AutoDiffXd>>,
               std::default_delete<
                   EventCollection<PublishEvent<::drake::AutoDiffXd>>>> (
               LeafSystem<::drake::AutoDiffXd>::*)() const>(
               &LeafSystem<
                   ::drake::AutoDiffXd>::AllocateForcedPublishEventCollection))
      .def("AllocateForcedUnrestrictedUpdateEventCollection",
           static_cast<::std::unique_ptr<
               EventCollection<UnrestrictedUpdateEvent<::drake::AutoDiffXd>>,
               std::default_delete<EventCollection<
                   UnrestrictedUpdateEvent<::drake::AutoDiffXd>>>> (
               LeafSystem<::drake::AutoDiffXd>::*)() const>(
               &LeafSystem<::drake::AutoDiffXd>::
                   AllocateForcedUnrestrictedUpdateEventCollection))
      .def("AllocateParameters",
           static_cast<::std::unique_ptr<
               Parameters<::drake::AutoDiffXd>,
               std::default_delete<Parameters<::drake::AutoDiffXd>>> (
               LeafSystem<::drake::AutoDiffXd>::*)() const>(
               &LeafSystem_Eigen_AutoDiffScalar_Eigen_VectorXd_publicist::
                   AllocateParameters))
      .def("AllocateTimeDerivatives",
           static_cast<::std::unique_ptr<
               ContinuousState<::drake::AutoDiffXd>,
               std::default_delete<ContinuousState<::drake::AutoDiffXd>>> (
               LeafSystem<::drake::AutoDiffXd>::*)() const>(
               &LeafSystem<::drake::AutoDiffXd>::AllocateTimeDerivatives))
      .def("DeclareAbstractInputPort",
           static_cast<InputPort<::drake::AutoDiffXd> &(
               LeafSystem<::drake::AutoDiffXd>::
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
           static_cast<InputPort<::drake::AutoDiffXd> &(
               LeafSystem<::drake::AutoDiffXd>::*)(::drake::AbstractValue const
                                                       &)>(
               &LeafSystem_Eigen_AutoDiffScalar_Eigen_VectorXd_publicist::
                   DeclareAbstractInputPort),
           py::arg("model_value"), py::return_value_policy::reference_internal)
      .def("DeclareAbstractOutputPort",
           [](LeafSystem<::drake::AutoDiffXd> &self,
              ::std::variant<std::basic_string<char, std::char_traits<char>,
                                               std::allocator<char>>,
                             UseDefaultName>
                  name,
              LeafOutputPort<::drake::AutoDiffXd>::AllocCallback alloc_function,
              LeafOutputPort<::drake::AutoDiffXd>::CalcCallback calc_function,
              ::std::set<drake::TypeSafeIndex<DependencyTag>,
                         std::less<drake::TypeSafeIndex<DependencyTag>>,
                         std::allocator<drake::TypeSafeIndex<DependencyTag>>>
                  prerequisites_of_calc) {
             return self.DeclareAbstractOutputPort(
                 name, alloc_function, calc_function, prerequisites_of_calc);
           })
      .def("DeclareAbstractOutputPort",
           [](LeafSystem<::drake::AutoDiffXd> &self,
              LeafOutputPort<::drake::AutoDiffXd>::AllocCallback alloc_function,
              LeafOutputPort<::drake::AutoDiffXd>::CalcCallback calc_function,
              ::std::set<drake::TypeSafeIndex<DependencyTag>,
                         std::less<drake::TypeSafeIndex<DependencyTag>>,
                         std::allocator<drake::TypeSafeIndex<DependencyTag>>>
                  prerequisites_of_calc) {
             return self.DeclareAbstractOutputPort(
                 alloc_function, calc_function, prerequisites_of_calc);
           })
      .def("DeclareAbstractParameter",
           static_cast<int (LeafSystem<::drake::AutoDiffXd>::*)(
               ::drake::AbstractValue const &)>(
               &LeafSystem_Eigen_AutoDiffScalar_Eigen_VectorXd_publicist::
                   DeclareAbstractParameter),
           py::arg("model_value"))
      .def("DeclareAbstractState",
           static_cast<AbstractStateIndex (LeafSystem<::drake::AutoDiffXd>::*)(
               ::drake::AbstractValue const &)>(
               &LeafSystem_Eigen_AutoDiffScalar_Eigen_VectorXd_publicist::
                   DeclareAbstractState),
           py::arg("abstract_state"))
      .def("DeclareAbstractState",
           [](LeafSystem<::drake::AutoDiffXd> &self,
              drake::AbstractValue abstract_state) {
             return self.DeclareAbstractState(
                 std::make_unique<drake::AbstractValue>(abstract_state));
           })
      .def("DeclareContinuousState",
           static_cast<void (LeafSystem<::drake::AutoDiffXd>::*)(int)>(
               &LeafSystem_Eigen_AutoDiffScalar_Eigen_VectorXd_publicist::
                   DeclareContinuousState),
           py::arg("num_state_variables"))
      .def(
          "DeclareContinuousState",
          static_cast<void (LeafSystem<::drake::AutoDiffXd>::*)(int, int, int)>(
              &LeafSystem_Eigen_AutoDiffScalar_Eigen_VectorXd_publicist::
                  DeclareContinuousState),
          py::arg("num_q"), py::arg("num_v"), py::arg("num_z"))
      .def("DeclareContinuousState",
           [](LeafSystem<::drake::AutoDiffXd> &self,
              BasicVector<::drake::AutoDiffXd> const &model_vector) {
             return self.DeclareContinuousState(model_vector);
           })
      .def("DeclareContinuousState",
           [](LeafSystem<::drake::AutoDiffXd> &self,
              BasicVector<::drake::AutoDiffXd> const &model_vector, int num_q,
              int num_v, int num_z) {
             return self.DeclareContinuousState(model_vector, num_q, num_v,
                                                num_z);
           })
      .def("DeclareDiscreteState",
           [](LeafSystem<::drake::AutoDiffXd> &self,
              BasicVector<::drake::AutoDiffXd> const &model_vector) {
             return self.DeclareDiscreteState(model_vector);
           })
      .def("DeclareDiscreteState",
           [](LeafSystem<::drake::AutoDiffXd> &self,
              ::Eigen::Ref<
                  const Eigen::Matrix<Eigen::AutoDiffScalar<Eigen::VectorXd>,
                                      -1, 1, 0, -1, 1>,
                  0, Eigen::InnerStride<1>> const &vector) {
             return self.DeclareDiscreteState(vector);
           })
      .def("DeclareDiscreteState",
           static_cast<DiscreteStateIndex (LeafSystem<::drake::AutoDiffXd>::*)(
               int)>(&LeafSystem_Eigen_AutoDiffScalar_Eigen_VectorXd_publicist::
                         DeclareDiscreteState),
           py::arg("num_state_variables"))
      .def("DeclareEqualityConstraint",
           [](LeafSystem<::drake::AutoDiffXd> &self,
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
           static_cast<void (LeafSystem<::drake::AutoDiffXd>::*)(int)>(
               &LeafSystem_Eigen_AutoDiffScalar_Eigen_VectorXd_publicist::
                   DeclareImplicitTimeDerivativesResidualSize),
           py::arg("n"))
      .def("DeclareInequalityConstraint",
           [](LeafSystem<::drake::AutoDiffXd> &self,
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
           [](LeafSystem<::drake::AutoDiffXd> &self,
              BasicVector<::drake::AutoDiffXd> const &model_vector) {
             return self.DeclareNumericParameter(model_vector);
           })
      .def("DeclarePeriodicDiscreteUpdate",
           static_cast<void (LeafSystem<::drake::AutoDiffXd>::*)(double,
                                                                 double)>(
               &LeafSystem_Eigen_AutoDiffScalar_Eigen_VectorXd_publicist::
                   DeclarePeriodicDiscreteUpdate),
           py::arg("period_sec"), py::arg("offset_sec") = double(0))
      .def("DeclarePeriodicPublish",
           static_cast<void (LeafSystem<::drake::AutoDiffXd>::*)(double,
                                                                 double)>(
               &LeafSystem_Eigen_AutoDiffScalar_Eigen_VectorXd_publicist::
                   DeclarePeriodicPublish),
           py::arg("period_sec"), py::arg("offset_sec") = double(0))
      .def("DeclarePeriodicUnrestrictedUpdate",
           static_cast<void (LeafSystem<::drake::AutoDiffXd>::*)(double,
                                                                 double)>(
               &LeafSystem_Eigen_AutoDiffScalar_Eigen_VectorXd_publicist::
                   DeclarePeriodicUnrestrictedUpdate),
           py::arg("period_sec"), py::arg("offset_sec") = double(0))
      .def(
          "DeclareVectorInputPort",
          [](LeafSystem<::drake::AutoDiffXd> &self,
             ::std::variant<std::basic_string<char, std::char_traits<char>,
                                              std::allocator<char>>,
                            UseDefaultName>
                 name,
             BasicVector<::drake::AutoDiffXd> const &model_vector,
             ::std::optional<drake::RandomDistribution> random_type) {
            return self.DeclareVectorInputPort(name, model_vector, random_type);
          },
          py::return_value_policy::reference_internal)
      .def(
          "DeclareVectorInputPort",
          [](LeafSystem<::drake::AutoDiffXd> &self,
             BasicVector<::drake::AutoDiffXd> const &model_vector,
             ::std::optional<drake::RandomDistribution> random_type) {
            return self.DeclareVectorInputPort(model_vector, random_type);
          },
          py::return_value_policy::reference_internal)
      .def(
          "DeclareVectorOutputPort",
          [](LeafSystem<::drake::AutoDiffXd> &self,
             ::std::variant<std::basic_string<char, std::char_traits<char>,
                                              std::allocator<char>>,
                            UseDefaultName>
                 name,
             BasicVector<::drake::AutoDiffXd> const &model_vector,
             LeafOutputPort<::drake::AutoDiffXd>::CalcVectorCallback
                 vector_calc_function,
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
          [](LeafSystem<::drake::AutoDiffXd> &self,
             BasicVector<::drake::AutoDiffXd> const &model_vector,
             LeafOutputPort<::drake::AutoDiffXd>::CalcVectorCallback
                 vector_calc_function,
             ::std::set<drake::TypeSafeIndex<DependencyTag>,
                        std::less<drake::TypeSafeIndex<DependencyTag>>,
                        std::allocator<drake::TypeSafeIndex<DependencyTag>>>
                 prerequisites_of_calc) {
            return self.DeclareVectorOutputPort(
                model_vector, vector_calc_function, prerequisites_of_calc);
          },
          py::return_value_policy::reference_internal)
      .def(
          "DoAllocateContext",
          static_cast<
              ::std::unique_ptr<ContextBase, std::default_delete<ContextBase>> (
                  LeafSystem<::drake::AutoDiffXd>::*)() const>(
              &LeafSystem<::drake::AutoDiffXd>::DoAllocateContext))
      .def("DoCalcDiscreteVariableUpdates",
           [](LeafSystem<::drake::AutoDiffXd> &self,
              Context<::drake::AutoDiffXd> const &context,
              ::std::vector<const DiscreteUpdateEvent<::drake::AutoDiffXd> *,
                            std::allocator<const DiscreteUpdateEvent<
                                ::drake::AutoDiffXd> *>> const &events,
              DiscreteValues<::drake::AutoDiffXd> *discrete_state) {
             return self.DoCalcDiscreteVariableUpdates(context, events,
                                                       discrete_state);
           })
      .def("DoCalcNextUpdateTime",
           [](LeafSystem<::drake::AutoDiffXd> &self,
              Context<::drake::AutoDiffXd> const &context,
              CompositeEventCollection<::drake::AutoDiffXd> *events,
              Eigen::Ref<::Eigen::AutoDiffScalar<Eigen::VectorXd> *, 0,
                         Eigen::Stride<Eigen::Dynamic, Eigen::Dynamic>>
                  time) {
             return self.DoCalcNextUpdateTime(context, events, time);
           })
      .def(
          "DoCalcUnrestrictedUpdate",
          [](LeafSystem<::drake::AutoDiffXd> &self,
             Context<::drake::AutoDiffXd> const &context,
             ::std::vector<const UnrestrictedUpdateEvent<::drake::AutoDiffXd> *,
                           std::allocator<const UnrestrictedUpdateEvent<
                               ::drake::AutoDiffXd> *>> const &events,
             State<::drake::AutoDiffXd> *state) {
            return self.DoCalcUnrestrictedUpdate(context, events, state);
          })
      .def("DoCalcWitnessValue",
           [](LeafSystem<::drake::AutoDiffXd> &self,
              Context<::drake::AutoDiffXd> const &context,
              WitnessFunction<::drake::AutoDiffXd> const &witness_func) {
             return self.DoCalcWitnessValue(context, witness_func);
           })
      .def("DoMakeLeafContext",
           static_cast<::std::unique_ptr<
               LeafContext<::drake::AutoDiffXd>,
               std::default_delete<LeafContext<::drake::AutoDiffXd>>> (
               LeafSystem<::drake::AutoDiffXd>::*)() const>(
               &LeafSystem_Eigen_AutoDiffScalar_Eigen_VectorXd_publicist::
                   DoMakeLeafContext))
      .def("DoPublish",
           [](LeafSystem<::drake::AutoDiffXd> &self,
              Context<::drake::AutoDiffXd> const &context,
              ::std::vector<const PublishEvent<::drake::AutoDiffXd> *,
                            std::allocator<const PublishEvent<
                                ::drake::AutoDiffXd> *>> const &events) {
             return self.DoPublish(context, events);
           })
      .def("DoValidateAllocatedLeafContext",
           [](LeafSystem<::drake::AutoDiffXd> &self,
              LeafContext<::drake::AutoDiffXd> const &context) {
             return self.DoValidateAllocatedLeafContext(context);
           })
      .def("GetDirectFeedthroughs",
           static_cast<
               ::std::multimap<int, int, std::less<int>,
                               std::allocator<std::pair<const int, int>>> (
                   LeafSystem<::drake::AutoDiffXd>::*)() const>(
               &LeafSystem<::drake::AutoDiffXd>::GetDirectFeedthroughs))
      .def("GetGraphvizFragment",
           static_cast<void (LeafSystem<::drake::AutoDiffXd>::*)(
               int, ::std::stringstream *) const>(
               &LeafSystem_Eigen_AutoDiffScalar_Eigen_VectorXd_publicist::
                   GetGraphvizFragment),
           py::arg("max_depth"), py::arg("dot"))
      .def("GetGraphvizInputPortToken",
           [](LeafSystem<::drake::AutoDiffXd> &self,
              InputPort<::drake::AutoDiffXd> const &port, int max_depth,
              ::std::stringstream *dot) {
             return self.GetGraphvizInputPortToken(port, max_depth, dot);
           })
      .def("GetGraphvizOutputPortToken",
           [](LeafSystem<::drake::AutoDiffXd> &self,
              OutputPort<::drake::AutoDiffXd> const &port, int max_depth,
              ::std::stringstream *dot) {
             return self.GetGraphvizOutputPortToken(port, max_depth, dot);
           })
      .def("MakeWitnessFunction",
           [](LeafSystem<::drake::AutoDiffXd> &self,
              ::std::string const &description,
              WitnessFunctionDirection const &direction_type,
              ::std::function<Eigen::AutoDiffScalar<Eigen::VectorXd>(
                  const Context<::drake::AutoDiffXd> &)>
                  calc) {
             return self.MakeWitnessFunction(description, direction_type, calc);
           })
      .def("MakeWitnessFunction",
           [](LeafSystem<::drake::AutoDiffXd> &self,
              ::std::string const &description,
              WitnessFunctionDirection const &direction_type,
              ::std::function<Eigen::AutoDiffScalar<Eigen::VectorXd>(
                  const Context<::drake::AutoDiffXd> &)>
                  calc,
              Event<::drake::AutoDiffXd> const &e) {
             return self.MakeWitnessFunction(description, direction_type, calc,
                                             e);
           })
      .def("SetDefaultParameters",
           [](LeafSystem<::drake::AutoDiffXd> &self,
              Context<::drake::AutoDiffXd> const &context,
              Parameters<::drake::AutoDiffXd> *parameters) {
             return self.SetDefaultParameters(context, parameters);
           })
      .def("SetDefaultState",
           [](LeafSystem<::drake::AutoDiffXd> &self,
              Context<::drake::AutoDiffXd> const &context,
              State<::drake::AutoDiffXd> *state) {
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
