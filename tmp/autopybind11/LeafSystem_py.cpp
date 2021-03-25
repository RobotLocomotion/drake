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
    : public LeafSystem<drake::AutoDiffXd> {
public:
  using LeafSystem<
      drake::AutoDiffXd>::AddTriggeredWitnessFunctionToCompositeEventCollection;
  using LeafSystem<drake::AutoDiffXd>::AllocateAbstractState;
  using LeafSystem<drake::AutoDiffXd>::AllocateContinuousState;
  using LeafSystem<drake::AutoDiffXd>::AllocateDiscreteState;
  using LeafSystem<drake::AutoDiffXd>::AllocateParameters;
  using LeafSystem<drake::AutoDiffXd>::DeclareAbstractInputPort;
  using LeafSystem<drake::AutoDiffXd>::DeclareAbstractInputPort;
  using LeafSystem<drake::AutoDiffXd>::DeclareAbstractOutputPort;
  using LeafSystem<drake::AutoDiffXd>::DeclareAbstractOutputPort;
  using LeafSystem<drake::AutoDiffXd>::DeclareAbstractParameter;
  using LeafSystem<drake::AutoDiffXd>::DeclareAbstractState;
  using LeafSystem<drake::AutoDiffXd>::DeclareAbstractState;
  using LeafSystem<drake::AutoDiffXd>::DeclareContinuousState;
  using LeafSystem<drake::AutoDiffXd>::DeclareContinuousState;
  using LeafSystem<drake::AutoDiffXd>::DeclareContinuousState;
  using LeafSystem<drake::AutoDiffXd>::DeclareContinuousState;
  using LeafSystem<drake::AutoDiffXd>::DeclareDiscreteState;
  using LeafSystem<drake::AutoDiffXd>::DeclareDiscreteState;
  using LeafSystem<drake::AutoDiffXd>::DeclareDiscreteState;
  using LeafSystem<drake::AutoDiffXd>::DeclareEqualityConstraint;
  using LeafSystem<
      drake::AutoDiffXd>::DeclareImplicitTimeDerivativesResidualSize;
  using LeafSystem<drake::AutoDiffXd>::DeclareInequalityConstraint;
  using LeafSystem<drake::AutoDiffXd>::DeclareNumericParameter;
  using LeafSystem<drake::AutoDiffXd>::DeclarePeriodicDiscreteUpdate;
  using LeafSystem<drake::AutoDiffXd>::DeclarePeriodicPublish;
  using LeafSystem<drake::AutoDiffXd>::DeclarePeriodicUnrestrictedUpdate;
  using LeafSystem<drake::AutoDiffXd>::DeclareVectorInputPort;
  using LeafSystem<drake::AutoDiffXd>::DeclareVectorInputPort;
  using LeafSystem<drake::AutoDiffXd>::DeclareVectorOutputPort;
  using LeafSystem<drake::AutoDiffXd>::DeclareVectorOutputPort;
  using LeafSystem<drake::AutoDiffXd>::DoCalcDiscreteVariableUpdates;
  using LeafSystem<drake::AutoDiffXd>::DoCalcNextUpdateTime;
  using LeafSystem<drake::AutoDiffXd>::DoCalcUnrestrictedUpdate;
  using LeafSystem<drake::AutoDiffXd>::DoCalcWitnessValue;
  using LeafSystem<drake::AutoDiffXd>::DoMakeLeafContext;
  using LeafSystem<drake::AutoDiffXd>::DoPublish;
  using LeafSystem<drake::AutoDiffXd>::DoValidateAllocatedLeafContext;
  using LeafSystem<drake::AutoDiffXd>::GetGraphvizFragment;
  using LeafSystem<drake::AutoDiffXd>::GetGraphvizInputPortToken;
  using LeafSystem<drake::AutoDiffXd>::GetGraphvizOutputPortToken;
  using LeafSystem<drake::AutoDiffXd>::MakeWitnessFunction;
  using LeafSystem<drake::AutoDiffXd>::MakeWitnessFunction;
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

template <typename Class, typename... Options>
py::class_<Class, Options...> DefineTemplateClass(py::handle scope,
                                                  const char *name,
                                                  const char *doc_string = "") {
  py::class_<Class, Options...> py_class(scope, name, doc_string);
  return py_class;
};

void apb11_pydrake_LeafSystem_py_register(py::module &m) {
  // Instantiation of LeafSystem<double>
  auto PyLeafSystem_double =
      DefineTemplateClass<LeafSystem<double>, System<double>>(
          m, "LeafSystem_double");

  PyLeafSystem_double
      .def("AddTriggeredWitnessFunctionToCompositeEventCollection",
           static_cast<void (LeafSystem<double>::*)(
               Event<double> *, CompositeEventCollection<double> *) const>(
               &LeafSystem_double_publicist::
                   AddTriggeredWitnessFunctionToCompositeEventCollection),
           py::arg("event"), py::arg("events"))
      .def("AllocateAbstractState",
           static_cast<::std::unique_ptr<AbstractValues,
                                         std::default_delete<AbstractValues>> (
               LeafSystem<double>::*)() const>(
               &LeafSystem_double_publicist::AllocateAbstractState))
      .def("AllocateCompositeEventCollection",
           static_cast<::std::unique_ptr<
               CompositeEventCollection<double>,
               std::default_delete<CompositeEventCollection<double>>> (
               LeafSystem<double>::*)() const>(
               &LeafSystem<double>::AllocateCompositeEventCollection))
      .def("AllocateContext",
           static_cast<::std::unique_ptr<
               LeafContext<double>, std::default_delete<LeafContext<double>>> (
               LeafSystem<double>::*)() const>(
               &LeafSystem<double>::AllocateContext))
      .def("AllocateContinuousState",
           static_cast<
               ::std::unique_ptr<ContinuousState<double>,
                                 std::default_delete<ContinuousState<double>>> (
                   LeafSystem<double>::*)() const>(
               &LeafSystem_double_publicist::AllocateContinuousState))
      .def("AllocateDiscreteState",
           static_cast<
               ::std::unique_ptr<DiscreteValues<double>,
                                 std::default_delete<DiscreteValues<double>>> (
                   LeafSystem<double>::*)() const>(
               &LeafSystem_double_publicist::AllocateDiscreteState))
      .def("AllocateDiscreteVariables",
           static_cast<
               ::std::unique_ptr<DiscreteValues<double>,
                                 std::default_delete<DiscreteValues<double>>> (
                   LeafSystem<double>::*)() const>(
               &LeafSystem<double>::AllocateDiscreteVariables))
      .def(
          "AllocateForcedDiscreteUpdateEventCollection",
          static_cast<
              ::std::unique_ptr<EventCollection<DiscreteUpdateEvent<double>>,
                                std::default_delete<EventCollection<
                                    DiscreteUpdateEvent<double>>>> (
                  LeafSystem<double>::*)() const>(
              &LeafSystem<double>::AllocateForcedDiscreteUpdateEventCollection))
      .def("AllocateForcedPublishEventCollection",
           static_cast<::std::unique_ptr<
               EventCollection<PublishEvent<double>>,
               std::default_delete<EventCollection<PublishEvent<double>>>> (
               LeafSystem<double>::*)() const>(
               &LeafSystem<double>::AllocateForcedPublishEventCollection))
      .def("AllocateForcedUnrestrictedUpdateEventCollection",
           static_cast<::std::unique_ptr<
               EventCollection<UnrestrictedUpdateEvent<double>>,
               std::default_delete<
                   EventCollection<UnrestrictedUpdateEvent<double>>>> (
               LeafSystem<double>::*)() const>(
               &LeafSystem<
                   double>::AllocateForcedUnrestrictedUpdateEventCollection))
      .def("AllocateParameters",
           static_cast<::std::unique_ptr<
               Parameters<double>, std::default_delete<Parameters<double>>> (
               LeafSystem<double>::*)() const>(
               &LeafSystem_double_publicist::AllocateParameters))
      .def("AllocateTimeDerivatives",
           static_cast<
               ::std::unique_ptr<ContinuousState<double>,
                                 std::default_delete<ContinuousState<double>>> (
                   LeafSystem<double>::*)() const>(
               &LeafSystem<double>::AllocateTimeDerivatives))
      .def("DeclareAbstractInputPort",
           static_cast<InputPort<double> &(
               LeafSystem<double>::
                   *)(::std::variant<
                          std::basic_string<char, std::char_traits<char>,
                                            std::allocator<char>>,
                          UseDefaultName>,
                      ::drake::AbstractValue const &)>(
               &LeafSystem_double_publicist::DeclareAbstractInputPort),
           py::arg("name"), py::arg("model_value"))
      .def("DeclareAbstractInputPort",
           static_cast<InputPort<double> &(
               LeafSystem<double>::*)(::drake::AbstractValue const &)>(
               &LeafSystem_double_publicist::DeclareAbstractInputPort),
           py::arg("model_value"))
      .def("DeclareAbstractOutputPort",
           [](LeafSystem<double> &self,
              ::std::variant<std::basic_string<char, std::char_traits<char>,
                                               std::allocator<char>>,
                             UseDefaultName>
                  name,
              LeafOutputPort<double>::AllocCallback alloc_function,
              LeafOutputPort<double>::CalcCallback calc_function,
              ::std::set<drake::TypeSafeIndex<DependencyTag>,
                         std::less<drake::TypeSafeIndex<DependencyTag>>,
                         std::allocator<drake::TypeSafeIndex<DependencyTag>>>
                  prerequisites_of_calc) {
             return self.DeclareAbstractOutputPort(
                 name, alloc_function, calc_function, prerequisites_of_calc);
           })
      .def("DeclareAbstractOutputPort",
           [](LeafSystem<double> &self,
              LeafOutputPort<double>::AllocCallback alloc_function,
              LeafOutputPort<double>::CalcCallback calc_function,
              ::std::set<drake::TypeSafeIndex<DependencyTag>,
                         std::less<drake::TypeSafeIndex<DependencyTag>>,
                         std::allocator<drake::TypeSafeIndex<DependencyTag>>>
                  prerequisites_of_calc) {
             return self.DeclareAbstractOutputPort(
                 alloc_function, calc_function, prerequisites_of_calc);
           })
      .def("DeclareAbstractParameter",
           static_cast<int (LeafSystem<double>::*)(
               ::drake::AbstractValue const &)>(
               &LeafSystem_double_publicist::DeclareAbstractParameter),
           py::arg("model_value"))
      .def("DeclareAbstractState",
           static_cast<AbstractStateIndex (LeafSystem<double>::*)(
               ::drake::AbstractValue const &)>(
               &LeafSystem_double_publicist::DeclareAbstractState),
           py::arg("abstract_state"))
      .def("DeclareAbstractState",
           [](LeafSystem<double> &self, drake::AbstractValue abstract_state) {
             return self.DeclareAbstractState(
                 std::make_unique<drake::AbstractValue>(abstract_state));
           })
      .def("DeclareContinuousState",
           static_cast<void (LeafSystem<double>::*)(int)>(
               &LeafSystem_double_publicist::DeclareContinuousState),
           py::arg("num_state_variables"))
      .def("DeclareContinuousState",
           static_cast<void (LeafSystem<double>::*)(int, int, int)>(
               &LeafSystem_double_publicist::DeclareContinuousState),
           py::arg("num_q"), py::arg("num_v"), py::arg("num_z"))
      .def("DeclareContinuousState",
           static_cast<void (LeafSystem<double>::*)(
               BasicVector<double> const &)>(
               &LeafSystem_double_publicist::DeclareContinuousState),
           py::arg("model_vector"))
      .def("DeclareContinuousState",
           static_cast<void (LeafSystem<double>::*)(BasicVector<double> const &,
                                                    int, int, int)>(
               &LeafSystem_double_publicist::DeclareContinuousState),
           py::arg("model_vector"), py::arg("num_q"), py::arg("num_v"),
           py::arg("num_z"))
      .def("DeclareDiscreteState",
           static_cast<DiscreteStateIndex (LeafSystem<double>::*)(
               BasicVector<double> const &)>(
               &LeafSystem_double_publicist::DeclareDiscreteState),
           py::arg("model_vector"))
      .def("DeclareDiscreteState",
           static_cast<DiscreteStateIndex (LeafSystem<double>::*)(
               ::Eigen::Ref<const Eigen::Matrix<double, -1, 1, 0, -1, 1>, 0,
                            Eigen::InnerStride<1>> const &)>(
               &LeafSystem_double_publicist::DeclareDiscreteState),
           py::arg("vector"))
      .def("DeclareDiscreteState",
           static_cast<DiscreteStateIndex (LeafSystem<double>::*)(int)>(
               &LeafSystem_double_publicist::DeclareDiscreteState),
           py::arg("num_state_variables"))
      .def("DeclareEqualityConstraint",
           static_cast<SystemConstraintIndex (LeafSystem<double>::*)(
               ::std::function<void(const Context<double> &,
                                    Eigen::Matrix<double, -1, 1, 0, -1, 1> *)>,
               int, ::std::string)>(
               &LeafSystem_double_publicist::DeclareEqualityConstraint),
           py::arg("calc"), py::arg("count"), py::arg("description"))
      .def("DeclareImplicitTimeDerivativesResidualSize",
           static_cast<void (LeafSystem<double>::*)(int)>(
               &LeafSystem_double_publicist::
                   DeclareImplicitTimeDerivativesResidualSize),
           py::arg("n"))
      .def("DeclareInequalityConstraint",
           static_cast<SystemConstraintIndex (LeafSystem<double>::*)(
               ::std::function<void(const Context<double> &,
                                    Eigen::Matrix<double, -1, 1, 0, -1, 1> *)>,
               SystemConstraintBounds, ::std::string)>(
               &LeafSystem_double_publicist::DeclareInequalityConstraint),
           py::arg("calc"), py::arg("bounds"), py::arg("description"))
      .def(
          "DeclareNumericParameter",
          static_cast<int (LeafSystem<double>::*)(BasicVector<double> const &)>(
              &LeafSystem_double_publicist::DeclareNumericParameter),
          py::arg("model_vector"))
      .def("DeclarePeriodicDiscreteUpdate",
           static_cast<void (LeafSystem<double>::*)(double, double)>(
               &LeafSystem_double_publicist::DeclarePeriodicDiscreteUpdate),
           py::arg("period_sec"), py::arg("offset_sec") = double(0))
      .def("DeclarePeriodicPublish",
           static_cast<void (LeafSystem<double>::*)(double, double)>(
               &LeafSystem_double_publicist::DeclarePeriodicPublish),
           py::arg("period_sec"), py::arg("offset_sec") = double(0))
      .def("DeclarePeriodicUnrestrictedUpdate",
           static_cast<void (LeafSystem<double>::*)(double, double)>(
               &LeafSystem_double_publicist::DeclarePeriodicUnrestrictedUpdate),
           py::arg("period_sec"), py::arg("offset_sec") = double(0))
      .def("DeclareVectorInputPort",
           static_cast<InputPort<double> &(
               LeafSystem<double>::
                   *)(::std::variant<
                          std::basic_string<char, std::char_traits<char>,
                                            std::allocator<char>>,
                          UseDefaultName>,
                      BasicVector<double> const &,
                      ::std::optional<drake::RandomDistribution>)>(
               &LeafSystem_double_publicist::DeclareVectorInputPort),
           py::arg("name"), py::arg("model_vector"),
           py::arg("random_type") =
               ::std::optional<drake::RandomDistribution>(std::nullopt))
      .def("DeclareVectorInputPort",
           static_cast<InputPort<double> &(
               LeafSystem<double>::*)(BasicVector<double> const &,
                                      ::std::optional<
                                          drake::RandomDistribution>)>(
               &LeafSystem_double_publicist::DeclareVectorInputPort),
           py::arg("model_vector"),
           py::arg("random_type") =
               ::std::optional<drake::RandomDistribution>(std::nullopt))
      .def(
          "DeclareVectorOutputPort",
          static_cast<LeafOutputPort<double> &(
              LeafSystem<double>::
                  *)(::std::variant<
                         std::basic_string<char, std::char_traits<char>,
                                           std::allocator<char>>,
                         UseDefaultName>,
                     BasicVector<double> const &,
                     LeafOutputPort<double>::CalcVectorCallback,
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
          static_cast<LeafOutputPort<double> &(
              LeafSystem<double>::
                  *)(BasicVector<double> const &,
                     LeafOutputPort<double>::CalcVectorCallback,
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
                  LeafSystem<double>::*)() const>(
              &LeafSystem<double>::DoAllocateContext))
      .def("DoCalcDiscreteVariableUpdates",
           static_cast<void (LeafSystem<double>::*)(
               Context<double> const &,
               ::std::vector<
                   const DiscreteUpdateEvent<double> *,
                   std::allocator<const DiscreteUpdateEvent<double> *>> const &,
               DiscreteValues<double> *) const>(
               &LeafSystem_double_publicist::DoCalcDiscreteVariableUpdates),
           py::arg("context"), py::arg("events"), py::arg("discrete_state"))
      .def("DoCalcNextUpdateTime",
           static_cast<void (LeafSystem<double>::*)(
               Context<double> const &, CompositeEventCollection<double> *,
               double *) const>(
               &LeafSystem_double_publicist::DoCalcNextUpdateTime),
           py::arg("context"), py::arg("events"), py::arg("time"))
      .def(
          "DoCalcUnrestrictedUpdate",
          static_cast<void (LeafSystem<double>::*)(
              Context<double> const &,
              ::std::vector<const UnrestrictedUpdateEvent<double> *,
                            std::allocator<const UnrestrictedUpdateEvent<double>
                                               *>> const &,
              State<double> *) const>(
              &LeafSystem_double_publicist::DoCalcUnrestrictedUpdate),
          py::arg("context"), py::arg("events"), py::arg("state"))
      .def("DoCalcWitnessValue",
           static_cast<double (LeafSystem<double>::*)(
               Context<double> const &, WitnessFunction<double> const &) const>(
               &LeafSystem_double_publicist::DoCalcWitnessValue),
           py::arg("context"), py::arg("witness_func"))
      .def("DoMakeLeafContext",
           static_cast<::std::unique_ptr<
               LeafContext<double>, std::default_delete<LeafContext<double>>> (
               LeafSystem<double>::*)() const>(
               &LeafSystem_double_publicist::DoMakeLeafContext))
      .def("DoPublish",
           static_cast<void (LeafSystem<double>::*)(
               Context<double> const &,
               ::std::vector<const PublishEvent<double> *,
                             std::allocator<const PublishEvent<double> *>> const
                   &) const>(&LeafSystem_double_publicist::DoPublish),
           py::arg("context"), py::arg("events"))
      .def("DoValidateAllocatedLeafContext",
           static_cast<void (LeafSystem<double>::*)(LeafContext<double> const &)
                           const>(
               &LeafSystem_double_publicist::DoValidateAllocatedLeafContext),
           py::arg("context"))
      .def("GetDirectFeedthroughs",
           static_cast<
               ::std::multimap<int, int, std::less<int>,
                               std::allocator<std::pair<const int, int>>> (
                   LeafSystem<double>::*)() const>(
               &LeafSystem<double>::GetDirectFeedthroughs))
      .def("GetGraphvizFragment",
           static_cast<void (LeafSystem<double>::*)(int, ::std::stringstream *)
                           const>(
               &LeafSystem_double_publicist::GetGraphvizFragment),
           py::arg("max_depth"), py::arg("dot"))
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
      .def("MakeWitnessFunction",
           static_cast<
               ::std::unique_ptr<WitnessFunction<double>,
                                 std::default_delete<WitnessFunction<double>>> (
                   LeafSystem<double>::*)(
                   ::std::string const &,
                   WitnessFunctionDirection const &,
                   ::std::function<double(const Context<double> &)>) const>(
               &LeafSystem_double_publicist::MakeWitnessFunction),
           py::arg("description"), py::arg("direction_type"), py::arg("calc"))
      .def("MakeWitnessFunction",
           static_cast<
               ::std::unique_ptr<WitnessFunction<double>,
                                 std::default_delete<WitnessFunction<double>>> (
                   LeafSystem<double>::*)(
                   ::std::string const &,
                   WitnessFunctionDirection const &,
                   ::std::function<double(const Context<double> &)>,
                   Event<double> const &) const>(
               &LeafSystem_double_publicist::MakeWitnessFunction),
           py::arg("description"), py::arg("direction_type"), py::arg("calc"),
           py::arg("e"))
      .def("SetDefaultParameters",
           static_cast<void (LeafSystem<double>::*)(
               Context<double> const &, Parameters<double> *) const>(
               &LeafSystem<double>::SetDefaultParameters),
           py::arg("context"), py::arg("parameters"))
      .def("SetDefaultState",
           static_cast<void (LeafSystem<double>::*)(Context<double> const &,
                                                    State<double> *) const>(
               &LeafSystem<double>::SetDefaultState),
           py::arg("context"), py::arg("state"))

      ;

  // Instantiation of LeafSystem<drake::AutoDiffXd>
  auto PyLeafSystem_Eigen_AutoDiffScalar_Eigen_VectorXd =
      DefineTemplateClass<LeafSystem<drake::AutoDiffXd>,
                          System<drake::AutoDiffXd>>(
          m, "LeafSystem_Eigen_AutoDiffScalar_Eigen_VectorXd");

  PyLeafSystem_Eigen_AutoDiffScalar_Eigen_VectorXd
      .def("AddTriggeredWitnessFunctionToCompositeEventCollection",
           static_cast<void (LeafSystem<drake::AutoDiffXd>::*)(
               Event<drake::AutoDiffXd> *,
               CompositeEventCollection<drake::AutoDiffXd> *) const>(
               &LeafSystem_Eigen_AutoDiffScalar_Eigen_VectorXd_publicist::
                   AddTriggeredWitnessFunctionToCompositeEventCollection),
           py::arg("event"), py::arg("events"))
      .def("AllocateAbstractState",
           static_cast<::std::unique_ptr<AbstractValues,
                                         std::default_delete<AbstractValues>> (
               LeafSystem<drake::AutoDiffXd>::*)() const>(
               &LeafSystem_Eigen_AutoDiffScalar_Eigen_VectorXd_publicist::
                   AllocateAbstractState))
      .def(
          "AllocateCompositeEventCollection",
          static_cast<
              ::std::unique_ptr<CompositeEventCollection<::drake::AutoDiffXd>,
                                std::default_delete<CompositeEventCollection<
                                    ::drake::AutoDiffXd>>> (
                  LeafSystem<drake::AutoDiffXd>::*)() const>(
              &LeafSystem<drake::AutoDiffXd>::AllocateCompositeEventCollection))
      .def("AllocateContext",
           static_cast<::std::unique_ptr<
               LeafContext<::drake::AutoDiffXd>,
               std::default_delete<LeafContext<::drake::AutoDiffXd>>> (
               LeafSystem<drake::AutoDiffXd>::*)() const>(
               &LeafSystem<drake::AutoDiffXd>::AllocateContext))
      .def("AllocateContinuousState",
           static_cast<::std::unique_ptr<
               ContinuousState<::drake::AutoDiffXd>,
               std::default_delete<ContinuousState<::drake::AutoDiffXd>>> (
               LeafSystem<drake::AutoDiffXd>::*)() const>(
               &LeafSystem_Eigen_AutoDiffScalar_Eigen_VectorXd_publicist::
                   AllocateContinuousState))
      .def("AllocateDiscreteState",
           static_cast<::std::unique_ptr<
               DiscreteValues<::drake::AutoDiffXd>,
               std::default_delete<DiscreteValues<::drake::AutoDiffXd>>> (
               LeafSystem<drake::AutoDiffXd>::*)() const>(
               &LeafSystem_Eigen_AutoDiffScalar_Eigen_VectorXd_publicist::
                   AllocateDiscreteState))
      .def("AllocateDiscreteVariables",
           static_cast<::std::unique_ptr<
               DiscreteValues<::drake::AutoDiffXd>,
               std::default_delete<DiscreteValues<::drake::AutoDiffXd>>> (
               LeafSystem<drake::AutoDiffXd>::*)() const>(
               &LeafSystem<drake::AutoDiffXd>::AllocateDiscreteVariables))
      .def("AllocateForcedDiscreteUpdateEventCollection",
           static_cast<::std::unique_ptr<
               EventCollection<DiscreteUpdateEvent<::drake::AutoDiffXd>>,
               std::default_delete<
                   EventCollection<DiscreteUpdateEvent<::drake::AutoDiffXd>>>> (
               LeafSystem<drake::AutoDiffXd>::*)() const>(
               &LeafSystem<drake::AutoDiffXd>::
                   AllocateForcedDiscreteUpdateEventCollection))
      .def("AllocateForcedPublishEventCollection",
           static_cast<::std::unique_ptr<
               EventCollection<PublishEvent<::drake::AutoDiffXd>>,
               std::default_delete<
                   EventCollection<PublishEvent<::drake::AutoDiffXd>>>> (
               LeafSystem<drake::AutoDiffXd>::*)() const>(
               &LeafSystem<
                   drake::AutoDiffXd>::AllocateForcedPublishEventCollection))
      .def("AllocateForcedUnrestrictedUpdateEventCollection",
           static_cast<::std::unique_ptr<
               EventCollection<UnrestrictedUpdateEvent<::drake::AutoDiffXd>>,
               std::default_delete<EventCollection<
                   UnrestrictedUpdateEvent<::drake::AutoDiffXd>>>> (
               LeafSystem<drake::AutoDiffXd>::*)() const>(
               &LeafSystem<drake::AutoDiffXd>::
                   AllocateForcedUnrestrictedUpdateEventCollection))
      .def("AllocateParameters",
           static_cast<::std::unique_ptr<
               Parameters<::drake::AutoDiffXd>,
               std::default_delete<Parameters<::drake::AutoDiffXd>>> (
               LeafSystem<drake::AutoDiffXd>::*)() const>(
               &LeafSystem_Eigen_AutoDiffScalar_Eigen_VectorXd_publicist::
                   AllocateParameters))
      .def("AllocateTimeDerivatives",
           static_cast<::std::unique_ptr<
               ContinuousState<::drake::AutoDiffXd>,
               std::default_delete<ContinuousState<::drake::AutoDiffXd>>> (
               LeafSystem<drake::AutoDiffXd>::*)() const>(
               &LeafSystem<drake::AutoDiffXd>::AllocateTimeDerivatives))
      .def("DeclareAbstractInputPort",
           static_cast<InputPort<drake::AutoDiffXd> &(
               LeafSystem<drake::AutoDiffXd>::
                   *)(::std::variant<
                          std::basic_string<char, std::char_traits<char>,
                                            std::allocator<char>>,
                          UseDefaultName>,
                      ::drake::AbstractValue const &)>(
               &LeafSystem_Eigen_AutoDiffScalar_Eigen_VectorXd_publicist::
                   DeclareAbstractInputPort),
           py::arg("name"), py::arg("model_value"))
      .def("DeclareAbstractInputPort",
           static_cast<InputPort<drake::AutoDiffXd> &(
               LeafSystem<drake::AutoDiffXd>::*)(::drake::AbstractValue const
                                                     &)>(
               &LeafSystem_Eigen_AutoDiffScalar_Eigen_VectorXd_publicist::
                   DeclareAbstractInputPort),
           py::arg("model_value"))
      .def("DeclareAbstractOutputPort",
           [](LeafSystem<drake::AutoDiffXd> &self,
              ::std::variant<std::basic_string<char, std::char_traits<char>,
                                               std::allocator<char>>,
                             UseDefaultName>
                  name,
              LeafOutputPort<drake::AutoDiffXd>::AllocCallback alloc_function,
              LeafOutputPort<drake::AutoDiffXd>::CalcCallback calc_function,
              ::std::set<drake::TypeSafeIndex<DependencyTag>,
                         std::less<drake::TypeSafeIndex<DependencyTag>>,
                         std::allocator<drake::TypeSafeIndex<DependencyTag>>>
                  prerequisites_of_calc) {
             return self.DeclareAbstractOutputPort(
                 name, alloc_function, calc_function, prerequisites_of_calc);
           })
      .def("DeclareAbstractOutputPort",
           [](LeafSystem<drake::AutoDiffXd> &self,
              LeafOutputPort<drake::AutoDiffXd>::AllocCallback alloc_function,
              LeafOutputPort<drake::AutoDiffXd>::CalcCallback calc_function,
              ::std::set<drake::TypeSafeIndex<DependencyTag>,
                         std::less<drake::TypeSafeIndex<DependencyTag>>,
                         std::allocator<drake::TypeSafeIndex<DependencyTag>>>
                  prerequisites_of_calc) {
             return self.DeclareAbstractOutputPort(
                 alloc_function, calc_function, prerequisites_of_calc);
           })
      .def("DeclareAbstractParameter",
           static_cast<int (LeafSystem<drake::AutoDiffXd>::*)(
               ::drake::AbstractValue const &)>(
               &LeafSystem_Eigen_AutoDiffScalar_Eigen_VectorXd_publicist::
                   DeclareAbstractParameter),
           py::arg("model_value"))
      .def("DeclareAbstractState",
           static_cast<AbstractStateIndex (LeafSystem<drake::AutoDiffXd>::*)(
               ::drake::AbstractValue const &)>(
               &LeafSystem_Eigen_AutoDiffScalar_Eigen_VectorXd_publicist::
                   DeclareAbstractState),
           py::arg("abstract_state"))
      .def("DeclareAbstractState",
           [](LeafSystem<drake::AutoDiffXd> &self,
              drake::AbstractValue abstract_state) {
             return self.DeclareAbstractState(
                 std::make_unique<drake::AbstractValue>(abstract_state));
           })
      .def("DeclareContinuousState",
           static_cast<void (LeafSystem<drake::AutoDiffXd>::*)(int)>(
               &LeafSystem_Eigen_AutoDiffScalar_Eigen_VectorXd_publicist::
                   DeclareContinuousState),
           py::arg("num_state_variables"))
      .def("DeclareContinuousState",
           static_cast<void (LeafSystem<drake::AutoDiffXd>::*)(int, int, int)>(
               &LeafSystem_Eigen_AutoDiffScalar_Eigen_VectorXd_publicist::
                   DeclareContinuousState),
           py::arg("num_q"), py::arg("num_v"), py::arg("num_z"))
      .def("DeclareContinuousState",
           static_cast<void (LeafSystem<drake::AutoDiffXd>::*)(
               BasicVector<drake::AutoDiffXd> const &)>(
               &LeafSystem_Eigen_AutoDiffScalar_Eigen_VectorXd_publicist::
                   DeclareContinuousState),
           py::arg("model_vector"))
      .def("DeclareContinuousState",
           static_cast<void (LeafSystem<drake::AutoDiffXd>::*)(
               BasicVector<drake::AutoDiffXd> const &, int, int, int)>(
               &LeafSystem_Eigen_AutoDiffScalar_Eigen_VectorXd_publicist::
                   DeclareContinuousState),
           py::arg("model_vector"), py::arg("num_q"), py::arg("num_v"),
           py::arg("num_z"))
      .def("DeclareDiscreteState",
           static_cast<DiscreteStateIndex (LeafSystem<drake::AutoDiffXd>::*)(
               BasicVector<drake::AutoDiffXd> const &)>(
               &LeafSystem_Eigen_AutoDiffScalar_Eigen_VectorXd_publicist::
                   DeclareDiscreteState),
           py::arg("model_vector"))
      .def("DeclareDiscreteState",
           static_cast<DiscreteStateIndex (LeafSystem<drake::AutoDiffXd>::*)(
               ::Eigen::Ref<
                   const Eigen::Matrix<drake::AutoDiffXd, -1, 1, 0, -1, 1>, 0,
                   Eigen::InnerStride<1>> const &)>(
               &LeafSystem_Eigen_AutoDiffScalar_Eigen_VectorXd_publicist::
                   DeclareDiscreteState),
           py::arg("vector"))
      .def("DeclareDiscreteState",
           static_cast<DiscreteStateIndex (LeafSystem<drake::AutoDiffXd>::*)(
               int)>(&LeafSystem_Eigen_AutoDiffScalar_Eigen_VectorXd_publicist::
                         DeclareDiscreteState),
           py::arg("num_state_variables"))
      .def("DeclareEqualityConstraint",
           static_cast<SystemConstraintIndex (LeafSystem<drake::AutoDiffXd>::*)(
               ::std::function<void(
                   const Context<::drake::AutoDiffXd> &,
                   Eigen::Matrix<drake::AutoDiffXd, -1, 1, 0, -1, 1> *)>,
               int, ::std::string)>(
               &LeafSystem_Eigen_AutoDiffScalar_Eigen_VectorXd_publicist::
                   DeclareEqualityConstraint),
           py::arg("calc"), py::arg("count"), py::arg("description"))
      .def("DeclareImplicitTimeDerivativesResidualSize",
           static_cast<void (LeafSystem<drake::AutoDiffXd>::*)(int)>(
               &LeafSystem_Eigen_AutoDiffScalar_Eigen_VectorXd_publicist::
                   DeclareImplicitTimeDerivativesResidualSize),
           py::arg("n"))
      .def("DeclareInequalityConstraint",
           static_cast<SystemConstraintIndex (LeafSystem<drake::AutoDiffXd>::*)(
               ::std::function<void(
                   const Context<::drake::AutoDiffXd> &,
                   Eigen::Matrix<drake::AutoDiffXd, -1, 1, 0, -1, 1> *)>,
               SystemConstraintBounds, ::std::string)>(
               &LeafSystem_Eigen_AutoDiffScalar_Eigen_VectorXd_publicist::
                   DeclareInequalityConstraint),
           py::arg("calc"), py::arg("bounds"), py::arg("description"))
      .def("DeclareNumericParameter",
           static_cast<int (LeafSystem<drake::AutoDiffXd>::*)(
               BasicVector<drake::AutoDiffXd> const &)>(
               &LeafSystem_Eigen_AutoDiffScalar_Eigen_VectorXd_publicist::
                   DeclareNumericParameter),
           py::arg("model_vector"))
      .def("DeclarePeriodicDiscreteUpdate",
           static_cast<void (LeafSystem<drake::AutoDiffXd>::*)(double, double)>(
               &LeafSystem_Eigen_AutoDiffScalar_Eigen_VectorXd_publicist::
                   DeclarePeriodicDiscreteUpdate),
           py::arg("period_sec"), py::arg("offset_sec") = double(0))
      .def("DeclarePeriodicPublish",
           static_cast<void (LeafSystem<drake::AutoDiffXd>::*)(double, double)>(
               &LeafSystem_Eigen_AutoDiffScalar_Eigen_VectorXd_publicist::
                   DeclarePeriodicPublish),
           py::arg("period_sec"), py::arg("offset_sec") = double(0))
      .def("DeclarePeriodicUnrestrictedUpdate",
           static_cast<void (LeafSystem<drake::AutoDiffXd>::*)(double, double)>(
               &LeafSystem_Eigen_AutoDiffScalar_Eigen_VectorXd_publicist::
                   DeclarePeriodicUnrestrictedUpdate),
           py::arg("period_sec"), py::arg("offset_sec") = double(0))
      .def("DeclareVectorInputPort",
           static_cast<InputPort<drake::AutoDiffXd> &(
               LeafSystem<drake::AutoDiffXd>::
                   *)(::std::variant<
                          std::basic_string<char, std::char_traits<char>,
                                            std::allocator<char>>,
                          UseDefaultName>,
                      BasicVector<drake::AutoDiffXd> const &,
                      ::std::optional<drake::RandomDistribution>)>(
               &LeafSystem_Eigen_AutoDiffScalar_Eigen_VectorXd_publicist::
                   DeclareVectorInputPort),
           py::arg("name"), py::arg("model_vector"),
           py::arg("random_type") =
               ::std::optional<drake::RandomDistribution>(std::nullopt))
      .def("DeclareVectorInputPort",
           static_cast<InputPort<drake::AutoDiffXd> &(
               LeafSystem<drake::AutoDiffXd>::
                   *)(BasicVector<drake::AutoDiffXd> const &,
                      ::std::optional<drake::RandomDistribution>)>(
               &LeafSystem_Eigen_AutoDiffScalar_Eigen_VectorXd_publicist::
                   DeclareVectorInputPort),
           py::arg("model_vector"),
           py::arg("random_type") =
               ::std::optional<drake::RandomDistribution>(std::nullopt))
      .def(
          "DeclareVectorOutputPort",
          static_cast<LeafOutputPort<drake::AutoDiffXd> &(
              LeafSystem<drake::AutoDiffXd>::
                  *)(::std::variant<
                         std::basic_string<char, std::char_traits<char>,
                                           std::allocator<char>>,
                         UseDefaultName>,
                     BasicVector<drake::AutoDiffXd> const &,
                     LeafOutputPort<drake::AutoDiffXd>::CalcVectorCallback,
                     ::std::set<
                         drake::TypeSafeIndex<DependencyTag>,
                         std::less<drake::TypeSafeIndex<DependencyTag>>,
                         std::allocator<drake::TypeSafeIndex<DependencyTag>>>)>(
              &LeafSystem_Eigen_AutoDiffScalar_Eigen_VectorXd_publicist::
                  DeclareVectorOutputPort),
          py::arg("name"), py::arg("model_vector"),
          py::arg("vector_calc_function"),
          py::arg("prerequisites_of_calc") =
              ::std::set<drake::TypeSafeIndex<DependencyTag>,
                         std::less<drake::TypeSafeIndex<DependencyTag>>,
                         std::allocator<drake::TypeSafeIndex<DependencyTag>>>(
                  {SystemBase::all_sources_ticket()}))
      .def(
          "DeclareVectorOutputPort",
          static_cast<LeafOutputPort<drake::AutoDiffXd> &(
              LeafSystem<drake::AutoDiffXd>::
                  *)(BasicVector<drake::AutoDiffXd> const &,
                     LeafOutputPort<drake::AutoDiffXd>::CalcVectorCallback,
                     ::std::set<
                         drake::TypeSafeIndex<DependencyTag>,
                         std::less<drake::TypeSafeIndex<DependencyTag>>,
                         std::allocator<drake::TypeSafeIndex<DependencyTag>>>)>(
              &LeafSystem_Eigen_AutoDiffScalar_Eigen_VectorXd_publicist::
                  DeclareVectorOutputPort),
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
                  LeafSystem<drake::AutoDiffXd>::*)() const>(
              &LeafSystem<drake::AutoDiffXd>::DoAllocateContext))
      .def("DoCalcDiscreteVariableUpdates",
           static_cast<void (LeafSystem<drake::AutoDiffXd>::*)(
               Context<drake::AutoDiffXd> const &,
               ::std::vector<const DiscreteUpdateEvent<::drake::AutoDiffXd> *,
                             std::allocator<const DiscreteUpdateEvent<
                                 ::drake::AutoDiffXd> *>> const &,
               DiscreteValues<drake::AutoDiffXd> *) const>(
               &LeafSystem_Eigen_AutoDiffScalar_Eigen_VectorXd_publicist::
                   DoCalcDiscreteVariableUpdates),
           py::arg("context"), py::arg("events"), py::arg("discrete_state"))
      .def("DoCalcNextUpdateTime",
           static_cast<void (LeafSystem<drake::AutoDiffXd>::*)(
               Context<drake::AutoDiffXd> const &,
               CompositeEventCollection<drake::AutoDiffXd> *,
               drake::AutoDiffXd *) const>(
               &LeafSystem_Eigen_AutoDiffScalar_Eigen_VectorXd_publicist::
                   DoCalcNextUpdateTime),
           py::arg("context"), py::arg("events"), py::arg("time"))
      .def("DoCalcUnrestrictedUpdate",
           static_cast<void (LeafSystem<drake::AutoDiffXd>::*)(
               Context<drake::AutoDiffXd> const &,
               ::std::vector<
                   const UnrestrictedUpdateEvent<::drake::AutoDiffXd> *,
                   std::allocator<const UnrestrictedUpdateEvent<
                       ::drake::AutoDiffXd> *>> const &,
               State<drake::AutoDiffXd> *) const>(
               &LeafSystem_Eigen_AutoDiffScalar_Eigen_VectorXd_publicist::
                   DoCalcUnrestrictedUpdate),
           py::arg("context"), py::arg("events"), py::arg("state"))
      .def("DoCalcWitnessValue",
           static_cast<drake::AutoDiffXd (LeafSystem<drake::AutoDiffXd>::*)(
               Context<drake::AutoDiffXd> const &,
               WitnessFunction<drake::AutoDiffXd> const &) const>(
               &LeafSystem_Eigen_AutoDiffScalar_Eigen_VectorXd_publicist::
                   DoCalcWitnessValue),
           py::arg("context"), py::arg("witness_func"))
      .def("DoMakeLeafContext",
           static_cast<::std::unique_ptr<
               LeafContext<::drake::AutoDiffXd>,
               std::default_delete<LeafContext<::drake::AutoDiffXd>>> (
               LeafSystem<drake::AutoDiffXd>::*)() const>(
               &LeafSystem_Eigen_AutoDiffScalar_Eigen_VectorXd_publicist::
                   DoMakeLeafContext))
      .def("DoPublish",
           static_cast<void (LeafSystem<drake::AutoDiffXd>::*)(
               Context<drake::AutoDiffXd> const &,
               ::std::vector<const PublishEvent<::drake::AutoDiffXd> *,
                             std::allocator<const PublishEvent<
                                 ::drake::AutoDiffXd> *>> const &) const>(
               &LeafSystem_Eigen_AutoDiffScalar_Eigen_VectorXd_publicist::
                   DoPublish),
           py::arg("context"), py::arg("events"))
      .def("DoValidateAllocatedLeafContext",
           static_cast<void (LeafSystem<drake::AutoDiffXd>::*)(
               LeafContext<drake::AutoDiffXd> const &) const>(
               &LeafSystem_Eigen_AutoDiffScalar_Eigen_VectorXd_publicist::
                   DoValidateAllocatedLeafContext),
           py::arg("context"))
      .def("GetDirectFeedthroughs",
           static_cast<
               ::std::multimap<int, int, std::less<int>,
                               std::allocator<std::pair<const int, int>>> (
                   LeafSystem<drake::AutoDiffXd>::*)() const>(
               &LeafSystem<drake::AutoDiffXd>::GetDirectFeedthroughs))
      .def("GetGraphvizFragment",
           static_cast<void (LeafSystem<drake::AutoDiffXd>::*)(
               int, ::std::stringstream *) const>(
               &LeafSystem_Eigen_AutoDiffScalar_Eigen_VectorXd_publicist::
                   GetGraphvizFragment),
           py::arg("max_depth"), py::arg("dot"))
      .def("GetGraphvizInputPortToken",
           static_cast<void (LeafSystem<drake::AutoDiffXd>::*)(
               InputPort<drake::AutoDiffXd> const &, int, ::std::stringstream *)
                           const>(
               &LeafSystem_Eigen_AutoDiffScalar_Eigen_VectorXd_publicist::
                   GetGraphvizInputPortToken),
           py::arg("port"), py::arg("max_depth"), py::arg("dot"))
      .def("GetGraphvizOutputPortToken",
           static_cast<void (LeafSystem<drake::AutoDiffXd>::*)(
               OutputPort<drake::AutoDiffXd> const &, int,
               ::std::stringstream *) const>(
               &LeafSystem_Eigen_AutoDiffScalar_Eigen_VectorXd_publicist::
                   GetGraphvizOutputPortToken),
           py::arg("port"), py::arg("max_depth"), py::arg("dot"))
      .def("MakeWitnessFunction",
           static_cast<::std::unique_ptr<
               WitnessFunction<::drake::AutoDiffXd>,
               std::default_delete<WitnessFunction<::drake::AutoDiffXd>>> (
               LeafSystem<drake::AutoDiffXd>::*)(
               ::std::string const &, WitnessFunctionDirection const &,
               ::std::function<drake::AutoDiffXd(
                   const Context<::drake::AutoDiffXd> &)>) const>(
               &LeafSystem_Eigen_AutoDiffScalar_Eigen_VectorXd_publicist::
                   MakeWitnessFunction),
           py::arg("description"), py::arg("direction_type"), py::arg("calc"))
      .def("MakeWitnessFunction",
           static_cast<::std::unique_ptr<
               WitnessFunction<::drake::AutoDiffXd>,
               std::default_delete<WitnessFunction<::drake::AutoDiffXd>>> (
               LeafSystem<drake::AutoDiffXd>::*)(
               ::std::string const &, WitnessFunctionDirection const &,
               ::std::function<drake::AutoDiffXd(
                   const Context<::drake::AutoDiffXd> &)>,
               Event<drake::AutoDiffXd> const &) const>(
               &LeafSystem_Eigen_AutoDiffScalar_Eigen_VectorXd_publicist::
                   MakeWitnessFunction),
           py::arg("description"), py::arg("direction_type"), py::arg("calc"),
           py::arg("e"))
      .def("SetDefaultParameters",
           static_cast<void (LeafSystem<drake::AutoDiffXd>::*)(
               Context<drake::AutoDiffXd> const &,
               Parameters<drake::AutoDiffXd> *) const>(
               &LeafSystem<drake::AutoDiffXd>::SetDefaultParameters),
           py::arg("context"), py::arg("parameters"))
      .def("SetDefaultState",
           static_cast<void (LeafSystem<drake::AutoDiffXd>::*)(
               Context<drake::AutoDiffXd> const &, State<drake::AutoDiffXd> *)
                           const>(
               &LeafSystem<drake::AutoDiffXd>::SetDefaultState),
           py::arg("context"), py::arg("state"))

      ;

  // Instantiation of LeafSystem<float>
  auto PyLeafSystem_float =
      DefineTemplateClass<LeafSystem<float>>(m, "LeafSystem_float");

  PyLeafSystem_float
      .def("AddTriggeredWitnessFunctionToCompositeEventCollection",
           static_cast<void (LeafSystem<float>::*)(
               Event<float> *, CompositeEventCollection<float> *) const>(
               &LeafSystem_float_publicist::
                   AddTriggeredWitnessFunctionToCompositeEventCollection),
           py::arg("event"), py::arg("events"))
      .def("AllocateAbstractState",
           static_cast<::std::unique_ptr<AbstractValues,
                                         std::default_delete<AbstractValues>> (
               LeafSystem<float>::*)() const>(
               &LeafSystem_float_publicist::AllocateAbstractState))
      .def("AllocateCompositeEventCollection",
           static_cast<::std::unique_ptr<
               CompositeEventCollection<float>,
               std::default_delete<CompositeEventCollection<float>>> (
               LeafSystem<float>::*)() const>(
               &LeafSystem<float>::AllocateCompositeEventCollection))
      .def("AllocateContext",
           static_cast<::std::unique_ptr<
               LeafContext<float>, std::default_delete<LeafContext<float>>> (
               LeafSystem<float>::*)() const>(
               &LeafSystem<float>::AllocateContext))
      .def("AllocateContinuousState",
           static_cast<
               ::std::unique_ptr<ContinuousState<float>,
                                 std::default_delete<ContinuousState<float>>> (
                   LeafSystem<float>::*)() const>(
               &LeafSystem_float_publicist::AllocateContinuousState))
      .def("AllocateDiscreteState",
           static_cast<
               ::std::unique_ptr<DiscreteValues<float>,
                                 std::default_delete<DiscreteValues<float>>> (
                   LeafSystem<float>::*)() const>(
               &LeafSystem_float_publicist::AllocateDiscreteState))
      .def("AllocateDiscreteVariables",
           static_cast<
               ::std::unique_ptr<DiscreteValues<float>,
                                 std::default_delete<DiscreteValues<float>>> (
                   LeafSystem<float>::*)() const>(
               &LeafSystem<float>::AllocateDiscreteVariables))
      .def(
          "AllocateForcedDiscreteUpdateEventCollection",
          static_cast<::std::unique_ptr<
              EventCollection<DiscreteUpdateEvent<float>>,
              std::default_delete<EventCollection<
                  DiscreteUpdateEvent<float>>>> (LeafSystem<float>::*)() const>(
              &LeafSystem<float>::AllocateForcedDiscreteUpdateEventCollection))
      .def("AllocateForcedPublishEventCollection",
           static_cast<::std::unique_ptr<
               EventCollection<PublishEvent<float>>,
               std::default_delete<EventCollection<PublishEvent<float>>>> (
               LeafSystem<float>::*)() const>(
               &LeafSystem<float>::AllocateForcedPublishEventCollection))
      .def("AllocateForcedUnrestrictedUpdateEventCollection",
           static_cast<::std::unique_ptr<
               EventCollection<UnrestrictedUpdateEvent<float>>,
               std::default_delete<
                   EventCollection<UnrestrictedUpdateEvent<float>>>> (
               LeafSystem<float>::*)() const>(
               &LeafSystem<
                   float>::AllocateForcedUnrestrictedUpdateEventCollection))
      .def("AllocateParameters",
           static_cast<::std::unique_ptr<
               Parameters<float>, std::default_delete<Parameters<float>>> (
               LeafSystem<float>::*)() const>(
               &LeafSystem_float_publicist::AllocateParameters))
      .def("AllocateTimeDerivatives",
           static_cast<
               ::std::unique_ptr<ContinuousState<float>,
                                 std::default_delete<ContinuousState<float>>> (
                   LeafSystem<float>::*)() const>(
               &LeafSystem<float>::AllocateTimeDerivatives))
      .def("DeclareAbstractOutputPort",
           [](LeafSystem<float> &self,
              ::std::variant<std::basic_string<char, std::char_traits<char>,
                                               std::allocator<char>>,
                             UseDefaultName>
                  name,
              LeafOutputPort<float>::AllocCallback alloc_function,
              LeafOutputPort<float>::CalcCallback calc_function,
              ::std::set<drake::TypeSafeIndex<DependencyTag>,
                         std::less<drake::TypeSafeIndex<DependencyTag>>,
                         std::allocator<drake::TypeSafeIndex<DependencyTag>>>
                  prerequisites_of_calc) {
             return self.DeclareAbstractOutputPort(
                 name, alloc_function, calc_function, prerequisites_of_calc);
           })
      .def("DeclareAbstractOutputPort",
           [](LeafSystem<float> &self,
              LeafOutputPort<float>::AllocCallback alloc_function,
              LeafOutputPort<float>::CalcCallback calc_function,
              ::std::set<drake::TypeSafeIndex<DependencyTag>,
                         std::less<drake::TypeSafeIndex<DependencyTag>>,
                         std::allocator<drake::TypeSafeIndex<DependencyTag>>>
                  prerequisites_of_calc) {
             return self.DeclareAbstractOutputPort(
                 alloc_function, calc_function, prerequisites_of_calc);
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
           static_cast<DiscreteStateIndex (LeafSystem<float>::*)(
               ::Eigen::Ref<const Eigen::Matrix<float, -1, 1, 0, -1, 1>, 0,
                            Eigen::InnerStride<1>> const &)>(
               &LeafSystem_float_publicist::DeclareDiscreteState),
           py::arg("vector"))
      .def("DeclareDiscreteState",
           static_cast<DiscreteStateIndex (LeafSystem<float>::*)(int)>(
               &LeafSystem_float_publicist::DeclareDiscreteState),
           py::arg("num_state_variables"))
      .def("DeclareEqualityConstraint",
           static_cast<SystemConstraintIndex (LeafSystem<float>::*)(
               ::std::function<void(const Context<float> &,
                                    Eigen::Matrix<float, -1, 1, 0, -1, 1> *)>,
               int, ::std::string)>(
               &LeafSystem_float_publicist::DeclareEqualityConstraint),
           py::arg("calc"), py::arg("count"), py::arg("description"))
      .def("DeclareImplicitTimeDerivativesResidualSize",
           static_cast<void (LeafSystem<float>::*)(int)>(
               &LeafSystem_float_publicist::
                   DeclareImplicitTimeDerivativesResidualSize),
           py::arg("n"))
      .def("DeclareInequalityConstraint",
           static_cast<SystemConstraintIndex (LeafSystem<float>::*)(
               ::std::function<void(const Context<float> &,
                                    Eigen::Matrix<float, -1, 1, 0, -1, 1> *)>,
               SystemConstraintBounds, ::std::string)>(
               &LeafSystem_float_publicist::DeclareInequalityConstraint),
           py::arg("calc"), py::arg("bounds"), py::arg("description"))
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
      .def(
          "DeclareVectorOutputPort",
          static_cast<LeafOutputPort<float> &(
              LeafSystem<float>::
                  *)(::std::variant<
                         std::basic_string<char, std::char_traits<char>,
                                           std::allocator<char>>,
                         UseDefaultName>,
                     BasicVector<float> const &,
                     LeafOutputPort<float>::CalcVectorCallback,
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
          static_cast<LeafOutputPort<float> &(
              LeafSystem<float>::
                  *)(BasicVector<float> const &,
                     LeafOutputPort<float>::CalcVectorCallback,
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
                  LeafSystem<float>::*)() const>(
              &LeafSystem<float>::DoAllocateContext))
      .def("DoCalcDiscreteVariableUpdates",
           static_cast<void (LeafSystem<float>::*)(
               Context<float> const &,
               ::std::vector<
                   const DiscreteUpdateEvent<float> *,
                   std::allocator<const DiscreteUpdateEvent<float> *>> const &,
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
               ::std::vector<const UnrestrictedUpdateEvent<float> *,
                             std::allocator<const UnrestrictedUpdateEvent<float>
                                                *>> const &,
               State<float> *) const>(
               &LeafSystem_float_publicist::DoCalcUnrestrictedUpdate),
           py::arg("context"), py::arg("events"), py::arg("state"))
      .def("DoMakeLeafContext",
           static_cast<::std::unique_ptr<
               LeafContext<float>, std::default_delete<LeafContext<float>>> (
               LeafSystem<float>::*)() const>(
               &LeafSystem_float_publicist::DoMakeLeafContext))
      .def("DoPublish",
           static_cast<void (LeafSystem<float>::*)(
               Context<float> const &,
               ::std::vector<const PublishEvent<float> *,
                             std::allocator<const PublishEvent<float> *>> const
                   &) const>(&LeafSystem_float_publicist::DoPublish),
           py::arg("context"), py::arg("events"))
      .def("DoValidateAllocatedLeafContext",
           static_cast<void (LeafSystem<float>::*)(LeafContext<float> const &)
                           const>(
               &LeafSystem_float_publicist::DoValidateAllocatedLeafContext),
           py::arg("context"))
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
           static_cast<
               ::std::unique_ptr<WitnessFunction<float>,
                                 std::default_delete<WitnessFunction<float>>> (
                   LeafSystem<float>::*)(
                   ::std::string const &, WitnessFunctionDirection const &,
                   ::std::function<float(const Context<float> &)>) const>(
               &LeafSystem_float_publicist::MakeWitnessFunction),
           py::arg("description"), py::arg("direction_type"), py::arg("calc"))
      .def("MakeWitnessFunction",
           static_cast<
               ::std::unique_ptr<WitnessFunction<float>,
                                 std::default_delete<WitnessFunction<float>>> (
                   LeafSystem<float>::*)(
                   ::std::string const &, WitnessFunctionDirection const &,
                   ::std::function<float(const Context<float> &)>,
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
