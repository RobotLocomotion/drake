#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "drake/bindings/generated_docstrings/systems_primitives.h"
#include "drake/bindings/pydrake/common/cpp_template_pybind.h"
#include "drake/bindings/pydrake/common/default_scalars_pybind.h"
#include "drake/bindings/pydrake/common/eigen_pybind.h"
#include "drake/bindings/pydrake/common/serialize_pybind.h"
#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/systems/primitives/adder.h"
#include "drake/systems/primitives/affine_system.h"
#include "drake/systems/primitives/barycentric_system.h"
#include "drake/systems/primitives/bus_creator.h"
#include "drake/systems/primitives/bus_selector.h"
#include "drake/systems/primitives/constant_value_source.h"
#include "drake/systems/primitives/constant_vector_source.h"
#include "drake/systems/primitives/demultiplexer.h"
#include "drake/systems/primitives/discrete_derivative.h"
#include "drake/systems/primitives/discrete_time_delay.h"
#include "drake/systems/primitives/discrete_time_integrator.h"
#include "drake/systems/primitives/first_order_low_pass_filter.h"
#include "drake/systems/primitives/gain.h"
#include "drake/systems/primitives/integrator.h"
#include "drake/systems/primitives/linear_system.h"
#include "drake/systems/primitives/linear_transform_density.h"
#include "drake/systems/primitives/matrix_gain.h"
#include "drake/systems/primitives/multilayer_perceptron.h"
#include "drake/systems/primitives/multiplexer.h"
#include "drake/systems/primitives/pass_through.h"
#include "drake/systems/primitives/port_switch.h"
#include "drake/systems/primitives/random_source.h"
#include "drake/systems/primitives/saturation.h"
#include "drake/systems/primitives/selector.h"
#include "drake/systems/primitives/shared_pointer_system.h"
#include "drake/systems/primitives/sine.h"
#include "drake/systems/primitives/sparse_matrix_gain.h"
#include "drake/systems/primitives/symbolic_vector_system.h"
#include "drake/systems/primitives/trajectory_affine_system.h"
#include "drake/systems/primitives/trajectory_linear_system.h"
#include "drake/systems/primitives/trajectory_source.h"
#include "drake/systems/primitives/vector_log_sink.h"
#include "drake/systems/primitives/wrap_to_system.h"
#include "drake/systems/primitives/zero_order_hold.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

namespace drake {

using symbolic::Expression;
using symbolic::Variable;

namespace pydrake {

PYDRAKE_MODULE(primitives, m) {
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::systems;

  m.doc() = "Bindings for the primitives portion of the Systems framework.";
  constexpr auto& doc = pydrake_doc_systems_primitives.drake.systems;

  py::module_::import_("pydrake.systems.framework");
  py::module_::import_("pydrake.trajectories");

  py::enum_<PerceptronActivationType>(
      m, "PerceptronActivationType", doc.PerceptronActivationType.doc)
      .value("kIdentity", PerceptronActivationType::kIdentity,
          doc.PerceptronActivationType.kIdentity.doc)
      .value("kReLU", PerceptronActivationType::kReLU,
          doc.PerceptronActivationType.kReLU.doc)
      .value("kTanh", PerceptronActivationType::kTanh,
          doc.PerceptronActivationType.kTanh.doc);

  {
    using Class = SelectorParams;
    class_<Class> cls(m, "SelectorParams", doc.SelectorParams.doc);
    {
      using Nested = Class::InputPortParams;
      class_<Nested> nested(
          cls, "InputPortParams", doc.SelectorParams.InputPortParams.doc);
      nested.def(ParamInit<Nested>());
      DefAttributesUsingSerialize(&nested, doc.SelectorParams.InputPortParams);
      DefReprUsingSerialize(&nested);
      DefCopyAndDeepCopy(&nested);
    }
    {
      using Nested = Class::OutputSelection;
      class_<Nested> nested(
          cls, "OutputSelection", doc.SelectorParams.OutputSelection.doc);
      nested.def(ParamInit<Nested>());
      DefAttributesUsingSerialize(&nested, doc.SelectorParams.OutputSelection);
      DefReprUsingSerialize(&nested);
      DefCopyAndDeepCopy(&nested);
    }
    {
      using Nested = Class::OutputPortParams;
      class_<Nested> nested(
          cls, "OutputPortParams", doc.SelectorParams.OutputPortParams.doc);
      nested.def(ParamInit<Nested>());
      DefAttributesUsingSerialize(&nested, doc.SelectorParams.OutputPortParams);
      DefReprUsingSerialize(&nested);
      DefCopyAndDeepCopy(&nested);
    }
    cls.def(ParamInit<Class>());
    DefAttributesUsingSerialize(&cls, doc.SelectorParams);
    DefReprUsingSerialize(&cls);
    DefCopyAndDeepCopy(&cls);
  }

  // N.B. Capturing `&doc` should not be required; workaround per #9600.
  auto bind_common_scalar_types = [&m, &doc](auto dummy) {
    using T = decltype(dummy);

    DefineTemplateClassWithDefault<Adder<T>, LeafSystem<T>>(
        m, "Adder", GetPyParam<T>(), doc.Adder.doc)
        .def(py::init<int, int>(), py::arg("num_inputs"), py::arg("size"),
            doc.Adder.ctor.doc);

    DefineTemplateClassWithDefault<AffineSystem<T>, LeafSystem<T>>(
        m, "AffineSystem", GetPyParam<T>(), doc.AffineSystem.doc)
        .def(py::init<const Eigen::Ref<const MatrixXd>&,
                 const Eigen::Ref<const MatrixXd>&,
                 const Eigen::Ref<const VectorXd>&,
                 const Eigen::Ref<const MatrixXd>&,
                 const Eigen::Ref<const MatrixXd>&,
                 const Eigen::Ref<const VectorXd>&, double>(),
            py::arg("A") = Eigen::MatrixXd(), py::arg("B") = Eigen::MatrixXd(),
            py::arg("f0") = Eigen::VectorXd(), py::arg("C") = Eigen::MatrixXd(),
            py::arg("D") = Eigen::MatrixXd(), py::arg("y0") = Eigen::VectorXd(),
            py::arg("time_period") = 0.0, doc.AffineSystem.ctor.doc_7args)
        // TODO(eric.cousineau): Fix these to return references instead of
        // copies.
        .def("A", overload_cast_explicit<const MatrixXd&>(&AffineSystem<T>::A),
            doc.AffineSystem.A.doc_0args)
        .def("B", overload_cast_explicit<const MatrixXd&>(&AffineSystem<T>::B),
            doc.AffineSystem.B.doc)
        .def("f0",
            overload_cast_explicit<const VectorXd&>(&AffineSystem<T>::f0),
            doc.AffineSystem.f0.doc)
        .def("C", overload_cast_explicit<const MatrixXd&>(&AffineSystem<T>::C),
            doc.AffineSystem.C.doc)
        .def("D", overload_cast_explicit<const MatrixXd&>(&AffineSystem<T>::D),
            doc.AffineSystem.D.doc)
        .def("y0",
            overload_cast_explicit<const VectorXd&>(&AffineSystem<T>::y0),
            doc.AffineSystem.y0.doc)
        .def("UpdateCoefficients", &AffineSystem<T>::UpdateCoefficients,
            py::arg("A") = Eigen::MatrixXd(), py::arg("B") = Eigen::MatrixXd(),
            py::arg("f0") = Eigen::VectorXd(), py::arg("C") = Eigen::MatrixXd(),
            py::arg("D") = Eigen::MatrixXd(), py::arg("y0") = Eigen::VectorXd(),
            doc.AffineSystem.UpdateCoefficients.doc)
        // Wrap a few methods from the TimeVaryingAffineSystem parent class.
        // TODO(russt): Move to TimeVaryingAffineSystem if/when that class is
        // wrapped.
        .def("time_period", &AffineSystem<T>::time_period,
            doc.TimeVaryingAffineSystem.time_period.doc)
        .def("num_states", &TrajectoryAffineSystem<T>::num_states,
            doc.TimeVaryingAffineSystem.num_states.doc)
        .def("num_inputs", &TrajectoryAffineSystem<T>::num_inputs,
            doc.TimeVaryingAffineSystem.num_inputs.doc)
        .def("num_outputs", &TrajectoryAffineSystem<T>::num_outputs,
            doc.TimeVaryingAffineSystem.num_outputs.doc)
        .def("configure_default_state",
            &TimeVaryingAffineSystem<T>::configure_default_state, py::arg("x0"),
            doc.TimeVaryingAffineSystem.configure_default_state.doc)
        .def("configure_random_state",
            &TimeVaryingAffineSystem<T>::configure_random_state,
            py::arg("covariance"),
            doc.TimeVaryingAffineSystem.configure_random_state.doc);

    DefineTemplateClassWithDefault<BusCreator<T>, LeafSystem<T>>(
        m, "BusCreator", GetPyParam<T>(), doc.BusCreator.doc)
        .def(py::init<std::variant<std::string, UseDefaultName>>(),
            py::arg("output_port_name") = kUseDefaultName,
            doc.BusCreator.ctor.doc)
        .def("DeclareVectorInputPort", &BusCreator<T>::DeclareVectorInputPort,
            py::arg("name"), py::arg("size"), py_rvp::reference_internal,
            doc.BusCreator.DeclareVectorInputPort.doc)
        .def("DeclareAbstractInputPort",
            &BusCreator<T>::DeclareAbstractInputPort, py::arg("name"),
            py::arg("model_value"), py_rvp::reference_internal,
            doc.BusCreator.DeclareAbstractInputPort.doc);

    DefineTemplateClassWithDefault<BusSelector<T>, LeafSystem<T>>(
        m, "BusSelector", GetPyParam<T>(), doc.BusSelector.doc)
        .def(py::init<std::variant<std::string, UseDefaultName>>(),
            py::arg("input_port_name") = kUseDefaultName,
            doc.BusSelector.ctor.doc)
        .def("DeclareVectorOutputPort",
            &BusSelector<T>::DeclareVectorOutputPort, py::arg("name"),
            py::arg("size"), py_rvp::reference_internal,
            doc.BusSelector.DeclareVectorOutputPort.doc)
        .def("DeclareAbstractOutputPort",
            &BusSelector<T>::DeclareAbstractOutputPort, py::arg("name"),
            py::arg("model_value"), py_rvp::reference_internal,
            doc.BusSelector.DeclareAbstractOutputPort.doc);

    DefineTemplateClassWithDefault<ConstantValueSource<T>, LeafSystem<T>>(
        m, "ConstantValueSource", GetPyParam<T>(), doc.ConstantValueSource.doc)
        .def(py::init<const AbstractValue&>(), py::arg("value"),
            doc.ConstantValueSource.ctor.doc);

    DefineTemplateClassWithDefault<ConstantVectorSource<T>, LeafSystem<T>>(m,
        "ConstantVectorSource", GetPyParam<T>(), doc.ConstantVectorSource.doc)
        .def(py::init<VectorX<T>>(), py::arg("source_value"),
            doc.ConstantVectorSource.ctor.doc)
        .def("get_source_value", &ConstantVectorSource<T>::get_source_value,
            py::arg("context"), py_rvp::reference_internal,
            doc.ConstantVectorSource.get_source_value.doc)
        .def("get_mutable_source_value",
            &ConstantVectorSource<T>::get_mutable_source_value,
            py::arg("context"), py_rvp::reference_internal,
            doc.ConstantVectorSource.get_mutable_source_value.doc);

    DefineTemplateClassWithDefault<Demultiplexer<T>, LeafSystem<T>>(
        m, "Demultiplexer", GetPyParam<T>(), doc.Demultiplexer.doc)
        .def(py::init<int, int>(), py::arg("size"),
            py::arg("output_ports_size") = 1, doc.Demultiplexer.ctor.doc_2args)
        .def(py::init<const std::vector<int>&>(), py::arg("output_ports_sizes"),
            doc.Demultiplexer.ctor.doc_1args)
        .def("get_output_ports_sizes",
            &Demultiplexer<T>::get_output_ports_sizes,
            doc.Demultiplexer.get_output_ports_sizes.doc);

    DefineTemplateClassWithDefault<DiscreteTimeDelay<T>, LeafSystem<T>>(
        m, "DiscreteTimeDelay", GetPyParam<T>(), doc.DiscreteTimeDelay.doc)
        .def(py::init<double, int, int>(), py::arg("update_sec"),
            py::arg("delay_time_steps"), py::arg("vector_size"),
            doc.DiscreteTimeDelay.ctor
                .doc_3args_update_sec_delay_time_steps_vector_size)
        .def(py::init<double, int, const AbstractValue&>(),
            py::arg("update_sec"), py::arg("delay_time_steps"),
            py::arg("abstract_model_value"),
            doc.DiscreteTimeDelay.ctor
                .doc_3args_update_sec_delay_time_steps_abstract_model_value);

    DefineTemplateClassWithDefault<DiscreteTimeIntegrator<T>, LeafSystem<T>>(m,
        "DiscreteTimeIntegrator", GetPyParam<T>(),
        doc.DiscreteTimeIntegrator.doc)
        .def(py::init<int, double>(), py::arg("size"), py::arg("time_step"),
            doc.DiscreteTimeIntegrator.ctor.doc)
        .def("set_integral_value",
            &DiscreteTimeIntegrator<T>::set_integral_value, py::arg("context"),
            py::arg("value"), doc.DiscreteTimeIntegrator.set_integral_value.doc)
        .def("time_step", &DiscreteTimeIntegrator<T>::time_step,
            doc.DiscreteTimeIntegrator.time_step.doc);

    DefineTemplateClassWithDefault<DiscreteDerivative<T>, LeafSystem<T>>(
        m, "DiscreteDerivative", GetPyParam<T>(), doc.DiscreteDerivative.doc)
        .def(py::init<int, double, bool>(), py::arg("num_inputs"),
            py::arg("time_step"), py::arg("suppress_initial_transient") = true,
            doc.DiscreteDerivative.ctor.doc)
        .def("time_step", &DiscreteDerivative<T>::time_step,
            doc.DiscreteDerivative.time_step.doc)
        .def("suppress_initial_transient",
            &DiscreteDerivative<T>::suppress_initial_transient,
            doc.DiscreteDerivative.suppress_initial_transient.doc);

    DefineTemplateClassWithDefault<
        FirstOrderLowPassFilter<T>, LeafSystem<T>>(
        m, "FirstOrderLowPassFilter", GetPyParam<T>(),
        doc.FirstOrderLowPassFilter.doc)
        .def(py::init<double, int>(), py::arg("time_constant"),
            py::arg("size") = 1, doc.FirstOrderLowPassFilter.ctor.doc_2args)
        .def(py::init<const VectorX<double>&>(), py::arg("time_constants"),
            doc.FirstOrderLowPassFilter.ctor.doc_1args)
        .def("get_time_constant",
            &FirstOrderLowPassFilter<T>::get_time_constant,
            doc.FirstOrderLowPassFilter.get_time_constant.doc)
        .def("get_time_constants_vector",
            &FirstOrderLowPassFilter<T>::get_time_constants_vector,
            doc.FirstOrderLowPassFilter.get_time_constants_vector.doc)
        .def("set_initial_output_value",
            &FirstOrderLowPassFilter<T>::set_initial_output_value,
            doc.FirstOrderLowPassFilter.set_initial_output_value.doc);

    DefineTemplateClassWithDefault<Gain<T>, LeafSystem<T>>(
        m, "Gain", GetPyParam<T>(), doc.Gain.doc)
        .def(py::init<double, int>(), py::arg("k"), py::arg("size"),
            doc.Gain.ctor.doc_2args)
        .def(py::init<const Eigen::Ref<const VectorXd>&>(), py::arg("k"),
            doc.Gain.ctor.doc_1args);

    DefineTemplateClassWithDefault<Selector<T>, LeafSystem<T>>(
        m, "Selector", GetPyParam<T>(), doc.Selector.doc)
        .def(py::init<SelectorParams>(), py::arg("params"),
            doc.Selector.ctor.doc);

    DefineTemplateClassWithDefault<Sine<T>, LeafSystem<T>>(
        m, "Sine", GetPyParam<T>(), doc.Sine.doc)
        .def(py::init<double, double, double, int, bool>(),
            py::arg("amplitude"), py::arg("frequency"), py::arg("phase"),
            py::arg("size"), py::arg("is_time_based") = true,
            doc.Sine.ctor.doc_5args)
        .def(py::init<const Eigen::Ref<const VectorXd>&,
                 const Eigen::Ref<const VectorXd>&,
                 const Eigen::Ref<const VectorXd>&, bool>(),
            py::arg("amplitudes"), py::arg("frequencies"), py::arg("phases"),
            py::arg("is_time_based") = true, doc.Sine.ctor.doc_4args);

    DefineTemplateClassWithDefault<Integrator<T>, LeafSystem<T>>(
        m, "Integrator", GetPyParam<T>(), doc.Integrator.doc)
        .def(py::init<int>(), py::arg("size"),
            doc.Integrator.ctor.doc_1args_size)
        .def(py::init<const VectorXd&>(), py::arg("initial_value"),
            doc.Integrator.ctor.doc_1args_initial_value)
        .def("set_default_integral_value",
            &Integrator<T>::set_default_integral_value,
            py::arg("initial_value"),
            doc.Integrator.set_default_integral_value.doc)
        .def("set_integral_value", &Integrator<T>::set_integral_value,
            py::arg("context"), py::arg("value"),
            doc.Integrator.set_integral_value.doc);

    DefineTemplateClassWithDefault<LinearSystem<T>, AffineSystem<T>>(
        m, "LinearSystem", GetPyParam<T>(), doc.LinearSystem.doc)
        .def(py::init<const Eigen::Ref<const MatrixXd>&,
                 const Eigen::Ref<const MatrixXd>&,
                 const Eigen::Ref<const MatrixXd>&,
                 const Eigen::Ref<const MatrixXd>&, double>(),
            py::arg("A") = Eigen::MatrixXd(), py::arg("B") = Eigen::MatrixXd(),
            py::arg("C") = Eigen::MatrixXd(), py::arg("D") = Eigen::MatrixXd(),
            py::arg("time_period") = 0.0, doc.LinearSystem.ctor.doc_5args);

    DefineTemplateClassWithDefault<MatrixGain<T>, LinearSystem<T>>(
        m, "MatrixGain", GetPyParam<T>(), doc.MatrixGain.doc)
        .def(py::init<const Eigen::Ref<const MatrixXd>&>(), py::arg("D"),
            doc.MatrixGain.ctor.doc_1args_D);

    DefineTemplateClassWithDefault<Multiplexer<T>, LeafSystem<T>>(
        m, "Multiplexer", GetPyParam<T>(), doc.Multiplexer.doc)
        .def(py::init<int>(), py::arg("num_scalar_inputs"),
            doc.Multiplexer.ctor.doc_1args_num_scalar_inputs)
        .def(py::init<std::vector<int>>(), py::arg("input_sizes"),
            doc.Multiplexer.ctor.doc_1args_input_sizes)
        .def(py::init<const BasicVector<T>&>(), py::arg("model_vector"),
            doc.Multiplexer.ctor.doc_1args_model_vector);

    PLACEHOLDER_TOO_LARGE