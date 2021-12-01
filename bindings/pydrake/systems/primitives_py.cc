#include "pybind11/eigen.h"
#include "pybind11/pybind11.h"
#include "pybind11/stl.h"

#include "drake/bindings/pydrake/common/cpp_template_pybind.h"
#include "drake/bindings/pydrake/common/default_scalars_pybind.h"
#include "drake/bindings/pydrake/documentation_pybind.h"
#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/systems/primitives/adder.h"
#include "drake/systems/primitives/affine_system.h"
#include "drake/systems/primitives/barycentric_system.h"
#include "drake/systems/primitives/constant_value_source.h"
#include "drake/systems/primitives/constant_vector_source.h"
#include "drake/systems/primitives/demultiplexer.h"
#include "drake/systems/primitives/discrete_derivative.h"
#include "drake/systems/primitives/discrete_time_delay.h"
#include "drake/systems/primitives/first_order_low_pass_filter.h"
#include "drake/systems/primitives/gain.h"
#include "drake/systems/primitives/integrator.h"
#include "drake/systems/primitives/linear_system.h"
#include "drake/systems/primitives/linear_transform_density.h"
#include "drake/systems/primitives/matrix_gain.h"
#include "drake/systems/primitives/multiplexer.h"
#include "drake/systems/primitives/pass_through.h"
#include "drake/systems/primitives/random_source.h"
#include "drake/systems/primitives/saturation.h"
#include "drake/systems/primitives/sine.h"
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

PYBIND11_MODULE(primitives, m) {
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::systems;

  m.doc() = "Bindings for the primitives portion of the Systems framework.";
  constexpr auto& doc = pydrake_doc.drake.systems;

  py::module::import("pydrake.systems.framework");
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
            py::arg("A"), py::arg("B"), py::arg("f0"), py::arg("C"),
            py::arg("D"), py::arg("y0"), py::arg("time_period") = 0.0,
            doc.AffineSystem.ctor.doc_7args)
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
        // Wrap a few methods from the TimeVaryingAffineSystem parent class.
        // TODO(russt): Move to TimeVaryingAffineSystem if/when that class is
        // wrapped.
        .def("time_period", &AffineSystem<T>::time_period,
            doc.TimeVaryingAffineSystem.time_period.doc)
        .def("configure_default_state",
            &TimeVaryingAffineSystem<T>::configure_default_state, py::arg("x0"),
            doc.TimeVaryingAffineSystem.configure_default_state.doc)
        .def("configure_random_state",
            &TimeVaryingAffineSystem<T>::configure_random_state,
            py::arg("covariance"),
            doc.TimeVaryingAffineSystem.configure_random_state.doc);

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
            py::arg("delay_timesteps"), py::arg("vector_size"),
            doc.DiscreteTimeDelay.ctor
                .doc_3args_update_sec_delay_timesteps_vector_size)
        .def(py::init<double, int, const AbstractValue&>(),
            py::arg("update_sec"), py::arg("delay_timesteps"),
            py::arg("abstract_model_value"),
            doc.DiscreteTimeDelay.ctor
                .doc_3args_update_sec_delay_timesteps_abstract_model_value);

    DefineTemplateClassWithDefault<DiscreteDerivative<T>, LeafSystem<T>>(
        m, "DiscreteDerivative", GetPyParam<T>(), doc.DiscreteDerivative.doc)
        .def(py::init<int, double, bool>(), py::arg("num_inputs"),
            py::arg("time_step"), py::arg("suppress_initial_transient") = false,
            doc.DiscreteDerivative.ctor.doc)
        .def("time_step", &DiscreteDerivative<T>::time_step,
            doc.DiscreteDerivative.time_step.doc)
        .def("suppress_initial_transient",
            &DiscreteDerivative<T>::suppress_initial_transient,
            doc.DiscreteDerivative.suppress_initial_transient.doc);

    DefineTemplateClassWithDefault<                  // BR
        FirstOrderLowPassFilter<T>, LeafSystem<T>>(  //
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
        .def(py::init<int>(), doc.Integrator.ctor.doc)
        .def("set_integral_value", &Integrator<T>::set_integral_value,
            py::arg("context"), py::arg("value"),
            doc.Integrator.set_integral_value.doc);

    DefineTemplateClassWithDefault<LinearSystem<T>, AffineSystem<T>>(
        m, "LinearSystem", GetPyParam<T>(), doc.LinearSystem.doc)
        .def(py::init<const Eigen::Ref<const MatrixXd>&,
                 const Eigen::Ref<const MatrixXd>&,
                 const Eigen::Ref<const MatrixXd>&,
                 const Eigen::Ref<const MatrixXd>&, double>(),
            py::arg("A"), py::arg("B"), py::arg("C"), py::arg("D"),
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

    DefineTemplateClassWithDefault<PassThrough<T>, LeafSystem<T>>(
        m, "PassThrough", GetPyParam<T>(), doc.PassThrough.doc)
        .def(py::init<int>(), py::arg("vector_size"),
            doc.PassThrough.ctor.doc_1args_vector_size)
        .def(py::init<const Eigen::Ref<const VectorXd>&>(), py::arg("value"),
            doc.PassThrough.ctor.doc_1args_value)
        .def(py::init<const AbstractValue&>(), py::arg("abstract_model_value"),
            doc.PassThrough.ctor.doc_1args_abstract_model_value);

    DefineTemplateClassWithDefault<Saturation<T>, LeafSystem<T>>(
        m, "Saturation", GetPyParam<T>(), doc.Saturation.doc)
        .def(py::init<const VectorX<T>&, const VectorX<T>&>(),
            py::arg("min_value"), py::arg("max_value"),
            doc.Saturation.ctor.doc_2args);

    DefineTemplateClassWithDefault<StateInterpolatorWithDiscreteDerivative<T>,
        Diagram<T>>(m, "StateInterpolatorWithDiscreteDerivative",
        GetPyParam<T>(), doc.StateInterpolatorWithDiscreteDerivative.doc)
        .def(py::init<int, double, bool>(), py::arg("num_positions"),
            py::arg("time_step"), py::arg("suppress_initial_transient") = false,
            doc.StateInterpolatorWithDiscreteDerivative.ctor.doc)
        .def("suppress_initial_transient",
            &StateInterpolatorWithDiscreteDerivative<
                T>::suppress_initial_transient,
            doc.StateInterpolatorWithDiscreteDerivative
                .suppress_initial_transient.doc)
        .def(
            "set_initial_position",
            [](const StateInterpolatorWithDiscreteDerivative<T>* self,
                Context<T>* context,
                const Eigen::Ref<const VectorX<T>>& position) {
              self->set_initial_position(context, position);
            },
            py::arg("context"), py::arg("position"),
            doc.StateInterpolatorWithDiscreteDerivative.set_initial_position
                .doc_2args_context_position)
        .def(
            "set_initial_position",
            [](const StateInterpolatorWithDiscreteDerivative<T>* self,
                State<T>* state, const Eigen::Ref<const VectorX<T>>& position) {
              self->set_initial_position(state, position);
            },
            py::arg("state"), py::arg("position"),
            doc.StateInterpolatorWithDiscreteDerivative.set_initial_position
                .doc_2args_state_position);

    DefineTemplateClassWithDefault<SymbolicVectorSystem<T>, LeafSystem<T>>(m,
        "SymbolicVectorSystem", GetPyParam<T>(), doc.SymbolicVectorSystem.doc)
        .def(py::init<std::optional<Variable>, VectorX<Variable>,
                 VectorX<Variable>, VectorX<Expression>, VectorX<Expression>,
                 double>(),
            py::arg("time") = std::nullopt,
            py::arg("state") = Vector0<Variable>{},
            py::arg("input") = Vector0<Variable>{},
            py::arg("dynamics") = Vector0<Expression>{},
            py::arg("output") = Vector0<Expression>{},
            py::arg("time_period") = 0.0,
            doc.SymbolicVectorSystem.ctor.doc_6args)
        .def(py::init<std::optional<Variable>, VectorX<Variable>,
                 VectorX<Variable>, VectorX<Variable>, VectorX<Expression>,
                 VectorX<Expression>, double>(),
            py::arg("time") = std::nullopt,
            py::arg("state") = Vector0<Variable>{},
            py::arg("input") = Vector0<Variable>{},
            py::arg("parameter") = Vector0<Variable>{},
            py::arg("dynamics") = Vector0<Expression>{},
            py::arg("output") = Vector0<Expression>{},
            py::arg("time_period") = 0.0,
            doc.SymbolicVectorSystem.ctor.doc_7args)
        .def("dynamics_for_variable",
            &SymbolicVectorSystem<T>::dynamics_for_variable, py::arg("var"),
            doc.SymbolicVectorSystem.dynamics_for_variable.doc);

    DefineTemplateClassWithDefault<VectorLog<T>>(
        m, "VectorLog", GetPyParam<T>(), doc.VectorLog.doc)
        .def_property_readonly_static("kDefaultCapacity",
            [](py::object) { return VectorLog<T>::kDefaultCapacity; })
        .def(py::init<int>(), py::arg("input_size"), doc.VectorLog.ctor.doc)
        .def("num_samples", &VectorLog<T>::num_samples,
            doc.VectorLog.num_samples.doc)
        .def(
            "sample_times",
            [](const VectorLog<T>* self) {
              // Reference
              return CopyIfNotPodType(self->sample_times());
            },
            return_value_policy_for_scalar_type<T>(),
            doc.VectorLog.sample_times.doc)
        .def(
            "data",
            [](const VectorLog<T>* self) {
              // Reference.
              return CopyIfNotPodType(self->data());
            },
            return_value_policy_for_scalar_type<T>(), doc.VectorLog.data.doc)
        .def("Clear", &VectorLog<T>::Clear, doc.VectorLog.Clear.doc)
        .def("Reserve", &VectorLog<T>::Reserve, doc.VectorLog.Reserve.doc)
        .def("AddData", &VectorLog<T>::AddData, py::arg("time"),
            py::arg("sample"), doc.VectorLog.AddData.doc)
        .def("get_input_size", &VectorLog<T>::get_input_size,
            doc.VectorLog.get_input_size.doc);

    DefineTemplateClassWithDefault<VectorLogSink<T>, LeafSystem<T>>(
        m, "VectorLogSink", GetPyParam<T>(), doc.VectorLogSink.doc)
        .def(py::init<int, double>(), py::arg("input_size"),
            py::arg("publish_period") = 0.0, doc.VectorLogSink.ctor.doc_2args)
        .def(py::init<int, const TriggerTypeSet&, double>(),
            py::arg("input_size"), py::arg("publish_triggers"),
            py::arg("publish_period") = 0.0, doc.VectorLogSink.ctor.doc_3args)
        .def(
            "GetLog",
            [](const VectorLogSink<T>* self, const Context<T>& context)
                -> const VectorLog<T>& { return self->GetLog(context); },
            py::arg("context"),
            // Keep alive, ownership: `return` keeps `context` alive.
            py::keep_alive<0, 2>(), py_rvp::reference,
            doc.VectorLogSink.GetLog.doc)
        .def(
            "GetMutableLog",
            [](const VectorLogSink<T>* self, Context<T>* context)
                -> VectorLog<T>& { return self->GetMutableLog(context); },
            py::arg("context"),
            // Keep alive, ownership: `return` keeps `context` alive.
            py::keep_alive<0, 2>(), py_rvp::reference,
            doc.VectorLogSink.GetMutableLog.doc)
        .def(
            "FindLog",
            [](const VectorLogSink<T>* self, const Context<T>& context)
                -> const VectorLog<T>& { return self->FindLog(context); },
            py::arg("context"),
            // Keep alive, ownership: `return` keeps `context` alive.
            py::keep_alive<0, 2>(), py_rvp::reference,
            doc.VectorLogSink.FindLog.doc)
        .def(
            "FindMutableLog",
            [](const VectorLogSink<T>* self, Context<T>* context)
                -> VectorLog<T>& { return self->FindMutableLog(context); },
            py::arg("context"),
            // Keep alive, ownership: `return` keeps `context` alive.
            py::keep_alive<0, 2>(), py_rvp::reference,
            doc.VectorLogSink.FindMutableLog.doc);

    m.def("LogVectorOutput",
        py::overload_cast<const OutputPort<T>&, DiagramBuilder<T>*, double>(
            &LogVectorOutput<T>),
        py::arg("src"), py::arg("builder"), py::arg("publish_period") = 0.0,
        // Keep alive, ownership: `return` keeps `builder` alive.
        py::keep_alive<0, 2>(),
        // See #11531 for why `py_rvp::reference` is needed.
        py_rvp::reference, doc.LogVectorOutput.doc_3args);

    m.def("LogVectorOutput",
        py::overload_cast<const OutputPort<T>&, DiagramBuilder<T>*,
            const TriggerTypeSet&, double>(&LogVectorOutput<T>),
        py::arg("src"), py::arg("builder"), py::arg("publish_triggers"),
        py::arg("publish_period") = 0.0,
        // Keep alive, ownership: `return` keeps `builder` alive.
        py::keep_alive<0, 2>(),
        // See #11531 for why `py_rvp::reference` is needed.
        py_rvp::reference, doc.LogVectorOutput.doc_4args);

    DefineTemplateClassWithDefault<WrapToSystem<T>, LeafSystem<T>>(
        m, "WrapToSystem", GetPyParam<T>(), doc.WrapToSystem.doc)
        .def(py::init<int>(), doc.WrapToSystem.ctor.doc)
        .def("set_interval", &WrapToSystem<T>::set_interval,
            doc.WrapToSystem.set_interval.doc);

    DefineTemplateClassWithDefault<ZeroOrderHold<T>, LeafSystem<T>>(
        m, "ZeroOrderHold", GetPyParam<T>(), doc.ZeroOrderHold.doc)
        .def(py::init<double, int>(), py::arg("period_sec"),
            py::arg("vector_size"),
            doc.ZeroOrderHold.ctor.doc_2args_period_sec_vector_size)
        .def(py::init<double, const AbstractValue&>(), py::arg("period_sec"),
            py::arg("abstract_model_value"),
            doc.ZeroOrderHold.ctor.doc_2args_period_sec_abstract_model_value);
  };
  type_visit(bind_common_scalar_types, CommonScalarPack{});

  // N.B. Capturing `&doc` should not be required; workaround per #9600.
  auto bind_non_symbolic_scalar_types = [m, &doc](auto dummy) {
    using T = decltype(dummy);

    DefineTemplateClassWithDefault<LinearTransformDensity<T>, LeafSystem<T>>(m,
        "LinearTransformDensity", GetPyParam<T>(),
        doc.LinearTransformDensity.doc)
        .def(py::init<RandomDistribution, int, int>(), py::arg("distribution"),
            py::arg("input_size"), py::arg("output_size"),
            doc.LinearTransformDensity.ctor.doc)
        .def("get_input_port_w_in",
            &LinearTransformDensity<T>::get_input_port_w_in,
            py_rvp::reference_internal,
            doc.LinearTransformDensity.get_input_port_w_in.doc)
        .def("get_input_port_A", &LinearTransformDensity<T>::get_input_port_A,
            py_rvp::reference_internal,
            doc.LinearTransformDensity.get_input_port_A.doc)
        .def("get_input_port_b", &LinearTransformDensity<T>::get_input_port_b,
            py_rvp::reference_internal,
            doc.LinearTransformDensity.get_input_port_b.doc)
        .def("get_output_port_w_out",
            &LinearTransformDensity<T>::get_output_port_w_out,
            py_rvp::reference_internal,
            doc.LinearTransformDensity.get_output_port_w_out.doc)
        .def("get_output_port_w_out_density",
            &LinearTransformDensity<T>::get_output_port_w_out_density,
            py_rvp::reference_internal,
            doc.LinearTransformDensity.get_output_port_w_out_density.doc)
        .def("get_distribution", &LinearTransformDensity<T>::get_distribution,
            doc.LinearTransformDensity.get_distribution.doc)
        .def("FixConstantA", &LinearTransformDensity<T>::FixConstantA,
            py::arg("context"), py::arg("A"), py_rvp::reference_internal,
            doc.LinearTransformDensity.FixConstantA.doc)
        .def("FixConstantB", &LinearTransformDensity<T>::FixConstantB,
            py::arg("context"), py::arg("b"), py_rvp::reference_internal,
            doc.LinearTransformDensity.FixConstantB.doc)
        .def("CalcDensity", &LinearTransformDensity<T>::CalcDensity,
            py::arg("context"), doc.LinearTransformDensity.CalcDensity.doc);

    DefineTemplateClassWithDefault<TrajectoryAffineSystem<T>, LeafSystem<T>>(m,
        "TrajectoryAffineSystem", GetPyParam<T>(),
        doc.TrajectoryAffineSystem.doc)
        .def(py::init<const trajectories::Trajectory<double>&,
                 const trajectories::Trajectory<double>&,
                 const trajectories::Trajectory<double>&,
                 const trajectories::Trajectory<double>&,
                 const trajectories::Trajectory<double>&,
                 const trajectories::Trajectory<double>&, double>(),
            py::arg("A"), py::arg("B"), py::arg("f0"), py::arg("C"),
            py::arg("D"), py::arg("y0"), py::arg("time_period") = 0.0,
            doc.TrajectoryAffineSystem.ctor.doc)
        .def("A",
            overload_cast_explicit<MatrixX<T>, const T&>(
                &TrajectoryAffineSystem<T>::A),
            doc.TrajectoryAffineSystem.A.doc)
        .def("B",
            overload_cast_explicit<MatrixX<T>, const T&>(
                &TrajectoryAffineSystem<T>::B),
            doc.TrajectoryAffineSystem.B.doc)
        .def("f0",
            overload_cast_explicit<VectorX<T>, const T&>(
                &TrajectoryAffineSystem<T>::f0),
            doc.TrajectoryAffineSystem.f0.doc)
        .def("C",
            overload_cast_explicit<MatrixX<T>, const T&>(
                &TrajectoryAffineSystem<T>::C),
            doc.TrajectoryAffineSystem.C.doc)
        .def("D",
            overload_cast_explicit<MatrixX<T>, const T&>(
                &TrajectoryAffineSystem<T>::D),
            doc.TrajectoryAffineSystem.D.doc)
        .def("y0",
            overload_cast_explicit<VectorX<T>, const T&>(
                &TrajectoryAffineSystem<T>::y0),
            doc.TrajectoryAffineSystem.y0.doc)
        // Wrap a few methods from the TimeVaryingAffineSystem parent class.
        // TODO(russt): Move to TimeVaryingAffineSystem if/when that class is
        // wrapped.
        .def("time_period", &TrajectoryAffineSystem<T>::time_period,
            doc.TimeVaryingAffineSystem.time_period.doc)
        .def("configure_default_state",
            &TimeVaryingAffineSystem<T>::configure_default_state, py::arg("x0"),
            doc.TimeVaryingAffineSystem.configure_default_state.doc)
        .def("configure_random_state",
            &TimeVaryingAffineSystem<T>::configure_random_state,
            py::arg("covariance"),
            doc.TimeVaryingAffineSystem.configure_random_state.doc);

    DefineTemplateClassWithDefault<TrajectoryLinearSystem<T>, LeafSystem<T>>(m,
        "TrajectoryLinearSystem", GetPyParam<T>(),
        doc.TrajectoryLinearSystem.doc)
        .def(py::init<const trajectories::Trajectory<double>&,
                 const trajectories::Trajectory<double>&,
                 const trajectories::Trajectory<double>&,
                 const trajectories::Trajectory<double>&, double>(),
            py::arg("A"), py::arg("B"), py::arg("C"), py::arg("D"),
            py::arg("time_period") = 0.0, doc.TrajectoryLinearSystem.ctor.doc)
        .def("A",
            overload_cast_explicit<MatrixX<T>, const T&>(
                &TrajectoryLinearSystem<T>::A),
            doc.TrajectoryLinearSystem.A.doc)
        .def("B",
            overload_cast_explicit<MatrixX<T>, const T&>(
                &TrajectoryLinearSystem<T>::B),
            doc.TrajectoryLinearSystem.B.doc)
        .def("C",
            overload_cast_explicit<MatrixX<T>, const T&>(
                &TrajectoryLinearSystem<T>::C),
            doc.TrajectoryLinearSystem.C.doc)
        .def("D",
            overload_cast_explicit<MatrixX<T>, const T&>(
                &TrajectoryLinearSystem<T>::D),
            doc.TrajectoryLinearSystem.D.doc)
        // Wrap a few methods from the TimeVaryingAffineSystem parent class.
        // TODO(russt): Move to TimeVaryingAffineSystem if/when that class is
        // wrapped.
        .def("time_period", &TrajectoryAffineSystem<T>::time_period,
            doc.TimeVaryingAffineSystem.time_period.doc)
        .def("configure_default_state",
            &TimeVaryingAffineSystem<T>::configure_default_state, py::arg("x0"),
            doc.TimeVaryingAffineSystem.configure_default_state.doc)
        .def("configure_random_state",
            &TimeVaryingAffineSystem<T>::configure_random_state,
            py::arg("covariance"),
            doc.TimeVaryingAffineSystem.configure_random_state.doc);
  };
  type_visit(bind_non_symbolic_scalar_types, NonSymbolicScalarPack{});

  py::class_<BarycentricMeshSystem<double>, LeafSystem<double>>(
      m, "BarycentricMeshSystem", doc.BarycentricMeshSystem.doc)
      .def(py::init<math::BarycentricMesh<double>,
               const Eigen::Ref<const MatrixX<double>>&>(),
          doc.BarycentricMeshSystem.ctor.doc)
      .def("get_mesh", &BarycentricMeshSystem<double>::get_mesh,
          doc.BarycentricMeshSystem.get_mesh.doc)
      .def("get_output_values",
          &BarycentricMeshSystem<double>::get_output_values,
          doc.BarycentricMeshSystem.get_output_values.doc);

  // TODO(jwnimmer-tri) Add more scalar types bindings for this class.
  py::class_<RandomSource<double>, LeafSystem<double>>(
      m, "RandomSource", doc.RandomSource.doc)
      .def(py::init<RandomDistribution, int, double>(), py::arg("distribution"),
          py::arg("num_outputs"), py::arg("sampling_interval_sec"),
          doc.RandomSource.ctor.doc);

  py::class_<TrajectorySource<double>, LeafSystem<double>>(
      m, "TrajectorySource", doc.TrajectorySource.doc)
      .def(py::init<const trajectories::Trajectory<double>&, int, bool>(),
          py::arg("trajectory"), py::arg("output_derivative_order") = 0,
          py::arg("zero_derivatives_beyond_limits") = true,
          doc.TrajectorySource.ctor.doc);

  m.def("AddRandomInputs", &AddRandomInputs<double>,
       py::arg("sampling_interval_sec"), py::arg("builder"),
       doc.AddRandomInputs.doc)
      .def("AddRandomInputs", &AddRandomInputs<AutoDiffXd>,
          py::arg("sampling_interval_sec"), py::arg("builder"),
          doc.AddRandomInputs.doc);

  m.def("Linearize", &Linearize, py::arg("system"), py::arg("context"),
      py::arg("input_port_index") =
          systems::InputPortSelection::kUseFirstInputIfItExists,
      py::arg("output_port_index") =
          systems::OutputPortSelection::kUseFirstOutputIfItExists,
      py::arg("equilibrium_check_tolerance") = 1e-6, doc.Linearize.doc);

  m.def("FirstOrderTaylorApproximation", &FirstOrderTaylorApproximation,
      py::arg("system"), py::arg("context"),
      py::arg("input_port_index") =
          systems::InputPortSelection::kUseFirstInputIfItExists,
      py::arg("output_port_index") =
          systems::OutputPortSelection::kUseFirstOutputIfItExists,
      doc.FirstOrderTaylorApproximation.doc);

  m.def("ControllabilityMatrix", &ControllabilityMatrix,
      doc.ControllabilityMatrix.doc);

  m.def("IsControllable", &IsControllable, py::arg("sys"),
      py::arg("threshold") = std::nullopt, doc.IsControllable.doc);

  m.def(
      "ObservabilityMatrix", &ObservabilityMatrix, doc.ObservabilityMatrix.doc);

  m.def("IsObservable", &IsObservable, py::arg("sys"),
      py::arg("threshold") = std::nullopt, doc.IsObservable.doc);

  // TODO(eric.cousineau): Add more systems as needed.
}  // NOLINT(readability/fn_size)

}  // namespace pydrake
}  // namespace drake
