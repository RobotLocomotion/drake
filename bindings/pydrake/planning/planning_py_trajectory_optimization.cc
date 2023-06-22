#include "drake/bindings/pydrake/common/deprecation_pybind.h"
#include "drake/bindings/pydrake/documentation_pybind.h"
#include "drake/bindings/pydrake/geometry/optimization_pybind.h"
#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/bindings/pydrake/symbolic_types_pybind.h"
#include "drake/planning/trajectory_optimization/direct_collocation.h"
#include "drake/planning/trajectory_optimization/direct_transcription.h"
#include "drake/planning/trajectory_optimization/gcs_trajectory_optimization.h"
#include "drake/planning/trajectory_optimization/kinematic_trajectory_optimization.h"

namespace drake {
namespace pydrake {
namespace internal {

template <typename C>
void RegisterAddConstraintToAllKnotPoints(
    py::class_<planning::trajectory_optimization::MultipleShooting>* cls) {
  using drake::planning::trajectory_optimization::MultipleShooting;
  constexpr auto& doc = pydrake_doc.drake.planning.trajectory_optimization;
  cls->def(
      "AddConstraintToAllKnotPoints",
      [](MultipleShooting* self, std::shared_ptr<C> constraint,
          const Eigen::Ref<const VectorX<symbolic::Variable>>& vars)
          -> std::vector<solvers::Binding<C>> {
        return self->AddConstraintToAllKnotPoints<C>(constraint, vars);
      },
      py::arg("constraint"), py::arg("vars"),
      doc.MultipleShooting.AddConstraintToAllKnotPoints.doc_shared_ptr);
}

void DefinePlanningTrajectoryOptimization(py::module m) {
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::planning::trajectory_optimization;
  constexpr auto& doc = pydrake_doc.drake.planning.trajectory_optimization;

  using solvers::MathematicalProgram;
  using solvers::MatrixXDecisionVariable;
  using solvers::VectorXDecisionVariable;

  // TODO(jwnimmer-tri) We should probably do all importing in our
  // planning_py.cc rather than in our helper functions. That will probably be
  // easier to topo-sort.
  py::module::import("pydrake.symbolic");
  py::module::import("pydrake.systems.framework");
  py::module::import("pydrake.systems.primitives");
  py::module::import("pydrake.solvers");

  {
    using Class = MultipleShooting;
    constexpr auto& cls_doc = doc.MultipleShooting;
    py::class_<Class> cls(m, "MultipleShooting", cls_doc.doc);
    cls  // BR
        .def("time", &Class::time, cls_doc.time.doc)
        .def("prog", overload_cast_explicit<MathematicalProgram&>(&Class::prog),
            py_rvp::reference_internal, cls_doc.prog.doc)
        .def("time_step", &Class::time_step, py::arg("index"),
            cls_doc.time_step.doc)
        .def("fixed_time_step", &Class::fixed_time_step,
            cls_doc.fixed_time_step.doc)
        // TODO(eric.cousineau): The original bindings returned references
        // instead of copies using VectorXBlock. Restore this once dtype=custom
        // is resolved.
        .def(
            "state",
            [](const Class& self) -> VectorXDecisionVariable {
              return self.state();
            },
            cls_doc.state.doc_0args)
        .def(
            "state",
            [](const Class& self, int index) -> VectorXDecisionVariable {
              return self.state(index);
            },
            py::arg("index"), cls_doc.state.doc_1args)
        .def(
            "initial_state",
            [](const Class& self) -> VectorXDecisionVariable {
              return self.initial_state();
            },
            cls_doc.initial_state.doc)
        .def(
            "final_state",
            [](const Class& self) -> VectorXDecisionVariable {
              return self.final_state();
            },
            cls_doc.final_state.doc)
        .def(
            "input",
            [](const Class& self) -> VectorXDecisionVariable {
              return self.input();
            },
            cls_doc.input.doc_0args)
        .def(
            "input",
            [](const Class& self, int index) -> VectorXDecisionVariable {
              return self.input(index);
            },
            py::arg("index"), cls_doc.input.doc_1args)
        .def(
            "NewSequentialVariable",
            [](Class& self, int rows,
                const std::string& name) -> VectorXDecisionVariable {
              return self.NewSequentialVariable(rows, name);
            },
            py::arg("rows"), py::arg("name"), cls_doc.NewSequentialVariable.doc)
        .def(
            "GetSequentialVariableAtIndex",
            [](const Class& self, const std::string& name,
                int index) -> VectorXDecisionVariable {
              return self.GetSequentialVariableAtIndex(name, index);
            },
            py::arg("name"), py::arg("index"),
            cls_doc.GetSequentialVariableAtIndex.doc)
        .def(
            "AddRunningCost",
            [](Class& prog, const symbolic::Expression& g) {
              prog.AddRunningCost(g);
            },
            py::arg("g"), cls_doc.AddRunningCost.doc_1args_g)
        .def(
            "AddRunningCost",
            [](Class& prog,
                const Eigen::Ref<const MatrixX<symbolic::Expression>>& g) {
              prog.AddRunningCost(g);
            },
            py::arg("g"), cls_doc.AddRunningCost.doc_1args_constEigenMatrixBase)
        .def(
            "AddConstraintToAllKnotPoints",
            [](Class* self, const symbolic::Formula& f) {
              return self->AddConstraintToAllKnotPoints(f);
            },
            py::arg("f"), cls_doc.AddConstraintToAllKnotPoints.doc)
        .def(
            "AddConstraintToAllKnotPoints",
            [](Class* self,
                const Eigen::Ref<const VectorX<symbolic::Formula>>& f) {
              return self->AddConstraintToAllKnotPoints(f);
            },
            py::arg("f"), cls_doc.AddConstraintToAllKnotPoints.doc_formulas)
        .def("AddTimeIntervalBounds", &Class::AddTimeIntervalBounds,
            py::arg("lower_bound"), py::arg("upper_bound"),
            cls_doc.AddTimeIntervalBounds.doc)
        .def("AddEqualTimeIntervalsConstraints",
            &Class::AddEqualTimeIntervalsConstraints,
            cls_doc.AddEqualTimeIntervalsConstraints.doc)
        .def("AddDurationBounds", &Class::AddDurationBounds,
            py::arg("lower_bound"), py::arg("upper_bound"),
            cls_doc.AddDurationBounds.doc)
        .def("AddFinalCost",
            py::overload_cast<const symbolic::Expression&>(
                &Class::AddFinalCost),
            py::arg("e"), cls_doc.AddFinalCost.doc_1args_e)
        .def("AddFinalCost",
            py::overload_cast<
                const Eigen::Ref<const MatrixX<symbolic::Expression>>&>(
                &Class::AddFinalCost),
            py::arg("matrix"), cls_doc.AddFinalCost.doc_1args_matrix)
        .def("AddInputTrajectoryCallback", &Class::AddInputTrajectoryCallback,
            py::arg("callback"), cls_doc.AddInputTrajectoryCallback.doc)
        .def("AddStateTrajectoryCallback", &Class::AddStateTrajectoryCallback,
            py::arg("callback"), cls_doc.AddStateTrajectoryCallback.doc)
        .def("AddCompleteTrajectoryCallback",
            &Class::AddCompleteTrajectoryCallback, py::arg("callback"),
            py::arg("names"), cls_doc.AddCompleteTrajectoryCallback.doc)
        .def("SetInitialTrajectory", &Class::SetInitialTrajectory,
            py::arg("traj_init_u"), py::arg("traj_init_x"),
            cls_doc.SetInitialTrajectory.doc)
        .def("GetSampleTimes",
            overload_cast_explicit<Eigen::VectorXd,
                const solvers::MathematicalProgramResult&>(
                &Class::GetSampleTimes),
            py::arg("result"), cls_doc.GetSampleTimes.doc_1args_h_var_values)
        .def("GetInputSamples",
            overload_cast_explicit<Eigen::MatrixXd,
                const solvers::MathematicalProgramResult&>(
                &Class::GetInputSamples),
            py::arg("result"), cls_doc.GetInputSamples.doc)
        .def("GetStateSamples",
            overload_cast_explicit<Eigen::MatrixXd,
                const solvers::MathematicalProgramResult&>(
                &Class::GetStateSamples),
            py::arg("result"), cls_doc.GetStateSamples.doc)
        .def("GetSequentialVariableSamples",
            overload_cast_explicit<Eigen::MatrixXd,
                const solvers::MathematicalProgramResult&, const std::string&>(
                &Class::GetSequentialVariableSamples),
            py::arg("result"), py::arg("name"),
            cls_doc.GetSequentialVariableSamples.doc)
        .def("ReconstructInputTrajectory",
            overload_cast_explicit<trajectories::PiecewisePolynomial<double>,
                const solvers::MathematicalProgramResult&>(
                &Class::ReconstructInputTrajectory),
            py::arg("result"), cls_doc.ReconstructInputTrajectory.doc)
        .def("ReconstructStateTrajectory",
            overload_cast_explicit<trajectories::PiecewisePolynomial<double>,
                const solvers::MathematicalProgramResult&>(
                &Class::ReconstructStateTrajectory),
            py::arg("result"), cls_doc.ReconstructStateTrajectory.doc);

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
    cls.def("timestep",
           WrapDeprecated(cls_doc.timestep.doc_deprecated, &Class::timestep),
           py::arg("index"), cls_doc.timestep.doc_deprecated)
        .def("fixed_timestep",
            WrapDeprecated(
                cls_doc.fixed_timestep.doc_deprecated, &Class::fixed_timestep),
            cls_doc.fixed_timestep.doc_deprecated);
#pragma GCC diagnostic pop

    RegisterAddConstraintToAllKnotPoints<solvers::BoundingBoxConstraint>(&cls);
    RegisterAddConstraintToAllKnotPoints<solvers::LinearEqualityConstraint>(
        &cls);
    RegisterAddConstraintToAllKnotPoints<solvers::LinearConstraint>(&cls);
    RegisterAddConstraintToAllKnotPoints<solvers::Constraint>(&cls);
  }

  {
    using Class = DirectCollocation;
    constexpr auto& cls_doc = doc.DirectCollocation;
    py::class_<Class, MultipleShooting>(m, "DirectCollocation", cls_doc.doc)
        .def(py::init<const systems::System<double>*,
                 const systems::Context<double>&, int, double, double,
                 std::variant<systems::InputPortSelection,
                     systems::InputPortIndex>,
                 bool, solvers::MathematicalProgram*>(),
            py::arg("system"), py::arg("context"), py::arg("num_time_samples"),
            py::arg("minimum_time_step"), py::arg("maximum_time_step"),
            py::arg("input_port_index") =
                systems::InputPortSelection::kUseFirstInputIfItExists,
            py::arg("assume_non_continuous_states_are_fixed") = false,
            py::arg("prog") = nullptr, cls_doc.ctor.doc)
        .def(
            py_init_deprecated<Class, const systems::System<double>*,
                const systems::Context<double>&, int, double, double,
                std::variant<systems::InputPortSelection,
                    systems::InputPortIndex>,
                bool, solvers::MathematicalProgram*>(
                "The arguments minimum_timestep and maximum_timestep have been "
                "renamed to minimum_time_step and maximum_time_step. This "
                "version will be removed on or after 2023-09-01."),
            py::arg("system"), py::arg("context"), py::arg("num_time_samples"),
            py::arg("minimum_timestep"), py::arg("maximum_timestep"),
            py::arg("input_port_index") =
                systems::InputPortSelection::kUseFirstInputIfItExists,
            py::arg("assume_non_continuous_states_are_fixed") = false,
            py::arg("prog") = nullptr, cls_doc.ctor.doc);
  }

  {
    using Class = DirectCollocationConstraint;
    constexpr auto& cls_doc = doc.DirectCollocationConstraint;
    py::class_<Class, solvers::Constraint, std::shared_ptr<Class>>(
        m, "DirectCollocationConstraint", cls_doc.doc)
        .def(py::init<const systems::System<double>&,
                 const systems::Context<double>&,
                 std::variant<systems::InputPortSelection,
                     systems::InputPortIndex>,
                 bool>(),
            py::arg("system"), py::arg("context"),
            py::arg("input_port_index") =
                systems::InputPortSelection::kUseFirstInputIfItExists,
            py::arg("assume_non_continuous_states_are_fixed") = false,
            cls_doc.ctor.doc_double);
  }

  m.def("AddDirectCollocationConstraint", &AddDirectCollocationConstraint,
      py::arg("constraint"), py::arg("time_step"), py::arg("state"),
      py::arg("next_state"), py::arg("input"), py::arg("next_input"),
      py::arg("prog"), doc.AddDirectCollocationConstraint.doc);
  m.def("AddDirectCollocationConstraint",
      WrapDeprecated("Argument timestep has been renamed to time_step. This "
                     "version will be removed on or after 2023-09-01.",
          AddDirectCollocationConstraint),
      py::arg("constraint"), py::arg("timestep"), py::arg("state"),
      py::arg("next_state"), py::arg("input"), py::arg("next_input"),
      py::arg("prog"), doc.AddDirectCollocationConstraint.doc);

  {
    using Class = DirectTranscription;
    constexpr auto& cls_doc = doc.DirectTranscription;
    py::class_<Class, MultipleShooting> cls(
        m, "DirectTranscription", doc.DirectTranscription.doc);

    // Inject the nested TimeStep so it's already bound when we bind the
    // constructor that depends on it. In C++ it's *not* nested. Nesting makes
    // things a bit more pythonic -- better would be kwonly named arg.
    py::class_<TimeStep>(cls, "TimeStep", doc.TimeStep.doc)
        .def(py::init<double>(), py::arg("value"))
        .def_readwrite("value", &TimeStep::value, doc.TimeStep.value.doc);

    cls  // BR
        .def(py::init<const systems::System<double>*,
                 const systems::Context<double>&, int,
                 std::variant<systems::InputPortSelection,
                     systems::InputPortIndex>>(),
            py::arg("system"), py::arg("context"), py::arg("num_time_samples"),
            py::arg("input_port_index") =
                systems::InputPortSelection::kUseFirstInputIfItExists,
            cls_doc.ctor.doc_4args)
        .def(py_init_deprecated<Class, const systems::System<double>*,
                 const systems::Context<double>&, int, TimeStep,
                 std::variant<systems::InputPortSelection,
                     systems::InputPortIndex>>(
                 "Argument fixed_timestep has been renamed to fixed_time_step. "
                 "This version will be removed on or after 2023-09-01."),
            py::arg("system"), py::arg("context"), py::arg("num_time_samples"),
            py::arg("fixed_timestep"),
            py::arg("input_port_index") =
                systems::InputPortSelection::kUseFirstInputIfItExists,
            cls_doc.ctor.doc_5args)
        .def(py::init<const systems::System<double>*,
                 const systems::Context<double>&, int, TimeStep,
                 std::variant<systems::InputPortSelection,
                     systems::InputPortIndex>>(),
            py::arg("system"), py::arg("context"), py::arg("num_time_samples"),
            py::arg("fixed_time_step"),
            py::arg("input_port_index") =
                systems::InputPortSelection::kUseFirstInputIfItExists,
            cls_doc.ctor.doc_5args);
  }

  {
    using Class = KinematicTrajectoryOptimization;
    constexpr auto& cls_doc = doc.KinematicTrajectoryOptimization;
    py::class_<Class>(m, "KinematicTrajectoryOptimization", cls_doc.doc)
        .def(py::init<int, int, int, double>(), py::arg("num_positions"),
            py::arg("num_control_points"), py::arg("spline_order") = 4,
            py::arg("duration") = 1.0, cls_doc.ctor.doc_4args)
        .def(py::init<const trajectories::BsplineTrajectory<double>&>(),
            py::arg("trajectory"), cls_doc.ctor.doc_1args)
        .def("num_positions", &Class::num_positions, cls_doc.num_positions.doc)
        .def("num_control_points", &Class::num_control_points,
            cls_doc.num_control_points.doc)
        .def("basis", &Class::basis, py_rvp::reference_internal,
            cls_doc.basis.doc)
        .def(
            "control_points",
            [](const Class& self) -> MatrixXDecisionVariable {
              return self.control_points();
            },
            cls_doc.control_points.doc)
        .def(
            "duration",
            [](const Class& self) -> symbolic::Variable {
              return self.duration();
            },
            cls_doc.duration.doc)
        .def("prog", &Class::prog, py_rvp::reference_internal, cls_doc.prog.doc)
        .def("get_mutable_prog", &Class::get_mutable_prog,
            py_rvp::reference_internal, cls_doc.get_mutable_prog.doc)
        .def("SetInitialGuess", &Class::SetInitialGuess, py::arg("trajectory"),
            cls_doc.SetInitialGuess.doc)
        .def("ReconstructTrajectory", &Class::ReconstructTrajectory,
            py::arg("result"), cls_doc.ReconstructTrajectory.doc)
        .def("AddPathPositionConstraint",
            py::overload_cast<const Eigen::Ref<const Eigen::VectorXd>&,
                const Eigen::Ref<const Eigen::VectorXd>&, double>(
                &Class::AddPathPositionConstraint),
            py::arg("lb"), py::arg("ub"), py::arg("s"),
            cls_doc.AddPathPositionConstraint.doc_3args)
        .def("AddPathPositionConstraint",
            py::overload_cast<const std::shared_ptr<solvers::Constraint>&,
                double>(&Class::AddPathPositionConstraint),
            py::arg("constraint"), py::arg("s"),
            cls_doc.AddPathPositionConstraint.doc_2args)
        .def("AddPathVelocityConstraint", &Class::AddPathVelocityConstraint,
            py::arg("lb"), py::arg("ub"), py::arg("s"),
            cls_doc.AddPathVelocityConstraint.doc)
        .def("AddVelocityConstraintAtNormalizedTime",
            &Class::AddVelocityConstraintAtNormalizedTime,
            py::arg("constraint"), py::arg("s"),
            cls_doc.AddVelocityConstraintAtNormalizedTime.doc)
        .def("AddPathAccelerationConstraint",
            &Class::AddPathAccelerationConstraint, py::arg("lb"), py::arg("ub"),
            py::arg("s"), cls_doc.AddPathAccelerationConstraint.doc)
        .def("AddDurationConstraint", &Class::AddDurationConstraint,
            py::arg("lb"), py::arg("ub"), cls_doc.AddDurationConstraint.doc)
        .def("AddPositionBounds", &Class::AddPositionBounds, py::arg("lb"),
            py::arg("ub"), cls_doc.AddPositionBounds.doc)
        .def("AddVelocityBounds", &Class::AddVelocityBounds, py::arg("lb"),
            py::arg("ub"), cls_doc.AddVelocityBounds.doc)
        .def("AddAccelerationBounds", &Class::AddAccelerationBounds,
            py::arg("lb"), py::arg("ub"), cls_doc.AddAccelerationBounds.doc)
        .def("AddJerkBounds", &Class::AddJerkBounds, py::arg("lb"),
            py::arg("ub"), cls_doc.AddJerkBounds.doc)
        .def("AddDurationCost", &Class::AddDurationCost,
            py::arg("weight") = 1.0, cls_doc.AddDurationCost.doc)
        .def("AddPathLengthCost", &Class::AddPathLengthCost,
            py::arg("weight") = 1.0, py::arg("use_conic_constraint") = false,
            cls_doc.AddPathLengthCost.doc);
  }

  {
    using Class = GcsTrajectoryOptimization;
    constexpr auto& cls_doc = doc.GcsTrajectoryOptimization;
    py::class_<Class> gcs_traj_opt(m, "GcsTrajectoryOptimization", cls_doc.doc);

    // Subgraph
    const auto& subgraph_doc = doc.GcsTrajectoryOptimization.Subgraph;
    py::class_<Class::Subgraph>(gcs_traj_opt, "Subgraph", subgraph_doc.doc)
        .def("name", &Class::Subgraph::name, subgraph_doc.name.doc)
        .def("order", &Class::Subgraph::order, subgraph_doc.order.doc)
        .def("size", &Class::Subgraph::size, subgraph_doc.size.doc)
        .def(
            "regions",
            [](Class::Subgraph* self) {
              std::vector<const geometry::optimization::ConvexSet*> regions;
              for (auto& region : self->regions()) {
                regions.push_back(region.get());
              }
              py::object self_py = py::cast(self, py_rvp::reference);
              return py_keep_alive_iterable<py::list>(regions, self_py);
            },
            subgraph_doc.regions.doc)
        .def("AddTimeCost", &Class::Subgraph::AddTimeCost,
            py::arg("weight") = 1.0, subgraph_doc.AddTimeCost.doc)
        .def("AddPathLengthCost",
            py::overload_cast<const Eigen::MatrixXd&>(
                &Class::Subgraph::AddPathLengthCost),
            py::arg("weight_matrix"),
            subgraph_doc.AddPathLengthCost.doc_1args_weight_matrix)
        .def("AddPathLengthCost",
            py::overload_cast<double>(&Class::Subgraph::AddPathLengthCost),
            py::arg("weight") = 1.0,
            subgraph_doc.AddPathLengthCost.doc_1args_weight)
        .def("AddVelocityBounds", &Class::Subgraph::AddVelocityBounds,
            py::arg("lb"), py::arg("ub"), subgraph_doc.AddVelocityBounds.doc);

    // EdgesBetweenSubgraphs
    const auto& subgraph_edges_doc =
        doc.GcsTrajectoryOptimization.EdgesBetweenSubgraphs;
    py::class_<Class::EdgesBetweenSubgraphs>(
        gcs_traj_opt, "EdgesBetweenSubgraphs", subgraph_edges_doc.doc)
        .def("AddVelocityBounds",
            &Class::EdgesBetweenSubgraphs::AddVelocityBounds, py::arg("lb"),
            py::arg("ub"), subgraph_edges_doc.AddVelocityBounds.doc);

    gcs_traj_opt  // BR
        .def(py::init<int>(), py::arg("num_positions"), cls_doc.ctor.doc)
        .def("num_positions", &Class::num_positions, cls_doc.num_positions.doc)
        .def("GetGraphvizString", &Class::GetGraphvizString,
            py::arg("result") = std::nullopt, py::arg("show_slack") = true,
            py::arg("precision") = 3, py::arg("scientific") = false,
            cls_doc.GetGraphvizString.doc)
        .def(
            "AddRegions",
            [](Class& self,
                const std::vector<geometry::optimization::ConvexSet*>& regions,
                const std::vector<std::pair<int, int>>& edges_between_regions,
                int order, double h_min, double h_max,
                std::string name) -> Class::Subgraph& {
              return self.AddRegions(CloneConvexSets(regions),
                  edges_between_regions, order, h_min, h_max, std::move(name));
            },
            py_rvp::reference_internal, py::arg("regions"),
            py::arg("edges_between_regions"), py::arg("order"),
            py::arg("h_min") = 1e-6, py::arg("h_max") = 20,
            py::arg("name") = "", cls_doc.AddRegions.doc_6args)
        .def(
            "AddRegions",
            [](Class& self,
                const std::vector<geometry::optimization::ConvexSet*>& regions,
                int order, double h_min, double h_max,
                std::string name) -> Class::Subgraph& {
              return self.AddRegions(CloneConvexSets(regions), order, h_min,
                  h_max, std::move(name));
            },
            py_rvp::reference_internal, py::arg("regions"), py::arg("order"),
            py::arg("h_min") = 1e-6, py::arg("h_max") = 20,
            py::arg("name") = "", cls_doc.AddRegions.doc_5args)
        .def("AddEdges", &Class::AddEdges, py_rvp::reference_internal,
            py::arg("from_subgraph"), py::arg("to_subgraph"),
            py::arg("subspace") = py::none(), cls_doc.AddEdges.doc)
        .def("AddTimeCost", &Class::AddTimeCost, py::arg("weight") = 1.0,
            cls_doc.AddTimeCost.doc)
        .def("AddPathLengthCost",
            py::overload_cast<const Eigen::MatrixXd&>(
                &Class::AddPathLengthCost),
            py::arg("weight_matrix"),
            cls_doc.AddPathLengthCost.doc_1args_weight_matrix)
        .def("AddPathLengthCost",
            py::overload_cast<double>(&Class::AddPathLengthCost),
            py::arg("weight") = 1.0, cls_doc.AddPathLengthCost.doc_1args_weight)
        .def("AddVelocityBounds", &Class::AddVelocityBounds, py::arg("lb"),
            py::arg("ub"), cls_doc.AddVelocityBounds.doc)
        .def("SolvePath", &Class::SolvePath, py::arg("source"),
            py::arg("target"),
            py::arg("options") =
                geometry::optimization::GraphOfConvexSetsOptions(),
            cls_doc.SolvePath.doc);
  }
}

}  // namespace internal
}  // namespace pydrake
}  // namespace drake
