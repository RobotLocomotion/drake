#include "pybind11/eigen.h"
#include "pybind11/functional.h"
#include "pybind11/pybind11.h"
#include "pybind11/stl.h"

#include "drake/bindings/pydrake/documentation_pybind.h"
#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/bindings/pydrake/symbolic_types_pybind.h"
#include "drake/systems/trajectory_optimization/direct_collocation.h"
#include "drake/systems/trajectory_optimization/direct_transcription.h"

namespace drake {
namespace pydrake {

PYBIND11_MODULE(trajectory_optimization, m) {
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::systems::trajectory_optimization;
  constexpr auto& doc = pydrake_doc.drake.systems.trajectory_optimization;

  using solvers::VectorXDecisionVariable;

  py::module::import("pydrake.symbolic");
  py::module::import("pydrake.systems.framework");
  py::module::import("pydrake.systems.primitives");
  py::module::import("pydrake.solvers.mathematicalprogram");

  py::class_<MultipleShooting, solvers::MathematicalProgram>(
      m, "MultipleShooting", doc.MultipleShooting.doc)
      .def("time", &MultipleShooting::time, doc.MultipleShooting.time.doc)
      .def("timestep", &MultipleShooting::timestep, py::arg("index"),
          doc.MultipleShooting.timestep.doc)
      .def("fixed_timestep", &MultipleShooting::fixed_timestep,
          doc.MultipleShooting.fixed_timestep.doc)
      // TODO(eric.cousineau): The original bindings returned references
      // instead of copies using VectorXBlock. Restore this once dtype=custom
      // is resolved.
      .def(
          "state",
          [](const MultipleShooting& self) -> VectorXDecisionVariable {
            return self.state();
          },
          doc.MultipleShooting.state.doc_0args)
      .def(
          "state",
          [](const MultipleShooting& self, int index)
              -> VectorXDecisionVariable { return self.state(index); },
          py::arg("index"), doc.MultipleShooting.state.doc_1args)
      .def(
          "initial_state",
          [](const MultipleShooting& self) -> VectorXDecisionVariable {
            return self.initial_state();
          },
          doc.MultipleShooting.initial_state.doc)
      .def(
          "final_state",
          [](const MultipleShooting& self) -> VectorXDecisionVariable {
            return self.final_state();
          },
          doc.MultipleShooting.final_state.doc)
      .def(
          "input",
          [](const MultipleShooting& self) -> VectorXDecisionVariable {
            return self.input();
          },
          doc.MultipleShooting.input.doc_0args)
      .def(
          "input",
          [](const MultipleShooting& self, int index)
              -> VectorXDecisionVariable { return self.input(index); },
          py::arg("index"), doc.MultipleShooting.input.doc_1args)
      .def(
          "NewSequentialVariable",
          [](MultipleShooting& self, int rows,
              const std::string& name) -> VectorXDecisionVariable {
            return self.NewSequentialVariable(rows, name);
          },
          py::arg("rows"), py::arg("name"),
          doc.MultipleShooting.NewSequentialVariable.doc)
      .def(
          "GetSequentialVariableAtIndex",
          [](const MultipleShooting& self, const std::string& name,
              int index) -> VectorXDecisionVariable {
            return self.GetSequentialVariableAtIndex(name, index);
          },
          py::arg("name"), py::arg("index"),
          doc.MultipleShooting.GetSequentialVariableAtIndex.doc)
      .def(
          "AddRunningCost",
          [](MultipleShooting& prog, const symbolic::Expression& g) {
            prog.AddRunningCost(g);
          },
          py::arg("g"), doc.MultipleShooting.AddRunningCost.doc_1args_g)
      .def(
          "AddRunningCost",
          [](MultipleShooting& prog,
              const Eigen::Ref<const MatrixX<symbolic::Expression>>& g) {
            prog.AddRunningCost(g);
          },
          py::arg("g"),
          doc.MultipleShooting.AddRunningCost.doc_1args_constEigenMatrixBase)
      .def("AddConstraintToAllKnotPoints",
          &MultipleShooting::AddConstraintToAllKnotPoints, py::arg("f"),
          doc.MultipleShooting.AddConstraintToAllKnotPoints.doc)
      .def("AddTimeIntervalBounds", &MultipleShooting::AddTimeIntervalBounds,
          py::arg("lower_bound"), py::arg("upper_bound"),
          doc.MultipleShooting.AddTimeIntervalBounds.doc)
      .def("AddEqualTimeIntervalsConstraints",
          &MultipleShooting::AddEqualTimeIntervalsConstraints,
          doc.MultipleShooting.AddEqualTimeIntervalsConstraints.doc)
      .def("AddDurationBounds", &MultipleShooting::AddDurationBounds,
          py::arg("lower_bound"), py::arg("upper_bound"),
          doc.MultipleShooting.AddDurationBounds.doc)
      .def("AddFinalCost",
          py::overload_cast<const symbolic::Expression&>(
              &MultipleShooting::AddFinalCost),
          py::arg("e"), doc.MultipleShooting.AddFinalCost.doc_1args_e)
      .def("AddFinalCost",
          py::overload_cast<
              const Eigen::Ref<const MatrixX<symbolic::Expression>>&>(
              &MultipleShooting::AddFinalCost),
          py::arg("matrix"), doc.MultipleShooting.AddFinalCost.doc_1args_matrix)
      .def("AddInputTrajectoryCallback",
          &MultipleShooting::AddInputTrajectoryCallback, py::arg("callback"),
          doc.MultipleShooting.AddInputTrajectoryCallback.doc)
      .def("AddStateTrajectoryCallback",
          &MultipleShooting::AddStateTrajectoryCallback, py::arg("callback"),
          doc.MultipleShooting.AddStateTrajectoryCallback.doc)
      .def("AddCompleteTrajectoryCallback",
          &MultipleShooting::AddCompleteTrajectoryCallback, py::arg("callback"),
          py::arg("names"),
          doc.MultipleShooting.AddCompleteTrajectoryCallback.doc)
      .def("SetInitialTrajectory", &MultipleShooting::SetInitialTrajectory,
          py::arg("traj_init_u"), py::arg("traj_init_x"),
          doc.MultipleShooting.SetInitialTrajectory.doc)
      .def("GetSampleTimes",
          overload_cast_explicit<Eigen::VectorXd,
              const solvers::MathematicalProgramResult&>(
              &MultipleShooting::GetSampleTimes),
          py::arg("result"),
          doc.MultipleShooting.GetSampleTimes.doc_1args_h_var_values)
      .def("GetInputSamples",
          overload_cast_explicit<Eigen::MatrixXd,
              const solvers::MathematicalProgramResult&>(
              &MultipleShooting::GetInputSamples),
          py::arg("result"), doc.MultipleShooting.GetInputSamples.doc)
      .def("GetStateSamples",
          overload_cast_explicit<Eigen::MatrixXd,
              const solvers::MathematicalProgramResult&>(
              &MultipleShooting::GetStateSamples),
          py::arg("result"), doc.MultipleShooting.GetStateSamples.doc)
      .def("GetSequentialVariableSamples",
          overload_cast_explicit<Eigen::MatrixXd,
              const solvers::MathematicalProgramResult&, const std::string&>(
              &MultipleShooting::GetSequentialVariableSamples),
          py::arg("result"), py::arg("name"),
          doc.MultipleShooting.GetSequentialVariableSamples.doc)
      .def("ReconstructInputTrajectory",
          overload_cast_explicit<trajectories::PiecewisePolynomial<double>,
              const solvers::MathematicalProgramResult&>(
              &MultipleShooting::ReconstructInputTrajectory),
          py::arg("result"),
          doc.MultipleShooting.ReconstructInputTrajectory.doc)
      .def("ReconstructStateTrajectory",
          overload_cast_explicit<trajectories::PiecewisePolynomial<double>,
              const solvers::MathematicalProgramResult&>(
              &MultipleShooting::ReconstructStateTrajectory),
          py::arg("result"),
          doc.MultipleShooting.ReconstructStateTrajectory.doc);

  py::class_<DirectCollocation, MultipleShooting>(
      m, "DirectCollocation", doc.DirectCollocation.doc)
      .def(py::init<const systems::System<double>*,
               const systems::Context<double>&, int, double, double,
               std::variant<systems::InputPortSelection,
                   systems::InputPortIndex>,
               bool>(),
          py::arg("system"), py::arg("context"), py::arg("num_time_samples"),
          py::arg("minimum_timestep"), py::arg("maximum_timestep"),
          py::arg("input_port_index") =
              systems::InputPortSelection::kUseFirstInputIfItExists,
          py::arg("assume_non_continuous_states_are_fixed") = false,
          doc.DirectCollocation.ctor.doc);

  py::class_<DirectCollocationConstraint, solvers::Constraint,
      std::shared_ptr<DirectCollocationConstraint>>(
      m, "DirectCollocationConstraint", doc.DirectCollocationConstraint.doc)
      .def(py::init<const systems::System<double>&,
               const systems::Context<double>&,
               std::variant<systems::InputPortSelection,
                   systems::InputPortIndex>,
               bool>(),
          py::arg("system"), py::arg("context"),
          py::arg("input_port_index") =
              systems::InputPortSelection::kUseFirstInputIfItExists,
          py::arg("assume_non_continuous_states_are_fixed") = false,
          doc.DirectCollocationConstraint.ctor.doc);

  m.def("AddDirectCollocationConstraint", &AddDirectCollocationConstraint,
      py::arg("constraint"), py::arg("timestep"), py::arg("state"),
      py::arg("next_state"), py::arg("input"), py::arg("next_input"),
      py::arg("prog"), doc.AddDirectCollocationConstraint.doc);

  // TimeStep
  py::class_<TimeStep>(m, "TimeStep", doc.TimeStep.doc)
      .def(py::init<double>(), py::arg("value"))
      .def_readwrite("value", &TimeStep::value, doc.TimeStep.value.doc);

  py::class_<DirectTranscription, MultipleShooting>(
      m, "DirectTranscription", doc.DirectTranscription.doc)
      .def(py::init<const systems::System<double>*,
               const systems::Context<double>&, int,
               std::variant<systems::InputPortSelection,
                   systems::InputPortIndex>>(),
          py::arg("system"), py::arg("context"), py::arg("num_time_samples"),
          py::arg("input_port_index") =
              systems::InputPortSelection::kUseFirstInputIfItExists,
          doc.DirectTranscription.ctor.doc_4args)
      .def(py::init<const systems::System<double>*,
               const systems::Context<double>&, int, TimeStep,
               std::variant<systems::InputPortSelection,
                   systems::InputPortIndex>>(),
          py::arg("system"), py::arg("context"), py::arg("num_time_samples"),
          py::arg("fixed_timestep"),
          py::arg("input_port_index") =
              systems::InputPortSelection::kUseFirstInputIfItExists,
          doc.DirectTranscription.ctor.doc_5args);
}

}  // namespace pydrake
}  // namespace drake
