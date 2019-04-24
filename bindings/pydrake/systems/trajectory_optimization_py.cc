#include "pybind11/eigen.h"
#include "pybind11/functional.h"
#include "pybind11/pybind11.h"
#include "pybind11/stl.h"

#include "drake/bindings/pydrake/common/deprecation_pybind.h"
#include "drake/bindings/pydrake/common/drake_variant_pybind.h"
#include "drake/bindings/pydrake/documentation_pybind.h"
#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/bindings/pydrake/symbolic_types_pybind.h"
#include "drake/systems/trajectory_optimization/direct_collocation.h"
#include "drake/systems/trajectory_optimization/direct_transcription.h"

namespace drake {
namespace pydrake {

namespace {
constexpr const char* const kSolveDeprecationString =
    "MathematicalProgram methods that assume the solution is stored inside "
    "the program are deprecated; for details and porting advice, see "
    "https://github.com/RobotLocomotion/drake/issues/9633.  This method "
    "will be removed on 2019-06-01.";
}  // namespace

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
      .def("timestep", &MultipleShooting::timestep,
          doc.MultipleShooting.timestep.doc)
      .def("fixed_timestep", &MultipleShooting::fixed_timestep,
          doc.MultipleShooting.fixed_timestep.doc)
      // TODO(eric.cousineau): The original bindings returned references
      // instead of copies using VectorXBlock. Restore this once dtype=custom
      // is resolved.
      .def("state",
          [](const MultipleShooting& self) -> VectorXDecisionVariable {
            return self.state();
          },
          doc.MultipleShooting.state.doc_0args)
      .def("state",
          [](const MultipleShooting& self, int index)
              -> VectorXDecisionVariable { return self.state(index); },
          doc.MultipleShooting.state.doc_1args)
      .def("initial_state",
          [](const MultipleShooting& self) -> VectorXDecisionVariable {
            return self.initial_state();
          },
          doc.MultipleShooting.initial_state.doc)
      .def("final_state",
          [](const MultipleShooting& self) -> VectorXDecisionVariable {
            return self.final_state();
          },
          doc.MultipleShooting.final_state.doc)
      .def("input",
          [](const MultipleShooting& self) -> VectorXDecisionVariable {
            return self.input();
          },
          doc.MultipleShooting.input.doc_0args)
      .def("input",
          [](const MultipleShooting& self, int index)
              -> VectorXDecisionVariable { return self.input(index); },
          doc.MultipleShooting.input.doc_1args)
      .def("AddRunningCost",
          [](MultipleShooting& prog, const symbolic::Expression& g) {
            prog.AddRunningCost(g);
          },
          doc.MultipleShooting.AddRunningCost.doc_1args_g)
      .def("AddRunningCost",
          [](MultipleShooting& prog,
              const Eigen::Ref<const MatrixX<symbolic::Expression>>& g) {
            prog.AddRunningCost(g);
          },
          doc.MultipleShooting.AddRunningCost.doc_1args_constEigenMatrixBase)
      .def("AddConstraintToAllKnotPoints",
          &MultipleShooting::AddConstraintToAllKnotPoints,
          doc.MultipleShooting.AddConstraintToAllKnotPoints.doc)
      .def("AddTimeIntervalBounds", &MultipleShooting::AddTimeIntervalBounds,
          doc.MultipleShooting.AddTimeIntervalBounds.doc)
      .def("AddEqualTimeIntervalsConstraints",
          &MultipleShooting::AddEqualTimeIntervalsConstraints,
          doc.MultipleShooting.AddEqualTimeIntervalsConstraints.doc)
      .def("AddDurationBounds", &MultipleShooting::AddDurationBounds,
          doc.MultipleShooting.AddDurationBounds.doc)
      .def("AddFinalCost",
          py::overload_cast<const symbolic::Expression&>(
              &MultipleShooting::AddFinalCost),
          doc.MultipleShooting.AddFinalCost.doc_1args_e)
      .def("AddFinalCost",
          py::overload_cast<
              const Eigen::Ref<const MatrixX<symbolic::Expression>>&>(
              &MultipleShooting::AddFinalCost),
          doc.MultipleShooting.AddFinalCost.doc_1args_matrix)
      .def("AddInputTrajectoryCallback",
          &MultipleShooting::AddInputTrajectoryCallback,
          doc.MultipleShooting.AddInputTrajectoryCallback.doc)
      .def("AddStateTrajectoryCallback",
          &MultipleShooting::AddStateTrajectoryCallback,
          doc.MultipleShooting.AddStateTrajectoryCallback.doc)
      .def("SetInitialTrajectory", &MultipleShooting::SetInitialTrajectory,
          doc.MultipleShooting.SetInitialTrajectory.doc)
      .def("GetSampleTimes",
          [](const MultipleShooting& prog) {
            WarnDeprecated(kSolveDeprecationString);
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
            return prog.GetSampleTimes();
#pragma GCC diagnostic pop
          },
          doc.MultipleShooting.GetSampleTimes.doc_deprecated)
      .def("GetSampleTimes",
          overload_cast_explicit<Eigen::VectorXd,
              const solvers::MathematicalProgramResult&>(
              &MultipleShooting::GetSampleTimes),
          doc.MultipleShooting.GetSampleTimes.doc)
      .def("GetInputSamples",
          [](const MultipleShooting& prog) {
            WarnDeprecated(kSolveDeprecationString);
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
            return prog.GetInputSamples();
#pragma GCC diagnostic pop
          },
          doc.MultipleShooting.GetInputSamples.doc_deprecated)
      .def("GetInputSamples",
          overload_cast_explicit<Eigen::MatrixXd,
              const solvers::MathematicalProgramResult&>(
              &MultipleShooting::GetInputSamples),
          doc.MultipleShooting.GetInputSamples.doc)
      .def("GetStateSamples",
          [](const MultipleShooting& prog) {
            WarnDeprecated(kSolveDeprecationString);
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
            return prog.GetStateSamples();
#pragma GCC diagnostic pop
          },
          doc.MultipleShooting.GetStateSamples.doc_deprecated)
      .def("GetStateSamples",
          overload_cast_explicit<Eigen::MatrixXd,
              const solvers::MathematicalProgramResult&>(
              &MultipleShooting::GetStateSamples),
          doc.MultipleShooting.GetStateSamples.doc)
      .def("ReconstructInputTrajectory",
          [](const MultipleShooting& prog) {
            WarnDeprecated(kSolveDeprecationString);
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
            return prog.ReconstructInputTrajectory();
#pragma GCC diagnostic pop
          },
          doc.MultipleShooting.ReconstructInputTrajectory.doc_deprecated)
      .def("ReconstructInputTrajectory",
          overload_cast_explicit<trajectories::PiecewisePolynomial<double>,
              const solvers::MathematicalProgramResult&>(
              &MultipleShooting::ReconstructInputTrajectory),
          doc.MultipleShooting.ReconstructInputTrajectory.doc)
      .def("ReconstructStateTrajectory",
          [](const MultipleShooting& prog) {
            WarnDeprecated(kSolveDeprecationString);
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
            return prog.ReconstructStateTrajectory();
#pragma GCC diagnostic pop
          },
          doc.MultipleShooting.ReconstructStateTrajectory.doc_deprecated)
      .def("ReconstructStateTrajectory",
          overload_cast_explicit<trajectories::PiecewisePolynomial<double>,
              const solvers::MathematicalProgramResult&>(
              &MultipleShooting::ReconstructStateTrajectory),
          doc.MultipleShooting.ReconstructStateTrajectory.doc);

  py::class_<DirectCollocation, MultipleShooting>(
      m, "DirectCollocation", doc.DirectCollocation.doc)
      .def(py::init<const systems::System<double>*,
               const systems::Context<double>&, int, double, double,
               variant<systems::InputPortSelection, systems::InputPortIndex>,
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
               variant<systems::InputPortSelection, systems::InputPortIndex>,
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
  py::class_<TimeStep>(m, "TimeStep")
      .def(py::init<double>())
      .def_readwrite("value", &TimeStep::value);

  py::class_<DirectTranscription, MultipleShooting>(
      m, "DirectTranscription", doc.DirectTranscription.doc)
      .def(py::init<const systems::System<double>*,
               const systems::Context<double>&, int,
               variant<systems::InputPortSelection, systems::InputPortIndex>>(),
          py::arg("system"), py::arg("context"), py::arg("num_time_samples"),
          py::arg("input_port_index") =
              systems::InputPortSelection::kUseFirstInputIfItExists,
          doc.DirectTranscription.ctor.doc_4args)
      .def(py::init<const systems::System<double>*,
               const systems::Context<double>&, int, TimeStep,
               variant<systems::InputPortSelection, systems::InputPortIndex>>(),
          py::arg("system"), py::arg("context"), py::arg("num_time_samples"),
          py::arg("fixed_timestep"),
          py::arg("input_port_index") =
              systems::InputPortSelection::kUseFirstInputIfItExists,
          doc.DirectTranscription.ctor.doc_5args);
}

}  // namespace pydrake
}  // namespace drake
