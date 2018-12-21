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
          overload_cast_explicit<Eigen::VectorXd>(
              &MultipleShooting::GetSampleTimes),
          doc.MultipleShooting.GetSampleTimes.doc_0args)
      .def("GetInputSamples", &MultipleShooting::GetInputSamples,
          doc.MultipleShooting.GetInputSamples.doc)
      .def("GetStateSamples", &MultipleShooting::GetStateSamples,
          doc.MultipleShooting.GetStateSamples.doc)
      .def("ReconstructInputTrajectory",
          &MultipleShooting::ReconstructInputTrajectory,
          doc.MultipleShooting.ReconstructInputTrajectory.doc)
      .def("ReconstructStateTrajectory",
          &MultipleShooting::ReconstructStateTrajectory,
          doc.MultipleShooting.ReconstructStateTrajectory.doc);

  py::class_<DirectCollocation, MultipleShooting>(
      m, "DirectCollocation", doc.DirectCollocation.doc)
      .def(py::init<const systems::System<double>*,
               const systems::Context<double>&, int, double, double>(),
          py::arg("system"), py::arg("context"), py::arg("num_time_samples"),
          py::arg("minimum_timestep"), py::arg("maximum_timestep"),
          doc.DirectCollocation.ctor.doc)
      .def("ReconstructInputTrajectory",
          &DirectCollocation::ReconstructInputTrajectory,
          doc.DirectCollocation.ReconstructInputTrajectory.doc)
      .def("ReconstructStateTrajectory",
          &DirectCollocation::ReconstructStateTrajectory,
          doc.DirectCollocation.ReconstructStateTrajectory.doc);

  py::class_<DirectCollocationConstraint, solvers::Constraint,
      std::shared_ptr<DirectCollocationConstraint>>(
      m, "DirectCollocationConstraint", doc.DirectCollocationConstraint.doc)
      .def(py::init<const systems::System<double>&,
               const systems::Context<double>&>(),
          doc.DirectCollocationConstraint.ctor.doc);

  m.def("AddDirectCollocationConstraint", &AddDirectCollocationConstraint,
      py::arg("constraint"), py::arg("timestep"), py::arg("state"),
      py::arg("next_state"), py::arg("input"), py::arg("next_input"),
      py::arg("prog"), doc.AddDirectCollocationConstraint.doc);

  py::class_<DirectTranscription, MultipleShooting>(
      m, "DirectTranscription", doc.DirectTranscription.doc)
      .def(py::init<const systems::System<double>*,
               const systems::Context<double>&, int>(),
          py::arg("system"), py::arg("context"), py::arg("num_time_samples"),
          doc.DirectTranscription.ctor
              .doc_3args_system_context_num_time_samples)
      .def(py::init<const systems::LinearSystem<double>*,
               const systems::Context<double>&, int>(),
          py::arg("linear_system"), py::arg("context"),
          py::arg("num_time_samples"),
          doc.DirectTranscription.ctor
              .doc_3args_linear_system_context_num_time_samples)
      // TODO(russt): Add this once TimeVaryingLinearSystem is bound.
      //      .def(py::init<const TimeVaryingLinearSystem<double>*,
      //                    const Context<double>&, int>())
      .def("ReconstructInputTrajectory",
          &DirectTranscription::ReconstructInputTrajectory,
          doc.DirectTranscription.ReconstructInputTrajectory.doc)
      .def("ReconstructStateTrajectory",
          &DirectTranscription::ReconstructStateTrajectory,
          doc.DirectTranscription.ReconstructStateTrajectory.doc);
}

}  // namespace pydrake
}  // namespace drake
