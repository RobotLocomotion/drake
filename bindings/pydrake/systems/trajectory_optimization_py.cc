#include "pybind11/eigen.h"
#include "pybind11/functional.h"
#include "pybind11/pybind11.h"
#include "pybind11/stl.h"

#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/bindings/pydrake/symbolic_types_pybind.h"
#include "drake/systems/trajectory_optimization/direct_collocation.h"
#include "drake/systems/trajectory_optimization/direct_transcription.h"

namespace drake {
namespace pydrake {

PYBIND11_MODULE(trajectory_optimization, m) {
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::systems::trajectory_optimization;

  using solvers::VectorXDecisionVariable;

  py::module::import("pydrake.symbolic");
  py::module::import("pydrake.systems.framework");
  py::module::import("pydrake.systems.primitives");
  py::module::import("pydrake.solvers._mathematicalprogram_py");

  py::class_<MultipleShooting, solvers::MathematicalProgram>(m,
                                                             "MultipleShooting")
      .def("time", &MultipleShooting::time)
      .def("timestep", &MultipleShooting::timestep)
      .def("fixed_timestep", &MultipleShooting::fixed_timestep)
      // TODO(eric.cousineau): The original bindings returned references
      // instead of copies using VectorXBlock. Restore this once dtype=custom
      // is resolved.
      .def("state",
           [](const MultipleShooting& self) -> VectorXDecisionVariable {
             return self.state();
           })
      .def("state",
           [](const MultipleShooting& self, int index) ->
               VectorXDecisionVariable {
             return self.state(index);
           })
      .def("initial_state",
           [](const MultipleShooting& self) -> VectorXDecisionVariable {
             return self.initial_state();
           })
      .def("final_state",
           [](const MultipleShooting& self) -> VectorXDecisionVariable {
             return self.final_state();
           })
      .def("input",
           [](const MultipleShooting& self) -> VectorXDecisionVariable {
             return self.input();
           })
      .def("input",
           [](const MultipleShooting& self, int index) ->
                VectorXDecisionVariable {
             return self.input(index);
           })
      .def("AddRunningCost",
           [](MultipleShooting& prog, const symbolic::Expression& g) {
             prog.AddRunningCost(g);
           })
      .def("AddRunningCost",
           [](MultipleShooting& prog,
              const Eigen::Ref<const MatrixX<symbolic::Expression>>& g) {
             prog.AddRunningCost(g);
           })
      .def("AddConstraintToAllKnotPoints",
           &MultipleShooting::AddConstraintToAllKnotPoints)
      .def("AddTimeIntervalBounds", &MultipleShooting::AddTimeIntervalBounds)
      .def("AddEqualTimeIntervalsConstraints",
           &MultipleShooting::AddEqualTimeIntervalsConstraints)
      .def("AddDurationBounds", &MultipleShooting::AddDurationBounds)
      .def("AddFinalCost", py::overload_cast<const symbolic::Expression&>(
                               &MultipleShooting::AddFinalCost))
      .def("AddFinalCost",
           py::overload_cast<
               const Eigen::Ref<const MatrixX<symbolic::Expression>>&>(
               &MultipleShooting::AddFinalCost))
      .def("AddInputTrajectoryCallback",
           &MultipleShooting::AddInputTrajectoryCallback)
      .def("AddStateTrajectoryCallback",
           &MultipleShooting::AddStateTrajectoryCallback)
      .def("SetInitialTrajectory", &MultipleShooting::SetInitialTrajectory)
      .def("GetSampleTimes", overload_cast_explicit<Eigen::VectorXd>(
                                 &MultipleShooting::GetSampleTimes))
      .def("GetInputSamples", &MultipleShooting::GetInputSamples)
      .def("GetStateSamples", &MultipleShooting::GetStateSamples)
      .def("ReconstructInputTrajectory",
           &MultipleShooting::ReconstructInputTrajectory)
      .def("ReconstructStateTrajectory",
           &MultipleShooting::ReconstructStateTrajectory);

  py::class_<DirectCollocation, MultipleShooting>(m, "DirectCollocation")
      .def(py::init<const systems::System<double>*,
                    const systems::Context<double>&, int, double, double>(),
           py::arg("system"), py::arg("context"), py::arg("num_time_samples"),
           py::arg("minimum_timestep"), py::arg("maximum_timestep"))
      .def("ReconstructInputTrajectory",
           &DirectCollocation::ReconstructInputTrajectory)
      .def("ReconstructStateTrajectory",
           &DirectCollocation::ReconstructStateTrajectory);

  py::class_<DirectCollocationConstraint, solvers::Constraint,
             std::shared_ptr<DirectCollocationConstraint>>(
      m, "DirectCollocationConstraint")
      .def(py::init<const systems::System<double>&,
                    const systems::Context<double>&>());

  m.def("AddDirectCollocationConstraint", &AddDirectCollocationConstraint,
        py::arg("constraint"), py::arg("timestep"), py::arg
            ("state"), py::arg("next_state"), py::arg("input"), py::arg
            ("next_input"), py::arg("prog"));

  py::class_<DirectTranscription, MultipleShooting>(m, "DirectTranscription")
      .def(py::init<const systems::System<double>*,
                    const systems::Context<double>&, int>(),
           py::arg("system"), py::arg("context"), py::arg("num_time_samples"))
      .def(py::init<const systems::LinearSystem<double>*,
                    const systems::Context<double>&, int>(),
           py::arg("system"), py::arg("context"), py::arg("num_time_samples"))
      // TODO(russt): Add this once TimeVaryingLinearSystem is bound.
      //      .def(py::init<const TimeVaryingLinearSystem<double>*,
      //                    const Context<double>&, int>())
      .def("ReconstructInputTrajectory",
           &DirectTranscription::ReconstructInputTrajectory)
      .def("ReconstructStateTrajectory",
           &DirectTranscription::ReconstructStateTrajectory);
}

}  // namespace pydrake
}  // namespace drake
