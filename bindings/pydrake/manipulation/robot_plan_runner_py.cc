#include "pybind11/eigen.h"
#include "pybind11/pybind11.h"
#include "pybind11/stl.h"

#include "drake/bindings/pydrake/common/deprecation_pybind.h"
#include "drake/bindings/pydrake/common/drake_optional_pybind.h"
#include "drake/bindings/pydrake/common/value_pybind.h"
#include "drake/bindings/pydrake/documentation_pybind.h"
#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/manipulation/robot_plan_runner/plan_sender.h"
#include "drake/manipulation/robot_plan_runner/robot_plan_runner.h"
#include "drake/manipulation/robot_plan_runner/robot_plans/joint_space_plan.h"
#include "drake/manipulation/robot_plan_runner/robot_plans/plan_base.h"

namespace drake {
namespace pydrake {

PYBIND11_MODULE(robot_plan_runner, m) {
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::manipulation::robot_plan_runner;

  m.doc() = "Bindings for the robot plan runner.";
  constexpr auto& doc = pydrake_doc.drake.manipulation.robot_plan_runner;

  using robot_plans::PlanData;
  using robot_plans::PlanType;
  using systems::Diagram;
  using systems::LeafSystem;

  py::module::import("pydrake.systems.framework");

  py::enum_<PlanType>(m, "PlanType")
      .value("kJointSpacePlan", PlanType::kJointSpacePlan,
          doc.robot_plans.PlanType.kJointSpacePlan.doc)
      .value("kTaskSpacePlan", PlanType::kTaskSpacePlan,
          doc.robot_plans.PlanType.kTaskSpacePlan.doc)
      .value("kEmptyPlan", PlanType::kEmptyPlan,
          doc.robot_plans.PlanType.kEmptyPlan.doc)
      .export_values();

  py::class_<PlanData> plan_data(m, "PlanData");
  plan_data
      .def(py::init([](PlanType plan_type,
                        optional<trajectories::PiecewisePolynomial<double>>
                            joint_traj) {
        return PlanData{plan_type, long{-1}, joint_traj};
      }),
          py::arg("plan_type") = PlanType::kEmptyPlan,
          py::arg("joint_traj") = nullopt)
      .def_readwrite("plan_type", &PlanData::plan_type,
          doc.robot_plans.PlanData.plan_type.doc)
      .def_readwrite("joint_traj", &PlanData::joint_traj,
          doc.robot_plans.PlanData.joint_traj.doc);

  AddValueInstantiation<PlanData>(m);

  py::class_<PlanSender, LeafSystem<double>>(m, "PlanSender")
      .def(py::init<const std::vector<PlanData>>(),
          py::arg("plan_data_list") = std::vector<PlanData>{},
          doc.PlanSender.ctor.doc)
      .def("get_all_plans_duration", &PlanSender::get_all_plans_duration,
          doc.PlanSender.get_all_plans_duration.doc);

  py::class_<RobotPlanRunner, Diagram<double>>(m, "RobotPlanRunner")
      .def(py::init<bool, double>(), py::arg("is_discrete"),
          py::arg("control_period_sec") = 0.005, doc.RobotPlanRunner.ctor.doc);
}

}  // namespace pydrake
}  // namespace drake
