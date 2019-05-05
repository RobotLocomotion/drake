#include "pybind11/eigen.h"
#include "pybind11/pybind11.h"
#include "pybind11/stl.h"

#include "drake/bindings/pydrake/common/deprecation_pybind.h"
#include "drake/bindings/pydrake/common/drake_optional_pybind.h"
#include "drake/bindings/pydrake/documentation_pybind.h"
#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/manipulation/robot_plan_runner/robot_plans.h"
#include "drake/manipulation/robot_plan_runner/plan_sender.h"
#include "drake/manipulation/robot_plan_runner/robot_plan_runner.h"

using std::make_unique;
using std::unique_ptr;
using std::vector;

namespace drake {
namespace pydrake {

PYBIND11_MODULE(manipulation_station, m) {
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::systems;
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::manipulation::robot_plan_runner;

  m.doc() = "Bindings for the robot plan runner.";
  constexpr auto& doc = pydrake_doc.drake.manipulation.robot_plan_runner;

  py::enum_<PlanType>(m, "PlanType")
      .value("kJointSpacePlan", PlanType::kJointSpacePlan,
             doc.PlanType.kJointSpacePlan.doc)
      .value("kTaskSpacePlan", PlanType::kTaskSpacePlan,
             doc.PlanType.kTaskSpacePlan.doc)
      .value("kEmptyPlan", PlanType::kEmptyPlan,
             doc.PlanType.kEmptyPlan.doc)
      .export_values();

  py::class_<PlanData> plan_data(m, "PlanData");
  plan_data.attr("plan_type") = PlanType::kEmptyPlan;
  plan_data.attr("plan_signature") = long{-1};
  plan_data.attr("joint_traj") = nullopt;
  // N.B. this datatype doesn't have bindings yet.
  plan_data.attr("ee_traj") = nullopt;

  py::class_<PlanSender, LeafSystem<double>>(m, "PlanSender")
      .def(py::init<const std::vector<PlanData>>(),
          py::arg("plan_data_list") = std::vector<PlanData>{},
          doc.PlanSender.ctor.doc)
      .def("get_all_plans_duration", &PlanSender::get_all_plans_duration,
          doc.PlanSender.get_all_plans_duration.doc);

  py::class_<RobotPlanRunner, Diagram<double>>(m, "RobotPlanRunner")
      .def(py::init<double>(), py::arg("control_period_sec") = 0.005,
           doc.RobotPlanRunner.ctor.doc);

  ExecuteExtraPythonCode(m);
}

}  // namespace pydrake
}  // namespace drake
