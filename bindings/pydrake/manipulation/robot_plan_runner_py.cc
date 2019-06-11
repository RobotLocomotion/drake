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
  // EeData is a struct defined within the scope of PlanData.
  py::class_<PlanData::EeData> ee_data_(plan_data, "EeData");
  ee_data_
      .def(py::init<Eigen::Vector3d, trajectories::PiecewisePolynomial<double>,
                    trajectories::PiecewiseQuaternionSlerp<double>>(),
           py::arg("p_ToQ_T"), py::arg("ee_xyz_traj"), py::arg("ee_quat_traj"))
      .def_readwrite("p_ToQ_T", &PlanData::EeData::p_ToQ_T,
                     doc.robot_plans.PlanData.EeData.p_ToQ_T.doc)
      .def_readwrite("ee_xyz_traj", &PlanData::EeData::ee_xyz_traj,
                     doc.robot_plans.PlanData.EeData.ee_xyz_traj.doc)
      .def_readwrite("ee_quat_traj", &PlanData::EeData::ee_quat_traj,
                     doc.robot_plans.PlanData.EeData.ee_quat_traj.doc);
  plan_data
      .def(py::init([](
               PlanType plan_type,
               optional<trajectories::PiecewisePolynomial<double>> joint_traj,
               optional<PlanData::EeData> ee_data) {
             return PlanData{plan_type, long{-1}, joint_traj, ee_data};
           }),
           py::arg("plan_type") = PlanType::kEmptyPlan,
           py::arg("joint_traj") = nullopt, py::arg("ee_data") = nullopt)
      .def_readwrite("plan_type", &PlanData::plan_type,
                     doc.robot_plans.PlanData.plan_type.doc)
      .def_readwrite("joint_traj", &PlanData::joint_traj,
                     doc.robot_plans.PlanData.joint_traj.doc)
      .def_readwrite("ee_data", &PlanData::ee_data,
                     doc.robot_plans.PlanData.ee_data.doc);

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
