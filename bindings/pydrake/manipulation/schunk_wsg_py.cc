#include "pybind11/eigen.h"
#include "pybind11/pybind11.h"
#include "pybind11/stl.h"

#include "drake/bindings/pydrake/documentation_pybind.h"
#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/manipulation/schunk_wsg/schunk_wsg_constants.h"
#include "drake/manipulation/schunk_wsg/schunk_wsg_lcm.h"
#include "drake/manipulation/schunk_wsg/schunk_wsg_position_controller.h"

namespace drake {
namespace pydrake {

PYBIND11_MODULE(schunk_wsg, m) {
  using drake::systems::Diagram;
  using drake::systems::LeafSystem;

  m.doc() = "Tools for schunk wsg.";
  constexpr auto& doc = pydrake_doc.drake.manipulation.schunk_wsg;

  py::module::import("pydrake.systems.framework");

  {
    using Class = manipulation::schunk_wsg::SchunkWsgPositionController;
    constexpr auto& cls_doc = doc.SchunkWsgPositionController;
    py::class_<Class, Diagram<double>>(
        m, "SchunkWsgPositionController", cls_doc.doc)
        .def(py::init<double, double, double, double, double, double>(),
            py::arg("time_step") = 0.05, py::arg("kp_command") = 200.,
            py::arg("kd_command") = 5., py::arg("kp_constraint") = 2000.,
            py::arg("kd_constraint") = 5.,
            py::arg("default_force_limit") = 40.0, cls_doc.ctor.doc)
        .def("get_desired_position_input_port",
            &Class::get_desired_position_input_port, py_rvp::reference_internal,
            cls_doc.get_desired_position_input_port.doc)
        .def("get_force_limit_input_port", &Class::get_force_limit_input_port,
            py_rvp::reference_internal, cls_doc.get_force_limit_input_port.doc)
        .def("get_state_input_port", &Class::get_state_input_port,
            py_rvp::reference_internal, cls_doc.get_state_input_port.doc)
        .def("get_generalized_force_output_port",
            &Class::get_generalized_force_output_port,
            py_rvp::reference_internal,
            cls_doc.get_generalized_force_output_port.doc)
        .def("get_grip_force_output_port", &Class::get_grip_force_output_port,
            py_rvp::reference_internal, cls_doc.get_grip_force_output_port.doc);
  }

  {
    using Class = manipulation::schunk_wsg::SchunkWsgCommandReceiver;
    constexpr auto& cls_doc = doc.SchunkWsgCommandReceiver;
    py::class_<Class, LeafSystem<double>>(
        m, "SchunkWsgCommandReceiver", cls_doc.doc)
        .def(py::init<double, double>(), py::arg("initial_position") = 0.02,
            py::arg("initial_force") = 40., cls_doc.ctor.doc)
        .def("get_position_output_port", &Class::get_position_output_port,
            py_rvp::reference_internal, cls_doc.get_position_output_port.doc)
        .def("get_force_limit_output_port", &Class::get_force_limit_output_port,
            py_rvp::reference_internal,
            cls_doc.get_force_limit_output_port.doc);
  }

  {
    using Class = manipulation::schunk_wsg::SchunkWsgCommandSender;
    constexpr auto& cls_doc = doc.SchunkWsgCommandSender;
    py::class_<Class, LeafSystem<double>>(
        m, "SchunkWsgCommandSender", cls_doc.doc)
        .def(py::init<double>(), py::arg("default_force_limit") = 40.0,
            cls_doc.ctor.doc)
        .def("get_position_input_port", &Class::get_position_input_port,
            py_rvp::reference_internal, cls_doc.get_position_input_port.doc)
        .def("get_force_limit_input_port", &Class::get_force_limit_input_port,
            py_rvp::reference_internal, cls_doc.get_force_limit_input_port.doc)
        .def("get_command_output_port", &Class::get_command_output_port,
            py_rvp::reference_internal, cls_doc.get_command_output_port.doc);
  }

  {
    using Class = manipulation::schunk_wsg::SchunkWsgStatusReceiver;
    constexpr auto& cls_doc = doc.SchunkWsgStatusReceiver;
    py::class_<Class, LeafSystem<double>>(
        m, "SchunkWsgStatusReceiver", cls_doc.doc)
        .def(py::init(), cls_doc.ctor.doc)
        .def("get_status_input_port", &Class::get_status_input_port,
            py_rvp::reference_internal, cls_doc.get_status_input_port.doc)
        .def("get_state_output_port", &Class::get_state_output_port,
            py_rvp::reference_internal, cls_doc.get_state_output_port.doc)
        .def("get_force_output_port", &Class::get_force_output_port,
            py_rvp::reference_internal, cls_doc.get_force_output_port.doc);
  }

  {
    using Class = manipulation::schunk_wsg::SchunkWsgStatusSender;
    constexpr auto& cls_doc = doc.SchunkWsgStatusSender;
    py::class_<Class, LeafSystem<double>>(
        m, "SchunkWsgStatusSender", cls_doc.doc)
        .def(py::init(), cls_doc.ctor.doc)
        .def("get_state_input_port", &Class::get_state_input_port,
            py_rvp::reference_internal, cls_doc.get_state_input_port.doc)
        .def("get_force_input_port", &Class::get_force_input_port,
            py_rvp::reference_internal, cls_doc.get_force_input_port.doc);
  }

  {
    using T = double;

    m.def(
        "GetSchunkWsgOpenPosition",
        []() {
          return manipulation::schunk_wsg::GetSchunkWsgOpenPosition<T>();
        },
        doc.GetSchunkWsgOpenPosition.doc);

    m.def(
        "MakeMultibodyStateToWsgStateSystem",
        []() {
          return manipulation::schunk_wsg::MakeMultibodyStateToWsgStateSystem<
              T>();
        },
        doc.MakeMultibodyStateToWsgStateSystem.doc);
  }
}

}  // namespace pydrake
}  // namespace drake
