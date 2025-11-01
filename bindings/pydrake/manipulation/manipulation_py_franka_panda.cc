#include "drake/bindings/generated_docstrings/manipulation_franka_panda.h"
#include "drake/bindings/pydrake/common/serialize_pybind.h"
#include "drake/bindings/pydrake/manipulation/manipulation_py.h"
#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/lcmt_panda_status.hpp"
#include "drake/manipulation/franka_panda/panda_command_receiver.h"
#include "drake/manipulation/franka_panda/panda_command_sender.h"
#include "drake/manipulation/franka_panda/panda_constants.h"
#include "drake/manipulation/franka_panda/panda_status_receiver.h"
#include "drake/manipulation/franka_panda/panda_status_sender.h"

namespace drake {
namespace pydrake {
namespace internal {

using systems::LeafSystem;

void DefineManipulationFrankaPanda(py::module m) {
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::manipulation::franka_panda;
  constexpr auto& doc =
      pydrake_doc_manipulation_franka_panda.drake.manipulation.franka_panda;

  // Constants.
  m.attr("kPandaArmNumJoints") = kPandaArmNumJoints;

  // Control mode enum
  py::enum_<PandaControlMode>(m, "PandaControlMode", py::arithmetic(),
      "Control modes for the Panda robot. "
      "These can be bitwise OR'd together.")
      .value("kNone", PandaControlMode::kNone)
      .value("kPosition", PandaControlMode::kPosition)
      .value("kVelocity", PandaControlMode::kVelocity)
      .value("kTorque", PandaControlMode::kTorque)
      .def("__or__",
          [](PandaControlMode a, PandaControlMode b) { return a | b; })
      .def("__and__",
          [](PandaControlMode a, PandaControlMode b) { return a & b; });
  {
    using Class = PandaCommandReceiver;
    constexpr auto& cls_doc = doc.PandaCommandReceiver;
    py::class_<Class, LeafSystem<double>>(
        m, "PandaCommandReceiver", cls_doc.doc)
        .def(py::init<int, PandaControlMode>(), py::arg("num_joints"),
            py::arg("control_mode"), cls_doc.ctor.doc)
        .def("LatchInitialPosition",
            overload_cast_explicit<void, drake::systems::Context<double>*>(
                &Class::LatchInitialPosition),
            py::arg("context"), cls_doc.LatchInitialPosition.doc)
        .def("get_message_input_port", &Class::get_message_input_port,
            py_rvp::reference_internal, cls_doc.get_message_input_port.doc)
        .def("get_position_measured_input_port",
            &Class::get_position_measured_input_port,
            py_rvp::reference_internal,
            cls_doc.get_position_measured_input_port.doc)
        .def("get_commanded_position_output_port",
            &Class::get_commanded_position_output_port,
            py_rvp::reference_internal,
            cls_doc.get_commanded_position_output_port.doc)
        .def("get_commanded_velocity_output_port",
            &Class::get_commanded_velocity_output_port,
            py_rvp::reference_internal,
            cls_doc.get_commanded_velocity_output_port.doc)
        .def("get_commanded_torque_output_port",
            &Class::get_commanded_torque_output_port,
            py_rvp::reference_internal,
            cls_doc.get_commanded_torque_output_port.doc);
  }

  {
    using Class = PandaCommandSender;
    constexpr auto& cls_doc = doc.PandaCommandSender;
    py::class_<Class, LeafSystem<double>>(m, "PandaCommandSender", cls_doc.doc)
        .def(py::init<int, PandaControlMode>(), py::arg("num_joints"),
            py::arg("control_mode"), cls_doc.ctor.doc)
        .def("get_position_input_port", &Class::get_position_input_port,
            py_rvp::reference_internal, cls_doc.get_position_input_port.doc)
        .def("get_velocity_input_port", &Class::get_velocity_input_port,
            py_rvp::reference_internal, cls_doc.get_velocity_input_port.doc)
        .def("get_torque_input_port", &Class::get_torque_input_port,
            py_rvp::reference_internal, cls_doc.get_torque_input_port.doc);
  }

  {
    using Class = PandaStatusReceiver;
    constexpr auto& cls_doc = doc.PandaStatusReceiver;
    py::class_<Class, LeafSystem<double>>(m, "PandaStatusReceiver", cls_doc.doc)
        .def(py::init<int>(), py::arg("num_joints") = kPandaArmNumJoints,
            cls_doc.ctor.doc)
        .def("get_position_commanded_output_port",
            &Class::get_position_commanded_output_port,
            py_rvp::reference_internal,
            cls_doc.get_position_commanded_output_port.doc)
        .def("get_position_measured_output_port",
            &Class::get_position_measured_output_port,
            py_rvp::reference_internal,
            cls_doc.get_position_measured_output_port.doc)
        .def("get_velocity_commanded_output_port",
            &Class::get_velocity_commanded_output_port,
            py_rvp::reference_internal,
            cls_doc.get_velocity_commanded_output_port.doc)
        .def("get_velocity_measured_output_port",
            &Class::get_velocity_measured_output_port,
            py_rvp::reference_internal,
            cls_doc.get_velocity_measured_output_port.doc)
        .def("get_acceleration_commanded_output_port",
            &Class::get_acceleration_commanded_output_port,
            py_rvp::reference_internal,
            cls_doc.get_acceleration_commanded_output_port.doc)
        .def("get_torque_commanded_output_port",
            &Class::get_torque_commanded_output_port,
            py_rvp::reference_internal,
            cls_doc.get_torque_commanded_output_port.doc)
        .def("get_torque_measured_output_port",
            &Class::get_torque_measured_output_port, py_rvp::reference_internal,
            cls_doc.get_torque_measured_output_port.doc)
        .def("get_torque_external_output_port",
            &Class::get_torque_external_output_port, py_rvp::reference_internal,
            cls_doc.get_torque_external_output_port.doc);
  }

  {
    using Class = PandaStatusSender;
    constexpr auto& cls_doc = doc.PandaStatusSender;
    py::class_<Class, LeafSystem<double>>(m, "PandaStatusSender", cls_doc.doc)
        .def(py::init<int>(), py::arg("num_joints") = kPandaArmNumJoints,
            cls_doc.ctor.doc)
        .def("get_position_commanded_input_port",
            &Class::get_position_commanded_input_port,
            py_rvp::reference_internal,
            cls_doc.get_position_commanded_input_port.doc)
        .def("get_position_measured_input_port",
            &Class::get_position_measured_input_port,
            py_rvp::reference_internal,
            cls_doc.get_position_measured_input_port.doc)
        .def("get_velocity_commanded_input_port",
            &Class::get_velocity_commanded_input_port,
            py_rvp::reference_internal,
            cls_doc.get_velocity_commanded_input_port.doc)
        .def("get_velocity_measured_input_port",
            &Class::get_velocity_measured_input_port,
            py_rvp::reference_internal,
            cls_doc.get_velocity_measured_input_port.doc)
        .def("get_acceleration_commanded_input_port",
            &Class::get_acceleration_commanded_input_port,
            py_rvp::reference_internal,
            cls_doc.get_acceleration_commanded_input_port.doc)
        .def("get_torque_commanded_input_port",
            &Class::get_torque_commanded_input_port, py_rvp::reference_internal,
            cls_doc.get_torque_commanded_input_port.doc)
        .def("get_torque_measured_input_port",
            &Class::get_torque_measured_input_port, py_rvp::reference_internal,
            cls_doc.get_torque_measured_input_port.doc)
        .def("get_torque_external_input_port",
            &Class::get_torque_external_input_port, py_rvp::reference_internal,
            cls_doc.get_torque_external_input_port.doc);
  }
}

}  // namespace internal
}  // namespace pydrake
}  // namespace drake
