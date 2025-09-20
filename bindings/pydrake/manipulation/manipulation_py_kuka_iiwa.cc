#include "drake/bindings/generated_docstrings/manipulation_kuka_iiwa.h"
#include "drake/bindings/pydrake/common/ref_cycle_pybind.h"
#include "drake/bindings/pydrake/common/serialize_pybind.h"
#include "drake/bindings/pydrake/manipulation/manipulation_py.h"
#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/bindings/pydrake/systems/builder_life_support_pybind.h"
#include "drake/manipulation/kuka_iiwa/build_iiwa_control.h"
#include "drake/manipulation/kuka_iiwa/iiwa_command_receiver.h"
#include "drake/manipulation/kuka_iiwa/iiwa_command_sender.h"
#include "drake/manipulation/kuka_iiwa/iiwa_constants.h"
#include "drake/manipulation/kuka_iiwa/iiwa_driver.h"
#include "drake/manipulation/kuka_iiwa/iiwa_driver_functions.h"
#include "drake/manipulation/kuka_iiwa/iiwa_status_receiver.h"
#include "drake/manipulation/kuka_iiwa/iiwa_status_sender.h"
#include "drake/manipulation/kuka_iiwa/sim_iiwa_driver.h"

namespace drake {
namespace pydrake {
namespace internal {

using systems::Diagram;
using systems::LeafSystem;

void DefineManipulationKukaIiwa(py::module m) {
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::manipulation::kuka_iiwa;
  constexpr auto& doc =
      pydrake_doc_manipulation_kuka_iiwa.drake.manipulation.kuka_iiwa;

  // Constants.
  m.attr("kIiwaArmNumJoints") = kIiwaArmNumJoints;
  m.def("get_iiwa_max_joint_velocities", &get_iiwa_max_joint_velocities,
      doc.get_iiwa_max_joint_velocities.doc);
  m.attr("kIiwaLcmStatusPeriod") = kIiwaLcmStatusPeriod;

  {
    using Class = IiwaControlMode;
    constexpr auto& cls_doc = doc.IiwaControlMode;
    py::enum_<Class>(m, "IiwaControlMode", cls_doc.doc)
        .value("kPositionOnly", Class::kPositionOnly, cls_doc.kPositionOnly.doc)
        .value("kTorqueOnly", Class::kTorqueOnly, cls_doc.kTorqueOnly.doc)
        .value("kPositionAndTorque", Class::kPositionAndTorque,
            cls_doc.kPositionAndTorque.doc);
  }

  m.def("position_enabled", &position_enabled, py::arg("control_mode"),
      doc.position_enabled.doc);
  m.def("torque_enabled", &torque_enabled, py::arg("control_mode"),
      doc.torque_enabled.doc);
  m.def("ParseIiwaControlMode", &ParseIiwaControlMode, py::arg("control_mode"),
      doc.ParseIiwaControlMode.doc);

  {
    using Class = IiwaCommandReceiver;
    constexpr auto& cls_doc = doc.IiwaCommandReceiver;
    py::class_<Class, LeafSystem<double>>(m, "IiwaCommandReceiver", cls_doc.doc)
        .def(py::init<int, IiwaControlMode>(),
            py::arg("num_joints") = kIiwaArmNumJoints,
            py::arg("control_mode") = IiwaControlMode::kPositionAndTorque,
            cls_doc.ctor.doc)
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
        .def("get_commanded_torque_output_port",
            &Class::get_commanded_torque_output_port,
            py_rvp::reference_internal,
            cls_doc.get_commanded_torque_output_port.doc)
        .def("get_time_output_port", &Class::get_time_output_port,
            py_rvp::reference_internal, cls_doc.get_time_output_port.doc);
  }

  {
    using Class = IiwaCommandSender;
    constexpr auto& cls_doc = doc.IiwaCommandSender;
    py::class_<Class, LeafSystem<double>>(m, "IiwaCommandSender", cls_doc.doc)
        .def(py::init<int, IiwaControlMode>(),
            py::arg("num_joints") = kIiwaArmNumJoints,
            py::arg("control_mode") = IiwaControlMode::kPositionAndTorque,
            cls_doc.ctor.doc)
        .def("get_time_input_port", &Class::get_time_input_port,
            py_rvp::reference_internal, cls_doc.get_time_input_port.doc)
        .def("get_position_input_port", &Class::get_position_input_port,
            py_rvp::reference_internal, cls_doc.get_position_input_port.doc)
        .def("get_torque_input_port", &Class::get_torque_input_port,
            py_rvp::reference_internal, cls_doc.get_torque_input_port.doc);
  }

  {
    using Class = IiwaStatusReceiver;
    constexpr auto& cls_doc = doc.IiwaStatusReceiver;
    py::class_<Class, LeafSystem<double>>(m, "IiwaStatusReceiver", cls_doc.doc)
        .def(py::init<int>(), py::arg("num_joints") = kIiwaArmNumJoints,
            cls_doc.ctor.doc)
        .def("get_time_measured_output_port",
            &Class::get_time_measured_output_port, py_rvp::reference_internal,
            cls_doc.get_time_measured_output_port.doc)
        .def("get_position_commanded_output_port",
            &Class::get_position_commanded_output_port,
            py_rvp::reference_internal,
            cls_doc.get_position_commanded_output_port.doc)
        .def("get_position_measured_output_port",
            &Class::get_position_measured_output_port,
            py_rvp::reference_internal,
            cls_doc.get_position_measured_output_port.doc)
        .def("get_velocity_estimated_output_port",
            &Class::get_velocity_estimated_output_port,
            py_rvp::reference_internal,
            cls_doc.get_velocity_estimated_output_port.doc)
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
    using Class = IiwaStatusSender;
    constexpr auto& cls_doc = doc.IiwaStatusSender;
    py::class_<Class, LeafSystem<double>>(m, "IiwaStatusSender", cls_doc.doc)
        .def(py::init<int>(), py::arg("num_joints") = kIiwaArmNumJoints,
            cls_doc.ctor.doc)
        .def("get_time_measured_input_port",
            &Class::get_time_measured_input_port, py_rvp::reference_internal,
            cls_doc.get_time_measured_input_port.doc)
        .def("get_position_commanded_input_port",
            &Class::get_position_commanded_input_port,
            py_rvp::reference_internal,
            cls_doc.get_position_commanded_input_port.doc)
        .def("get_position_measured_input_port",
            &Class::get_position_measured_input_port,
            py_rvp::reference_internal,
            cls_doc.get_position_measured_input_port.doc)
        .def("get_velocity_estimated_input_port",
            &Class::get_velocity_estimated_input_port,
            py_rvp::reference_internal,
            cls_doc.get_velocity_estimated_input_port.doc)
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

  {
    using Class = IiwaDriver;
    constexpr auto& cls_doc = doc.IiwaDriver;
    py::class_<Class> cls(m, "IiwaDriver", cls_doc.doc);
    cls  // BR
        .def(ParamInit<Class>());
    DefAttributesUsingSerialize(&cls, cls_doc);
    DefReprUsingSerialize(&cls);
    DefCopyAndDeepCopy(&cls);
  }

  {
    using Class = SimIiwaDriver<double>;
    constexpr auto& cls_doc = doc.SimIiwaDriver;
    py::class_<Class, Diagram<double>>(m, "SimIiwaDriver", cls_doc.doc)
        .def(py::init<const IiwaDriver&,
                 const multibody::MultibodyPlant<double>*>(),
            py::arg("driver_config"), py::arg("controller_plant"),
            // Keep alive, ownership: `self` keeps `controller_plant` alive.
            py::keep_alive<1, 3>(), cls_doc.ctor.doc)
        .def_static("AddToBuilder", &Class::AddToBuilder, py::arg("builder"),
            py::arg("plant"), py::arg("iiwa_instance"),
            py::arg("driver_config"), py::arg("controller_plant"),
            // Using builder_life_support_stash makes the
            // builder temporarily immortal (uncollectible self cycle). This
            // will be resolved by the Build() step. See BuilderLifeSupport
            // for rationale.
            internal::builder_life_support_stash<double, 1>(),
            // `return` and `builder` join ref cycle.
            internal::ref_cycle<0, 1>(),
            // Keep alive, reference: `return` keeps `controller_plant` alive.
            py::keep_alive<0, 5>(), py_rvp::reference,
            cls_doc.AddToBuilder.doc);
  }

  {
    m.def("ApplyDriverConfig", &ApplyDriverConfig, py::arg("driver_config"),
        py::arg("model_instance_name"), py::arg("sim_plant"),
        py::arg("models_from_directives"), py::arg("lcms"), py::arg("builder"),
        doc.ApplyDriverConfig.doc);

    m.def("BuildIiwaControl",
        overload_cast_explicit<void, systems::DiagramBuilder<double>*,
            lcm::DrakeLcmInterface*, const multibody::MultibodyPlant<double>&,
            const multibody::ModelInstanceIndex, const IiwaDriver&,
            const multibody::MultibodyPlant<double>&>(&BuildIiwaControl),
        py::arg("builder"), py::arg("lcm"), py::arg("plant"),
        py::arg("iiwa_instance"), py::arg("driver_config"),
        py::arg("controller_plant"),
        // Using builder_life_support_stash makes the
        // builder temporarily immortal (uncollectible self cycle). This
        // will be resolved by the Build() step. See BuilderLifeSupport
        // for rationale.
        internal::builder_life_support_stash<double, 1>(),
        // Keep alive, reference: `builder` keeps `controller_plant` alive.  It
        // would be preferable to attach the keep-alive to the SimIiwaDriver
        // diagram/system, but it does not appear in the function signature.
        // Use the builder as an adequate lifetime proxy, since it will be kept
        // alive via lifetime management associated with Build() calls.
        py::keep_alive<1, 6>(), doc.BuildIiwaControl.doc);
  }
}

}  // namespace internal
}  // namespace pydrake
}  // namespace drake
