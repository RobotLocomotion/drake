#include "pybind11/eigen.h"
#include "pybind11/pybind11.h"

#include "drake/bindings/pydrake/common/value_pybind.h"
#include "drake/bindings/pydrake/documentation_pybind.h"
#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/multibody/rigid_body_plant/drake_visualizer.h"
#include "drake/multibody/rigid_body_plant/rigid_body_plant.h"

using std::unique_ptr;
using std::vector;

namespace drake {

using drake::lcm::DrakeLcmInterface;

namespace pydrake {

PYBIND11_MODULE(rigid_body_plant, m) {
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::systems;
  constexpr auto& doc = pydrake_doc.drake.systems;

  // Ensure we have bindings for dependencies.
  py::module::import("pydrake.attic.multibody.rigid_body_tree");
  py::module::import("pydrake.lcm");
  py::module::import("pydrake.systems.framework");

  using T = double;

  {
    using Class = CompliantContactModelParameters;
    constexpr auto& cls_doc = doc.CompliantContactModelParameters;
    py::class_<Class> cls(m, "CompliantContactModelParameters", cls_doc.doc);
    cls  // BR
        .def(
            // Implicit constructor does not have documentation.
            py::init(
                [](double v_stiction_tolerance, double characteristic_radius) {
                  return Class{v_stiction_tolerance, characteristic_radius};
                }),
            py::arg("v_stiction_tolerance") = Class::kDefaultVStictionTolerance,
            py::arg("characteristic_radius") =
                Class::kDefaultCharacteristicRadius)
        .def_readwrite("v_stiction_tolerance", &Class::v_stiction_tolerance,
            cls_doc.v_stiction_tolerance.doc)
        .def_readwrite("characteristic_radius", &Class::characteristic_radius,
            cls_doc.characteristic_radius.doc);
    cls.attr("kDefaultVStictionTolerance") = Class::kDefaultVStictionTolerance;
    cls.attr("kDefaultCharacteristicRadius") =
        Class::kDefaultCharacteristicRadius;
  }

  {
    using Class = ContactInfo<T>;
    constexpr auto& cls_doc = doc.ContactInfo;
    py::class_<Class> cls(m, "ContactInfo", cls_doc.doc);
    cls  // BR
        .def("get_element_id_1", &Class::get_element_id_1,
            cls_doc.get_element_id_1.doc)
        .def("get_element_id_2", &Class::get_element_id_2,
            cls_doc.get_element_id_2.doc)
        .def("get_resultant_force", &Class::get_resultant_force,
            py_reference_internal, cls_doc.get_resultant_force.doc);
  }

  {
    using Class = ContactForce<T>;
    constexpr auto& cls_doc = doc.ContactForce;
    py::class_<Class> cls(m, "ContactForce", cls_doc.doc);
    cls  // BR
        .def(py::init<const Vector3<T>&, const Vector3<T>&, const Vector3<T>&,
                 const Vector3<T>&>(),
            py::arg("application_point"), py::arg("normal"), py::arg("force"),
            py::arg("torque") = Vector3<T>::Zero(), cls_doc.ctor.doc_4args)
        .def("get_reaction_force", &Class::get_reaction_force,
            cls_doc.get_reaction_force.doc)
        .def("get_application_point", &Class::get_application_point,
            cls_doc.get_application_point.doc)
        .def("get_force", &Class::get_force, cls_doc.get_force.doc)
        .def("get_normal_force", &Class::get_normal_force,
            cls_doc.get_normal_force.doc)
        .def("get_tangent_force", &Class::get_tangent_force,
            cls_doc.get_tangent_force.doc)
        .def("get_torque", &Class::get_torque, cls_doc.get_torque.doc)
        .def("get_normal", &Class::get_normal, cls_doc.get_normal.doc);
  }

  {
    using Class = ContactResults<T>;
    constexpr auto& cls_doc = doc.ContactResults;
    py::class_<Class> cls(m, "ContactResults", cls_doc.doc);
    cls  // BR
        .def(py::init<>(), cls_doc.ctor.doc)
        .def("get_num_contacts", &Class::get_num_contacts,
            cls_doc.get_num_contacts.doc)
        .def("get_contact_info", &Class::get_contact_info,
            py_reference_internal, cls_doc.get_contact_info.doc)
        .def("set_generalized_contact_force",
            [](Class* self, const Eigen::VectorXd& f) {
              self->set_generalized_contact_force(f);
            },
            cls_doc.set_generalized_contact_force.doc)
        .def("get_generalized_contact_force",
            &Class::get_generalized_contact_force, py_reference_internal,
            cls_doc.get_generalized_contact_force.doc)
        .def("AddContact", &Class::AddContact, py_reference_internal,
            py::arg("element_a"), py::arg("element_b"), cls_doc.AddContact.doc)
        .def("Clear", &Class::Clear, cls_doc.Clear.doc);
    AddValueInstantiation<Class>(m);
  }

  {
    using Class = CompliantMaterial;
    constexpr auto& cls_doc = doc.CompliantMaterial;
    py::class_<Class> cls(m, "CompliantMaterial", cls_doc.doc);
    cls  // BR
        .def(py::init<>(), cls_doc.ctor.doc_0args)
        .def(py::init<double, double, double, double>(),
            py::arg("youngs_modulus"), py::arg("dissipation"),
            py::arg("static_friction"), py::arg("dynamic_friction"),
            cls_doc.ctor.doc_4args)
        // youngs_modulus
        .def("set_youngs_modulus", &Class::set_youngs_modulus, py_reference,
            cls_doc.set_youngs_modulus.doc)
        .def("youngs_modulus", &Class::youngs_modulus,
            py::arg("default_value") = Class::kDefaultYoungsModulus,
            cls_doc.youngs_modulus.doc)
        .def("youngs_modulus_is_default", &Class::youngs_modulus_is_default,
            cls_doc.youngs_modulus_is_default.doc)
        .def("set_youngs_modulus_to_default",
            &Class::set_youngs_modulus_to_default,
            cls_doc.set_youngs_modulus_to_default.doc)
        // dissipation
        .def("set_dissipation", &Class::set_dissipation, py_reference,
            cls_doc.set_dissipation.doc)
        .def("dissipation", &Class::dissipation,
            py::arg("default_value") = Class::kDefaultDissipation,
            cls_doc.dissipation.doc)
        .def("dissipation_is_default", &Class::dissipation_is_default,
            cls_doc.dissipation_is_default.doc)
        .def("set_dissipation_to_default", &Class::set_dissipation_to_default,
            cls_doc.set_dissipation_to_default.doc)
        // friction
        .def("set_friction", py::overload_cast<double>(&Class::set_friction),
            py::arg("value"), py_reference, cls_doc.set_friction.doc_1args)
        .def("set_friction",
            py::overload_cast<double, double>(&Class::set_friction),
            py::arg("static_friction"), py::arg("dynamic_friction"),
            py_reference, cls_doc.set_friction.doc_2args)
        .def("static_friction", &Class::static_friction,
            py::arg("default_value") = Class::kDefaultStaticFriction,
            cls_doc.static_friction.doc)
        .def("dynamic_friction", &Class::dynamic_friction,
            py::arg("default_value") = Class::kDefaultDynamicFriction,
            cls_doc.dynamic_friction.doc)
        .def("friction_is_default", &Class::friction_is_default,
            cls_doc.friction_is_default.doc)
        .def("set_friction_to_default", &Class::set_friction_to_default,
            cls_doc.set_friction_to_default.doc);
    cls.attr("kDefaultYoungsModulus") = Class::kDefaultYoungsModulus;
    cls.attr("kDefaultDissipation") = Class::kDefaultDissipation;
    cls.attr("kDefaultStaticFriction") = Class::kDefaultStaticFriction;
    cls.attr("kDefaultDynamicFriction") = Class::kDefaultDynamicFriction;
  }

  {
    using Class = RigidBodyPlant<T>;
    constexpr auto& cls_doc = doc.RigidBodyPlant;
    // Defined in order of declaration in `rigid_body_plant.h`.
    py::class_<Class, LeafSystem<T>>(m, "RigidBodyPlant", cls_doc.doc)
        .def(py::init<unique_ptr<const RigidBodyTree<T>>, double>(),
            py::arg("tree"), py::arg("timestep") = 0.0, cls_doc.ctor.doc)
        .def("set_contact_model_parameters",
            &Class::set_contact_model_parameters,
            cls_doc.set_contact_model_parameters.doc)
        .def("set_default_compliant_material",
            &Class::set_default_compliant_material,
            cls_doc.set_default_compliant_material.doc)
        .def("get_rigid_body_tree", &Class::get_rigid_body_tree,
            py_reference_internal, cls_doc.get_rigid_body_tree.doc)
        .def("get_num_bodies", &Class::get_num_bodies,
            cls_doc.get_num_bodies.doc)
        .def("get_num_positions",
            overload_cast_explicit<int>(&Class::get_num_positions),
            cls_doc.get_num_positions.doc_0args)
        .def("get_num_positions",
            overload_cast_explicit<int, int>(&Class::get_num_positions),
            cls_doc.get_num_positions.doc_1args)
        .def("get_num_velocities",
            overload_cast_explicit<int>(&Class::get_num_velocities),
            cls_doc.get_num_velocities.doc_0args)
        .def("get_num_velocities",
            overload_cast_explicit<int, int>(&Class::get_num_velocities),
            cls_doc.get_num_velocities.doc_1args)
        .def("get_num_states",
            overload_cast_explicit<int>(&Class::get_num_states),
            cls_doc.get_num_states.doc_0args)
        .def("get_num_states",
            overload_cast_explicit<int, int>(&Class::get_num_states),
            cls_doc.get_num_states.doc_1args)
        .def("get_num_actuators",
            overload_cast_explicit<int>(&Class::get_num_actuators),
            cls_doc.get_num_actuators.doc_0args)
        .def("get_num_actuators",
            overload_cast_explicit<int, int>(&Class::get_num_actuators),
            cls_doc.get_num_actuators.doc_1args)
        .def("get_num_model_instances", &Class::get_num_model_instances,
            cls_doc.get_num_model_instances.doc)
        .def("get_input_size", &Class::get_input_size,
            cls_doc.get_input_size.doc)
        .def("get_output_size", &Class::get_output_size,
            cls_doc.get_output_size.doc)
        .def("set_position", &Class::set_position, cls_doc.set_position.doc)
        .def("set_velocity", &Class::set_velocity, cls_doc.set_velocity.doc)
        .def("set_state_vector",
            overload_cast_explicit<void, Context<T>*,
                const Eigen::Ref<const VectorX<T>>>(&Class::set_state_vector),
            cls_doc.set_state_vector.doc)
        .def("set_state_vector",
            overload_cast_explicit<void, State<T>*,
                const Eigen::Ref<const VectorX<T>>>(&Class::set_state_vector),
            cls_doc.set_state_vector.doc)
        .def("SetDefaultState", &Class::SetDefaultState,
            cls_doc.SetDefaultState.doc)
        .def("FindInstancePositionIndexFromWorldIndex",
            &Class::FindInstancePositionIndexFromWorldIndex,
            cls_doc.FindInstancePositionIndexFromWorldIndex.doc)
        .def("actuator_command_input_port", &Class::actuator_command_input_port,
            py_reference_internal, cls_doc.actuator_command_input_port.doc)
        .def("model_instance_has_actuators",
            &Class::model_instance_has_actuators,
            cls_doc.model_instance_has_actuators.doc)
        .def("model_instance_actuator_command_input_port",
            &Class::model_instance_actuator_command_input_port,
            py_reference_internal,
            cls_doc.model_instance_actuator_command_input_port.doc)
        .def("state_output_port", &Class::state_output_port,
            py_reference_internal, cls_doc.state_output_port.doc)
        .def("state_derivative_output_port",
            &Class::state_derivative_output_port, py_reference_internal,
            cls_doc.state_derivative_output_port.doc)
        .def("model_instance_state_output_port",
            &Class::model_instance_state_output_port, py_reference_internal,
            cls_doc.model_instance_state_output_port.doc)
        .def("torque_output_port", &Class::torque_output_port,
            py_reference_internal, cls_doc.torque_output_port.doc)
        .def("model_instance_torque_output_port",
            &Class::model_instance_torque_output_port, py_reference_internal,
            cls_doc.model_instance_torque_output_port.doc)
        .def("kinematics_results_output_port",
            &Class::kinematics_results_output_port, py_reference_internal,
            cls_doc.kinematics_results_output_port.doc)
        .def("contact_results_output_port", &Class::contact_results_output_port,
            py_reference_internal, cls_doc.contact_results_output_port.doc)
        .def("GetStateVector",
            [](const Class* self,
                const Context<T>& context) -> Eigen::Ref<const VectorX<T>> {
              return self->GetStateVector(context);
            },
            py::arg("context"), py_reference,
            // Keep alive, ownership: `return` keeps `context` alive.
            py::keep_alive<0, 2>(), cls_doc.GetStateVector.doc)
        .def("is_state_discrete", &Class::is_state_discrete,
            cls_doc.is_state_discrete.doc)
        .def("get_time_step", &Class::get_time_step, cls_doc.get_time_step.doc);
  }

  {
    using Class = DrakeVisualizer;
    constexpr auto& cls_doc = doc.DrakeVisualizer;
    py::class_<Class, LeafSystem<T>>(m, "DrakeVisualizer", cls_doc.doc)
        .def(py::init<const RigidBodyTree<T>&, DrakeLcmInterface*, bool>(),
            py::arg("tree"), py::arg("lcm"), py::arg("enable_playback") = false,
            // Keep alive, reference: `self` keeps `tree` alive.
            py::keep_alive<1, 2>(),
            // Keep alive, reference: `self` keeps `lcm` alive.
            py::keep_alive<1, 3>(), cls_doc.ctor.doc)
        .def("set_publish_period", &Class::set_publish_period,
            py::arg("period"), cls_doc.set_publish_period.doc)
        .def("ReplayCachedSimulation", &Class::ReplayCachedSimulation,
            cls_doc.ReplayCachedSimulation.doc)
        .def("PublishLoadRobot", &Class::PublishLoadRobot,
            cls_doc.PublishLoadRobot.doc);
    // TODO(eric.cousineau): Bind `PlaybackTrajectory` when
    // `PiecewisePolynomial` has bindings.
  }
}

}  // namespace pydrake
}  // namespace drake
