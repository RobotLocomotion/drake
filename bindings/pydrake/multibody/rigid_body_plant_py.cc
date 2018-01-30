#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>

#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/multibody/rigid_body_plant/rigid_body_plant.h"

using std::unique_ptr;
using std::vector;

namespace drake {
namespace pydrake {

PYBIND11_MODULE(rigid_body_plant, m) {
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::systems;

  // Ensure we have bindings for dependencies.
  py::module::import("pydrake.rbtree");
  py::module::import("pydrake.systems.framework");

  using T = double;

  {
    using Class = CompliantContactModelParameters;
    py::class_<Class> cls(m, "CompliantContactModelParameters");
    cls
        .def(
            py::init(
                [](double v_stiction_tolerance, double characteristic_radius) {
                  return Class{v_stiction_tolerance, characteristic_radius};
                }),
            py::arg("v_stiction_tolerance") =
                Class::kDefaultVStictionTolerance,
            py::arg("characteristic_radius") =
                Class::kDefaultCharacteristicRadius)
        .def_readwrite("v_stiction_tolerance", &Class::v_stiction_tolerance)
        .def_readwrite("characteristic_radius", &Class::characteristic_radius);
    cls.attr("kDefaultVStictionTolerance") =
        Class::kDefaultVStictionTolerance;
    cls.attr("kDefaultCharacteristicRadius") =
        Class::kDefaultCharacteristicRadius;
  }

  {
    using Class = CompliantMaterial;
    py::class_<Class> cls(m, "CompliantMaterial");
    cls
        .def(py::init<>())
        .def(py::init<double, double, double, double>(),
             py::arg("youngs_modulus"),
             py::arg("dissipation"),
             py::arg("static_friction"),
             py::arg("dynamic_friction"))
        // youngs_modulus
        .def("set_youngs_modulus", &Class::set_youngs_modulus, py_reference)
        .def("youngs_modulus", &Class::youngs_modulus,
             py::arg("default_value") = Class::kDefaultYoungsModulus)
        .def("youngs_modulus_is_default", &Class::youngs_modulus_is_default)
        .def("set_youngs_modulus_to_default",
             &Class::set_youngs_modulus_to_default)
        // dissipation
        .def("set_dissipation", &Class::set_dissipation, py_reference)
        .def("dissipation", &Class::dissipation,
             py::arg("default_value") = Class::kDefaultDissipation)
        .def("dissipation_is_default", &Class::dissipation_is_default)
        .def("set_dissipation_to_default", &Class::set_dissipation_to_default)
        // friction
        .def("set_friction",
             py::overload_cast<double>(&Class::set_friction),
             py::arg("value"), py_reference)
        .def("set_friction",
             py::overload_cast<double, double>(&Class::set_friction),
             py::arg("static_friction"), py::arg("dynamic_friction"),
             py_reference)
        .def("static_friction", &Class::static_friction,
             py::arg("default_value") = Class::kDefaultStaticFriction)
        .def("dynamic_friction", &Class::dynamic_friction,
             py::arg("default_value") = Class::kDefaultDynamicFriction)
        .def("friction_is_default", &Class::friction_is_default)
        .def("set_friction_to_default", &Class::set_friction_to_default);
    cls.attr("kDefaultYoungsModulus") = Class::kDefaultYoungsModulus;
    cls.attr("kDefaultDissipation") = Class::kDefaultDissipation;
    cls.attr("kDefaultStaticFriction") = Class::kDefaultStaticFriction;
    cls.attr("kDefaultDynamicFriction") = Class::kDefaultDynamicFriction;
  }

  {
    using Class = RigidBodyPlant<T>;
    // Defined in order of declaration in `rigid_body_plant.h`.
    py::class_<Class, LeafSystem<T>>(m, "RigidBodyPlant")
        .def(py::init<unique_ptr<const RigidBodyTree<T>>, double>(),
             py::arg("tree"), py::arg("timestep") = 0.0)
        .def("set_contact_model_parameters",
             &Class::set_contact_model_parameters)
        .def("set_default_compliant_material",
             &Class::set_default_compliant_material)
        .def("get_rigid_body_tree", &Class::get_rigid_body_tree,
             py_reference_internal)
        .def("get_num_bodies", &Class::get_num_bodies)
        .def("get_num_positions",
             overload_cast_explicit<int>(&Class::get_num_positions))
        .def("get_num_positions",
             overload_cast_explicit<int, int>(&Class::get_num_positions))
        .def("get_num_velocities",
             overload_cast_explicit<int>(&Class::get_num_velocities))
        .def("get_num_velocities",
             overload_cast_explicit<int, int>(&Class::get_num_velocities))
        .def("get_num_states",
             overload_cast_explicit<int>(&Class::get_num_states))
        .def("get_num_states",
             overload_cast_explicit<int, int>(&Class::get_num_states))
        .def("get_num_actuators",
             overload_cast_explicit<int>(&Class::get_num_actuators))
        .def("get_num_actuators",
             overload_cast_explicit<int, int>(&Class::get_num_actuators))
        .def("get_num_model_instances", &Class::get_num_model_instances)
        .def("get_input_size", &Class::get_input_size)
        .def("get_output_size", &Class::get_output_size)
        .def("set_position", &Class::set_position)
        .def("set_velocity", &Class::set_velocity)
        .def("set_state_vector",
             overload_cast_explicit<
                 void, Context<T>*, const Eigen::Ref<const VectorX<T>>>(
                     &Class::set_state_vector))
        .def("set_state_vector",
             overload_cast_explicit<
                 void, State<T>*, const Eigen::Ref<const VectorX<T>>>(
                     &Class::set_state_vector))
        .def("SetDefaultState", &Class::SetDefaultState)
        .def("FindInstancePositionIndexFromWorldIndex",
             &Class::FindInstancePositionIndexFromWorldIndex)
        .def("actuator_command_input_port",
             &Class::actuator_command_input_port,
             py_reference_internal)
        .def("model_instance_has_actuators",
             &Class::model_instance_has_actuators)
        .def("model_instance_actuator_command_input_port",
             &Class::model_instance_actuator_command_input_port,
             py_reference_internal)
        .def("state_output_port",
             &Class::state_output_port, py_reference_internal)
        .def("model_instance_state_output_port",
             &Class::model_instance_state_output_port, py_reference_internal)
        .def("torque_output_port", &Class::torque_output_port,
             py_reference_internal)
        .def("model_instance_torque_output_port",
             &Class::model_instance_torque_output_port, py_reference_internal)
        .def("kinematics_results_output_port",
             &Class::kinematics_results_output_port, py_reference_internal)
        .def("contact_results_output_port",
             &Class::contact_results_output_port, py_reference_internal)
        .def("GetStateVector",
             [](const Class* self, const Context<T>& context)
                    -> Eigen::Ref<const VectorX<T>> {
                return self->GetStateVector(context);
             }, py_reference,
             // Keep alive, ownership: `return` keeps `Context` alive.
             py::keep_alive<0, 2>())
        .def("is_state_discrete", &Class::is_state_discrete)
        .def("get_time_step", &Class::get_time_step);
  }
}

}  // namespace pydrake
}  // namespace drake
