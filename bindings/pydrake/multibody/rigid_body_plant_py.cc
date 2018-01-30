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
    using Class = RigidBodyPlant<T>;
    // Defined in order of declaration in `rigid_body_plant.h`.
    py::class_<Class, LeafSystem<T>>(m, "RigidBodyPlant")
        .def(py::init<unique_ptr<const RigidBodyTree<T>>, double>(),
             py::arg("tree"), py::arg("timestep") = 0.0)
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
