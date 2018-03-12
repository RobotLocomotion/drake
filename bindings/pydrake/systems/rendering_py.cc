#include <Eigen/Dense>
#include "pybind11/eigen.h"
#include "pybind11/pybind11.h"

#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/bindings/pydrake/systems/systems_pybind.h"
#include "drake/bindings/pydrake/util/eigen_geometry_pybind.h"
#include "drake/multibody/multibody_tree/math/spatial_velocity.h"
#include "drake/systems/rendering/frame_velocity.h"
#include "drake/systems/rendering/pose_aggregator.h"
#include "drake/systems/rendering/pose_vector.h"

namespace drake {
namespace pydrake {

PYBIND11_MODULE(rendering, m) {
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::systems;
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::systems::rendering;

  m.doc() = "Bindings for the rendering portion of the Systems framework.";

  py::module::import("pydrake.systems.framework");

  using T = double;

  py::class_<PoseVector<T>, BasicVector<T>> pose_vector(m, "PoseVector");
  pose_vector
    .def(py::init())
    .def(py::init<const Eigen::Quaternion<T>&,
                  const Eigen::Translation<T, 3>&>(),
         py::arg("rotation"), py::arg("translation"))
    .def("get_isometry", &PoseVector<T>::get_isometry)
    .def("get_translation", &PoseVector<T>::get_translation)
    .def("set_translation", &PoseVector<T>::set_translation)
    .def("get_rotation", &PoseVector<T>::get_rotation)
    .def("set_rotation", &PoseVector<T>::set_rotation);

  pose_vector.attr("kSize") = int{PoseVector<T>::kSize};

  py::class_<FrameVelocity<T>, BasicVector<T>> frame_velocity(
      m, "FrameVelocity");
  frame_velocity
    .def(py::init())
    .def(py::init<const multibody::SpatialVelocity<T>&>(), py::arg("velocity"))
    .def("get_velocity", &FrameVelocity<T>::get_velocity)
    .def("set_velocity", &FrameVelocity<T>::set_velocity, py::arg("velocity"));

  frame_velocity.attr("kSize") = int{FrameVelocity<T>::kSize};

  py::class_<PoseBundle<T>>(m, "PoseBundle")
    .def(py::init<int>())
    .def("get_num_poses", &PoseBundle<T>::get_num_poses)
    .def("get_pose", &PoseBundle<T>::get_pose)
    .def("set_pose", &PoseBundle<T>::set_pose)
    .def("get_velocity", &PoseBundle<T>::get_velocity)
    .def("set_velocity", &PoseBundle<T>::set_velocity)
    .def("get_name", &PoseBundle<T>::get_name)
    .def("set_name", &PoseBundle<T>::set_name)
    .def("get_model_instance_id", &PoseBundle<T>::get_model_instance_id)
    .def("set_model_instance_id", &PoseBundle<T>::set_model_instance_id);
  pysystems::AddValueInstantiation<PoseBundle<T>>(m);

  py::class_<PoseVelocityInputPortDescriptors<T>>(
      m, "PoseVelocityInputPortDescriptors")
      .def("pose_descriptor",
           [](PoseVelocityInputPortDescriptors<T>* self) ->
           const InputPortDescriptor<T>& {
             return self->pose_descriptor;
           }, py_reference_internal)
      .def("velocity_descriptor",
           [](PoseVelocityInputPortDescriptors<T>* self) ->
           const InputPortDescriptor<T>& {
             return self->velocity_descriptor;
           }, py_reference_internal);

  py::class_<PoseAggregator<T>, LeafSystem<T>>(m, "PoseAggregator")
    .def(py::init<>())
    .def("AddSingleInput", &PoseAggregator<T>::AddSingleInput,
         py_reference_internal)
    .def("AddSinglePoseAndVelocityInput",
         &PoseAggregator<T>::AddSinglePoseAndVelocityInput)
    .def("AddBundleInput", &PoseAggregator<T>::AddBundleInput,
         py_reference_internal);

  // TODO(eric.cousineau): Add more systems as needed.
}

}  // namespace pydrake
}  // namespace drake
