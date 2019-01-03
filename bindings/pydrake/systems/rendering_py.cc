#include <Eigen/Dense>
#include "pybind11/eigen.h"
#include "pybind11/pybind11.h"

#include "drake/bindings/pydrake/documentation_pybind.h"
#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/bindings/pydrake/systems/systems_pybind.h"
#include "drake/bindings/pydrake/util/eigen_geometry_pybind.h"
#include "drake/multibody/math/spatial_velocity.h"
#include "drake/systems/rendering/frame_velocity.h"
#include "drake/systems/rendering/multibody_position_to_geometry_pose.h"
#include "drake/systems/rendering/pose_aggregator.h"
#include "drake/systems/rendering/pose_vector.h"

namespace drake {
namespace pydrake {

PYBIND11_MODULE(rendering, m) {
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::systems;
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::systems::rendering;
  constexpr auto& doc = pydrake_doc.drake.systems.rendering;

  m.doc() = "Bindings for the rendering portion of the Systems framework.";

  py::module::import("pydrake.systems.framework");

  using T = double;

  py::class_<PoseVector<T>, BasicVector<T>> pose_vector(
      m, "PoseVector", doc.PoseVector.doc);
  pose_vector  // BR
      .def(py::init(), doc.PoseVector.ctor.doc_0args)
      .def(py::init<const Eigen::Quaternion<T>&,
               const Eigen::Translation<T, 3>&>(),
          py::arg("rotation"), py::arg("translation"),
          doc.PoseVector.ctor.doc_2args)
      .def("get_isometry", &PoseVector<T>::get_isometry,
          doc.PoseVector.get_isometry.doc)
      .def("get_translation", &PoseVector<T>::get_translation,
          doc.PoseVector.get_translation.doc)
      .def("set_translation", &PoseVector<T>::set_translation,
          doc.PoseVector.set_translation.doc)
      .def("get_rotation", &PoseVector<T>::get_rotation,
          doc.PoseVector.get_rotation.doc)
      .def("set_rotation", &PoseVector<T>::set_rotation,
          doc.PoseVector.set_rotation.doc);

  pose_vector.attr("kSize") = int{PoseVector<T>::kSize};

  py::class_<FrameVelocity<T>, BasicVector<T>> frame_velocity(
      m, "FrameVelocity", doc.FrameVelocity.doc);
  frame_velocity  // BR
      .def(py::init(), doc.FrameVelocity.ctor.doc_0args)
      .def(py::init<const multibody::SpatialVelocity<T>&>(),
          py::arg("velocity"), doc.FrameVelocity.ctor.doc_1args)
      .def("get_velocity", &FrameVelocity<T>::get_velocity,
          doc.FrameVelocity.get_velocity.doc)
      .def("set_velocity", &FrameVelocity<T>::set_velocity, py::arg("velocity"),
          doc.FrameVelocity.set_velocity.doc);

  frame_velocity.attr("kSize") = int{FrameVelocity<T>::kSize};

  py::class_<PoseBundle<T>>(m, "PoseBundle", doc.PoseBundle.doc)
      .def(py::init<int>(), py::arg("num_poses"), doc.PoseBundle.ctor.doc)
      .def("get_num_poses", &PoseBundle<T>::get_num_poses,
          doc.PoseBundle.get_num_poses.doc)
      .def("get_pose", &PoseBundle<T>::get_pose, doc.PoseBundle.get_pose.doc)
      .def("set_pose", &PoseBundle<T>::set_pose, doc.PoseBundle.set_pose.doc)
      .def("get_velocity", &PoseBundle<T>::get_velocity,
          doc.PoseBundle.get_velocity.doc)
      .def("set_velocity", &PoseBundle<T>::set_velocity,
          doc.PoseBundle.set_velocity.doc)
      .def("get_name", &PoseBundle<T>::get_name, doc.PoseBundle.get_name.doc)
      .def("set_name", &PoseBundle<T>::set_name, doc.PoseBundle.set_name.doc)
      .def("get_model_instance_id", &PoseBundle<T>::get_model_instance_id,
          doc.PoseBundle.get_model_instance_id.doc)
      .def("set_model_instance_id", &PoseBundle<T>::set_model_instance_id,
          doc.PoseBundle.set_model_instance_id.doc);
  pysystems::AddValueInstantiation<PoseBundle<T>>(m);

  py::class_<PoseVelocityInputPorts<T>>(
      m, "PoseVelocityInputPorts", doc.PoseVelocityInputPorts.doc)
      // N.B. We use lambdas below since we cannot use `def_readonly` with
      // reference members.
      .def_property_readonly("pose_input_port",
          [](PoseVelocityInputPorts<T>* self) -> const InputPort<T>& {
            return self->pose_input_port;
          },
          doc.PoseVelocityInputPorts.pose_input_port.doc)
      .def_property_readonly("velocity_input_port",
          [](PoseVelocityInputPorts<T>* self) -> const InputPort<T>& {
            return self->velocity_input_port;
          },
          doc.PoseVelocityInputPorts.velocity_input_port.doc);

  py::class_<PoseAggregator<T>, LeafSystem<T>>(
      m, "PoseAggregator", doc.PoseAggregator.doc)
      .def(py::init<>(), doc.PoseAggregator.ctor.doc)
      .def("AddSingleInput", &PoseAggregator<T>::AddSingleInput,
          py_reference_internal, doc.PoseAggregator.AddSingleInput.doc)
      .def("AddSinglePoseAndVelocityInput",
          &PoseAggregator<T>::AddSinglePoseAndVelocityInput,
          doc.PoseAggregator.AddSinglePoseAndVelocityInput.doc)
      .def("AddBundleInput", &PoseAggregator<T>::AddBundleInput,
          py_reference_internal, doc.PoseAggregator.AddBundleInput.doc);

  py::class_<MultibodyPositionToGeometryPose<T>, LeafSystem<T>>(m,
      "MultibodyPositionToGeometryPose",
      doc.MultibodyPositionToGeometryPose.doc)
      .def(py::init<const multibody::MultibodyPlant<T>&>(),
          doc.MultibodyPositionToGeometryPose.ctor.doc)
      .def("get_input_port",
          &MultibodyPositionToGeometryPose<T>::get_input_port,
          py_reference_internal,
          doc.MultibodyPositionToGeometryPose.get_input_port.doc)
      .def("get_output_port",
          &MultibodyPositionToGeometryPose<T>::get_output_port,
          py_reference_internal,
          doc.MultibodyPositionToGeometryPose.get_output_port.doc);

  // TODO(eric.cousineau): Add more systems as needed.
}

}  // namespace pydrake
}  // namespace drake
