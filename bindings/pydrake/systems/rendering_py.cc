#include "pybind11/eigen.h"
#include "pybind11/pybind11.h"
#include <Eigen/Dense>

#include "drake/bindings/pydrake/common/deprecation_pybind.h"
#include "drake/bindings/pydrake/common/eigen_geometry_pybind.h"
#include "drake/bindings/pydrake/common/value_pybind.h"
#include "drake/bindings/pydrake/documentation_pybind.h"
#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/multibody/math/spatial_algebra.h"
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

  // Basically *all* of these classes/structs are deprecated, so we'll put the
  // whole block of them inside the pragma.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"

  // Because we've deprecated the *classes* and not the methods, applying
  // deprecation to the constructors requires augmenting the doc string into
  // something with an appropriately formatted deprecation warning.
  const std::string deprecated_rendering(
      " This class has been deprecated and will be removed on or after "
      "2021-12-01.");

  py::class_<PoseVector<T>, BasicVector<T>> pose_vector(
      m, "PoseVector", doc.PoseVector.doc_deprecated);
  pose_vector  // BR
      .def(py_init_deprecated<PoseVector<T>>(
               doc.PoseVector.ctor.doc_0args + deprecated_rendering),
          (doc.PoseVector.ctor.doc_0args + deprecated_rendering).c_str())
      .def(py_init_deprecated<PoseVector<T>, const Eigen::Quaternion<T>&,
               const Eigen::Translation<T, 3>&>(
               doc.PoseVector.ctor.doc_2args + deprecated_rendering),
          py::arg("rotation"), py::arg("translation"),
          (doc.PoseVector.ctor.doc_2args + deprecated_rendering).c_str())
      .def("get_transform", &PoseVector<T>::get_transform,
          doc.PoseVector.get_transform.doc)
      .def("set_transform", &PoseVector<T>::set_transform,
          doc.PoseVector.set_transform.doc)
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
      m, "FrameVelocity", doc.FrameVelocity.doc_deprecated);
  frame_velocity  // BR
      .def(py_init_deprecated<FrameVelocity<T>>(
               doc.FrameVelocity.ctor.doc_0args + deprecated_rendering),
          (doc.FrameVelocity.ctor.doc_0args + deprecated_rendering).c_str())
      .def(py_init_deprecated<FrameVelocity<T>,
               const multibody::SpatialVelocity<T>&>(
               doc.FrameVelocity.ctor.doc_1args + deprecated_rendering),
          py::arg("velocity"),
          (doc.FrameVelocity.ctor.doc_1args + deprecated_rendering).c_str())
      .def("get_velocity", &FrameVelocity<T>::get_velocity,
          doc.FrameVelocity.get_velocity.doc)
      .def("set_velocity", &FrameVelocity<T>::set_velocity, py::arg("velocity"),
          doc.FrameVelocity.set_velocity.doc);

  frame_velocity.attr("kSize") = int{FrameVelocity<T>::kSize};

  py::class_<PoseBundle<T>> pose_bundle(
      m, "PoseBundle", doc.PoseBundle.doc_deprecated);
  pose_bundle
      .def(py_init_deprecated<PoseBundle<T>, int>(
               doc.PoseBundle.ctor.doc + deprecated_rendering),
          py::arg("num_poses"),
          (doc.PoseBundle.ctor.doc + deprecated_rendering).c_str())
      .def("get_num_poses", &PoseBundle<T>::get_num_poses,
          doc.PoseBundle.get_num_poses.doc)
      .def("get_transform", &PoseBundle<T>::get_transform,
          doc.PoseBundle.get_transform.doc)
      .def("set_transform", &PoseBundle<T>::set_transform,
          doc.PoseBundle.set_transform.doc)
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
  AddValueInstantiation<PoseBundle<T>>(m);

  py::class_<PoseVelocityInputPorts<T>> pose_vel_input_port_cls(
      m, "PoseVelocityInputPorts", doc.PoseVelocityInputPorts.doc_deprecated);
  pose_vel_input_port_cls
      // N.B. We use lambdas below since we cannot use `def_readonly` with
      // reference members.
      .def_property_readonly(
          "pose_input_port",
          [](PoseVelocityInputPorts<T>* self) -> const InputPort<T>& {
            return self->pose_input_port;
          },
          doc.PoseVelocityInputPorts.pose_input_port.doc)
      .def_property_readonly(
          "velocity_input_port",
          [](PoseVelocityInputPorts<T>* self) -> const InputPort<T>& {
            return self->velocity_input_port;
          },
          doc.PoseVelocityInputPorts.velocity_input_port.doc);
  DeprecateAttribute(
      pose_vel_input_port_cls, "pose_input_port", deprecated_rendering);
  DeprecateAttribute(
      pose_vel_input_port_cls, "velocity_input_port", deprecated_rendering);

  py::class_<PoseAggregator<T>, LeafSystem<T>>(
      m, "PoseAggregator", doc.PoseAggregator.doc_deprecated)
      .def(py_init_deprecated<PoseAggregator<T>>(
               doc.PoseAggregator.ctor.doc + deprecated_rendering),
          (doc.PoseAggregator.ctor.doc + deprecated_rendering).c_str())
      .def("AddSingleInput", &PoseAggregator<T>::AddSingleInput,
          py_rvp::reference_internal, doc.PoseAggregator.AddSingleInput.doc)
      .def("AddSinglePoseAndVelocityInput",
          &PoseAggregator<T>::AddSinglePoseAndVelocityInput,
          doc.PoseAggregator.AddSinglePoseAndVelocityInput.doc)
      .def("AddBundleInput", &PoseAggregator<T>::AddBundleInput,
          py_rvp::reference_internal, doc.PoseAggregator.AddBundleInput.doc);

#pragma GCC diagnostic pop

  // See the todo in multibody_position_to_geometry_pose.h. This should
  // ultimately move into a different module.
  py::class_<MultibodyPositionToGeometryPose<T>, LeafSystem<T>>(m,
      "MultibodyPositionToGeometryPose",
      doc.MultibodyPositionToGeometryPose.doc)
      .def(py::init<const multibody::MultibodyPlant<T>&, bool>(),
          py::arg("plant"), py::arg("input_multibody_state") = false,
          // Keep alive, reference: `self` keeps `plant` alive.
          py::keep_alive<1, 2>(),
          doc.MultibodyPositionToGeometryPose.ctor
              .doc_2args_plant_input_multibody_state);
}

}  // namespace pydrake
}  // namespace drake
