#include "drake/bindings/pydrake/multibody/tree_py.h"

#include <limits>
#include <memory>
#include <set>
#include <string>
#include <utility>

#include "pybind11/eval.h"

#include "drake/bindings/generated_docstrings/multibody_tree.h"
#include "drake/bindings/pydrake/common/cpp_template_pybind.h"
#include "drake/bindings/pydrake/common/default_scalars_pybind.h"
#include "drake/bindings/pydrake/common/deprecation_pybind.h"
#include "drake/bindings/pydrake/common/eigen_pybind.h"
#include "drake/bindings/pydrake/common/identifier_pybind.h"
#include "drake/bindings/pydrake/common/serialize_pybind.h"
#include "drake/bindings/pydrake/common/type_pack.h"
#include "drake/bindings/pydrake/common/type_safe_index_pybind.h"
#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/tree/ball_rpy_joint.h"
#include "drake/multibody/tree/door_hinge.h"
#include "drake/multibody/tree/force_density_field.h"
#include "drake/multibody/tree/force_element.h"
#include "drake/multibody/tree/frame.h"
#include "drake/multibody/tree/joint.h"
#include "drake/multibody/tree/joint_actuator.h"
#include "drake/multibody/tree/linear_bushing_roll_pitch_yaw.h"
#include "drake/multibody/tree/linear_spring_damper.h"
#include "drake/multibody/tree/multibody_forces.h"
#include "drake/multibody/tree/multibody_tree.h"
#include "drake/multibody/tree/multibody_tree_indexes.h"
#include "drake/multibody/tree/planar_joint.h"
#include "drake/multibody/tree/prismatic_joint.h"
#include "drake/multibody/tree/prismatic_spring.h"
#include "drake/multibody/tree/quaternion_floating_joint.h"
#include "drake/multibody/tree/revolute_joint.h"
#include "drake/multibody/tree/revolute_spring.h"
#include "drake/multibody/tree/rigid_body.h"
#include "drake/multibody/tree/rpy_floating_joint.h"
#include "drake/multibody/tree/screw_joint.h"
#include "drake/multibody/tree/universal_joint.h"
#include "drake/multibody/tree/weld_joint.h"

namespace drake {
namespace pydrake {

using Eigen::Vector3d;
using std::string;

using math::RigidTransform;
using multibody::SpatialAcceleration;
using multibody::SpatialVelocity;

namespace {

// NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
using namespace drake::multibody;
constexpr auto& doc = pydrake_doc_multibody_tree.drake.multibody;

// Negative case for checking T::name().
// https://stackoverflow.com/a/16000226/7829525
template <typename T, typename = void>
struct has_name_func : std::false_type {};

// Positive case for checking T::name().
template <typename T>
struct has_name_func<T, decltype(std::declval<T>().name(), void())>
    : std::true_type {};

// Binds `MultibodyElement` methods.
// N.B. We do this rather than inheritance because this template is more of a
// mixin than it is a parent class (since it is not used for its dynamic
// polymorphism).
// TODO(jamiesnape): Add documentation for bindings generated with this
// function.
template <typename T, typename PyClass>
void BindMultibodyElementMixin(PyClass* pcls) {
  using Class = typename PyClass::type;
  // TODO(eric.cousineau): Fix docstring generation for `MultibodyElement`.
  auto& cls = *pcls;
  cls  // BR
      .def("index", &Class::index)
      .def("model_instance", &Class::model_instance)
      .def("is_ephemeral", &Class::is_ephemeral)
      .def("GetParentPlant",
          [](const Class& self) -> const multibody::MultibodyPlant<T>& {
            return self.GetParentPlant();
          })
      .def("__repr__", [](const Class& self) {
        py::str cls_name =
            internal::PrettyClassName(py::cast(&self).get_type());
        const int index = self.index();
        const int model_instance = self.model_instance();
        if constexpr (has_name_func<Class>::value) {
          return py::str("<{} name='{}' index={} model_instance={}>")
              .format(cls_name, self.name(), index, model_instance);
        } else {
          return py::str("<{} index={} model_instance={}>")
              .format(cls_name, index, model_instance);
        }
      });
}

void DoScalarIndependentDefinitions(py::module m) {
  // To simplify checking binding coverage, these are defined in the same order
  // as `multibody_tree_indexes.h`.
  BindTypeSafeIndex<FrameIndex>(m, "FrameIndex", doc.FrameIndex.doc);
  BindTypeSafeIndex<BodyIndex>(m, "BodyIndex", doc.BodyIndex.doc);
  BindTypeSafeIndex<ForceElementIndex>(
      m, "ForceElementIndex", doc.ForceElementIndex.doc);
  BindTypeSafeIndex<JointIndex>(m, "JointIndex", doc.JointIndex.doc);
  BindTypeSafeIndex<JointActuatorIndex>(
      m, "JointActuatorIndex", doc.JointActuatorIndex.doc);
  BindIdentifier<MultibodyConstraintId>(
      m, "MultibodyConstraintId", doc.MultibodyConstraintId.doc);
  BindTypeSafeIndex<ModelInstanceIndex>(
      m, "ModelInstanceIndex", doc.ModelInstanceIndex.doc);
  BindIdentifier<DeformableBodyId>(
      m, "DeformableBodyId", doc.DeformableBodyId.doc);
  BindTypeSafeIndex<DeformableBodyIndex>(
      m, "DeformableBodyIndex", doc.DeformableBodyIndex.doc);
  m.def("world_index", &world_index, doc.world_index.doc);
  m.def("world_frame_index", &world_frame_index, doc.world_frame_index.doc);
  m.def("world_model_instance", &world_model_instance,
      doc.world_model_instance.doc);
  m.def("default_model_instance", &default_model_instance,
      doc.default_model_instance.doc);

  {
    using Class = DoorHingeConfig;
    constexpr auto& cls_doc = doc.DoorHingeConfig;
    py::class_<Class> cls(m, "DoorHingeConfig", cls_doc.doc);
    cls  // BR
        .def(ParamInit<Class>(), cls_doc.ctor.doc);
    DefAttributesUsingSerialize(&cls, cls_doc);
    DefReprUsingSerialize(&cls);
    DefCopyAndDeepCopy(&cls);
  }

  {
    using Enum = JacobianWrtVariable;
    constexpr auto& enum_doc = doc.JacobianWrtVariable;
    py::enum_<Enum> enum_py(m, "JacobianWrtVariable", enum_doc.doc);
    enum_py  // BR
        .value("kQDot", Enum::kQDot, enum_doc.kQDot.doc)
        .value("kV", Enum::kV, enum_doc.kV.doc);
  }

  {
    using Class = ScopedName;
    constexpr auto& cls_doc = doc.ScopedName;
    py::class_<Class> cls(m, "ScopedName", cls_doc.doc);
    cls  // BR
        .def(py::init<>(), cls_doc.ctor.doc_0args)
        .def(py::init<std::string_view, std::string_view>(),
            py::arg("namespace_name"), py::arg("element_name"),
            cls_doc.ctor.doc_2args)
        .def_static("Make", &Class::Make, py::arg("namespace_name"),
            py::arg("element_name"), cls_doc.Make.doc)
        .def_static("Join", &Class::Join, py::arg("name1"), py::arg("name2"),
            cls_doc.Join.doc)
        .def_static(
            "Parse", &Class::Parse, py::arg("scoped_name"), cls_doc.Parse.doc)
        .def("get_namespace", &Class::get_namespace, cls_doc.get_namespace.doc)
        .def("get_element", &Class::get_element, cls_doc.get_element.doc)
        .def("get_full", &Class::get_full, cls_doc.get_full.doc)
        .def("to_string", &Class::to_string, cls_doc.to_string.doc)
        .def("set_namespace", &Class::set_namespace, py::arg("namespace_name"),
            cls_doc.set_namespace.doc)
        .def("set_element", &Class::set_element, py::arg("element_name"),
            cls_doc.set_element.doc)
        .def("__str__", &Class::to_string, cls_doc.to_string.doc)
        .def("__repr__", [](const Class& self) {
          py::str py_namespace = std::string{self.get_namespace()};
          py::str py_element = std::string{self.get_element()};
          return fmt::format("ScopedName({}, {})",
              fmt_streamed(py::repr(py_namespace)),
              fmt_streamed(py::repr(py_element)));
        });
    DefCopyAndDeepCopy(&cls);
  }

  {
    using Class = PdControllerGains;
    constexpr auto& cls_doc = doc.PdControllerGains;
    py::class_<Class> cls(m, "PdControllerGains", cls_doc.doc);
    cls  // BR
        .def(ParamInit<Class>());
    cls  // BR
        .def_readwrite("p", &Class::p, cls_doc.p.doc)
        .def_readwrite("d", &Class::d, cls_doc.d.doc);
    DefCopyAndDeepCopy(&cls);
  }
}

// TODO(jwnimmer-tri) This function is just a grab-bag of several classes. We
// should split it up into smaller pieces.
template <typename T>
void DoScalarDependentDefinitions(py::module m, T) {
  py::tuple param = GetPyParam<T>();

  // Frames.
  {
    using Class = Frame<T>;
    constexpr auto& cls_doc = doc.Frame;
    auto cls =
        DefineTemplateClassWithDefault<Class>(m, "Frame", param, cls_doc.doc);
    BindMultibodyElementMixin<T>(&cls);
    cls  // BR
        .def("body", &Class::body, py_rvp::reference_internal, cls_doc.body.doc)
        .def("is_world_frame", &Class::is_world_frame,
            cls_doc.is_world_frame.doc)
        .def("is_body_frame", &Class::is_body_frame, cls_doc.is_body_frame.doc)
        .def("name", &Class::name, cls_doc.name.doc)
        .def("scoped_name", &Class::scoped_name, cls_doc.scoped_name.doc)
        .def("GetFixedPoseInBodyFrame", &Frame<T>::GetFixedPoseInBodyFrame,
            cls_doc.GetFixedPoseInBodyFrame.doc)
        .def("CalcPoseInBodyFrame", &Frame<T>::CalcPoseInBodyFrame,
            py::arg("context"), cls_doc.CalcPoseInBodyFrame.doc)
        .def("CalcRotationMatrixInBodyFrame",
            &Frame<T>::CalcRotationMatrixInBodyFrame, py::arg("context"),
            cls_doc.CalcRotationMatrixInBodyFrame.doc)
        .def("GetFixedPoseInBodyFrame", &Class::GetFixedPoseInBodyFrame,
            cls_doc.GetFixedPoseInBodyFrame.doc)
        .def("GetFixedRotationMatrixInBodyFrame",
            &Class::GetFixedRotationMatrixInBodyFrame,
            cls_doc.GetFixedRotationMatrixInBodyFrame.doc)
        .def("CalcOffsetPoseInBody",
            overload_cast_explicit<RigidTransform<T>,
                const systems::Context<T>&, const RigidTransform<T>&>(
                &Class::CalcOffsetPoseInBody),
            py::arg("context"), py::arg("X_FQ"),
            cls_doc.CalcOffsetPoseInBody.doc)
        .def("CalcOffsetRotationMatrixInBody",
            overload_cast_explicit<math::RotationMatrix<T>,
                const systems::Context<T>&, const math::RotationMatrix<T>&>(
                &Class::CalcOffsetRotationMatrixInBody),
            py::arg("context"), py::arg("R_FQ"),
            cls_doc.CalcOffsetRotationMatrixInBody.doc)
        .def("GetFixedOffsetPoseInBody", &Class::GetFixedOffsetPoseInBody,
            py::arg("X_FQ"), cls_doc.GetFixedOffsetPoseInBody.doc)
        .def("GetFixedRotationMatrixInBody",
            &Class::GetFixedRotationMatrixInBody, py::arg("R_FQ"),
            cls_doc.GetFixedRotationMatrixInBody.doc)
        .def("CalcPoseInWorld", &Class::CalcPoseInWorld, py::arg("context"),
            cls_doc.CalcPoseInWorld.doc)
        .def("CalcPose", &Class::CalcPose, py::arg("context"),
            py::arg("frame_M"), cls_doc.CalcPose.doc)
        .def("CalcRotationMatrix", &Class::CalcRotationMatrix,
            py::arg("context"), py::arg("frame_M"),
            cls_doc.CalcRotationMatrix.doc)
        .def("CalcRotationMatrixInWorld", &Class::CalcRotationMatrixInWorld,
            py::arg("context"), cls_doc.CalcRotationMatrixInWorld.doc)
        .def("EvalAngularVelocityInWorld", &Class::EvalAngularVelocityInWorld,
            py::arg("context"), cls_doc.EvalAngularVelocityInWorld.doc)
        .def("CalcAngularVelocity", &Class::CalcAngularVelocity,
            py::arg("context"), py::arg("measured_in_frame"),
            py::arg("expressed_in_frame"), cls_doc.CalcAngularVelocity.doc)
        .def("CalcSpatialVelocityInWorld", &Class::CalcSpatialVelocityInWorld,
            py::arg("context"), cls_doc.CalcSpatialVelocityInWorld.doc)
        .def("CalcSpatialVelocity", &Class::CalcSpatialVelocity,
            py::arg("context"), py::arg("frame_M"), py::arg("frame_E"),
            cls_doc.CalcSpatialVelocity.doc)
        .def("CalcRelativeSpatialVelocityInWorld",
            &Class::CalcRelativeSpatialVelocityInWorld, py::arg("context"),
            py::arg("other_frame"),
            cls_doc.CalcRelativeSpatialVelocityInWorld.doc)
        .def("CalcRelativeSpatialVelocity", &Class::CalcRelativeSpatialVelocity,
            py::arg("context"), py::arg("other_frame"),
            py::arg("measured_in_frame"), py::arg("expressed_in_frame"),
            cls_doc.CalcRelativeSpatialVelocity.doc)
        .def("CalcSpatialAccelerationInWorld",
            &Class::CalcSpatialAccelerationInWorld, py::arg("context"),
            cls_doc.CalcSpatialAccelerationInWorld.doc)
        .def("CalcSpatialAcceleration", &Class::CalcSpatialAcceleration,
            py::arg("context"), py::arg("measured_in_frame"),
            py::arg("expressed_in_frame"), cls_doc.CalcSpatialAcceleration.doc)
        .def("CalcRelativeSpatialAccelerationInWorld",
            &Class::CalcRelativeSpatialAccelerationInWorld, py::arg("context"),
            py::arg("other_frame"),
            cls_doc.CalcRelativeSpatialAccelerationInWorld.doc)
        .def("CalcRelativeSpatialAcceleration",
            &Class::CalcRelativeSpatialAcceleration, py::arg("context"),
            py::arg("other_frame"), py::arg("measured_in_frame"),
            py::arg("expressed_in_frame"),
            cls_doc.CalcRelativeSpatialAcceleration.doc);
  }

  {
    using Class = RigidBodyFrame<T>;
    constexpr auto& cls_doc = doc.RigidBodyFrame;
    auto cls = DefineTemplateClassWithDefault<Class, Frame<T>>(
        m, "RigidBodyFrame", param, cls_doc.doc);
    // No need to re-bind element mixins from `Frame`.
  }

  {
    using Class = FixedOffsetFrame<T>;
    constexpr auto& cls_doc = doc.FixedOffsetFrame;
    auto cls = DefineTemplateClassWithDefault<Class, Frame<T>>(
        m, "FixedOffsetFrame", param, cls_doc.doc);
    cls  // BR
        .def(py::init<const std::string&, const Frame<T>&,
                 const RigidTransform<double>&,
                 std::optional<ModelInstanceIndex>>(),
            py::arg("name"), py::arg("P"), py::arg("X_PF"),
            py::arg("model_instance") = std::nullopt, cls_doc.ctor.doc_4args)
        .def(py::init<const std::string&, const RigidBody<T>&,
                 const math::RigidTransform<double>&>(),
            py::arg("name"), py::arg("bodyB"), py::arg("X_BF"),
            cls_doc.ctor.doc_3args)
        .def("SetPoseInParentFrame", &Class::SetPoseInParentFrame,
            py::arg("context"), py::arg("X_PF"),
            cls_doc.SetPoseInParentFrame.doc)
        .def("GetPoseInParentFrame", &Class::GetPoseInParentFrame,
            py::arg("context"), cls_doc.GetPoseInParentFrame.doc);
  }

  // Rigid bodies.
  {
    using Class = RigidBody<T>;
    constexpr auto& cls_doc = doc.RigidBody;
    auto cls = DefineTemplateClassWithDefault<Class>(
        m, "RigidBody", param, cls_doc.doc);
    BindMultibodyElementMixin<T>(&cls);
    cls  // BR
        .def(py::init<const std::string&, const SpatialInertia<double>&>(),
            py::arg("body_name"),
            py::arg("M_BBo_B") = SpatialInertia<double>::Zero(),
            cls_doc.ctor.doc_2args)
        .def(py::init<const std::string&, ModelInstanceIndex,
                 const SpatialInertia<double>&>(),
            py::arg("body_name"), py::arg("model_instance"),
            py::arg("M_BBo_B") = SpatialInertia<double>::Zero(),
            cls_doc.ctor.doc_3args)
        .def("name", &Class::name, cls_doc.name.doc)
        .def("scoped_name", &Class::scoped_name, cls_doc.scoped_name.doc)
        .def("body_frame", &Class::body_frame, py_rvp::reference_internal,
            cls_doc.body_frame.doc)
        .def("is_floating_base_body", &Class::is_floating_base_body,
            cls_doc.is_floating_base_body.doc)
        .def("has_quaternion_dofs", &Class::has_quaternion_dofs,
            cls_doc.has_quaternion_dofs.doc)
        .def("floating_positions_start", &Class::floating_positions_start,
            cls_doc.floating_positions_start.doc)
        .def("floating_velocities_start_in_v",
            &Class::floating_velocities_start_in_v,
            cls_doc.floating_velocities_start_in_v.doc)
        .def("floating_position_suffix", &Class::floating_position_suffix,
            cls_doc.floating_position_suffix.doc)
        .def("floating_velocity_suffix", &Class::floating_velocity_suffix,
            cls_doc.floating_velocity_suffix.doc)
        .def("default_mass", &Class::default_mass, cls_doc.default_mass.doc)
        .def("get_mass", &Class::get_mass, py::arg("context"),
            cls_doc.get_mass.doc)
        .def("CalcCenterOfMassInBodyFrame", &Class::CalcCenterOfMassInBodyFrame,
            py::arg("context"), cls_doc.CalcCenterOfMassInBodyFrame.doc)
        .def("CalcCenterOfMassTranslationalVelocityInWorld",
            &Class::CalcCenterOfMassTranslationalVelocityInWorld,
            py::arg("context"),
            cls_doc.CalcCenterOfMassTranslationalVelocityInWorld.doc)
        .def("CalcCenterOfMassTranslationalAccelerationInWorld",
            &Class::CalcCenterOfMassTranslationalAccelerationInWorld,
            py::arg("context"),
            cls_doc.CalcCenterOfMassTranslationalAccelerationInWorld.doc)
        .def("CalcSpatialInertiaInBodyFrame",
            &Class::CalcSpatialInertiaInBodyFrame, py::arg("context"),
            cls_doc.CalcSpatialInertiaInBodyFrame.doc)
        .def("EvalPoseInWorld", &Class::EvalPoseInWorld, py::arg("context"),
            cls_doc.EvalPoseInWorld.doc)
        .def("EvalSpatialVelocityInWorld", &Class::EvalSpatialVelocityInWorld,
            py::arg("context"), cls_doc.EvalSpatialVelocityInWorld.doc)
        .def("EvalSpatialAccelerationInWorld",
            &Class::EvalSpatialAccelerationInWorld, py::arg("context"),
            cls_doc.EvalSpatialAccelerationInWorld.doc)
        .def("GetForceInWorld", &Class::GetForceInWorld, py::arg("context"),
            py::arg("forces"), cls_doc.GetForceInWorld.doc)
        .def("AddInForceInWorld", &Class::AddInForceInWorld, py::arg("context"),
            py::arg("F_Bo_W"), py::arg("forces"), cls_doc.AddInForceInWorld.doc)
        .def("AddInForce", &Class::AddInForce, py::arg("context"),
            py::arg("p_BP_E"), py::arg("F_Bp_E"), py::arg("frame_E"),
            py::arg("forces"), cls_doc.AddInForce.doc)
        .def("Lock", &Class::Lock, py::arg("context"), cls_doc.Lock.doc)
        .def("Unlock", &Class::Unlock, py::arg("context"), cls_doc.Unlock.doc)
        .def("is_locked", &Class::is_locked, py::arg("context"),
            cls_doc.is_locked.doc)
        .def("default_mass", &Class::default_mass, cls_doc.default_mass.doc)
        .def("default_com", &Class::default_com, py_rvp::reference_internal,
            cls_doc.default_com.doc)
        .def("default_unit_inertia", &Class::default_unit_inertia,
            py_rvp::reference_internal, cls_doc.default_unit_inertia.doc)
        .def("default_rotational_inertia", &Class::default_rotational_inertia,
            py_rvp::reference_internal, cls_doc.default_rotational_inertia.doc)
        .def("default_spatial_inertia", &Class::default_spatial_inertia,
            py_rvp::reference_internal, cls_doc.default_spatial_inertia.doc)
        .def("SetMass", &Class::SetMass, py::arg("context"), py::arg("mass"),
            cls_doc.SetMass.doc)
        .def("SetCenterOfMassInBodyFrame", &Class::SetCenterOfMassInBodyFrame,
            py::arg("context"), py::arg("com"),
            cls_doc.SetCenterOfMassInBodyFrame.doc)
        .def("SetSpatialInertiaInBodyFrame",
            &Class::SetSpatialInertiaInBodyFrame, py::arg("context"),
            py::arg("M_Bo_B"), cls_doc.SetSpatialInertiaInBodyFrame.doc);
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
    cls  // BR
        .def("is_floating",
            WrapDeprecated(
                cls_doc.is_floating.doc_deprecated, &Class::is_floating),
            cls_doc.is_floating.doc_deprecated);
#pragma GCC diagnostic pop

    // Aliases for backwards compatibility (dispreferred).
    m.attr("Body") = m.attr("RigidBody");
    m.attr("Body_") = m.attr("RigidBody_");
  }

  // Joints.
  {
    using Class = Joint<T>;
    constexpr auto& cls_doc = doc.Joint;
    auto cls =
        DefineTemplateClassWithDefault<Class>(m, "Joint", param, cls_doc.doc);
    BindMultibodyElementMixin<T>(&cls);
    cls  // BR
        .def("index", &Class::index, cls_doc.index.doc)
        .def("name", &Class::name, cls_doc.name.doc)
        .def("parent_body", &Class::parent_body, py_rvp::reference_internal,
            cls_doc.parent_body.doc)
        .def("child_body", &Class::child_body, py_rvp::reference_internal,
            cls_doc.child_body.doc)
        .def("frame_on_parent", &Class::frame_on_parent,
            py_rvp::reference_internal, cls_doc.frame_on_parent.doc)
        .def("frame_on_child", &Class::frame_on_child,
            py_rvp::reference_internal, cls_doc.frame_on_child.doc)
        .def("type_name", &Class::type_name, cls_doc.type_name.doc)
        .def("position_start", &Class::position_start,
            cls_doc.position_start.doc)
        .def("velocity_start", &Class::velocity_start,
            cls_doc.velocity_start.doc)
        .def("num_positions", &Class::num_positions, cls_doc.num_positions.doc)
        .def("num_velocities", &Class::num_velocities,
            cls_doc.num_velocities.doc)
        .def("can_rotate", &Class::can_rotate, cls_doc.can_rotate.doc)
        .def("can_translate", &Class::can_translate, cls_doc.can_translate.doc)
        .def("position_suffix", &Class::position_suffix,
            cls_doc.position_suffix.doc)
        .def("velocity_suffix", &Class::velocity_suffix,
            cls_doc.velocity_suffix.doc)
        .def("GetOnePosition", &Class::GetOnePosition, py::arg("context"),
            cls_doc.GetOnePosition.doc)
        .def("GetOneVelocity", &Class::GetOneVelocity, py::arg("context"),
            cls_doc.GetOneVelocity.doc)
        .def("AddInOneForce", &Class::AddInOneForce, py::arg("context"),
            py::arg("joint_dof"), py::arg("joint_tau"), py::arg("forces"),
            cls_doc.AddInOneForce.doc)
        .def("AddInDamping", &Class::AddInDamping, py::arg("context"),
            py::arg("forces"), cls_doc.AddInDamping.doc)
        .def("position_lower_limits", &Class::position_lower_limits,
            cls_doc.position_lower_limits.doc)
        .def("position_upper_limits", &Class::position_upper_limits,
            cls_doc.position_upper_limits.doc)
        .def("velocity_lower_limits", &Class::velocity_lower_limits,
            cls_doc.velocity_lower_limits.doc)
        .def("velocity_upper_limits", &Class::velocity_upper_limits,
            cls_doc.velocity_upper_limits.doc)
        .def("acceleration_lower_limits", &Class::acceleration_lower_limits,
            cls_doc.acceleration_lower_limits.doc)
        .def("acceleration_upper_limits", &Class::acceleration_upper_limits,
            cls_doc.acceleration_upper_limits.doc)
        .def("default_positions", &Class::default_positions,
            cls_doc.default_positions.doc)
        .def("set_position_limits", &Class::set_position_limits,
            py::arg("lower_limits"), py::arg("upper_limits"),
            cls_doc.set_position_limits.doc)
        .def("set_velocity_limits", &Class::set_velocity_limits,
            py::arg("lower_limits"), py::arg("upper_limits"),
            cls_doc.set_velocity_limits.doc)
        .def("set_acceleration_limits", &Class::set_acceleration_limits,
            py::arg("lower_limits"), py::arg("upper_limits"),
            cls_doc.set_acceleration_limits.doc)
        .def("set_default_positions", &Class::set_default_positions,
            py::arg("default_positions"), cls_doc.set_default_positions.doc)
        .def("SetDefaultPose", &Class::SetDefaultPose, py::arg("X_FM"),
            cls_doc.SetDefaultPose.doc)
        .def("SetDefaultPosePair", &Class::SetDefaultPosePair, py::arg("q_FM"),
            py::arg("p_FM"), cls_doc.SetDefaultPosePair.doc)
        .def("GetDefaultPose", &Class::GetDefaultPose,
            cls_doc.GetDefaultPose.doc)
        .def("GetDefaultPosePair", &Class::GetDefaultPosePair,
            cls_doc.GetDefaultPosePair.doc)
        .def("Lock", &Class::Lock, py::arg("context"), cls_doc.Lock.doc)
        .def("Unlock", &Class::Unlock, py::arg("context"), cls_doc.Unlock.doc)
        .def("is_locked", &Class::is_locked, py::arg("context"),
            cls_doc.is_locked.doc)
        .def("default_damping_vector", &Class::default_damping_vector,
            return_value_policy_for_scalar_type<T>(),
            cls_doc.default_damping_vector.doc)
        .def("set_default_damping_vector", &Class::set_default_damping_vector,
            py::arg("damping"), cls_doc.set_default_damping_vector.doc)
        .def("GetDampingVector", &Class::GetDampingVector, py::arg("context"),
            // Keep alive, ownership: `return` keeps `context` alive.
            py::keep_alive<0, 2>(), return_value_policy_for_scalar_type<T>(),
            cls_doc.GetDampingVector.doc)
        .def("SetDampingVector", &Class::SetDampingVector, py::arg("context"),
            py::arg("damping"), cls_doc.SetDampingVector.doc);
  }

  // BallRpyJoint
  {
    using Class = BallRpyJoint<T>;
    constexpr auto& cls_doc = doc.BallRpyJoint;
    auto cls = DefineTemplateClassWithDefault<Class, Joint<T>>(
        m, "BallRpyJoint", param, cls_doc.doc);
    cls  // BR
        .def_property_readonly_static(
            "kTypeName", [](py::object /* self */) { return Class::kTypeName; })
        .def(
            py::init<const string&, const Frame<T>&, const Frame<T>&, double>(),
            py::arg("name"), py::arg("frame_on_parent"),
            py::arg("frame_on_child"), py::arg("damping") = 0, cls_doc.ctor.doc)
        .def("default_damping", &Class::default_damping,
            cls_doc.default_damping.doc)
        .def("get_angles", &Class::get_angles, py::arg("context"),
            cls_doc.get_angles.doc)
        .def("set_angles", &Class::set_angles, py::arg("context"),
            py::arg("angles"), cls_doc.set_angles.doc)
        .def("set_random_angles_distribution",
            &Class::set_random_angles_distribution, py::arg("angles"),
            cls_doc.set_random_angles_distribution.doc)
        .def("get_angular_velocity", &Class::get_angular_velocity,
            py::arg("context"), cls_doc.get_angular_velocity.doc)
        .def("set_angular_velocity", &Class::set_angular_velocity,
            py::arg("context"), py::arg("w_FM"),
            cls_doc.set_angular_velocity.doc)
        .def("get_default_angles", &Class::get_default_angles,
            cls_doc.get_default_angles.doc)
        .def("set_default_angles", &Class::set_default_angles,
            py::arg("angles"), cls_doc.set_default_angles.doc);
  }

  // PlanarJoint
  {
    using Class = PlanarJoint<T>;
    constexpr auto& cls_doc = doc.PlanarJoint;
    auto cls = DefineTemplateClassWithDefault<Class, Joint<T>>(
        m, "PlanarJoint", param, cls_doc.doc);
    cls  // BR
        .def_property_readonly_static(
            "kTypeName", [](py::object /* self */) { return Class::kTypeName; })
        .def(py::init<const string&, const Frame<T>&, const Frame<T>&,
                 Vector3<double>>(),
            py::arg("name"), py::arg("frame_on_parent"),
            py::arg("frame_on_child"),
            py::arg("damping") = Vector3<double>::Zero(), cls_doc.ctor.doc)
        .def("default_damping", &Class::default_damping,
            cls_doc.default_damping.doc)
        .def("get_translation", &Class::get_translation, py::arg("context"),
            cls_doc.get_translation.doc)
        .def("set_translation", &Class::set_translation, py::arg("context"),
            py::arg("p_FoMo_F"), cls_doc.set_translation.doc)
        .def("get_rotation", &Class::get_rotation, py::arg("context"),
            cls_doc.get_rotation.doc)
        .def("set_rotation", &Class::set_rotation, py::arg("context"),
            py::arg("theta"), cls_doc.set_rotation.doc)
        .def("set_pose", &Class::set_pose, py::arg("context"),
            py::arg("p_FoMo_F"), py::arg("theta"), cls_doc.set_pose.doc)
        .def("get_translational_velocity", &Class::get_translational_velocity,
            py::arg("context"), cls_doc.get_translational_velocity.doc)
        .def("set_translational_velocity", &Class::set_translational_velocity,
            py::arg("context"), py::arg("v_FoMo_F"),
            cls_doc.set_translational_velocity.doc)
        .def("get_angular_velocity", &Class::get_angular_velocity,
            py::arg("context"), cls_doc.get_angular_velocity.doc)
        .def("set_angular_velocity", &Class::set_angular_velocity,
            py::arg("context"), py::arg("theta_dot"),
            cls_doc.set_angular_velocity.doc)
        .def("get_default_translation", &Class::get_default_translation,
            cls_doc.get_default_translation.doc)
        .def("set_default_translation", &Class::set_default_translation,
            py::arg("p_FoMo_F"), cls_doc.set_default_translation.doc)
        .def("get_default_rotation", &Class::get_default_rotation,
            cls_doc.get_default_rotation.doc)
        .def("set_default_rotation", &Class::set_default_rotation,
            py::arg("theta"), cls_doc.set_default_rotation.doc)
        .def("set_default_pose", &Class::set_default_pose, py::arg("p_FoMo_F"),
            py::arg("theta"), cls_doc.set_default_pose.doc)
        .def("set_random_pose_distribution",
            &Class::set_random_pose_distribution, py::arg("p_FoMo_F"),
            py::arg("theta"), cls_doc.set_random_pose_distribution.doc);
  }

  // PrismaticJoint
  {
    using Class = PrismaticJoint<T>;
    constexpr auto& cls_doc = doc.PrismaticJoint;
    auto cls = DefineTemplateClassWithDefault<Class, Joint<T>>(
        m, "PrismaticJoint", param, cls_doc.doc);
    cls  // BR
        .def_property_readonly_static(
            "kTypeName", [](py::object /* self */) { return Class::kTypeName; })
        .def(py::init<const string&, const Frame<T>&, const Frame<T>&,
                 const Vector3<double>&, double, double, double>(),
            py::arg("name"), py::arg("frame_on_parent"),
            py::arg("frame_on_child"), py::arg("axis"),
            py::arg("pos_lower_limit") =
                -std::numeric_limits<double>::infinity(),
            py::arg("pos_upper_limit") =
                std::numeric_limits<double>::infinity(),
            py::arg("damping") = 0, cls_doc.ctor.doc)
        .def("translation_axis", &Class::translation_axis,
            cls_doc.translation_axis.doc)
        .def("default_damping", &Class::default_damping,
            cls_doc.default_damping.doc)
        .def("set_default_damping", &Class::set_default_damping,
            py::arg("damping"), cls_doc.set_default_damping.doc)
        .def("position_lower_limit", &Class::position_lower_limit,
            cls_doc.position_lower_limit.doc)
        .def("position_upper_limit", &Class::position_upper_limit,
            cls_doc.position_upper_limit.doc)
        .def("velocity_lower_limit", &Class::velocity_lower_limit,
            cls_doc.velocity_lower_limit.doc)
        .def("velocity_upper_limit", &Class::velocity_upper_limit,
            cls_doc.velocity_upper_limit.doc)
        .def("acceleration_lower_limit", &Class::acceleration_lower_limit,
            cls_doc.acceleration_lower_limit.doc)
        .def("acceleration_upper_limit", &Class::acceleration_upper_limit,
            cls_doc.acceleration_upper_limit.doc)
        .def("get_translation", &Class::get_translation, py::arg("context"),
            cls_doc.get_translation.doc)
        .def("set_translation", &Class::set_translation, py::arg("context"),
            py::arg("translation"), cls_doc.set_translation.doc)
        .def("get_translation_rate", &Class::get_translation_rate,
            py::arg("context"), cls_doc.get_translation_rate.doc)
        .def("set_translation_rate", &Class::set_translation_rate,
            py::arg("context"), py::arg("translation_dot"),
            cls_doc.set_translation_rate.doc)
        .def("get_default_translation", &Class::get_default_translation,
            cls_doc.get_default_translation.doc)
        .def("set_default_translation", &Class::set_default_translation,
            py::arg("translation"), cls_doc.set_default_translation.doc)
        .def("set_random_translation_distribution",
            &Class::set_random_translation_distribution, py::arg("translation"),
            cls_doc.set_random_translation_distribution.doc)
        .def("GetDamping", &Class::GetDamping, py::arg("context"),
            cls_doc.GetDamping.doc)
        .def("SetDamping", &Class::SetDamping, py::arg("context"),
            py::arg("damping"), cls_doc.SetDamping.doc);
  }

  // QuaternionFloatingJoint
  {
    using Class = QuaternionFloatingJoint<T>;
    constexpr auto& cls_doc = doc.QuaternionFloatingJoint;
    auto cls = DefineTemplateClassWithDefault<Class, Joint<T>>(
        m, "QuaternionFloatingJoint", param, cls_doc.doc);
    cls  // BR
        .def_property_readonly_static(
            "kTypeName", [](py::object /* self */) { return Class::kTypeName; })
        .def(py::init<const string&, const Frame<T>&, const Frame<T>&, double,
                 double>(),
            py::arg("name"), py::arg("frame_on_parent"),
            py::arg("frame_on_child"), py::arg("angular_damping") = 0,
            py::arg("translational_damping") = 0, cls_doc.ctor.doc)
        .def("default_angular_damping", &Class::default_angular_damping,
            cls_doc.default_angular_damping.doc)
        .def("default_translational_damping",
            &Class::default_translational_damping,
            cls_doc.default_translational_damping.doc)
        .def("get_quaternion", &Class::get_quaternion, py::arg("context"),
            cls_doc.get_quaternion.doc)
        .def("get_translation", &Class::get_translation, py::arg("context"),
            cls_doc.get_translation.doc)
        .def(
            "GetPose", &Class::GetPose, py::arg("context"), cls_doc.GetPose.doc)
        .def("get_angular_velocity", &Class::get_angular_velocity,
            py::arg("context"), cls_doc.get_angular_velocity.doc)
        .def("get_translational_velocity", &Class::get_translational_velocity,
            py::arg("context"), cls_doc.get_translational_velocity.doc)
        .def("SetQuaternion", &Class::SetQuaternion, py::arg("context"),
            py::arg("q_FM"), cls_doc.SetQuaternion.doc)
        .def("SetOrientation", &Class::SetOrientation, py::arg("context"),
            py::arg("R"), cls_doc.SetOrientation.doc)
        .def("SetTranslation", &Class::SetTranslation, py::arg("context"),
            py::arg("p_FM"), cls_doc.SetTranslation.doc)
        .def("SetPose", &Class::SetPose, py::arg("context"), py::arg("X_FM"),
            cls_doc.SetPose.doc)
        .def("set_angular_velocity", &Class::set_angular_velocity,
            py::arg("context"), py::arg("w_FM"),
            cls_doc.set_angular_velocity.doc)
        .def("set_translational_velocity", &Class::set_translational_velocity,
            py::arg("context"), py::arg("v_FM"),
            cls_doc.set_translational_velocity.doc)
        .def("set_random_translation_distribution",
            &Class::set_random_translation_distribution, py::arg("translation"),
            cls_doc.set_random_translation_distribution.doc)
        .def("set_random_quaternion_distribution",
            &Class::set_random_quaternion_distribution, py::arg("q_FM"),
            cls_doc.set_random_quaternion_distribution.doc)
        .def("set_random_quaternion_distribution_to_uniform",
            &Class::set_random_quaternion_distribution_to_uniform,
            cls_doc.set_random_quaternion_distribution_to_uniform.doc)
        .def("get_default_quaternion", &Class::get_default_quaternion,
            cls_doc.get_default_quaternion.doc)
        .def("get_default_translation", &Class::get_default_translation,
            cls_doc.get_default_translation.doc)
        .def("set_default_quaternion", &Class::set_default_quaternion,
            py::arg("q_FM"), cls_doc.set_default_quaternion.doc)
        .def("set_default_translation", &Class::set_default_translation,
            py::arg("translation"), cls_doc.set_default_translation.doc);
  }

  // RevoluteJoint
  {
    using Class = RevoluteJoint<T>;
    constexpr auto& cls_doc = doc.RevoluteJoint;
    auto cls = DefineTemplateClassWithDefault<Class, Joint<T>>(
        m, "RevoluteJoint", param, cls_doc.doc);
    cls  // BR
        .def_property_readonly_static(
            "kTypeName", [](py::object /* self */) { return Class::kTypeName; })
        .def(py::init<const string&, const Frame<T>&, const Frame<T>&,
                 const Vector3<double>&, double>(),
            py::arg("name"), py::arg("frame_on_parent"),
            py::arg("frame_on_child"), py::arg("axis"), py::arg("damping") = 0,
            cls_doc.ctor.doc_5args)
        .def(py::init<const std::string&, const Frame<T>&, const Frame<T>&,
                 const Vector3<double>&, double, double, double>(),
            py::arg("name"), py::arg("frame_on_parent"),
            py::arg("frame_on_child"), py::arg("axis"),
            py::arg("pos_lower_limit"), py::arg("pos_upper_limit"),
            py::arg("damping") = 0.0, cls_doc.ctor.doc_7args)
        .def("revolute_axis", &Class::revolute_axis, cls_doc.revolute_axis.doc)
        .def("default_damping", &Class::default_damping,
            cls_doc.default_damping.doc)
        .def("set_default_damping", &Class::set_default_damping,
            py::arg("damping"), cls_doc.set_default_damping.doc)
        .def("position_lower_limit", &Class::position_lower_limit,
            cls_doc.position_lower_limit.doc)
        .def("position_upper_limit", &Class::position_upper_limit,
            cls_doc.position_upper_limit.doc)
        .def("velocity_lower_limit", &Class::velocity_lower_limit,
            cls_doc.velocity_lower_limit.doc)
        .def("velocity_upper_limit", &Class::velocity_upper_limit,
            cls_doc.velocity_upper_limit.doc)
        .def("acceleration_lower_limit", &Class::acceleration_lower_limit,
            cls_doc.acceleration_lower_limit.doc)
        .def("acceleration_upper_limit", &Class::acceleration_upper_limit,
            cls_doc.acceleration_upper_limit.doc)
        .def("get_angle", &Class::get_angle, py::arg("context"),
            cls_doc.get_angle.doc)
        .def("set_angle", &Class::set_angle, py::arg("context"),
            py::arg("angle"), cls_doc.set_angle.doc)
        .def("set_random_angle_distribution",
            &Class::set_random_angle_distribution, py::arg("angle"),
            cls_doc.set_random_angle_distribution.doc)
        .def("get_angular_rate", &Class::get_angular_rate, py::arg("context"),
            cls_doc.get_angular_rate.doc)
        .def("set_angular_rate", &Class::set_angular_rate, py::arg("context"),
            py::arg("angle"), cls_doc.set_angular_rate.doc)
        .def("get_default_angle", &Class::get_default_angle,
            cls_doc.get_default_angle.doc)
        .def("set_default_angle", &Class::set_default_angle, py::arg("angle"),
            cls_doc.set_default_angle.doc)
        .def("GetDamping", &Class::GetDamping, py::arg("context"),
            cls_doc.GetDamping.doc)
        .def("SetDamping", &Class::SetDamping, py::arg("context"),
            py::arg("damping"), cls_doc.SetDamping.doc);
  }

  // RpyFloatingJoint
  {
    using Class = RpyFloatingJoint<T>;
    constexpr auto& cls_doc = doc.RpyFloatingJoint;
    auto cls = DefineTemplateClassWithDefault<Class, Joint<T>>(
        m, "RpyFloatingJoint", param, cls_doc.doc);
    cls  // BR
        .def_property_readonly_static(
            "kTypeName", [](py::object /* self */) { return Class::kTypeName; })
        .def(py::init<const string&, const Frame<T>&, const Frame<T>&, double,
                 double>(),
            py::arg("name"), py::arg("frame_on_parent"),
            py::arg("frame_on_child"), py::arg("angular_damping") = 0,
            py::arg("translational_damping") = 0, cls_doc.ctor.doc)
        .def("default_angular_damping", &Class::default_angular_damping,
            cls_doc.default_angular_damping.doc)
        .def("default_translational_damping",
            &Class::default_translational_damping,
            cls_doc.default_translational_damping.doc)
        .def("get_angles", &Class::get_angles, py::arg("context"),
            cls_doc.get_angles.doc)
        .def("set_angles", &Class::set_angles, py::arg("context"),
            py::arg("angles"), cls_doc.set_angles.doc)
        .def("SetOrientation", &Class::SetOrientation, py::arg("context"),
            py::arg("R_FM"), cls_doc.SetOrientation.doc)
        .def("get_translation", &Class::get_translation, py::arg("context"),
            cls_doc.get_translation.doc)
        .def("SetTranslation", &Class::SetTranslation, py::arg("context"),
            py::arg("p_FM"), cls_doc.SetTranslation.doc)
        .def(
            "GetPose", &Class::GetPose, py::arg("context"), cls_doc.GetPose.doc)
        .def("SetPose", &Class::SetPose, py::arg("context"), py::arg("X_FM"),
            cls_doc.SetPose.doc)
        .def("get_angular_velocity", &Class::get_angular_velocity,
            py::arg("context"), cls_doc.get_angular_velocity.doc)
        .def("set_angular_velocity", &Class::set_angular_velocity,
            py::arg("context"), py::arg("w_FM"),
            cls_doc.set_angular_velocity.doc)
        .def("get_translational_velocity", &Class::get_translational_velocity,
            py::arg("context"), cls_doc.get_translational_velocity.doc)
        .def("set_translational_velocity", &Class::set_translational_velocity,
            py::arg("context"), py::arg("v_FM"),
            cls_doc.set_translational_velocity.doc)
        .def("set_random_angles_distribution",
            &Class::set_random_angles_distribution, py::arg("angles"),
            cls_doc.set_random_angles_distribution.doc)
        .def("set_random_translation_distribution",
            &Class::set_random_translation_distribution, py::arg("p_FM"),
            cls_doc.set_random_translation_distribution.doc)
        .def("get_default_angles", &Class::get_default_angles,
            cls_doc.get_default_angles.doc)
        .def("set_default_angles", &Class::set_default_angles,
            py::arg("angles"), cls_doc.set_default_angles.doc)
        .def("get_default_translation", &Class::get_default_translation,
            cls_doc.get_default_translation.doc)
        .def("set_default_translation", &Class::set_default_translation,
            py::arg("p_FM"), cls_doc.set_default_translation.doc);
  }

  // ScrewJoint
  {
    using Class = ScrewJoint<T>;
    constexpr auto& cls_doc = doc.ScrewJoint;
    auto cls = DefineTemplateClassWithDefault<Class, Joint<T>>(
        m, "ScrewJoint", param, cls_doc.doc);
    cls  // BR
        .def_property_readonly_static(
            "kTypeName", [](py::object /* self */) { return Class::kTypeName; })
        .def(py::init<const string&, const Frame<T>&, const Frame<T>&, double,
                 double>(),
            py::arg("name"), py::arg("frame_on_parent"),
            py::arg("frame_on_child"), py::arg("screw_pitch"),
            py::arg("damping"), cls_doc.ctor.doc_5args)
        .def(py::init<const string&, const Frame<T>&, const Frame<T>&,
                 const Vector3<double>&, double, double>(),
            py::arg("name"), py::arg("frame_on_parent"),
            py::arg("frame_on_child"), py::arg("axis"), py::arg("screw_pitch"),
            py::arg("damping"), cls_doc.ctor.doc_6args)
        .def("screw_pitch", &Class::screw_pitch, cls_doc.screw_pitch.doc)
        .def("default_damping", &Class::default_damping,
            cls_doc.default_damping.doc)
        .def("get_default_translation", &Class::get_default_translation,
            cls_doc.get_default_translation.doc)
        .def("set_default_translation", &Class::set_default_translation,
            py::arg("z"), cls_doc.set_default_translation.doc)
        .def("get_default_rotation", &Class::get_default_rotation,
            cls_doc.get_default_rotation.doc)
        .def("set_default_rotation", &Class::set_default_rotation,
            py::arg("theta"), cls_doc.set_default_rotation.doc)
        .def("get_translation", &Class::get_translation, py::arg("context"),
            cls_doc.get_translation.doc)
        .def("set_translation", &Class::set_translation, py::arg("context"),
            py::arg("translation"), cls_doc.set_translation.doc)
        .def("get_translational_velocity", &Class::get_translational_velocity,
            py::arg("context"), cls_doc.get_translational_velocity.doc)
        .def("set_translational_velocity", &Class::set_translational_velocity,
            py::arg("context"), py::arg("translation_dot"),
            cls_doc.set_translational_velocity.doc)
        .def("get_rotation", &Class::get_rotation, py::arg("context"),
            cls_doc.get_rotation.doc)
        .def("get_angular_velocity", &Class::get_angular_velocity,
            py::arg("context"), cls_doc.get_angular_velocity.doc)
        .def("set_angular_velocity", &Class::set_angular_velocity,
            py::arg("context"), py::arg("theta_dot"),
            cls_doc.set_angular_velocity.doc)
        .def("set_random_pose_distribution",
            &Class::set_random_pose_distribution, py::arg("theta"),
            cls_doc.set_random_pose_distribution.doc)
        .def("GetDamping", &Class::GetDamping, py::arg("context"),
            cls_doc.GetDamping.doc)
        .def("SetDamping", &Class::SetDamping, py::arg("context"),
            py::arg("damping"), cls_doc.SetDamping.doc);
  }

  // UniversalJoint
  {
    using Class = UniversalJoint<T>;
    constexpr auto& cls_doc = doc.UniversalJoint;
    auto cls = DefineTemplateClassWithDefault<Class, Joint<T>>(
        m, "UniversalJoint", param, cls_doc.doc);
    cls  // BR
        .def_property_readonly_static(
            "kTypeName", [](py::object /* self */) { return Class::kTypeName; })
        .def(
            py::init<const string&, const Frame<T>&, const Frame<T>&, double>(),
            py::arg("name"), py::arg("frame_on_parent"),
            py::arg("frame_on_child"), py::arg("damping") = 0, cls_doc.ctor.doc)
        .def("default_damping", &Class::default_damping,
            cls_doc.default_damping.doc)
        .def("get_angles", &Class::get_angles, py::arg("context"),
            cls_doc.get_angles.doc)
        .def("set_angles", &Class::set_angles, py::arg("context"),
            py::arg("angles"), cls_doc.set_angles.doc)
        .def("get_angular_rates", &Class::get_angular_rates, py::arg("context"),
            cls_doc.get_angular_rates.doc)
        .def("set_angular_rates", &Class::set_angular_rates, py::arg("context"),
            py::arg("theta_dot"), cls_doc.set_angular_rates.doc)
        .def("get_default_angles", &Class::get_default_angles,
            cls_doc.get_default_angles.doc)
        .def("set_default_angles", &Class::set_default_angles,
            py::arg("angles"), cls_doc.set_default_angles.doc)
        .def("set_random_angles_distribution",
            &Class::set_random_angles_distribution, py::arg("angles"),
            cls_doc.set_random_angles_distribution.doc);
  }

  // WeldJoint
  {
    using Class = WeldJoint<T>;
    constexpr auto& cls_doc = doc.WeldJoint;
    auto cls = DefineTemplateClassWithDefault<Class, Joint<T>>(
        m, "WeldJoint", param, cls_doc.doc);
    cls  // BR
        .def_property_readonly_static(
            "kTypeName", [](py::object /* self */) { return Class::kTypeName; })
        .def(py::init<const string&, const Frame<T>&, const Frame<T>&,
                 const RigidTransform<double>&>(),
            py::arg("name"), py::arg("frame_on_parent_F"),
            py::arg("frame_on_child_M"), py::arg("X_FM"), cls_doc.ctor.doc)
        .def("X_FM", &Class::X_FM, cls_doc.X_FM.doc);
  }

  // Actuators.
  {
    using Class = JointActuator<T>;
    constexpr auto& cls_doc = doc.JointActuator;
    auto cls = DefineTemplateClassWithDefault<Class>(
        m, "JointActuator", param, cls_doc.doc);
    BindMultibodyElementMixin<T>(&cls);
    cls  // BR
        .def("name", &Class::name, cls_doc.name.doc)
        .def("joint", &Class::joint, py_rvp::reference_internal,
            cls_doc.joint.doc)
        .def(
            "get_actuation_vector",
            [](const Class& self, const VectorX<T>& u) -> VectorX<T> {
              return self.get_actuation_vector(u);
            },
            py::arg("u"), cls_doc.get_actuation_vector.doc)
        .def("set_actuation_vector", &Class::set_actuation_vector,
            py::arg("u_actuator"), py::arg("u"),
            cls_doc.set_actuation_vector.doc)
        .def("input_start", &Class::input_start, cls_doc.input_start.doc)
        .def("num_inputs", &Class::num_inputs, cls_doc.num_inputs.doc)
        .def("effort_limit", &Class::effort_limit, cls_doc.effort_limit.doc)
        .def("set_effort_limit", &Class::set_effort_limit,
            py::arg("effort_limit"), cls_doc.set_effort_limit.doc)
        .def("default_rotor_inertia", &Class::default_rotor_inertia,
            cls_doc.default_rotor_inertia.doc)
        .def("default_gear_ratio", &Class::default_gear_ratio,
            cls_doc.default_gear_ratio.doc)
        .def("set_default_rotor_inertia", &Class::set_default_rotor_inertia,
            py::arg("rotor_inertia"), cls_doc.set_default_rotor_inertia.doc)
        .def("set_default_gear_ratio", &Class::set_default_gear_ratio,
            py::arg("gear_ratio"), cls_doc.set_default_gear_ratio.doc)
        .def("default_reflected_inertia", &Class::default_reflected_inertia,
            cls_doc.default_reflected_inertia.doc)
        .def("rotor_inertia", &Class::rotor_inertia, py::arg("context"),
            cls_doc.rotor_inertia.doc)
        .def("gear_ratio", &Class::gear_ratio, py::arg("context"),
            cls_doc.gear_ratio.doc)
        .def("SetRotorInertia", &Class::SetRotorInertia, py::arg("context"),
            py::arg("rotor_inertia"), cls_doc.SetRotorInertia.doc)
        .def("SetGearRatio", &Class::SetGearRatio, py::arg("context"),
            py::arg("gear_ratio"), cls_doc.SetGearRatio.doc)
        .def("calc_reflected_inertia", &Class::calc_reflected_inertia,
            py::arg("context"), cls_doc.calc_reflected_inertia.doc)
        .def("get_controller_gains", &Class::get_controller_gains,
            cls_doc.get_controller_gains.doc)
        .def("set_controller_gains", &Class::set_controller_gains,
            py::arg("gains"), cls_doc.set_controller_gains.doc)
        .def("has_controller", &Class::has_controller,
            cls_doc.has_controller.doc);
  }

  // Force Elements.
  {
    using Class = ForceElement<T>;
    constexpr auto& cls_doc = doc.ForceElement;
    auto cls = DefineTemplateClassWithDefault<Class>(
        m, "ForceElement", param, cls_doc.doc);
    BindMultibodyElementMixin<T>(&cls);
  }

  {
    using Class = LinearSpringDamper<T>;
    constexpr auto& cls_doc = doc.LinearSpringDamper;
    auto cls = DefineTemplateClassWithDefault<Class, ForceElement<T>>(
        m, "LinearSpringDamper", param, cls_doc.doc);
    cls  // BR
        .def(py::init<const RigidBody<T>&, const Vector3<double>&,
                 const RigidBody<T>&, const Vector3<double>&, double, double,
                 double>(),
            py::arg("bodyA"), py::arg("p_AP"), py::arg("bodyB"),
            py::arg("p_BQ"), py::arg("free_length"), py::arg("stiffness"),
            py::arg("damping"), cls_doc.ctor.doc)
        .def("bodyA", &Class::bodyA, py_rvp::reference_internal,
            cls_doc.bodyA.doc)
        .def("bodyB", &Class::bodyB, py_rvp::reference_internal,
            cls_doc.bodyB.doc)
        .def("p_AP", &Class::p_AP, cls_doc.p_AP.doc)
        .def("p_BQ", &Class::p_BQ, cls_doc.p_BQ.doc)
        .def("free_length", &Class::free_length, cls_doc.free_length.doc)
        .def("stiffness", &Class::stiffness, cls_doc.stiffness.doc)
        .def("damping", &Class::damping, cls_doc.damping.doc);
  }

  {
    using Class = PrismaticSpring<T>;
    constexpr auto& cls_doc = doc.PrismaticSpring;
    auto cls = DefineTemplateClassWithDefault<Class, ForceElement<T>>(
        m, "PrismaticSpring", param, cls_doc.doc);
    cls  // BR
        .def(py::init<const PrismaticJoint<T>&, double, double>(),
            py::arg("joint"), py::arg("nominal_position"), py::arg("stiffness"),
            cls_doc.ctor.doc)
        .def("joint", &Class::joint, py_rvp::reference_internal,
            cls_doc.joint.doc)
        .def("nominal_position", &Class::nominal_position,
            cls_doc.nominal_position.doc)
        .def("stiffness", &Class::stiffness, cls_doc.stiffness.doc);
  }

  {
    using Class = RevoluteSpring<T>;
    constexpr auto& cls_doc = doc.RevoluteSpring;
    auto cls = DefineTemplateClassWithDefault<Class, ForceElement<T>>(
        m, "RevoluteSpring", param, cls_doc.doc);
    cls  // BR
        .def(py::init<const RevoluteJoint<T>&, double, double>(),
            py::arg("joint"), py::arg("nominal_angle"), py::arg("stiffness"),
            cls_doc.ctor.doc)
        .def("joint", &Class::joint, py_rvp::reference_internal,
            cls_doc.joint.doc)
        .def("default_stiffness", &Class::default_stiffness,
            cls_doc.default_stiffness.doc)
        .def("GetStiffness", &Class::GetStiffness, py::arg("context"),
            cls_doc.GetStiffness.doc)
        .def("SetStiffness", &Class::SetStiffness, py::arg("context"),
            py::arg("stiffness"), cls_doc.SetStiffness.doc)
        .def("default_nominal_angle", &Class::default_nominal_angle,
            cls_doc.default_nominal_angle.doc)
        .def("GetNominalAngle", &Class::GetNominalAngle, py::arg("context"),
            cls_doc.GetNominalAngle.doc)
        .def("SetNominalAngle", &Class::SetNominalAngle, py::arg("context"),
            py::arg("nominal_angle"), cls_doc.SetNominalAngle.doc);
  }

  {
    using Class = UniformGravityFieldElement<T>;
    constexpr auto& cls_doc = doc.UniformGravityFieldElement;
    auto cls = DefineTemplateClassWithDefault<Class, ForceElement<T>>(
        m, "UniformGravityFieldElement", param, cls_doc.doc);
    cls  // BR
        .def_readonly_static("kDefaultStrength", &Class::kDefaultStrength)
        .def(py::init<>(), cls_doc.ctor.doc_0args)
        .def(
            py::init<Vector3<double>>(), py::arg("g_W"), cls_doc.ctor.doc_1args)
        .def("gravity_vector", &Class::gravity_vector,
            cls_doc.gravity_vector.doc)
        .def("set_gravity_vector", &Class::set_gravity_vector,
            cls_doc.set_gravity_vector.doc)
        .def("set_enabled", &Class::set_enabled, py::arg("model_instance"),
            py::arg("is_enabled"), cls_doc.set_enabled.doc)
        .def("is_enabled", &Class::is_enabled, py::arg("model_instance"),
            cls_doc.is_enabled.doc)
        .def("CalcGravityGeneralizedForces",
            &Class::CalcGravityGeneralizedForces, py::arg("context"),
            cls_doc.CalcGravityGeneralizedForces.doc);
  }

  {
    using Class = DoorHinge<T>;
    constexpr auto& cls_doc = doc.DoorHinge;
    auto cls = DefineTemplateClassWithDefault<Class, ForceElement<T>>(
        m, "DoorHinge", param, cls_doc.doc);
    cls  // BR
        .def(py::init<const RevoluteJoint<T>&, const DoorHingeConfig&>(),
            py::arg("joint"), py::arg("config"), cls_doc.ctor.doc)
        .def("joint", &Class::joint, py_rvp::reference_internal,
            cls_doc.joint.doc)
        .def("config", &Class::config, py_rvp::reference_internal,
            cls_doc.config.doc)
        .def("CalcHingeFrictionalTorque", &Class::CalcHingeFrictionalTorque,
            py::arg("angular_rate"), cls_doc.CalcHingeFrictionalTorque.doc)
        .def("CalcHingeSpringTorque", &Class::CalcHingeSpringTorque,
            py::arg("angle"), cls_doc.CalcHingeSpringTorque.doc)
        .def("CalcHingeTorque", &Class::CalcHingeTorque, py::arg("angle"),
            py::arg("angular_rate"), cls_doc.CalcHingeTorque.doc);
  }

  {
    using Class = LinearBushingRollPitchYaw<T>;
    constexpr auto& cls_doc = doc.LinearBushingRollPitchYaw;
    auto cls = DefineTemplateClassWithDefault<Class, ForceElement<T>>(
        m, "LinearBushingRollPitchYaw", param, cls_doc.doc);
    cls  // BR
        .def(py::init<const Frame<T>&, const Frame<T>&, const Vector3d&,
                 const Vector3d&, const Vector3d&, const Vector3d&>(),
            py::arg("frameA"), py::arg("frameC"),
            py::arg("torque_stiffness_constants"),
            py::arg("torque_damping_constants"),
            py::arg("force_stiffness_constants"),
            py::arg("force_damping_constants"), cls_doc.ctor.doc)
        .def("link0", &Class::link0, py_rvp::reference_internal,
            cls_doc.link0.doc)
        .def("link1", &Class::link1, py_rvp::reference_internal,
            cls_doc.link1.doc)
        .def("frameA", &Class::frameA, py_rvp::reference_internal,
            cls_doc.frameA.doc)
        .def("frameC", &Class::frameC, py_rvp::reference_internal,
            cls_doc.frameC.doc)
        .def("torque_stiffness_constants", &Class::torque_stiffness_constants,
            cls_doc.torque_stiffness_constants.doc)
        .def("torque_damping_constants", &Class::torque_damping_constants,
            cls_doc.torque_damping_constants.doc)
        .def("force_stiffness_constants", &Class::force_stiffness_constants,
            cls_doc.force_stiffness_constants.doc)
        .def("force_damping_constants", &Class::force_damping_constants,
            cls_doc.force_damping_constants.doc)
        .def("GetTorqueStiffnessConstants", &Class::GetTorqueStiffnessConstants,
            py::arg("context"), cls_doc.GetTorqueStiffnessConstants.doc)
        .def("GetTorqueDampingConstants", &Class::GetTorqueDampingConstants,
            py::arg("context"), cls_doc.GetTorqueDampingConstants.doc)
        .def("GetForceStiffnessConstants", &Class::GetForceStiffnessConstants,
            py::arg("context"), cls_doc.GetForceStiffnessConstants.doc)
        .def("GetForceDampingConstants", &Class::GetForceDampingConstants,
            py::arg("context"), cls_doc.GetForceDampingConstants.doc)
        .def("SetTorqueStiffnessConstants", &Class::SetTorqueStiffnessConstants,
            py::arg("context"), py::arg("torque_stiffness"),
            cls_doc.SetTorqueStiffnessConstants.doc)
        .def("SetTorqueDampingConstants", &Class::SetTorqueDampingConstants,
            py::arg("context"), py::arg("torque_damping"),
            cls_doc.SetTorqueDampingConstants.doc)
        .def("SetForceStiffnessConstants", &Class::SetForceStiffnessConstants,
            py::arg("context"), py::arg("force_stiffness"),
            cls_doc.SetForceStiffnessConstants.doc)
        .def("SetForceDampingConstants", &Class::SetForceDampingConstants,
            py::arg("context"), py::arg("force_damping"),
            cls_doc.SetForceDampingConstants.doc)
        .def("CalcBushingSpatialForceOnFrameA",
            &Class::CalcBushingSpatialForceOnFrameA, py::arg("context"),
            cls_doc.CalcBushingSpatialForceOnFrameA.doc)
        .def("CalcBushingSpatialForceOnFrameC",
            &Class::CalcBushingSpatialForceOnFrameC, py::arg("context"),
            cls_doc.CalcBushingSpatialForceOnFrameC.doc);
  }
  // NOLINTNEXTLINE(readability/fn_size)
}

template <typename T>
void DefineMultibodyForces(py::module m, T) {
  py::tuple param = GetPyParam<T>();
  {
    using Class = MultibodyForces<T>;
    constexpr auto& cls_doc = doc.MultibodyForces;
    auto cls = DefineTemplateClassWithDefault<Class>(
        m, "MultibodyForces", param, cls_doc.doc);
    // Custom constructor so that in Python we can take a MultibodyPlant
    // instead of a MultibodyTreeSystem.
    // N.B. This depends on `plant_py.cc`, but this codepath will only be
    // activated if this module is present, and thus should not create a runtime
    // error.
    cls  // BR
        .def(py::init([](const MultibodyPlant<T>& plant) {
          return std::make_unique<Class>(plant);
        }),
            py::arg("plant"), cls_doc.ctor.doc_1args_plant)
        .def(py::init<int, int>(), py::arg("nb"), py::arg("nv"),
            cls_doc.ctor.doc_2args_nb_nv)
        .def("SetZero", &Class::SetZero, cls_doc.SetZero.doc)
        .def("num_bodies", &Class::num_bodies, cls_doc.num_bodies.doc)
        .def("num_velocities", &Class::num_velocities,
            cls_doc.num_velocities.doc)
        .def("generalized_forces", &Class::generalized_forces,
            cls_doc.generalized_forces.doc)
        .def("mutable_generalized_forces", &Class::mutable_generalized_forces,
            py_rvp::reference_internal, cls_doc.mutable_generalized_forces.doc)
        // WARNING: Do not bind `body_forces` or `mutable_body_forces` because
        // they use `internal::MobodIndex`. Instead, use force API in Body.
        .def("AddInForces", &Class::AddInForces, py::arg("addend"),
            cls_doc.AddInForces.doc);
    DefCopyAndDeepCopy(&cls);
  }
}

template <typename T>
class ForceDensityFieldPublic : public ForceDensityField<T> {
 public:
  // Expose protected methods for binding, take MultibodyPlant instead of
  // MultibodyTreeSystem as argument so these methods can be bound in Python.
  static systems::CacheEntry& DeclareCacheEntry(MultibodyPlant<T>* plant,
      std::string description, systems::ValueProducer value_producer,
      std::set<systems::DependencyTicket> prerequisites_of_calc) {
    return ForceDensityField<T>::DeclareCacheEntry(
        plant, description, value_producer, prerequisites_of_calc);
  }

  static systems::InputPort<T>& DeclareAbstractInputPort(
      MultibodyPlant<T>* plant, std::string name,
      const AbstractValue& model_value) {
    return ForceDensityField<T>::DeclareAbstractInputPort(
        plant, name, model_value);
  }

  static systems::InputPort<T>& DeclareVectorInputPort(MultibodyPlant<T>* plant,
      std::string name, const systems::BasicVector<T>& model_vector) {
    return ForceDensityField<T>::DeclareVectorInputPort(
        plant, name, model_vector);
  }

 protected:
  explicit ForceDensityFieldPublic(ForceDensityType density_type)
      : ForceDensityField<T>(density_type) {}
};

template <typename T>
class DelegatedForceDensityField final : public ForceDensityField<T> {
 public:
  explicit DelegatedForceDensityField(
      std::shared_ptr<ForceDensityField<T>> impl)
      : ForceDensityField<T>(impl->density_type()), impl_(std::move(impl)) {
    DRAKE_THROW_UNLESS(impl_ != nullptr);
  }

 private:
  Vector3<T> DoEvaluateAt(
      const systems::Context<T>& context, const Vector3<T>& p_WQ) const final {
    return impl_->EvaluateAt(context, p_WQ);
  }

  std::unique_ptr<ForceDensityFieldBase<T>> DoClone() const final {
    return impl_->Clone();
  }

  std::shared_ptr<ForceDensityField<T>> impl_;
};

template <typename T>
class PyForceDensityField : public ForceDensityFieldPublic<T> {
 public:
  explicit PyForceDensityField(ForceDensityType density_type)
      : ForceDensityFieldPublic<T>(density_type) {}

  Vector3<T> DoEvaluateAt(const systems::Context<T>& context,
      const Vector3<T>& p_WQ) const override {
    py::gil_scoped_acquire gil;
    py::function override = py::get_override(
        static_cast<const ForceDensityField<T>*>(this), "DoEvaluateAt");
    if (!override) {
      throw std::logic_error(
          "Python class derived from ForceDensityField<T> must implement "
          "DoEvaluateAt().");
    }
    // Call Python-side DoEvaluateAt.
    py::object result_obj =
        override(py::cast(context, py_rvp::reference), py::cast(p_WQ));
    try {
      return result_obj.cast<Vector3<T>>();
    } catch (const py::cast_error& e) {
      throw std::logic_error(
          "DoEvaluateAt() must return a 3-element list or NumPy array that can "
          "be converted to Vector3<T>. Got " +
          py::str(result_obj).cast<std::string>() + ".");
    }
  }

  std::unique_ptr<ForceDensityFieldBase<T>> DoClone() const override {
    py::gil_scoped_acquire gil;
    py::function override = py::get_override(
        static_cast<const ForceDensityField<T>*>(this), "DoClone");
    if (!override) {
      throw std::logic_error(
          "Python class derived from ForceDensityField<T> must implement "
          "DoClone().");
    }
    // Call Python-side DoClone.
    py::object result_obj = override();
    std::shared_ptr<ForceDensityField<T>> cloned;
    try {
      cloned = result_obj.cast<std::shared_ptr<ForceDensityField<T>>>();
    } catch (const py::cast_error& e) {
      throw std::logic_error(
          "DoClone() must return a `ForceDensityField<T>`. Got " +
          py::str(result_obj.get_type()).cast<std::string>() +
          " Make sure your DoClone() returns a new instance of the same "
          "Python class, e.g., `return MyForceDensityField(...)`.");
    }
    if (cloned.get() == nullptr) {
      throw std::logic_error(
          "DoClone() must not return None. Did you forget to return a new "
          "instance of your class, e.g., `return MyForceDensityField(...)`.");
    } else if (cloned.get() == this) {
      throw std::logic_error(
          "DoClone() must return a clone, not itself. Return a new instance of "
          "your class, e.g., `return MyForceDensityField(...)`.");
    }
    return std::make_unique<DelegatedForceDensityField<T>>(std::move(cloned));
  }

  void DoDeclareCacheEntries(MultibodyPlant<T>* plant) override {
    PYBIND11_OVERRIDE(void, ForceDensityField<T>, DoDeclareCacheEntries, plant);
  }

  void DoDeclareInputPorts(MultibodyPlant<T>* plant) override {
    PYBIND11_OVERRIDE(void, ForceDensityField<T>, DoDeclareInputPorts, plant);
  }
};

template <typename T>
void DefineForceDensityField(py::module m, T) {
  py::tuple param = GetPyParam<T>();
  {
    constexpr auto& cls_doc = doc.ForceDensityField;
    auto cls = DefineTemplateClassWithDefault<ForceDensityField<T>,
        PyForceDensityField<T>, ForceDensityFieldBase<T>,
        std::shared_ptr<ForceDensityField<T>>>(
        m, "ForceDensityField", param, cls_doc.doc);
    cls  // BR
        .def(py::init<ForceDensityType>(),
            py::arg("density_type") = ForceDensityType::kPerCurrentVolume,
            cls_doc.ctor.doc)
        .def("has_parent_system", &ForceDensityField<T>::has_parent_system,
            cls_doc.has_parent_system.doc)
        .def("parent_system_or_throw",
            &ForceDensityField<T>::parent_system_or_throw,
            py_rvp::reference_internal, cls_doc.parent_system_or_throw.doc)
        .def_static("DeclareCacheEntry",
            &ForceDensityFieldPublic<T>::DeclareCacheEntry, py::arg("plant"),
            py::arg("description"), py::arg("value_producer"),
            py::arg("prerequisites_of_calc"), py_rvp::reference_internal,
            cls_doc.DeclareCacheEntry.doc)
        .def_static("DeclareAbstractInputPort",
            &ForceDensityFieldPublic<T>::DeclareAbstractInputPort,
            py::arg("plant"), py::arg("name"), py::arg("model_value"),
            py_rvp::reference_internal, cls_doc.DeclareAbstractInputPort.doc)
        .def_static("DeclareVectorInputPort",
            &ForceDensityFieldPublic<T>::DeclareVectorInputPort,
            py::arg("plant"), py::arg("name"), py::arg("model_vector"),
            py_rvp::reference_internal, cls_doc.DeclareVectorInputPort.doc);
  }

  {
    constexpr auto& cls_doc = doc.GravityForceField;
    auto cls = DefineTemplateClassWithDefault<GravityForceField<T>,
        ForceDensityField<T>, std::shared_ptr<GravityForceField<T>>>(
        m, "GravityForceField", param, cls_doc.doc);
    cls  // BR
        .def(py::init<const Vector3<T>&, const T&>(), py::arg("gravity_vector"),
            py::arg("mass_density"), cls_doc.ctor.doc);
  }
}

void DefineDeformableBody(py::module m) {
  using Class = DeformableBody<double>;
  constexpr auto& cls_doc = doc.DeformableBody;
  py::class_<Class> cls(m, "DeformableBody", cls_doc.doc);
  BindMultibodyElementMixin<double>(&cls);
  cls  // BR
      .def("body_id", &Class::body_id, cls_doc.body_id.doc)
      .def("name", &Class::name, cls_doc.name.doc)
      .def("scoped_name", &Class::scoped_name, cls_doc.scoped_name.doc)
      .def("geometry_id", &Class::geometry_id, cls_doc.geometry_id.doc)
      .def("config", &Class::config, py_rvp::reference_internal,
          cls_doc.config.doc)
      .def("num_dofs", &Class::num_dofs, cls_doc.num_dofs.doc)
      .def("reference_positions", &Class::reference_positions,
          py_rvp::reference_internal, cls_doc.reference_positions.doc)
      // TODO(xuchenhan-tri): Bind fem_model() or make it internal.
      .def("external_forces", &Class::external_forces,
          py_rvp::reference_internal, cls_doc.external_forces.doc)
      .def("discrete_state_index", &Class::discrete_state_index,
          cls_doc.discrete_state_index.doc)
      .def("is_enabled_parameter_index", &Class::is_enabled_parameter_index,
          cls_doc.is_enabled_parameter_index.doc)
      .def("SetWallBoundaryCondition", &Class::SetWallBoundaryCondition,
          py::arg("p_WQ"), py::arg("n_W"), cls_doc.SetWallBoundaryCondition.doc)
      .def("AddFixedConstraint", &Class::AddFixedConstraint, py::arg("body_B"),
          py::arg("X_BA"), py::arg("shape_G"), py::arg("X_BG"),
          cls_doc.AddFixedConstraint.doc)
      .def("has_fixed_constraint", &Class::has_fixed_constraint,
          cls_doc.has_fixed_constraint.doc)
      .def("SetPositions", &Class::SetPositions, py::arg("context"),
          py::arg("q"), cls_doc.SetPositions.doc)
      .def("GetPositions", &Class::GetPositions, py::arg("context"),
          cls_doc.GetPositions.doc)
      .def("SetVelocities", &Class::SetVelocities, py::arg("context"),
          py::arg("v"), cls_doc.SetVelocities.doc)
      .def("GetVelocities", &Class::GetVelocities, py::arg("context"),
          cls_doc.GetVelocities.doc)
      .def("SetPositionsAndVelocities", &Class::SetPositionsAndVelocities,
          py::arg("context"), py::arg("q"), py::arg("v"),
          cls_doc.SetPositionsAndVelocities.doc)
      .def("GetPositionsAndVelocities", &Class::GetPositionsAndVelocities,
          py::arg("context"), cls_doc.GetPositionsAndVelocities.doc)
      .def("is_enabled", &Class::is_enabled, py::arg("context"),
          cls_doc.is_enabled.doc)
      .def("Disable", &Class::Disable, py::arg("context"), cls_doc.Disable.doc)
      .def("Enable", &Class::Enable, py::arg("context"), cls_doc.Enable.doc)
      .def("set_default_pose", &Class::set_default_pose, py::arg("X_WD"),
          cls_doc.set_default_pose.doc)
      .def("get_default_pose", &Class::get_default_pose,
          cls_doc.get_default_pose.doc)
      .def("CalcCenterOfMassPositionInWorld",
          &Class::CalcCenterOfMassPositionInWorld, py::arg("context"),
          cls_doc.CalcCenterOfMassPositionInWorld.doc)
      .def("CalcCenterOfMassTranslationalVelocityInWorld",
          &Class::CalcCenterOfMassTranslationalVelocityInWorld,
          py::arg("context"),
          cls_doc.CalcCenterOfMassTranslationalVelocityInWorld.doc)
      .def("CalcEffectiveAngularVelocity", &Class::CalcEffectiveAngularVelocity,
          py::arg("context"), cls_doc.CalcEffectiveAngularVelocity.doc);
}

}  // namespace

PYBIND11_MODULE(tree, m) {
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::multibody;

  m.doc() = "Bindings for MultibodyTree and related components.";

  py::module::import("pydrake.common.eigen_geometry");
  py::module::import("pydrake.multibody.math");
  py::module::import("pydrake.multibody.fem");
  py::module::import("pydrake.systems.framework");
  py::module::import("pydrake.geometry");

  internal::DefineTreeInertia(m);
  DoScalarIndependentDefinitions(m);
  type_visit(
      [m](auto dummy) {
        DefineMultibodyForces(m, dummy);
        DefineForceDensityField(m, dummy);
        DoScalarDependentDefinitions(m, dummy);
      },
      CommonScalarPack{});
  DefineDeformableBody(m);

  ExecuteExtraPythonCode(m);
}

}  // namespace pydrake
}  // namespace drake
