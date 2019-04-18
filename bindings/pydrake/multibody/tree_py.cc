#include "pybind11/eigen.h"
#include "pybind11/eval.h"
#include "pybind11/pybind11.h"

#include "drake/bindings/pydrake/common/deprecation_pybind.h"
#include "drake/bindings/pydrake/common/drake_optional_pybind.h"
#include "drake/bindings/pydrake/common/type_safe_index_pybind.h"
#include "drake/bindings/pydrake/documentation_pybind.h"
#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/tree/body.h"
#include "drake/multibody/tree/force_element.h"
#include "drake/multibody/tree/frame.h"
#include "drake/multibody/tree/joint.h"
#include "drake/multibody/tree/joint_actuator.h"
#include "drake/multibody/tree/multibody_forces.h"
#include "drake/multibody/tree/multibody_tree.h"  // `JacobianWrtVariable`
#include "drake/multibody/tree/multibody_tree_indexes.h"
#include "drake/multibody/tree/prismatic_joint.h"
#include "drake/multibody/tree/revolute_joint.h"
#include "drake/multibody/tree/rigid_body.h"
#include "drake/multibody/tree/weld_joint.h"

namespace drake {
namespace pydrake {

// TODO(eric.cousineau): Expose available scalar types.
using T = double;

using std::string;

using math::RigidTransform;

// Binds `MultibodyTreeElement` methods.
// N.B. We do this rather than inheritance because this template is more of a
// mixin than it is a parent class (since it is not used for its dynamic
// polymorphism).
// TODO(jamiesnape): Add documentation for bindings generated with this
// function.
template <typename PyClass>
void BindMultibodyTreeElementMixin(PyClass* pcls) {
  using Class = typename PyClass::type;
  // TODO(eric.cousineau): Fix docstring generation for `MultibodyTreeElement`.
  auto& cls = *pcls;
  cls  // BR
      .def("index", &Class::index)
      .def("model_instance", &Class::model_instance);
  // Deprecated:
  cls.def("get_parent_tree", &Class::get_parent_tree, py_reference_internal);
  DeprecateAttribute(
      cls, "get_parent_tree", "`get_parent_tree()` will soon be internal.");
}

constexpr char doc_iso3_deprecation[] = R"""(
This API using Isometry3 is / will be deprecated soon with the resolution of
#9865. We only offer it for backwards compatibility. DO NOT USE!.
)""";

PYBIND11_MODULE(tree, m) {
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::multibody;
  constexpr auto& doc = pydrake_doc.drake.multibody;

  m.doc() = "Bindings for MultibodyTree and related components.";

  py::module::import("pydrake.common.eigen_geometry");
  py::module::import("pydrake.multibody.math");

  // To simplify checking binding coverage, these are defined in the same order
  // as `multibody_tree_indexes.h`.
  BindTypeSafeIndex<FrameIndex>(m, "FrameIndex", doc.FrameIndex.doc);
  BindTypeSafeIndex<BodyIndex>(m, "BodyIndex", doc.BodyIndex.doc);
  BindTypeSafeIndex<ForceElementIndex>(
      m, "ForceElementIndex", doc.ForceElementIndex.doc);
  BindTypeSafeIndex<JointIndex>(m, "JointIndex", doc.JointIndex.doc);
  BindTypeSafeIndex<JointActuatorIndex>(
      m, "JointActuatorIndex", doc.JointActuatorIndex.doc);
  BindTypeSafeIndex<ModelInstanceIndex>(
      m, "ModelInstanceIndex", doc.ModelInstanceIndex.doc);
  m.def("world_index", &world_index, doc.world_index.doc);

  // Frames.
  {
    using Class = Frame<T>;
    constexpr auto& cls_doc = doc.Frame;
    py::class_<Class> cls(m, "Frame", cls_doc.doc);
    BindMultibodyTreeElementMixin(&cls);
    cls  // BR
        .def("name", &Class::name, cls_doc.name.doc)
        .def("body", &Class::body, py_reference_internal, cls_doc.body.doc)
        .def("GetFixedPoseInBodyFrame", &Frame<double>::GetFixedPoseInBodyFrame,
            cls_doc.GetFixedPoseInBodyFrame.doc);
  }

  {
    using Class = BodyFrame<T>;
    constexpr auto& cls_doc = doc.BodyFrame;
    py::class_<Class, Frame<T>> cls(m, "BodyFrame", cls_doc.doc);
    // No need to re-bind element mixins from `Frame`.
  }

  {
    using Class = FixedOffsetFrame<T>;
    constexpr auto& cls_doc = doc.FixedOffsetFrame;
    py::class_<Class, Frame<T>>(m, "FixedOffsetFrame", cls_doc.doc)
        .def(py::init<const std::string&, const Frame<T>&,
                 const RigidTransform<T>&, optional<ModelInstanceIndex>>(),
            py::arg("name"), py::arg("P"), py::arg("X_PF"),
            py::arg("model_instance") = nullopt, cls_doc.ctor.doc_4args)
        .def(py::init([](const std::string& name, const Frame<T>& P,
                          const Isometry3<T>& X_PF,
                          optional<ModelInstanceIndex> model_instance) {
          WarnDeprecated(doc_iso3_deprecation);
          return std::make_unique<Class>(
              name, P, RigidTransform<T>(X_PF), model_instance);
        }),
            py::arg("name"), py::arg("P"), py::arg("X_PF"),
            py::arg("model_instance") = nullopt, doc_iso3_deprecation);
  }

  // Bodies.
  {
    using Class = Body<T>;
    constexpr auto& cls_doc = doc.Body;
    py::class_<Class> cls(m, "Body", cls_doc.doc);
    BindMultibodyTreeElementMixin(&cls);
    cls  // BR
        .def("name", &Class::name, cls_doc.name.doc)
        .def("body_frame", &Class::body_frame, py_reference_internal,
            cls_doc.body_frame.doc)
        .def("GetForceInWorld", &Class::GetForceInWorld, py::arg("context"),
            py::arg("forces"), cls_doc.GetForceInWorld.doc)
        .def("AddInForceInWorld", &Class::AddInForceInWorld, py::arg("context"),
            py::arg("F_Bo_W"), py::arg("forces"), cls_doc.AddInForceInWorld.doc)
        .def("AddInForce", &Class::AddInForce, py::arg("context"),
            py::arg("p_BP_E"), py::arg("F_Bp_E"), py::arg("frame_E"),
            py::arg("forces"), cls_doc.AddInForce.doc);
  }

  {
    using Class = RigidBody<T>;
    constexpr auto& cls_doc = doc.RigidBody;
    py::class_<Class, Body<T>> cls(m, "RigidBody", cls_doc.doc);
  }

  // Joints.
  {
    using Class = Joint<T>;
    constexpr auto& cls_doc = doc.Joint;
    py::class_<Class> cls(m, "Joint", cls_doc.doc);
    BindMultibodyTreeElementMixin(&cls);
    cls  // BR
        .def("name", &Class::name, cls_doc.name.doc)
        .def("parent_body", &Class::parent_body, py_reference_internal,
            cls_doc.parent_body.doc)
        .def("child_body", &Class::child_body, py_reference_internal,
            cls_doc.child_body.doc)
        .def("frame_on_parent", &Class::frame_on_parent, py_reference_internal,
            cls_doc.frame_on_parent.doc)
        .def("frame_on_child", &Class::frame_on_child, py_reference_internal,
            cls_doc.frame_on_child.doc)
        .def("position_start", &Class::position_start,
            cls_doc.position_start.doc)
        .def("velocity_start", &Class::velocity_start,
            cls_doc.velocity_start.doc)
        .def("num_positions", &Class::num_positions, cls_doc.num_positions.doc)
        .def("num_velocities", &Class::num_velocities,
            cls_doc.num_velocities.doc)
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
            cls_doc.acceleration_upper_limits.doc);
  }

  {
    using Class = PrismaticJoint<T>;
    constexpr auto& cls_doc = doc.PrismaticJoint;
    py::class_<Class, Joint<T>> cls(m, "PrismaticJoint", cls_doc.doc);
    cls  // BR
        .def(py::init<const string&, const Frame<T>&, const Frame<T>&,
                 const Vector3<T>&, double, double, double>(),
            py::arg("name"), py::arg("frame_on_parent"),
            py::arg("frame_on_child"), py::arg("axis"),
            py::arg("pos_lower_limit") =
                -std::numeric_limits<double>::infinity(),
            py::arg("pos_upper_limit") =
                std::numeric_limits<double>::infinity(),
            py::arg("damping") = 0, cls_doc.ctor.doc)
        .def("get_translation", &Class::get_translation, py::arg("context"),
            cls_doc.get_translation.doc)
        .def("set_translation", &Class::set_translation, py::arg("context"),
            py::arg("translation"), cls_doc.set_translation.doc)
        .def("get_translation_rate", &Class::get_translation_rate,
            py::arg("context"), cls_doc.get_translation_rate.doc)
        .def("set_translation_rate", &Class::set_translation_rate,
            py::arg("context"), py::arg("translation_dot"),
            cls_doc.set_translation_rate.doc)
        .def("set_random_translation_distribution",
            &Class::set_random_translation_distribution, py::arg("translation"),
            cls_doc.set_random_translation_distribution.doc);
  }

  {
    using Class = RevoluteJoint<T>;
    constexpr auto& cls_doc = doc.RevoluteJoint;
    py::class_<Class, Joint<T>> cls(m, "RevoluteJoint", cls_doc.doc);
    cls  // BR
        .def(py::init<const string&, const Frame<T>&, const Frame<T>&,
                 const Vector3<T>&, double>(),
            py::arg("name"), py::arg("frame_on_parent"),
            py::arg("frame_on_child"), py::arg("axis"), py::arg("damping") = 0,
            cls_doc.ctor.doc_5args)
        .def("get_angle", &Class::get_angle, py::arg("context"),
            cls_doc.get_angle.doc)
        .def("set_angle", &Class::set_angle, py::arg("context"),
            py::arg("angle"), cls_doc.set_angle.doc)
        .def("set_random_angle_distribution",
            &Class::set_random_angle_distribution, py::arg("angle"),
            cls_doc.set_random_angle_distribution.doc);
  }

  {
    using Class = WeldJoint<T>;
    constexpr auto& cls_doc = doc.WeldJoint;
    py::class_<Class, Joint<T>> cls(m, "WeldJoint", cls_doc.doc);
    cls  // BR
        .def(py::init<const string&, const Frame<T>&, const Frame<T>&,
                 const RigidTransform<T>&>(),
            py::arg("name"), py::arg("parent_frame_P"),
            py::arg("child_frame_C"), py::arg("X_PC"), cls_doc.ctor.doc)
        .def(py::init(
                 [](const std::string& name, const Frame<T>& parent_frame_P,
                     const Frame<T>& child_frame_C, const Isometry3<T>& X_PC) {
                   WarnDeprecated(doc_iso3_deprecation);
                   return std::make_unique<Class>(name, parent_frame_P,
                       child_frame_C, RigidTransform<T>(X_PC));
                 }),
            py::arg("name"), py::arg("parent_frame_P"),
            py::arg("child_frame_C"), py::arg("X_PC"), doc_iso3_deprecation);
  }

  // Actuators.
  {
    using Class = JointActuator<T>;
    constexpr auto& cls_doc = doc.JointActuator;
    py::class_<Class> cls(m, "JointActuator", cls_doc.doc);
    BindMultibodyTreeElementMixin(&cls);
    cls  // BR
        .def("name", &Class::name, cls_doc.name.doc)
        .def("joint", &Class::joint, py_reference_internal, cls_doc.joint.doc);
  }

  // Force Elements.
  {
    using Class = ForceElement<T>;
    constexpr auto& cls_doc = doc.ForceElement;
    py::class_<Class> cls(m, "ForceElement", cls_doc.doc);
    BindMultibodyTreeElementMixin(&cls);
  }

  {
    using Class = UniformGravityFieldElement<T>;
    constexpr auto& cls_doc = doc.UniformGravityFieldElement;
    py::class_<Class, ForceElement<T>>(
        m, "UniformGravityFieldElement", cls_doc.doc)
        .def(py::init<>(), cls_doc.ctor.doc_0args)
        .def(py::init<Vector3<double>>(), py::arg("g_W"),
            cls_doc.ctor.doc_1args);
  }

  // MultibodyForces
  {
    using Class = MultibodyForces<T>;
    constexpr auto& cls_doc = doc.MultibodyForces;
    py::class_<Class> cls(m, "MultibodyForces", cls_doc.doc);
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
        .def("SetZero", &Class::SetZero, cls_doc.SetZero.doc)
        .def("generalized_forces", &Class::generalized_forces,
            cls_doc.generalized_forces.doc)
        .def("mutable_generalized_forces", &Class::mutable_generalized_forces,
            py_reference_internal, cls_doc.mutable_generalized_forces.doc)
        // WARNING: Do not bind `body_forces` or `mutable_body_forces` because
        // they use `internal::BodyNodeIndex`. Instead, use force API in Body.
        .def("AddInForces", &Class::AddInForces, py::arg("addend"),
            cls_doc.AddInForces.doc);
  }

  {
    using Enum = JacobianWrtVariable;
    constexpr auto& enum_doc = doc.JacobianWrtVariable;
    py::enum_<Enum> enum_py(m, "JacobianWrtVariable", enum_doc.doc);
    enum_py  // BR
        .value("kQDot", Enum::kQDot, enum_doc.kQDot.doc)
        .value("kV", Enum::kV, enum_doc.kV.doc);
  }

  // Inertias
  {
    using Class = UnitInertia<T>;
    constexpr auto& cls_doc = doc.UnitInertia;
    py::class_<Class> cls(m, "UnitInertia", cls_doc.doc);
    cls  // BR
        .def(py::init(), cls_doc.ctor.doc_0args)
        .def(py::init<const T&, const T&, const T&>(), py::arg("Ixx"),
            py::arg("Iyy"), py::arg("Izz"), cls_doc.ctor.doc_3args);
  }
  {
    using Class = SpatialInertia<T>;
    constexpr auto& cls_doc = doc.SpatialInertia;
    py::class_<Class> cls(m, "SpatialInertia", cls_doc.doc);
    cls  // BR
        .def(py::init(), cls_doc.ctor.doc_0args)
        .def(py::init<const T&, const Eigen::Ref<const Vector3<T>>&,
                 const UnitInertia<T>&>(),
            py::arg("mass"), py::arg("p_PScm_E"), py::arg("G_SP_E"),
            cls_doc.ctor.doc_3args);
  }
}

}  // namespace pydrake
}  // namespace drake
