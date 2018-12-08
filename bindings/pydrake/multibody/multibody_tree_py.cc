#include "pybind11/eigen.h"
#include "pybind11/eval.h"
#include "pybind11/pybind11.h"

#include "drake/bindings/pydrake/documentation_pybind.h"
#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/bindings/pydrake/systems/systems_pybind.h"
#include "drake/bindings/pydrake/util/deprecation_pybind.h"
#include "drake/bindings/pydrake/util/drake_optional_pybind.h"
#include "drake/bindings/pydrake/util/eigen_geometry_pybind.h"
#include "drake/bindings/pydrake/util/type_safe_index_pybind.h"
#include "drake/geometry/query_results/penetration_as_point_pair.h"
#include "drake/multibody/multibody_tree/joints/prismatic_joint.h"
#include "drake/multibody/multibody_tree/joints/revolute_joint.h"
#include "drake/multibody/multibody_tree/joints/weld_joint.h"
#include "drake/multibody/multibody_tree/math/spatial_force.h"
#include "drake/multibody/multibody_tree/math/spatial_vector.h"
#include "drake/multibody/multibody_tree/math/spatial_velocity.h"
#include "drake/multibody/multibody_tree/multibody_forces.h"
#include "drake/multibody/multibody_tree/multibody_plant/contact_info.h"
#include "drake/multibody/multibody_tree/multibody_plant/contact_results.h"
#include "drake/multibody/multibody_tree/multibody_plant/multibody_plant.h"
#include "drake/multibody/multibody_tree/multibody_tree.h"
#include "drake/multibody/parsing/sdf_parser.h"

namespace drake {
namespace pydrake {
namespace {

using std::string;

using geometry::SceneGraph;

// TODO(eric.cousineau): Expose available scalar types.
using T = double;

// Binds `MultibodyTreeElement` methods.
// N.B. We do this rather than inheritance because this template is more of a
// mixin than it is a parent class (since it is not used for its dynamic
// polymorphism).
// TODO(jamiesnape): Add documentation for bindings generated with this
// function.
template <typename PyClass>
void BindMultibodyTreeElementMixin(PyClass* pcls) {
  using Class = typename PyClass::type;
  auto& cls = *pcls;
  cls  // BR
      .def("get_parent_tree", &Class::get_parent_tree, py_reference_internal)
      .def("index", &Class::index)
      .def("model_instance", &Class::model_instance);
}

void init_module(py::module m) {
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::multibody;
  constexpr auto& doc = pydrake_doc.drake.multibody;

  using systems::Context;

  // To simplify checking binding coverage, these are defined in the same order
  // as `multibody_tree_indexes.h`.
  // TODO(jamiesnape): Extract documentation automatically.
  BindTypeSafeIndex<FrameIndex>(m, "FrameIndex",
      "Type used to identify frames by index in a multibody tree system.");
  BindTypeSafeIndex<BodyIndex>(m, "BodyIndex",
      "Type used to identify bodies by index in a multibody tree system.");
  BindTypeSafeIndex<MobilizerIndex>(m, "MobilizerIndex",
      "Type used to identify mobilizers by index in a multibody tree system.");
  BindTypeSafeIndex<BodyNodeIndex>(m, "BodyNodeIndex",
      "Type used to identify tree nodes by index within a multibody tree "
      "system.");
  BindTypeSafeIndex<ForceElementIndex>(m, "ForceElementIndex",
      "Type used to identify force elements by index within a multibody tree "
      "system.");
  BindTypeSafeIndex<JointIndex>(m, "JointIndex",
      "Type used to identify joints by index within a multibody tree system.");
  BindTypeSafeIndex<JointActuatorIndex>(m, "JointActuatorIndex",
      "Type used to identify actuators by index within a multibody tree "
      "system.");
  BindTypeSafeIndex<ModelInstanceIndex>(m, "ModelInstanceIndex",
      "Type used to identify model instances by index within a multibody tree "
      "system.");
  m.def("world_index", &world_index, doc.world_index.doc);

  // Frames.
  {
    using Class = Frame<T>;
    py::class_<Class> cls(m, "Frame", doc.Frame.doc);
    BindMultibodyTreeElementMixin(&cls);
    cls  // BR
        .def("name", &Class::name, doc.Frame.name.doc)
        .def("body", &Class::body, py_reference_internal, doc.Frame.body.doc);
  }

  {
    using Class = BodyFrame<T>;
    py::class_<Class, Frame<T>> cls(m, "BodyFrame", doc.BodyFrame.doc);
    // No need to re-bind element mixins from `Frame`.
  }

  // Bodies.
  {
    using Class = Body<T>;
    py::class_<Class> cls(m, "Body", doc.Body.doc);
    BindMultibodyTreeElementMixin(&cls);
    cls  // BR
        .def("name", &Class::name, doc.Body.name.doc)
        .def("body_frame", &Class::body_frame, py_reference_internal,
            doc.Body.body_frame.doc);
  }

  {
    using Class = RigidBody<T>;
    py::class_<Class, Body<T>> cls(m, "RigidBody", doc.RigidBody.doc);
  }

  // Joints.
  {
    using Class = Joint<T>;
    py::class_<Class> cls(m, "Joint", doc.Joint.doc);
    BindMultibodyTreeElementMixin(&cls);
    cls  // BR
        .def("name", &Class::name, doc.Joint.name.doc)
        .def("parent_body", &Class::parent_body, py_reference_internal,
            doc.Joint.parent_body.doc)
        .def("child_body", &Class::child_body, py_reference_internal,
            doc.Joint.child_body.doc)
        .def("frame_on_parent", &Class::frame_on_parent, py_reference_internal,
            doc.Joint.frame_on_parent.doc)
        .def("frame_on_child", &Class::frame_on_child, py_reference_internal,
            doc.Joint.frame_on_child.doc)
        .def("position_start", &Class::position_start,
            doc.Joint.position_start.doc)
        .def("velocity_start", &Class::velocity_start,
            doc.Joint.velocity_start.doc)
        .def(
            "num_positions", &Class::num_positions, doc.Joint.num_positions.doc)
        .def("num_velocities", &Class::num_velocities,
            doc.Joint.num_velocities.doc)
        .def("lower_limits", &Class::lower_limits, doc.Joint.lower_limits.doc)
        .def("upper_limits", &Class::upper_limits, doc.Joint.upper_limits.doc);

    // Add deprecated methods.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
    cls.def("num_dofs", &Class::num_dofs, doc.Joint.num_dofs.doc);
#pragma GCC diagnostic pop  // pop -Wdeprecated-declarations
    cls.attr("message_num_dofs") = "Please use num_velocities().";
    DeprecateAttribute(cls, "num_dofs", cls.attr("message_num_dofs"));
  }

  {
    using Class = PrismaticJoint<T>;
    py::class_<Class, Joint<T>> cls(
        m, "PrismaticJoint", doc.PrismaticJoint.doc);
    cls  // BR
        .def("get_translation", &Class::get_translation, py::arg("context"),
            doc.PrismaticJoint.get_translation.doc)
        .def("set_translation", &Class::set_translation, py::arg("context"),
            py::arg("translation"), doc.PrismaticJoint.set_translation.doc)
        .def("get_translation_rate", &Class::get_translation_rate,
            py::arg("context"), doc.PrismaticJoint.get_translation_rate.doc)
        .def("set_translation_rate", &Class::set_translation_rate,
            py::arg("context"), py::arg("translation_dot"),
            doc.PrismaticJoint.set_translation_rate.doc);
  }

  {
    using Class = RevoluteJoint<T>;
    py::class_<Class, Joint<T>> cls(m, "RevoluteJoint", doc.RevoluteJoint.doc);
    cls  // BR
        .def(py::init<const string&, const Frame<T>&, const Frame<T>&,
                 const Vector3<T>&, double>(),
            py::arg("name"), py::arg("frame_on_parent"),
            py::arg("frame_on_child"), py::arg("axis"), py::arg("damping") = 0,
            doc.RevoluteJoint.ctor.doc_5args)
        .def("get_angle", &Class::get_angle, py::arg("context"),
            doc.RevoluteJoint.get_angle.doc)
        .def("set_angle", &Class::set_angle, py::arg("context"),
            py::arg("angle"), doc.RevoluteJoint.set_angle.doc);
  }

  {
    using Class = WeldJoint<T>;
    py::class_<Class, Joint<T>> cls(m, "WeldJoint", doc.WeldJoint.doc);
    cls  // BR
        .def(py::init<const string&, const Frame<T>&, const Frame<T>&,
                 const Isometry3<double>&>(),
            py::arg("name"), py::arg("parent_frame_P"),
            py::arg("child_frame_C"), py::arg("X_PC"),
            doc.WeldJoint.ctor.doc_4args);
  }

  // Actuators.
  {
    using Class = JointActuator<T>;
    py::class_<Class> cls(m, "JointActuator", doc.JointActuator.doc);
    BindMultibodyTreeElementMixin(&cls);
    cls  // BR
        .def("name", &Class::name, doc.JointActuator.name.doc)
        .def("joint", &Class::joint, py_reference_internal,
            doc.JointActuator.joint.doc);
  }

  // Force Elements.
  {
    using Class = ForceElement<T>;
    py::class_<Class> cls(m, "ForceElement", doc.ForceElement.doc);
    BindMultibodyTreeElementMixin(&cls);
  }

  {
    using Class = UniformGravityFieldElement<T>;
    py::class_<Class, ForceElement<T>>(
        m, "UniformGravityFieldElement", doc.UniformGravityFieldElement.doc)
        .def(py::init<Vector3<double>>(), py::arg("g_W"),
            doc.UniformGravityFieldElement.ctor.doc_1args);
  }

  // MultibodyForces
  {
    using Class = MultibodyForces<T>;
    py::class_<Class> cls(m, "MultibodyForces", doc.MultibodyForces.doc);
    cls  // BR
        .def(py::init<MultibodyTree<double>&>(), py::arg("model"),
            doc.MultibodyForces.ctor.doc_1args);
  }

  // Tree.
  {
    // N.B. Pending a concrete direction on #9366, a minimal subset of the
    // `MultibodyTree` API will be exposed.
    using Class = MultibodyTree<T>;
    py::class_<Class> cls(m, "MultibodyTree", doc.MultibodyTree.doc);

    cls  // BR
        .def("CalcRelativeTransform", &Class::CalcRelativeTransform,
            py::arg("context"), py::arg("frame_A"), py::arg("frame_B"),
            doc.MultibodyTree.CalcRelativeTransform.doc)
        .def("num_frames", &Class::num_frames, doc.MultibodyTree.num_frames.doc)
        .def("get_body", &Class::get_body, py::arg("body_index"),
            py_reference_internal, doc.MultibodyTree.get_body.doc)
        .def("get_joint", &Class::get_joint, py::arg("joint_index"),
            py_reference_internal, doc.MultibodyTree.get_joint.doc)
        .def("get_joint_actuator", &Class::get_joint_actuator,
            py::arg("actuator_index"), py_reference_internal,
            doc.MultibodyTree.get_joint_actuator.doc)
        .def("get_frame", &Class::get_frame, py::arg("frame_index"),
            py_reference_internal, doc.MultibodyTree.get_frame.doc)
        .def("GetModelInstanceName",
            overload_cast_explicit<const string&, ModelInstanceIndex>(
                &Class::GetModelInstanceName),
            py::arg("model_instance"), py_reference_internal,
            doc.MultibodyTree.GetModelInstanceName.doc)
        .def("GetPositionsAndVelocities",
            [](const MultibodyTree<T>* self,
                const Context<T>& context) -> Eigen::Ref<const VectorX<T>> {
              return self->GetPositionsAndVelocities(context);
            },
            py_reference,
            // Keep alive, ownership: `return` keeps `Context` alive.
            py::keep_alive<0, 2>(), py::arg("context"),
            doc.MultibodyTree.GetPositionsAndVelocities.doc_1args)
        .def("GetMutablePositionsAndVelocities",
            [](const MultibodyTree<T>* self,
                Context<T>* context) -> Eigen::Ref<VectorX<T>> {
              return self->GetMutablePositionsAndVelocities(context);
            },
            py_reference,
            // Keep alive, ownership: `return` keeps `Context` alive.
            py::keep_alive<0, 2>(), py::arg("context"),
            doc.MultibodyTree.GetMutablePositionsAndVelocities.doc)
        .def("CalcPointsPositions",
            [](const Class* self, const Context<T>& context,
                const Frame<T>& frame_B,
                const Eigen::Ref<const MatrixX<T>>& p_BQi,
                const Frame<T>& frame_A) {
              MatrixX<T> p_AQi(p_BQi.rows(), p_BQi.cols());
              self->CalcPointsPositions(
                  context, frame_B, p_BQi, frame_A, &p_AQi);
              return p_AQi;
            },
            py::arg("context"), py::arg("frame_B"), py::arg("p_BQi"),
            py::arg("frame_A"), doc.MultibodyTree.CalcPointsPositions.doc)
        .def("CalcFrameGeometricJacobianExpressedInWorld",
            [](const Class* self, const Context<T>& context,
                const Frame<T>& frame_B, const Vector3<T>& p_BoFo_B) {
              MatrixX<T> Jv_WF(6, self->num_velocities());
              self->CalcFrameGeometricJacobianExpressedInWorld(
                  context, frame_B, p_BoFo_B, &Jv_WF);
              return Jv_WF;
            },
            py::arg("context"), py::arg("frame_B"),
            py::arg("p_BoFo_B") = Vector3<T>::Zero().eval(),
            doc.MultibodyTree.CalcFrameGeometricJacobianExpressedInWorld.doc)
        .def("CalcInverseDynamics",
            overload_cast_explicit<VectorX<T>, const Context<T>&,
                const VectorX<T>&, const MultibodyForces<T>&>(
                &Class::CalcInverseDynamics),
            py::arg("context"), py::arg("known_vdot"),
            py::arg("external_forces"),
            doc.MultibodyTree.CalcInverseDynamics.doc_3args)
        .def("SetFreeBodyPoseOrThrow",
            overload_cast_explicit<void, const Body<T>&, const Isometry3<T>&,
                systems::Context<T>*>(&Class::SetFreeBodyPoseOrThrow),
            py::arg("body"), py::arg("X_WB"), py::arg("context"),
            doc.MultibodyTree.SetFreeBodyPoseOrThrow.doc_3args)
        .def("GetPositionsFromArray", &Class::GetPositionsFromArray,
            py::arg("model_instance"), py::arg("q_array"),
            doc.MultibodyTree.get_positions_from_array.doc)
        .def("GetVelocitiesFromArray", &Class::GetVelocitiesFromArray,
            py::arg("model_instance"), py::arg("v_array"),
            doc.MultibodyTree.get_velocities_from_array.doc)
        .def("SetFreeBodySpatialVelocityOrThrow",
            [](const Class* self, const Body<T>& body,
                const SpatialVelocity<T>& V_WB, Context<T>* context) {
              self->SetFreeBodySpatialVelocityOrThrow(body, V_WB, context);
            },
            py::arg("body"), py::arg("V_WB"), py::arg("context"),
            doc.MultibodyTree.SetFreeBodySpatialVelocityOrThrow.doc_3args)
        .def("CalcAllBodySpatialVelocitiesInWorld",
            [](const Class* self, const Context<T>& context) {
              std::vector<SpatialVelocity<T>> V_WB;
              self->CalcAllBodySpatialVelocitiesInWorld(context, &V_WB);
              return V_WB;
            },
            py::arg("context"),
            doc.MultibodyTree.CalcAllBodySpatialVelocitiesInWorld.doc)
        .def("EvalBodyPoseInWorld",
            [](const Class* self, const Context<T>& context,
                const Body<T>& body_B) {
              return self->EvalBodyPoseInWorld(context, body_B);
            },
            py::arg("context"), py::arg("body"),
            doc.MultibodyTree.EvalBodyPoseInWorld.doc)
        .def("EvalBodySpatialVelocityInWorld",
            [](const Class* self, const Context<T>& context,
                const Body<T>& body_B) {
              return self->EvalBodySpatialVelocityInWorld(context, body_B);
            },
            py::arg("context"), py::arg("body"),
            doc.MultibodyTree.EvalBodySpatialVelocityInWorld.doc)
        .def("CalcAllBodyPosesInWorld",
            [](const Class* self, const Context<T>& context) {
              std::vector<Isometry3<T>> X_WB;
              self->CalcAllBodyPosesInWorld(context, &X_WB);
              return X_WB;
            },
            py::arg("context"), doc.MultibodyTree.CalcAllBodyPosesInWorld.doc)
        .def("CalcMassMatrixViaInverseDynamics",
            [](const Class* self, const Context<T>& context) {
              MatrixX<T> H;
              const int n = self->num_velocities();
              H.resize(n, n);
              self->CalcMassMatrixViaInverseDynamics(context, &H);
              return H;
            },
            py::arg("context"))
        .def("CalcBiasTerm",
            [](const Class* self, const Context<T>& context) {
              VectorX<T> Cv;
              const int n = self->num_velocities();
              Cv.resize(n);
              self->CalcBiasTerm(context, &Cv);
              return Cv;
            },
            py::arg("context"), doc.MultibodyTree.CalcBiasTerm.doc);
    // Add deprecated methods.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
    cls.def("get_multibody_state_vector",
        [](const MultibodyTree<T>* self,
            const Context<T>& context) -> Eigen::Ref<const VectorX<T>> {
          return self->get_multibody_state_vector(context);
        },
        py_reference,
        // Keep alive, ownership: `return` keeps `Context` alive.
        py::keep_alive<0, 2>(), py::arg("context"),
        doc.MultibodyTree.get_multibody_state_vector.doc_1args);
    cls.def("get_mutable_multibody_state_vector",
        [](const MultibodyTree<T>* self,
            Context<T>* context) -> Eigen::Ref<VectorX<T>> {
          return self->get_mutable_multibody_state_vector(context);
        },
        py_reference,
        // Keep alive, ownership: `return` keeps `Context` alive.
        py::keep_alive<0, 2>(), py::arg("context"),
        doc.MultibodyTree.get_mutable_multibody_state_vector.doc);
#pragma GCC diagnostic pop  // pop -Wdeprecated-declarations
    cls.attr("message_get_mutable_multibody_state_vector") =
        "Please use GetMutablePositionsAndVelocities().";
    DeprecateAttribute(cls, "get_mutable_multibody_state_vector",
        cls.attr("message_get_mutable_multibody_state_vector"));
    cls.attr("message_get_multibody_state_vector") =
        "Please use GetPositionsAndVelocities().";
    DeprecateAttribute(cls, "get_multibody_state_vector",
        cls.attr("message_get_multibody_state_vector"));
  }
}

void init_math(py::module m) {
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::multibody;
  constexpr auto& doc = pydrake_doc.drake.multibody;

  m.doc() = "MultibodyTree math functionality.";

  py::class_<SpatialVector<SpatialVelocity, T>>(
      m, "SpatialVector", doc.SpatialVector.doc)
      .def("rotational",
          [](const SpatialVector<SpatialVelocity, T>* self)
              -> const Vector3<T>& { return self->rotational(); },
          py_reference_internal, doc.SpatialVector.rotational.doc)
      .def("translational",
          [](const SpatialVector<SpatialVelocity, T>* self)
              -> const Vector3<T>& { return self->translational(); },
          py_reference_internal, doc.SpatialVector.translational.doc);

  py::class_<SpatialVelocity<T>, SpatialVector<SpatialVelocity, T>>(
      m, "SpatialVelocity")
      .def(py::init(), doc.SpatialVelocity.ctor.doc_3)
      .def(py::init<const Eigen::Ref<const Vector3<T>>&,
               const Eigen::Ref<const Vector3<T>>&>(),
          py::arg("w"), py::arg("v"), doc.SpatialVelocity.ctor.doc_4);
}

void init_multibody_plant(py::module m) {
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::multibody;
  constexpr auto& doc = pydrake_doc.drake.multibody;

  py::module::import("pydrake.geometry");
  py::module::import("pydrake.systems.framework");

  {
    using Class = MultibodyPlant<T>;
    py::class_<Class, systems::LeafSystem<T>> cls(
        m, "MultibodyPlant", doc.MultibodyPlant.doc);
    // N.B. These are defined as they appear in the class declaration.
    // TODO(eric.cousineau): Add model-instance based overloads beyond
    // forwarded methods.
    // Forwarded methods from `MultibodyTree`.
    cls  // BR
        .def(py::init<double>(), py::arg("time_step") = 0.)
        .def(
            "num_bodies", &Class::num_bodies, doc.MultibodyPlant.num_bodies.doc)
        .def(
            "num_joints", &Class::num_joints, doc.MultibodyPlant.num_joints.doc)
        .def("num_actuators", &Class::num_actuators,
            doc.MultibodyPlant.num_actuators.doc)
        .def("num_model_instances", &Class::num_model_instances,
            doc.MultibodyPlant.num_model_instances.doc)
        .def("num_positions",
            overload_cast_explicit<int>(&Class::num_positions),
            doc.MultibodyPlant.num_positions.doc_0args)
        .def("num_positions",
            overload_cast_explicit<int, ModelInstanceIndex>(
                &Class::num_positions),
            py::arg("model_instance"),
            doc.MultibodyPlant.num_positions.doc_1args)
        .def("num_velocities",
            overload_cast_explicit<int>(&Class::num_velocities),
            doc.MultibodyPlant.num_velocities.doc_0args)
        .def("num_velocities",
            overload_cast_explicit<int, ModelInstanceIndex>(
                &Class::num_velocities),
            doc.MultibodyPlant.num_velocities.doc_1args)
        .def("num_multibody_states", &Class::num_multibody_states,
            doc.MultibodyPlant.num_multibody_states.doc)
        .def("num_actuated_dofs",
            overload_cast_explicit<int>(&Class::num_actuated_dofs),
            doc.MultibodyPlant.num_actuated_dofs.doc_0args);
    // Construction.
    cls  // BR
        .def("AddJoint",
            [](Class * self, std::unique_ptr<Joint<T>> joint) -> auto& {
              return self->AddJoint(std::move(joint));
            },
            py::arg("joint"), py_reference_internal,
            doc.MultibodyPlant.AddJoint.doc)
        .def("WeldFrames", &Class::WeldFrames, py::arg("A"), py::arg("B"),
            py::arg("X_AB") = Isometry3<double>::Identity(),
            py_reference_internal, doc.MultibodyPlant.WeldFrames.doc)
        .def("AddForceElement",
            [](Class * self,
                std::unique_ptr<ForceElement<T>> force_element) -> auto& {
              return self->AddForceElement<ForceElement>(
                  std::move(force_element));
            },
            py::arg("force_element"), py_reference_internal,
            doc.MultibodyPlant.AddForceElement.doc);
    // Topology queries.
    cls  // BR
        .def("HasBodyNamed",
            overload_cast_explicit<bool, const string&>(&Class::HasBodyNamed),
            py::arg("name"), doc.MultibodyPlant.HasBodyNamed.doc_1args)
        .def("HasBodyNamed",
            overload_cast_explicit<bool, const string&, ModelInstanceIndex>(
                &Class::HasBodyNamed),
            py::arg("name"), py::arg("model_instance"),
            doc.MultibodyPlant.HasBodyNamed.doc_2args)
        .def("HasJointNamed",
            overload_cast_explicit<bool, const string&>(&Class::HasJointNamed),
            py::arg("name"), doc.MultibodyPlant.HasJointNamed.doc_1args)
        .def("HasJointNamed",
            overload_cast_explicit<bool, const string&, ModelInstanceIndex>(
                &Class::HasJointNamed),
            py::arg("name"), py::arg("model_instance"),
            doc.MultibodyPlant.HasJointNamed.doc_2args)
        .def("GetFrameByName",
            overload_cast_explicit<const Frame<T>&, const string&>(
                &Class::GetFrameByName),
            py::arg("name"), py_reference_internal,
            doc.MultibodyPlant.GetFrameByName.doc_1args)
        .def("GetFrameByName",
            overload_cast_explicit<const Frame<T>&, const string&,
                ModelInstanceIndex>(&Class::GetFrameByName),
            py::arg("name"), py::arg("model_instance"), py_reference_internal,
            doc.MultibodyPlant.GetFrameByName.doc_2args)
        .def("GetBodyByName",
            overload_cast_explicit<const Body<T>&, const string&>(
                &Class::GetBodyByName),
            py::arg("name"), py_reference_internal,
            doc.MultibodyPlant.GetBodyByName.doc_1args)
        .def("GetBodyByName",
            overload_cast_explicit<const Body<T>&, const string&,
                ModelInstanceIndex>(&Class::GetBodyByName),
            py::arg("name"), py::arg("model_instance"), py_reference_internal,
            doc.MultibodyPlant.GetBodyByName.doc_2args)
        .def("GetJointByName",
            [](const Class* self, const string& name) -> auto& {
              return self->GetJointByName(name);
            },
            py::arg("name"), py_reference_internal,
            doc.MultibodyPlant.GetJointByName.doc)
        .def("GetJointByName",
            [](const Class* self, const string& name,
                ModelInstanceIndex model_instance) -> auto& {
              return self->GetJointByName(name, model_instance);
            },
            py::arg("name"), py::arg("model_instance"), py_reference_internal,
            doc.MultibodyPlant.GetJointByName.doc_2)
        .def("GetJointActuatorByName",
            overload_cast_explicit<const JointActuator<T>&, const string&>(
                &Class::GetJointActuatorByName),
            py::arg("name"), py_reference_internal,
            doc.MultibodyPlant.GetJointActuatorByName.doc_1args)
        .def("GetModelInstanceByName",
            overload_cast_explicit<ModelInstanceIndex, const string&>(
                &Class::GetModelInstanceByName),
            py::arg("name"), py_reference_internal,
            doc.MultibodyPlant.GetModelInstanceByName.doc);
    // Geometry.
    cls  // BR
        .def("RegisterAsSourceForSceneGraph",
            &Class::RegisterAsSourceForSceneGraph, py::arg("scene_graph"),
            doc.MultibodyPlant.RegisterAsSourceForSceneGraph.doc)
        .def("get_source_id", &Class::get_source_id,
            doc.MultibodyPlant.get_source_id.doc)
        .def("get_geometry_query_input_port",
            &Class::get_geometry_query_input_port, py_reference_internal,
            doc.MultibodyPlant.get_geometry_query_input_port.doc)
        .def("get_geometry_poses_output_port",
            &Class::get_geometry_poses_output_port, py_reference_internal,
            doc.MultibodyPlant.get_geometry_poses_output_port.doc)
        .def("geometry_source_is_registered",
            &Class::geometry_source_is_registered,
            doc.MultibodyPlant.geometry_source_is_registered.doc)
        .def("GetBodyFromFrameId", &Class::GetBodyFromFrameId,
            py_reference_internal, doc.MultibodyPlant.GetBodyFromFrameId.doc);
    // Port accessors.
    cls  // BR
        .def("get_actuation_input_port",
            overload_cast_explicit<const systems::InputPort<T>&>(
                &Class::get_actuation_input_port),
            py_reference_internal,
            doc.MultibodyPlant.get_actuation_input_port.doc_0args)
        .def("get_continuous_state_output_port",
            overload_cast_explicit<const systems::OutputPort<T>&>(
                &Class::get_continuous_state_output_port),
            py_reference_internal,
            doc.MultibodyPlant.get_continuous_state_output_port.doc_0args)
        .def("get_contact_results_output_port",
            overload_cast_explicit<const systems::OutputPort<T>&>(
                &Class::get_contact_results_output_port),
            py_reference_internal,
            doc.MultibodyPlant.get_contact_results_output_port.doc);
    // Property accessors.
    cls  // BR
        .def("world_body", &Class::world_body, py_reference_internal,
            doc.MultibodyPlant.world_body.doc)
        .def("world_frame", &Class::world_frame, py_reference_internal,
            doc.MultibodyPlant.world_frame.doc)
        .def("tree", &Class::tree, py_reference_internal,
            pydrake_doc.drake.multibody.MultibodyTreeSystem.tree.doc)
        .def("is_finalized", &Class::is_finalized,
            doc.MultibodyPlant.is_finalized.doc)
        .def("Finalize", py::overload_cast<SceneGraph<T>*>(&Class::Finalize),
            py::arg("scene_graph") = nullptr, doc.MultibodyPlant.Finalize.doc);
    // Position and velocity accessors and mutators.
    cls  // BR
        .def("GetMutablePositionsAndVelocities",
            [](const MultibodyPlant<T>* self,
                systems::Context<T>* context) -> Eigen::Ref<VectorX<T>> {
              return self->GetMutablePositionsAndVelocities(context);
            },
            py_reference,
            // Keep alive, ownership: `return` keeps `Context` alive.
            py::keep_alive<0, 2>(), py::arg("context"),
            doc.MultibodyPlant.GetMutablePositionsAndVelocities.doc)
        .def("GetMutablePositions",
            [](const MultibodyPlant<T>* self,
                systems::Context<T>* context) -> Eigen::Ref<VectorX<T>> {
              return self->GetMutablePositions(context);
            },
            py_reference,
            // Keep alive, ownership: `return` keeps `Context` alive.
            py::keep_alive<0, 2>(), py::arg("context"),
            doc.MultibodyPlant.GetMutablePositions.doc)
        .def("GetMutableVelocities",
            [](const MultibodyPlant<T>* self,
                systems::Context<T>* context) -> Eigen::Ref<VectorX<T>> {
              return self->GetMutableVelocities(context);
            },
            py_reference,
            // Keep alive, ownership: `return` keeps `Context` alive.
            py::keep_alive<0, 2>(), py::arg("context"),
            doc.MultibodyPlant.GetMutableVelocities.doc)
        .def("GetPositions",
            [](const MultibodyPlant<T>* self,
                const systems::Context<T>& context) -> VectorX<T> {
              return self->GetPositions(context);
            },
            py_reference, py::arg("context"),
            doc.MultibodyPlant.GetPositions.doc_1args)
        .def("GetPositions",
            [](const MultibodyPlant<T>* self,
                const systems::Context<T>& context,
                multibody::ModelInstanceIndex model_instance) -> VectorX<T> {
              return self->GetPositions(context, model_instance);
            },
            py_reference, py::arg("context"), py::arg("model_instance"),
            doc.MultibodyPlant.GetPositions.doc_2args)
        .def("SetPositions",
            [](const MultibodyPlant<T>* self, systems::Context<T>* context,
                const VectorX<T>& q) { self->SetPositions(context, q); },
            py_reference, py::arg("context"), py::arg("q"),
            doc.MultibodyPlant.SetPositions.doc_2args)
        .def("SetPositions",
            [](const MultibodyPlant<T>* self, systems::Context<T>* context,
                multibody::ModelInstanceIndex model_instance,
                const VectorX<T>& q) {
              self->SetPositions(context, model_instance, q);
            },
            py_reference, py::arg("context"), py::arg("model_instance"),
            py::arg("q"), doc.MultibodyPlant.SetPositions.doc_2args)
        .def("GetVelocities",
            [](const MultibodyPlant<T>* self,
                const systems::Context<T>& context) -> VectorX<T> {
              return self->GetVelocities(context);
            },
            py_reference, py::arg("context"),
            doc.MultibodyPlant.GetVelocities.doc_1args)
        .def("GetVelocities",
            [](const MultibodyPlant<T>* self,
                const systems::Context<T>& context,
                multibody::ModelInstanceIndex model_instance) -> VectorX<T> {
              return self->GetVelocities(context, model_instance);
            },
            py_reference, py::arg("context"), py::arg("model_instance"),
            doc.MultibodyPlant.GetVelocities.doc_2args)
        .def("SetVelocities",
            [](const MultibodyPlant<T>* self, systems::Context<T>* context,
                const VectorX<T>& v) { self->SetVelocities(context, v); },
            py_reference, py::arg("context"), py::arg("v"),
            doc.MultibodyPlant.SetVelocities.doc_2args)
        .def("SetVelocities",
            [](const MultibodyPlant<T>* self, systems::Context<T>* context,
                ModelInstanceIndex model_instance, const VectorX<T>& v) {
              self->SetVelocities(context, model_instance, v);
            },
            py_reference, py::arg("context"), py::arg("model_instance"),
            py::arg("v"), doc.MultibodyPlant.SetVelocities.doc_3args)
        .def("GetPositionsAndVelocities",
            [](const MultibodyPlant<T>* self,
                const systems::Context<T>& context) -> VectorX<T> {
              return self->GetPositionsAndVelocities(context);
            },
            py_reference, py::arg("context"),
            doc.MultibodyPlant.GetPositionsAndVelocities.doc_1args)
        .def("GetPositionsAndVelocities",
            [](const MultibodyPlant<T>* self,
                const systems::Context<T>& context,
                multibody::ModelInstanceIndex model_instance) -> VectorX<T> {
              return self->GetPositionsAndVelocities(context, model_instance);
            },
            py_reference, py::arg("context"), py::arg("model_instance"),
            doc.MultibodyPlant.GetPositionsAndVelocities.doc_2args)
        .def("SetPositionsAndVelocities",
            [](const MultibodyPlant<T>* self, systems::Context<T>* context,
                const VectorX<T>& q_v) {
              self->SetPositionsAndVelocities(context, q_v);
            },
            py_reference, py::arg("context"), py::arg("q_v"),
            doc.MultibodyPlant.SetPositionsAndVelocities.doc_2args)
        .def("SetPositionsAndVelocities",
            [](const MultibodyPlant<T>* self, systems::Context<T>* context,
                multibody::ModelInstanceIndex model_instance,
                const VectorX<T>& q_v) {
              self->SetPositionsAndVelocities(context, model_instance, q_v);
            },
            py_reference, py::arg("context"), py::arg("model_instance"),
            py::arg("q_v"),
            doc.MultibodyPlant.SetPositionsAndVelocities.doc_3args);

    // Add deprecated methods.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
    cls.def("model", &Class::model, py_reference_internal);
#pragma GCC diagnostic pop  // pop -Wdeprecated-declarations
    cls.attr("message_model") = "Please use tree().";
    DeprecateAttribute(cls, "model", cls.attr("message_model"));
  }

  // PointPairContactInfo
  {
    using Class = PointPairContactInfo<T>;
    py::class_<Class>(m, "PointPairContactInfo")
        .def(py::init<BodyIndex, BodyIndex, const Vector3<T>, const Vector3<T>,
                 const T&, const T&,
                 const geometry::PenetrationAsPointPair<T>>(),
            py::arg("bodyA_index"), py::arg("bodyB_index"), py::arg("f_Bc_W"),
            py::arg("p_WC"), py::arg("separation_speed"), py::arg("slip_speed"),
            py::arg("point_pair"))
        .def("bodyA_index", &Class::bodyA_index)
        .def("bodyB_index", &Class::bodyB_index)
        .def("contact_force", &Class::contact_force)
        .def("contact_point", &Class::contact_point)
        .def("slip_speed", &Class::slip_speed)
        .def("separation_speed", &Class::separation_speed);
  }

  // ContactResults
  {
    using Class = ContactResults<T>;
    py::class_<Class>(m, "ContactResults")
        .def(py::init<>())
        .def("num_contacts", &Class::num_contacts)
        .def("AddContactInfo", &Class::AddContactInfo,
            py::arg("point_pair_info"))
        .def("contact_info", &Class::contact_info, py::arg("i"));
    pysystems::AddValueInstantiation<Class>(m);
  }
}

void init_parsing(py::module m) {
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::multibody;
  constexpr auto& doc = pydrake_doc.drake.multibody.parsing;

  // Stub in a deprecation shim for the Parser class.
  // TODO(jwnimmer-tri) Remove this stub on or about 2019-01-01.
  py::dict vars = m.attr("__dict__");
  py::exec(
      "class Parser(object):\n"
      "    def __init__(self, *args, **kwargs):\n"
      "        import pydrake.multibody.parsing\n"
      "        self._x = pydrake.multibody.parsing.Parser(*args, **kwargs)\n"
      "    def AddModelFromFile(self, *args, **kwargs):\n"
      "        return self._x.AddModelFromFile(*args, **kwargs)\n"
      "    def AddAllModelsFromFile(self, *args, **kwargs):\n"
      "        return self._x.AddAllModelsFromFile(*args, **kwargs)\n",
      py::globals(), vars);
  py::object cls = m.attr("Parser");
  const char* const message =
      "Please use class pydrake.multibody.parsing.Parser instead of "
      "class pydrake.multibody.multibody_tree.parsing.Parser.";
  DeprecateAttribute(cls, "AddModelFromFile", message);
  DeprecateAttribute(cls, "AddAllModelsFromFile", message);

  // Bind the deprecated free functions.
  // TODO(jwnimmer-tri) Remove these stubs on or about 2019-03-01.
  m.def("AddModelFromSdfFile",
      [](const string& file_name, const string& model_name,
          MultibodyPlant<double>* plant, SceneGraph<double>* scene_graph) {
        WarnDeprecated(
            "AddModelFromSdfFile is deprecated; please use the class "
            "pydrake.multibody.parsing.Parser instead.");
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
        return parsing::AddModelFromSdfFile(
            file_name, model_name, plant, scene_graph);
#pragma GCC diagnostic pop
      },
      py::arg("file_name"), py::arg("model_name"), py::arg("plant"),
      py::arg("scene_graph") = nullptr, doc.AddModelFromSdfFile.doc_4args);
  m.def("AddModelFromSdfFile",
      [](const string& file_name, MultibodyPlant<double>* plant,
          SceneGraph<double>* scene_graph) {
        WarnDeprecated(
            "AddModelFromSdfFile is deprecated; please use the class "
            "pydrake.multibody.parsing.Parser instead.");
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
        return parsing::AddModelFromSdfFile(file_name, plant, scene_graph);
#pragma GCC diagnostic pop
      },
      py::arg("file_name"), py::arg("plant"), py::arg("scene_graph") = nullptr,
      doc.AddModelFromSdfFile.doc_3args);
}

void init_all(py::module m) {
  py::dict vars = m.attr("__dict__");
  py::exec(
      "from pydrake.multibody.multibody_tree import *\n"
      "from pydrake.multibody.multibody_tree.math import *\n"
      "from pydrake.multibody.multibody_tree.multibody_plant import *\n"
      "from pydrake.multibody.multibody_tree.parsing import *\n",
      py::globals(), vars);
}

}  // namespace

PYBIND11_MODULE(multibody_tree, m) {
  PYDRAKE_PREVENT_PYTHON3_MODULE_REIMPORT(m);
  m.doc() = "MultibodyTree functionality.";

  // TODO(eric.cousineau): Split this into separate files. See discussion in
  // #8282 for info relating to the current implementation.
  init_module(m);
  init_math(m.def_submodule("math"));
  init_multibody_plant(m.def_submodule("multibody_plant"));
  init_parsing(m.def_submodule("parsing"));
  init_all(m.def_submodule("all"));
}

}  // namespace pydrake
}  // namespace drake
