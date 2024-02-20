#include "drake/bindings/pydrake/common/cpp_template_pybind.h"
#include "drake/bindings/pydrake/common/default_scalars_pybind.h"
#include "drake/bindings/pydrake/common/deprecation_pybind.h"
#include "drake/bindings/pydrake/common/eigen_geometry_pybind.h"
#include "drake/bindings/pydrake/common/eigen_pybind.h"
#include "drake/bindings/pydrake/common/identifier_pybind.h"
#include "drake/bindings/pydrake/common/serialize_pybind.h"
#include "drake/bindings/pydrake/common/type_pack.h"
#include "drake/bindings/pydrake/common/value_pybind.h"
#include "drake/bindings/pydrake/documentation_pybind.h"
#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/common/eigen_types.h"
#include "drake/geometry/query_results/penetration_as_point_pair.h"
#include "drake/geometry/scene_graph.h"
#include "drake/math/rigid_transform.h"
#include "drake/multibody/plant/contact_results.h"
#include "drake/multibody/plant/contact_results_to_lcm.h"
#include "drake/multibody/plant/deformable_model.h"
#include "drake/multibody/plant/externally_applied_spatial_force.h"
#include "drake/multibody/plant/externally_applied_spatial_force_multiplexer.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/plant/multibody_plant_config.h"
#include "drake/multibody/plant/multibody_plant_config_functions.h"
#include "drake/multibody/plant/point_pair_contact_info.h"
#include "drake/multibody/plant/propeller.h"
#include "drake/multibody/plant/wing.h"
#include "drake/multibody/tree/spatial_inertia.h"

namespace drake {
namespace pydrake {

using std::string;
using std::string_view;

using geometry::SceneGraph;
using math::RigidTransform;
using systems::Context;
using systems::State;

namespace {

template <typename T>
int GetVariableSize(const multibody::MultibodyPlant<T>& plant,
    multibody::JacobianWrtVariable wrt) {
  switch (wrt) {
    case multibody::JacobianWrtVariable::kQDot:
      return plant.num_positions();
    case multibody::JacobianWrtVariable::kV:
      return plant.num_velocities();
  }
  DRAKE_UNREACHABLE();
}

/**
 * Adds Python bindings for its contents to module `m`, for template `T`.
 * @param m Module.
 * @param T Template.
 */
template <typename T>
void DoScalarDependentDefinitions(py::module m, T) {
  py::tuple param = GetPyParam<T>();
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::multibody;
  constexpr auto& doc = pydrake_doc.drake.multibody;
  // PointPairContactInfo
  {
    using Class = PointPairContactInfo<T>;
    constexpr auto& cls_doc = doc.PointPairContactInfo;
    auto cls = DefineTemplateClassWithDefault<Class>(
        m, "PointPairContactInfo", param, cls_doc.doc);
    cls  // BR
        .def(py::init<BodyIndex, BodyIndex, const Vector3<T>, const Vector3<T>,
                 const T&, const T&,
                 const geometry::PenetrationAsPointPair<T>>(),
            py::arg("bodyA_index"), py::arg("bodyB_index"), py::arg("f_Bc_W"),
            py::arg("p_WC"), py::arg("separation_speed"), py::arg("slip_speed"),
            py::arg("point_pair"), cls_doc.ctor.doc)
        .def("bodyA_index", &Class::bodyA_index, cls_doc.bodyA_index.doc)
        .def("bodyB_index", &Class::bodyB_index, cls_doc.bodyB_index.doc)
        .def("contact_force", &Class::contact_force, cls_doc.contact_force.doc)
        .def("contact_point", &Class::contact_point, cls_doc.contact_point.doc)
        .def("slip_speed", &Class::slip_speed, cls_doc.slip_speed.doc)
        .def("separation_speed", &Class::separation_speed,
            cls_doc.separation_speed.doc)
        .def("point_pair", &Class::point_pair, cls_doc.point_pair.doc);
    DefCopyAndDeepCopy(&cls);
  }

  // HydroelasticContactInfo
  {
    using Class = HydroelasticContactInfo<T>;
    constexpr auto& cls_doc = doc.HydroelasticContactInfo;
    auto cls = DefineTemplateClassWithDefault<Class>(
        m, "HydroelasticContactInfo", param, cls_doc.doc);
    cls  // BR
        .def("contact_surface", &Class::contact_surface,
            cls_doc.contact_surface.doc)
        .def("F_Ac_W", &Class::F_Ac_W, cls_doc.F_Ac_W.doc);
    DefCopyAndDeepCopy(&cls);
    AddValueInstantiation<Class>(m);
  }

  // ContactResults
  {
    using Class = ContactResults<T>;
    constexpr auto& cls_doc = doc.ContactResults;
    auto cls = DefineTemplateClassWithDefault<Class>(
        m, "ContactResults", param, cls_doc.doc);
    cls  // BR
        .def(py::init<>(), cls_doc.ctor.doc)
        .def("num_point_pair_contacts", &Class::num_point_pair_contacts,
            cls_doc.num_point_pair_contacts.doc)
        .def("point_pair_contact_info", &Class::point_pair_contact_info,
            py::arg("i"), cls_doc.point_pair_contact_info.doc)
        .def("num_hydroelastic_contacts", &Class::num_hydroelastic_contacts,
            cls_doc.num_hydroelastic_contacts.doc)
        .def("hydroelastic_contact_info", &Class::hydroelastic_contact_info,
            py::arg("i"), cls_doc.hydroelastic_contact_info.doc)
        .def("plant", &Class::plant, py_rvp::reference, cls_doc.plant.doc)
        .def("SelectHydroelastic", &Class::SelectHydroelastic,
            py::arg("selector"), cls_doc.SelectHydroelastic.doc);
    DefCopyAndDeepCopy(&cls);
    AddValueInstantiation<Class>(m);
  }

  // CoulombFriction
  {
    using Class = CoulombFriction<T>;
    constexpr auto& cls_doc = doc.CoulombFriction;
    auto cls = DefineTemplateClassWithDefault<Class>(
        m, "CoulombFriction", param, cls_doc.doc);
    cls  // BR
        .def(py::init<>(), cls_doc.ctor.doc_0args)
        .def(py::init<const T&, const T&>(), py::arg("static_friction"),
            py::arg("dynamic_friction"), cls_doc.ctor.doc_2args)
        .def("static_friction", &Class::static_friction,
            cls_doc.static_friction.doc)
        .def("dynamic_friction", &Class::dynamic_friction,
            cls_doc.dynamic_friction.doc)
        .def(py::pickle(
            [](const Class& self) {
              return std::pair(self.static_friction(), self.dynamic_friction());
            },
            [](std::pair<T, T> frictions) {
              return Class(frictions.first, frictions.second);
            }));
    DefCopyAndDeepCopy(&cls);

    AddValueInstantiation<CoulombFriction<T>>(m);

    m.def(
        "CalcContactFrictionFromSurfaceProperties",
        [](const CoulombFriction<T>& surface_properties1,
            const CoulombFriction<T>& surface_properties2) {
          return CalcContactFrictionFromSurfaceProperties(
              surface_properties1, surface_properties2);
        },
        py::arg("surface_properties1"), py::arg("surface_properties2"),
        py_rvp::reference, doc.CalcContactFrictionFromSurfaceProperties.doc);
  }

  {
    using Class = MultibodyPlant<T>;
    constexpr auto& cls_doc = doc.MultibodyPlant;
    auto cls = DefineTemplateClassWithDefault<Class, systems::LeafSystem<T>>(
        m, "MultibodyPlant", param, cls_doc.doc);
    // N.B. These are defined as they appear in the class declaration.
    // Forwarded methods from `MultibodyTree`.
    cls  // BR
        .def(py::init<double>(), py::arg("time_step"), cls_doc.ctor.doc)
        .def("time_step", &Class::time_step, cls_doc.time_step.doc)
        .def("num_bodies", &Class::num_bodies, cls_doc.num_bodies.doc)
        .def("num_joints", &Class::num_joints, cls_doc.num_joints.doc)
        .def("num_actuators",
            overload_cast_explicit<int>(&Class::num_actuators),
            cls_doc.num_actuators.doc_0args)
        .def("num_actuators",
            overload_cast_explicit<int, ModelInstanceIndex>(
                &Class::num_actuators),
            py::arg("model_instance"), cls_doc.num_actuators.doc_1args)
        .def("num_force_elements", &Class::num_force_elements,
            cls_doc.num_force_elements.doc)
        .def("num_constraints", &Class::num_constraints,
            cls_doc.num_constraints.doc)
        .def("num_model_instances", &Class::num_model_instances,
            cls_doc.num_model_instances.doc)
        .def("num_positions",
            overload_cast_explicit<int>(&Class::num_positions),
            cls_doc.num_positions.doc_0args)
        .def("num_positions",
            overload_cast_explicit<int, ModelInstanceIndex>(
                &Class::num_positions),
            py::arg("model_instance"), cls_doc.num_positions.doc_1args)
        .def("num_velocities",
            overload_cast_explicit<int>(&Class::num_velocities),
            cls_doc.num_velocities.doc_0args)
        .def("num_velocities",
            overload_cast_explicit<int, ModelInstanceIndex>(
                &Class::num_velocities),
            py::arg("model_instance"), cls_doc.num_velocities.doc_1args)
        .def("num_multibody_states",
            overload_cast_explicit<int>(&Class::num_multibody_states),
            cls_doc.num_multibody_states.doc_0args)
        .def("num_multibody_states",
            overload_cast_explicit<int, ModelInstanceIndex>(
                &Class::num_multibody_states),
            py::arg("model_instance"), cls_doc.num_multibody_states.doc_1args)
        .def("num_actuated_dofs",
            overload_cast_explicit<int>(&Class::num_actuated_dofs),
            cls_doc.num_actuated_dofs.doc_0args)
        .def("num_actuated_dofs",
            overload_cast_explicit<int, ModelInstanceIndex>(
                &Class::num_actuated_dofs),
            py::arg("model_instance"), cls_doc.num_actuated_dofs.doc_1args);
    // Construction.
    cls  // BR
        .def(
            "AddJoint",
            [](Class * self, std::unique_ptr<Joint<T>> joint) -> auto& {
              return self->AddJoint(std::move(joint));
            },
            py::arg("joint"), py_rvp::reference_internal,
            cls_doc.AddJoint.doc_1args)
        .def("AddJointActuator", &Class::AddJointActuator,
            py_rvp::reference_internal, py::arg("name"), py::arg("joint"),
            py::arg("effort_limit") = std::numeric_limits<double>::infinity(),
            cls_doc.AddJointActuator.doc)
        .def(
            "AddFrame",
            [](Class * self, std::unique_ptr<Frame<T>> frame) -> auto& {
              return self->AddFrame(std::move(frame));
            },
            py_rvp::reference_internal, py::arg("frame"), cls_doc.AddFrame.doc)
        .def("AddModelInstance", &Class::AddModelInstance, py::arg("name"),
            cls_doc.AddModelInstance.doc)
        .def("RenameModelInstance", &Class::RenameModelInstance,
            py::arg("model_instance"), py::arg("name"),
            cls_doc.RenameModelInstance.doc)
        .def(
            "AddRigidBody",
            [](Class * self, const std::string& name,
                const SpatialInertia<double>& s) -> auto& {
              return self->AddRigidBody(name, s);
            },
            py::arg("name"), py::arg("M_BBo_B"), py_rvp::reference_internal,
            cls_doc.AddRigidBody.doc_2args)
        .def("AddRigidBody",
            py::overload_cast<const std::string&, ModelInstanceIndex,
                const SpatialInertia<double>&>(&Class::AddRigidBody),
            py::arg("name"), py::arg("model_instance"), py::arg("M_BBo_B"),
            py_rvp::reference_internal, cls_doc.AddRigidBody.doc_3args)
        .def("WeldFrames", &Class::WeldFrames, py::arg("frame_on_parent_F"),
            py::arg("frame_on_child_M"),
            py::arg("X_FM") = RigidTransform<double>::Identity(),
            py_rvp::reference_internal, cls_doc.WeldFrames.doc)
        .def(
            "AddForceElement",
            [](Class * self,
                std::unique_ptr<ForceElement<T>> force_element) -> auto& {
              return self->template AddForceElement<ForceElement>(
                  std::move(force_element));
            },
            py::arg("force_element"), py_rvp::reference_internal,
            cls_doc.AddForceElement.doc)
        .def("SetConstraintActiveStatus", &Class::SetConstraintActiveStatus,
            py::arg("context"), py::arg("id"), py::arg("status"),
            cls_doc.SetConstraintActiveStatus.doc)
        .def("GetConstraintActiveStatus",
            overload_cast_explicit<bool, const Context<T>&,
                MultibodyConstraintId>(&Class::GetConstraintActiveStatus),
            py::arg("context"), py::arg("id"),
            cls_doc.GetConstraintActiveStatus.doc)
        .def("AddCouplerConstraint", &Class::AddCouplerConstraint,
            py::arg("joint0"), py::arg("joint1"), py::arg("gear_ratio"),
            py::arg("offset") = 0.0, py_rvp::reference_internal,
            cls_doc.AddCouplerConstraint.doc)
        .def("AddDistanceConstraint", &Class::AddDistanceConstraint,
            py::arg("body_A"), py::arg("p_AP"), py::arg("body_B"),
            py::arg("p_BQ"), py::arg("distance"),
            py::arg("stiffness") = std::numeric_limits<double>::infinity(),
            py::arg("damping") = 0.0, py_rvp::reference_internal,
            cls_doc.AddDistanceConstraint.doc)
        .def("AddBallConstraint", &Class::AddBallConstraint, py::arg("body_A"),
            py::arg("p_AP"), py::arg("body_B"), py::arg("p_BQ"),
            py_rvp::reference_internal, cls_doc.AddBallConstraint.doc)
        .def("AddWeldConstraint", &Class::AddWeldConstraint, py::arg("body_A"),
            py::arg("X_AP"), py::arg("body_B"), py::arg("X_BQ"),
            py_rvp::reference_internal, cls_doc.AddWeldConstraint.doc)
        .def("RemoveConstraint", &Class::RemoveConstraint, py::arg("id"),
            cls_doc.RemoveConstraint.doc);
    // Mathy bits
    cls  // BR
        .def(
            "CalcPointsPositions",
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
            py::arg("frame_A"), cls_doc.CalcPointsPositions.doc);
    cls  // BR
        .def("CalcTotalMass",
            overload_cast_explicit<T, const Context<T>&>(&Class::CalcTotalMass),
            py::arg("context"), cls_doc.CalcTotalMass.doc_1args)
        .def("CalcTotalMass",
            overload_cast_explicit<T, const Context<T>&,
                const std::vector<ModelInstanceIndex>&>(&Class::CalcTotalMass),
            py::arg("context"), py::arg("model_instances"),
            cls_doc.CalcTotalMass.doc_2args)
        .def("CalcCenterOfMassPositionInWorld",
            overload_cast_explicit<Vector3<T>, const Context<T>&>(
                &Class::CalcCenterOfMassPositionInWorld),
            py::arg("context"),
            cls_doc.CalcCenterOfMassPositionInWorld.doc_1args)
        .def("CalcCenterOfMassPositionInWorld",
            overload_cast_explicit<Vector3<T>, const Context<T>&,
                const std::vector<ModelInstanceIndex>&>(
                &Class::CalcCenterOfMassPositionInWorld),
            py::arg("context"), py::arg("model_instances"),
            cls_doc.CalcCenterOfMassPositionInWorld.doc_2args)
        .def(
            "CalcSpatialInertia",
            [](const Class* self, const Context<T>& context,
                const Frame<T>& frame_F,
                const std::vector<BodyIndex>& body_indexes) {
              return self->CalcSpatialInertia(context, frame_F, body_indexes);
            },
            py::arg("context"), py::arg("frame_F"), py::arg("body_indexes"),
            cls_doc.CalcSpatialInertia.doc)
        .def(
            "CalcSpatialMomentumInWorldAboutPoint",
            [](const Class* self, const Context<T>& context,
                const Vector3<T>& p_WoP_W) {
              return self->CalcSpatialMomentumInWorldAboutPoint(
                  context, p_WoP_W);
            },
            py::arg("context"), py::arg("p_WoP_W"),
            cls_doc.CalcSpatialMomentumInWorldAboutPoint.doc_2args)
        .def(
            "CalcSpatialMomentumInWorldAboutPoint",
            [](const Class* self, const Context<T>& context,
                const std::vector<ModelInstanceIndex>& model_instances,
                const Vector3<T>& p_WoP_W) {
              return self->CalcSpatialMomentumInWorldAboutPoint(
                  context, model_instances, p_WoP_W);
            },
            py::arg("context"), py::arg("model_instances"), py::arg("p_WoP_W"),
            cls_doc.CalcSpatialMomentumInWorldAboutPoint.doc_3args)
        .def(
            "CalcBiasCenterOfMassTranslationalAcceleration",
            [](const Class* self, const Context<T>& context,
                JacobianWrtVariable with_respect_to, const Frame<T>& frame_A,
                const Frame<T>& frame_E) {
              return self->CalcBiasCenterOfMassTranslationalAcceleration(
                  context, with_respect_to, frame_A, frame_E);
            },
            py::arg("context"), py::arg("with_respect_to"), py::arg("frame_A"),
            py::arg("frame_E"),
            cls_doc.CalcBiasCenterOfMassTranslationalAcceleration.doc)
        .def(
            "CalcJacobianCenterOfMassTranslationalVelocity",
            [](const Class* self, const Context<T>& context,
                JacobianWrtVariable with_respect_to, const Frame<T>& frame_A,
                const Frame<T>& frame_E) {
              Matrix3X<T> Js_v_ACcm_E(
                  3, GetVariableSize<T>(*self, with_respect_to));
              self->CalcJacobianCenterOfMassTranslationalVelocity(
                  context, with_respect_to, frame_A, frame_E, &Js_v_ACcm_E);
              return Js_v_ACcm_E;
            },
            py::arg("context"), py::arg("with_respect_to"), py::arg("frame_A"),
            py::arg("frame_E"),
            cls_doc.CalcJacobianCenterOfMassTranslationalVelocity.doc_5args)
        .def(
            "CalcJacobianCenterOfMassTranslationalVelocity",
            [](const Class* self, const Context<T>& context,
                const std::vector<ModelInstanceIndex>& model_instances,
                JacobianWrtVariable with_respect_to, const Frame<T>& frame_A,
                const Frame<T>& frame_E) {
              Matrix3X<T> Js_v_ACcm_E(
                  3, GetVariableSize<T>(*self, with_respect_to));
              self->CalcJacobianCenterOfMassTranslationalVelocity(context,
                  model_instances, with_respect_to, frame_A, frame_E,
                  &Js_v_ACcm_E);
              return Js_v_ACcm_E;
            },
            py::arg("context"), py::arg("model_instances"),
            py::arg("with_respect_to"), py::arg("frame_A"), py::arg("frame_E"),
            cls_doc.CalcJacobianCenterOfMassTranslationalVelocity.doc_6args)
        .def("GetFreeBodyPose", &Class::GetFreeBodyPose, py::arg("context"),
            py::arg("body"), cls_doc.GetFreeBodyPose.doc)
        .def("SetFreeBodyPose",
            overload_cast_explicit<void, Context<T>*, const RigidBody<T>&,
                const RigidTransform<T>&>(&Class::SetFreeBodyPose),
            py::arg("context"), py::arg("body"), py::arg("X_WB"),
            cls_doc.SetFreeBodyPose.doc_3args)
        .def("SetDefaultFreeBodyPose", &Class::SetDefaultFreeBodyPose,
            py::arg("body"), py::arg("X_WB"),
            cls_doc.SetDefaultFreeBodyPose.doc)
        .def("GetDefaultFreeBodyPose", &Class::GetDefaultFreeBodyPose,
            py::arg("body"), cls_doc.GetDefaultFreeBodyPose.doc)
        .def("SetActuationInArray", &Class::SetActuationInArray,
            py::arg("model_instance"), py::arg("u_instance"), py::arg("u"),
            cls_doc.SetActuationInArray.doc)
        .def("GetPositionsFromArray",
            overload_cast_explicit<VectorX<T>, ModelInstanceIndex,
                const Eigen::Ref<const VectorX<T>>&>(
                &Class::GetPositionsFromArray),
            py::arg("model_instance"), py::arg("q"),
            cls_doc.GetPositionsFromArray.doc_2args)
        .def("SetPositionsInArray", &Class::SetPositionsInArray,
            py::arg("model_instance"), py::arg("q_instance"), py::arg("q"),
            cls_doc.SetPositionsInArray.doc)
        .def("GetVelocitiesFromArray",
            overload_cast_explicit<VectorX<T>, ModelInstanceIndex,
                const Eigen::Ref<const VectorX<T>>&>(
                &Class::GetVelocitiesFromArray),
            py::arg("model_instance"), py::arg("v"),
            cls_doc.GetVelocitiesFromArray.doc_2args)
        .def("SetVelocitiesInArray", &Class::SetVelocitiesInArray,
            py::arg("model_instance"), py::arg("v_instance"), py::arg("v"),
            cls_doc.SetVelocitiesInArray.doc)
        // TODO(eric.cousineau): Ensure all of these return either references,
        // or copies, consistently. At present, `GetX(context)` returns a
        // reference, while `GetX(context, model_instance)` returns a copy.
        .def(
            "GetPositions",
            [](const Class* self, const Context<T>& context) {
              // Reference.
              return CopyIfNotPodType(self->GetPositions(context));
            },
            py::arg("context"), return_value_policy_for_scalar_type<T>(),
            // Keep alive, ownership: `return` keeps `context` alive.
            py::keep_alive<0, 2>(), cls_doc.GetPositions.doc_1args)
        .def(
            "GetPositions",
            [](const Class* self, const Context<T>& context,
                ModelInstanceIndex model_instance) {
              // Copy.
              return self->GetPositions(context, model_instance);
            },
            py::arg("context"), py::arg("model_instance"),
            cls_doc.GetPositions.doc_2args)
        .def(
            "GetVelocities",
            [](const Class* self, const Context<T>& context) {
              // Reference.
              return CopyIfNotPodType(self->GetVelocities(context));
            },
            py::arg("context"), return_value_policy_for_scalar_type<T>(),
            // Keep alive, ownership: `return` keeps `context` alive.
            py::keep_alive<0, 2>(), cls_doc.GetVelocities.doc_1args)
        .def(
            "GetVelocities",
            [](const Class* self, const Context<T>& context,
                ModelInstanceIndex model_instance) {
              // Copy.
              return self->GetVelocities(context, model_instance);
            },
            py::arg("context"), py::arg("model_instance"),
            cls_doc.GetVelocities.doc_2args)
        .def(
            "SetFreeBodySpatialVelocity",
            [](const Class* self, const RigidBody<T>& body,
                const SpatialVelocity<T>& V_WB, Context<T>* context) {
              self->SetFreeBodySpatialVelocity(context, body, V_WB);
            },
            py::arg("body"), py::arg("V_WB"), py::arg("context"),
            cls_doc.SetFreeBodySpatialVelocity.doc_3args)
        .def("HasUniqueFreeBaseBody", &Class::HasUniqueFreeBaseBody,
            py::arg("model_instance"), cls_doc.HasUniqueFreeBaseBody.doc)
        .def("GetUniqueFreeBaseBodyOrThrow",
            &Class::GetUniqueFreeBaseBodyOrThrow, py::arg("model_instance"),
            py_rvp::reference_internal,
            cls_doc.GetUniqueFreeBaseBodyOrThrow.doc)
        .def(
            "EvalBodyPoseInWorld",
            [](const Class* self, const Context<T>& context,
                const RigidBody<T>& body_B) {
              return self->EvalBodyPoseInWorld(context, body_B);
            },
            py::arg("context"), py::arg("body"),
            cls_doc.EvalBodyPoseInWorld.doc)
        .def(
            "EvalBodySpatialVelocityInWorld",
            [](const Class* self, const Context<T>& context,
                const RigidBody<T>& body_B) {
              return self->EvalBodySpatialVelocityInWorld(context, body_B);
            },
            py::arg("context"), py::arg("body"),
            cls_doc.EvalBodySpatialVelocityInWorld.doc);
    auto CalcJacobianSpatialVelocity =
        [](const Class* self, const systems::Context<T>& context,
            JacobianWrtVariable with_respect_to, const Frame<T>& frame_B,
            const Eigen::Ref<const Vector3<T>>& p_BoBp_B,
            const Frame<T>& frame_A, const Frame<T>& frame_E) {
          MatrixX<T> Js_V_ABp_E(6, GetVariableSize<T>(*self, with_respect_to));
          self->CalcJacobianSpatialVelocity(context, with_respect_to, frame_B,
              p_BoBp_B, frame_A, frame_E, &Js_V_ABp_E);
          return Js_V_ABp_E;
        };
    cls  // BR
        .def("CalcJacobianSpatialVelocity", CalcJacobianSpatialVelocity,
            py::arg("context"), py::arg("with_respect_to"), py::arg("frame_B"),
            py::arg("p_BoBp_B"), py::arg("frame_A"), py::arg("frame_E"),
            cls_doc.CalcJacobianSpatialVelocity.doc)
        .def(
            "CalcJacobianAngularVelocity",
            [](const Class* self, const Context<T>& context,
                JacobianWrtVariable with_respect_to, const Frame<T>& frame_B,
                const Frame<T>& frame_A, const Frame<T>& frame_E) {
              Matrix3X<T> Js_w_AB_E(
                  3, GetVariableSize<T>(*self, with_respect_to));
              self->CalcJacobianAngularVelocity(context, with_respect_to,
                  frame_B, frame_A, frame_E, &Js_w_AB_E);
              return Js_w_AB_E;
            },
            py::arg("context"), py::arg("with_respect_to"), py::arg("frame_B"),
            py::arg("frame_A"), py::arg("frame_E"),
            cls_doc.CalcJacobianAngularVelocity.doc)
        .def(
            "CalcJacobianTranslationalVelocity",
            [](const Class* self, const Context<T>& context,
                JacobianWrtVariable with_respect_to, const Frame<T>& frame_B,
                const Eigen::Ref<const Matrix3X<T>>& p_BoBi_B,
                const Frame<T>& frame_A, const Frame<T>& frame_E) {
              const int num_points = p_BoBi_B.cols();
              MatrixX<T> Js_v_ABi_E(
                  3 * num_points, GetVariableSize<T>(*self, with_respect_to));
              self->CalcJacobianTranslationalVelocity(context, with_respect_to,
                  frame_B, p_BoBi_B, frame_A, frame_E, &Js_v_ABi_E);
              return Js_v_ABi_E;
            },
            py::arg("context"), py::arg("with_respect_to"), py::arg("frame_B"),
            py::arg("p_BoBi_B"), py::arg("frame_A"), py::arg("frame_E"),
            cls_doc.CalcJacobianTranslationalVelocity.doc)
        .def(
            "CalcJacobianPositionVector",
            [](const Class* self, const Context<T>& context,
                const Frame<T>& frame_B,
                const Eigen::Ref<const Matrix3X<T>>& p_BoBi_B,
                const Frame<T>& frame_A, const Frame<T>& frame_E) {
              const int num_points = p_BoBi_B.cols();
              MatrixX<T> Jq_p_AoBi_E(3 * num_points, self->num_positions());
              self->CalcJacobianPositionVector(
                  context, frame_B, p_BoBi_B, frame_A, frame_E, &Jq_p_AoBi_E);
              return Jq_p_AoBi_E;
            },
            py::arg("context"), py::arg("frame_B"), py::arg("p_BoBi_B"),
            py::arg("frame_A"), py::arg("frame_E"),
            cls_doc.CalcJacobianPositionVector.doc)
        .def(
            "CalcSpatialAccelerationsFromVdot",
            [](const Class* self, const Context<T>& context,
                const Eigen::Ref<const VectorX<T>>& known_vdot) {
              std::vector<SpatialAcceleration<T>> A_WB_array(
                  self->num_bodies());
              self->CalcSpatialAccelerationsFromVdot(
                  context, known_vdot, &A_WB_array);
              return A_WB_array;
            },
            py::arg("context"), py::arg("known_vdot"),
            cls_doc.CalcSpatialAccelerationsFromVdot.doc)
        .def("CalcInverseDynamics", &Class::CalcInverseDynamics,
            py::arg("context"), py::arg("known_vdot"),
            py::arg("external_forces"), cls_doc.CalcInverseDynamics.doc)
        .def("CalcForceElementsContribution",
            &Class::CalcForceElementsContribution, py::arg("context"),
            py::arg("forces"), cls_doc.CalcForceElementsContribution.doc)
        .def("GetPositionLowerLimits", &Class::GetPositionLowerLimits,
            cls_doc.GetPositionLowerLimits.doc)
        .def("GetPositionUpperLimits", &Class::GetPositionUpperLimits,
            cls_doc.GetPositionUpperLimits.doc)
        .def("GetVelocityLowerLimits", &Class::GetVelocityLowerLimits,
            cls_doc.GetVelocityLowerLimits.doc)
        .def("GetVelocityUpperLimits", &Class::GetVelocityUpperLimits,
            cls_doc.GetVelocityUpperLimits.doc)
        .def("GetAccelerationLowerLimits", &Class::GetAccelerationLowerLimits,
            cls_doc.GetAccelerationLowerLimits.doc)
        .def("GetAccelerationUpperLimits", &Class::GetAccelerationUpperLimits,
            cls_doc.GetAccelerationUpperLimits.doc)
        .def("GetEffortLowerLimits", &Class::GetEffortLowerLimits,
            cls_doc.GetEffortLowerLimits.doc)
        .def("GetEffortUpperLimits", &Class::GetEffortUpperLimits,
            cls_doc.GetEffortUpperLimits.doc)
        .def(
            "CalcMassMatrixViaInverseDynamics",
            [](const Class* self, const Context<T>& context) {
              MatrixX<T> H;
              const int n = self->num_velocities();
              H.resize(n, n);
              self->CalcMassMatrixViaInverseDynamics(context, &H);
              return H;
            },
            py::arg("context"), cls_doc.CalcMassMatrixViaInverseDynamics.doc)
        .def(
            "CalcMassMatrix",
            [](const Class* self, const Context<T>& context) {
              MatrixX<T> H;
              const int n = self->num_velocities();
              H.resize(n, n);
              self->CalcMassMatrix(context, &H);
              return H;
            },
            py::arg("context"), cls_doc.CalcMassMatrix.doc)
        .def(
            "CalcBiasSpatialAcceleration",
            [](const Class* self, const systems::Context<T>& context,
                JacobianWrtVariable with_respect_to, const Frame<T>& frame_B,
                const Eigen::Ref<const Vector3<T>>& p_BoBp_B,
                const Frame<T>& frame_A, const Frame<T>& frame_E) {
              return self->CalcBiasSpatialAcceleration(context, with_respect_to,
                  frame_B, p_BoBp_B, frame_A, frame_E);
            },
            py::arg("context"), py::arg("with_respect_to"), py::arg("frame_B"),
            py::arg("p_BoBp_B"), py::arg("frame_A"), py::arg("frame_E"),
            cls_doc.CalcBiasSpatialAcceleration.doc)
        .def(
            "CalcBiasTranslationalAcceleration",
            [](const Class* self, const Context<T>& context,
                JacobianWrtVariable with_respect_to, const Frame<T>& frame_B,
                const Eigen::Ref<const Matrix3X<T>>& p_BoBi_B,
                const Frame<T>& frame_A, const Frame<T>& frame_E) {
              return self->CalcBiasTranslationalAcceleration(context,
                  with_respect_to, frame_B, p_BoBi_B, frame_A, frame_E);
            },
            py::arg("context"), py::arg("with_respect_to"), py::arg("frame_B"),
            py::arg("p_BoBi_B"), py::arg("frame_A"), py::arg("frame_E"),
            cls_doc.CalcBiasTranslationalAcceleration.doc)
        .def(
            "CalcBiasTerm",
            [](const Class* self, const Context<T>& context) {
              VectorX<T> Cv(self->num_velocities());
              self->CalcBiasTerm(context, &Cv);
              return Cv;
            },
            py::arg("context"), cls_doc.CalcBiasTerm.doc)
        .def("CalcGravityGeneralizedForces",
            &Class::CalcGravityGeneralizedForces, py::arg("context"),
            cls_doc.CalcGravityGeneralizedForces.doc)
        .def(
            "CalcGeneralizedForces",
            [](const Class* self, const Context<T>& context,
                const MultibodyForces<T>& forces) {
              VectorX<T> tau(self->num_velocities());
              self->CalcGeneralizedForces(context, forces, &tau);
              return tau;
            },
            py::arg("context"), py::arg("forces"),
            cls_doc.CalcGeneralizedForces.doc)
        .def("MakeActuationMatrix", &Class::MakeActuationMatrix,
            cls_doc.MakeActuationMatrix.doc)
        .def("MakeActuationMatrixPseudoinverse",
            &Class::MakeActuationMatrixPseudoinverse,
            cls_doc.MakeActuationMatrixPseudoinverse.doc)
        .def(
            "MakeActuatorSelectorMatrix",
            [](const Class* self, const std::vector<JointActuatorIndex>&
                                      user_to_actuator_index_map) {
              return self->MakeActuatorSelectorMatrix(
                  user_to_actuator_index_map);
            },
            py::arg("user_to_actuator_index_map"),
            cls_doc.MakeActuatorSelectorMatrix
                .doc_1args_user_to_actuator_index_map)
        .def(
            "MakeActuatorSelectorMatrix",
            [](const Class* self,
                const std::vector<JointIndex>& user_to_joint_index_map) {
              return self->MakeActuatorSelectorMatrix(user_to_joint_index_map);
            },
            py::arg("user_to_joint_index_map"),
            cls_doc.MakeActuatorSelectorMatrix
                .doc_1args_user_to_joint_index_map)
        .def("MakeStateSelectorMatrix", &Class::MakeStateSelectorMatrix,
            py::arg("user_to_joint_index_map"),
            cls_doc.MakeStateSelectorMatrix.doc)
        .def("IsVelocityEqualToQDot", &Class::IsVelocityEqualToQDot,
            cls_doc.IsVelocityEqualToQDot.doc)
        .def(
            "MapVelocityToQDot",
            [](const Class* self, const Context<T>& context,
                const Eigen::Ref<const VectorX<T>>& v) {
              VectorX<T> qdot(self->num_positions());
              self->MapVelocityToQDot(context, v, &qdot);
              return qdot;
            },
            py::arg("context"), py::arg("v"), cls_doc.MapVelocityToQDot.doc)
        .def(
            "MapQDotToVelocity",
            [](const Class* self, const Context<T>& context,
                const Eigen::Ref<const VectorX<T>>& qdot) {
              VectorX<T> v(self->num_velocities());
              self->MapQDotToVelocity(context, qdot, &v);
              return v;
            },
            py::arg("context"), py::arg("qdot"), cls_doc.MapQDotToVelocity.doc)
        .def("CalcRelativeTransform", &Class::CalcRelativeTransform,
            py::arg("context"), py::arg("frame_A"), py::arg("frame_B"),
            cls_doc.CalcRelativeTransform.doc);
    if constexpr (std::is_same_v<T, double>) {
      cls  // BR
          .def("MakeVelocityToQDotMap", &Class::MakeVelocityToQDotMap,
              py::arg("context"), cls_doc.MakeVelocityToQDotMap.doc)
          .def("MakeQDotToVelocityMap", &Class::MakeQDotToVelocityMap,
              py::arg("context"), cls_doc.MakeQDotToVelocityMap.doc);
    }
    // Topology queries.
    cls  // BR
        .def("num_frames", &Class::num_frames, cls_doc.num_frames.doc)
        .def("get_body", &Class::get_body, py::arg("body_index"),
            py_rvp::reference_internal, cls_doc.get_body.doc)
        .def("get_joint", &Class::get_joint, py::arg("joint_index"),
            py_rvp::reference_internal, cls_doc.get_joint.doc)
        .def("get_mutable_joint", &Class::get_mutable_joint,
            py::arg("joint_index"), py_rvp::reference_internal,
            cls_doc.get_mutable_joint.doc)
        .def("get_joint_actuator", &Class::get_joint_actuator,
            py::arg("actuator_index"), py_rvp::reference_internal,
            cls_doc.get_joint_actuator.doc)
        .def("get_mutable_joint_actuator", &Class::get_mutable_joint_actuator,
            py::arg("actuator_index"), py_rvp::reference_internal,
            cls_doc.get_mutable_joint_actuator.doc)
        .def("get_frame", &Class::get_frame, py::arg("frame_index"),
            py_rvp::reference_internal, cls_doc.get_frame.doc)
        .def("gravity_field", &Class::gravity_field, py_rvp::reference_internal,
            cls_doc.gravity_field.doc)
        .def("mutable_gravity_field", &Class::mutable_gravity_field,
            py_rvp::reference_internal, cls_doc.mutable_gravity_field.doc)
        .def("set_gravity_enabled", &Class::set_gravity_enabled,
            py::arg("model_instance"), py::arg("is_enabled"),
            cls_doc.set_gravity_enabled.doc)
        .def("is_gravity_enabled", &Class::is_gravity_enabled,
            py::arg("model_instance"), cls_doc.is_gravity_enabled.doc)
        .def("GetJointIndices", &Class::GetJointIndices,
            py::arg("model_instance"), cls_doc.GetJointIndices.doc)
        .def("GetJointActuatorIndices", &Class::GetJointActuatorIndices,
            py::arg("model_instance"), cls_doc.GetJointActuatorIndices.doc)
        .def("GetActuatedJointIndices", &Class::GetActuatedJointIndices,
            py::arg("model_instance"), cls_doc.GetActuatedJointIndices.doc)
        .def("GetModelInstanceName",
            overload_cast_explicit<const string&, ModelInstanceIndex>(
                &Class::GetModelInstanceName),
            py::arg("model_instance"), py_rvp::reference_internal,
            cls_doc.GetModelInstanceName.doc)
        .def("HasModelInstanceNamed", &Class::HasModelInstanceNamed,
            py::arg("name"), cls_doc.HasModelInstanceNamed.doc)
        .def("HasBodyNamed",
            overload_cast_explicit<bool, string_view>(&Class::HasBodyNamed),
            py::arg("name"), cls_doc.HasBodyNamed.doc_1args)
        .def("HasBodyNamed",
            overload_cast_explicit<bool, string_view, ModelInstanceIndex>(
                &Class::HasBodyNamed),
            py::arg("name"), py::arg("model_instance"),
            cls_doc.HasBodyNamed.doc_2args)
        .def("HasJointNamed",
            overload_cast_explicit<bool, string_view>(&Class::HasJointNamed),
            py::arg("name"), cls_doc.HasJointNamed.doc_1args)
        .def("HasJointNamed",
            overload_cast_explicit<bool, string_view, ModelInstanceIndex>(
                &Class::HasJointNamed),
            py::arg("name"), py::arg("model_instance"),
            cls_doc.HasJointNamed.doc_2args)
        .def("HasJointActuatorNamed",
            overload_cast_explicit<bool, string_view>(
                &Class::HasJointActuatorNamed),
            py::arg("name"), cls_doc.HasJointActuatorNamed.doc_1args)
        .def("HasJointActuatorNamed",
            overload_cast_explicit<bool, string_view, ModelInstanceIndex>(
                &Class::HasJointActuatorNamed),
            py::arg("name"), py::arg("model_instance"),
            cls_doc.HasJointActuatorNamed.doc_2args)
        .def("HasFrameNamed",
            overload_cast_explicit<bool, string_view>(&Class::HasFrameNamed),
            py::arg("name"), cls_doc.HasFrameNamed.doc_1args)
        .def("HasFrameNamed",
            overload_cast_explicit<bool, string_view, ModelInstanceIndex>(
                &Class::HasFrameNamed),
            py::arg("name"), py::arg("model_instance"),
            cls_doc.HasFrameNamed.doc_2args)
        .def("GetFrameByName",
            overload_cast_explicit<const Frame<T>&, string_view>(
                &Class::GetFrameByName),
            py::arg("name"), py_rvp::reference_internal,
            cls_doc.GetFrameByName.doc_1args)
        .def("GetFrameByName",
            overload_cast_explicit<const Frame<T>&, string_view,
                ModelInstanceIndex>(&Class::GetFrameByName),
            py::arg("name"), py::arg("model_instance"),
            py_rvp::reference_internal, cls_doc.GetFrameByName.doc_2args)
        .def("GetFrameIndices", &Class::GetFrameIndices,
            py::arg("model_instance"), cls_doc.GetFrameIndices.doc)
        .def("GetBodyByName",
            overload_cast_explicit<const RigidBody<T>&, string_view>(
                &Class::GetBodyByName),
            py::arg("name"), py_rvp::reference_internal,
            cls_doc.GetBodyByName.doc_1args)
        .def("GetBodyByName",
            overload_cast_explicit<const RigidBody<T>&, string_view,
                ModelInstanceIndex>(&Class::GetBodyByName),
            py::arg("name"), py::arg("model_instance"),
            py_rvp::reference_internal, cls_doc.GetBodyByName.doc_2args)
        .def("GetBodyFrameIdOrThrow", &Class::GetBodyFrameIdOrThrow,
            py::arg("body_index"), cls_doc.GetBodyFrameIdOrThrow.doc)
        .def("GetBodyIndices", &Class::GetBodyIndices,
            py::arg("model_instance"), cls_doc.GetBodyIndices.doc)
        .def("GetRigidBodyByName",
            overload_cast_explicit<const RigidBody<T>&, string_view>(
                &Class::GetRigidBodyByName),
            py::arg("name"), py_rvp::reference_internal,
            cls_doc.GetRigidBodyByName.doc_1args)
        .def("GetRigidBodyByName",
            overload_cast_explicit<const RigidBody<T>&, string_view,
                ModelInstanceIndex>(&Class::GetRigidBodyByName),
            py::arg("name"), py::arg("model_instance"),
            py_rvp::reference_internal, cls_doc.GetRigidBodyByName.doc_2args)
        .def(
            "GetJointByName",
            [](const Class* self, string_view name,
                std::optional<ModelInstanceIndex> model_instance) -> auto& {
              return self->GetJointByName(name, model_instance);
            },
            py::arg("name"), py::arg("model_instance") = std::nullopt,
            py_rvp::reference_internal, cls_doc.GetJointByName.doc)
        .def(
            "GetMutableJointByName",
            [](Class * self, string_view name,
                std::optional<ModelInstanceIndex> model_instance) -> auto& {
              return self->GetMutableJointByName(name, model_instance);
            },
            py::arg("name"), py::arg("model_instance") = std::nullopt,
            py_rvp::reference_internal, cls_doc.GetJointByName.doc)
        .def("GetJointActuatorByName",
            overload_cast_explicit<const JointActuator<T>&, string_view>(
                &Class::GetJointActuatorByName),
            py::arg("name"), py_rvp::reference_internal,
            cls_doc.GetJointActuatorByName.doc_1args)
        .def("GetJointActuatorByName",
            overload_cast_explicit<const JointActuator<T>&, string_view,
                ModelInstanceIndex>(&Class::GetJointActuatorByName),
            py::arg("name"), py::arg("model_instance"),
            py_rvp::reference_internal,
            cls_doc.GetJointActuatorByName.doc_2args)
        .def("GetModelInstanceByName",
            overload_cast_explicit<ModelInstanceIndex, string_view>(
                &Class::GetModelInstanceByName),
            py::arg("name"), py_rvp::reference_internal,
            cls_doc.GetModelInstanceByName.doc)
        .def("GetBodiesWeldedTo", &Class::GetBodiesWeldedTo, py::arg("body"),
            py_rvp::reference_internal, cls_doc.GetBodiesWeldedTo.doc)
        .def("GetBodiesKinematicallyAffectedBy",
            &Class::GetBodiesKinematicallyAffectedBy, py::arg("joint_indexes"),
            cls_doc.GetBodiesKinematicallyAffectedBy.doc)
        .def("GetTopologyGraphvizString", &Class::GetTopologyGraphvizString,
            cls_doc.GetTopologyGraphvizString.doc)
        .def("get_force_element", &Class::get_force_element,
            py::arg("force_element_index"), py_rvp::reference_internal,
            cls_doc.get_force_element.doc);
    // Geometry.
    cls  // BR
        .def("RegisterAsSourceForSceneGraph",
            &Class::RegisterAsSourceForSceneGraph, py::arg("scene_graph"),
            cls_doc.RegisterAsSourceForSceneGraph.doc)
        .def("RegisterVisualGeometry",
            py::overload_cast<const RigidBody<T>&,
                const RigidTransform<double>&, const geometry::Shape&,
                const std::string&, const Vector4<double>&>(
                &Class::RegisterVisualGeometry),
            py::arg("body"), py::arg("X_BG"), py::arg("shape"), py::arg("name"),
            py::arg("diffuse_color"),
            cls_doc.RegisterVisualGeometry
                .doc_5args_body_X_BG_shape_name_diffuse_color)
        .def("RegisterCollisionGeometry",
            py::overload_cast<const RigidBody<T>&,
                const RigidTransform<double>&, const geometry::Shape&,
                const std::string&, geometry::ProximityProperties>(
                &Class::RegisterCollisionGeometry),
            py::arg("body"), py::arg("X_BG"), py::arg("shape"), py::arg("name"),
            py::arg("properties"),
            cls_doc.RegisterCollisionGeometry
                .doc_5args_body_X_BG_shape_name_properties)
        .def("RegisterCollisionGeometry",
            py::overload_cast<const RigidBody<T>&,
                const RigidTransform<double>&, const geometry::Shape&,
                const std::string&, const CoulombFriction<double>&>(
                &Class::RegisterCollisionGeometry),
            py::arg("body"), py::arg("X_BG"), py::arg("shape"), py::arg("name"),
            py::arg("coulomb_friction"),
            cls_doc.RegisterCollisionGeometry
                .doc_5args_body_X_BG_shape_name_coulomb_friction)
        .def("GetFloatingBaseBodies", &Class::GetFloatingBaseBodies,
            cls_doc.GetFloatingBaseBodies.doc)
        .def("get_source_id", &Class::get_source_id, cls_doc.get_source_id.doc)
        .def("get_geometry_query_input_port",
            &Class::get_geometry_query_input_port, py_rvp::reference_internal,
            cls_doc.get_geometry_query_input_port.doc)
        .def("get_geometry_poses_output_port",
            &Class::get_geometry_poses_output_port, py_rvp::reference_internal,
            cls_doc.get_geometry_poses_output_port.doc)
        .def("geometry_source_is_registered",
            &Class::geometry_source_is_registered,
            cls_doc.geometry_source_is_registered.doc)
        .def("GetBodyFromFrameId", &Class::GetBodyFromFrameId,
            py_rvp::reference_internal, cls_doc.GetBodyFromFrameId.doc)
        .def("GetBodyFrameIdIfExists", &Class::GetBodyFrameIdIfExists,
            py::arg("body_index"), py_rvp::reference_internal,
            cls_doc.GetBodyFrameIdIfExists.doc)
        .def("GetCollisionGeometriesForBody",
            &Class::GetCollisionGeometriesForBody, py::arg("body"),
            py_rvp::reference_internal,
            cls_doc.GetCollisionGeometriesForBody.doc)
        .def("num_collision_geometries", &Class::num_collision_geometries,
            cls_doc.num_collision_geometries.doc)
        .def("CollectRegisteredGeometries", &Class::CollectRegisteredGeometries,
            py::arg("bodies"), cls_doc.CollectRegisteredGeometries.doc);
    // Port accessors.
    cls  // BR
        .def("get_actuation_input_port",
            overload_cast_explicit<const systems::InputPort<T>&>(
                &Class::get_actuation_input_port),
            py_rvp::reference_internal,
            cls_doc.get_actuation_input_port.doc_0args)
        .def("get_actuation_input_port",
            overload_cast_explicit<const systems::InputPort<T>&,
                ModelInstanceIndex>(&Class::get_actuation_input_port),
            py::arg("model_instance"), py_rvp::reference_internal,
            cls_doc.get_actuation_input_port.doc_1args)
        .def("get_net_actuation_output_port",
            overload_cast_explicit<const systems::OutputPort<T>&>(
                &Class::get_net_actuation_output_port),
            py_rvp::reference_internal,
            cls_doc.get_net_actuation_output_port.doc_0args)
        .def("get_net_actuation_output_port",
            overload_cast_explicit<const systems::OutputPort<T>&,
                ModelInstanceIndex>(&Class::get_net_actuation_output_port),
            py::arg("model_instance"), py_rvp::reference_internal,
            cls_doc.get_net_actuation_output_port.doc_1args)
        .def("get_desired_state_input_port",
            overload_cast_explicit<const systems::InputPort<T>&,
                multibody::ModelInstanceIndex>(
                &Class::get_desired_state_input_port),
            py::arg("model_instance"), py_rvp::reference_internal,
            cls_doc.get_desired_state_input_port.doc)
        .def("get_applied_generalized_force_input_port",
            overload_cast_explicit<const systems::InputPort<T>&>(
                &Class::get_applied_generalized_force_input_port),
            py_rvp::reference_internal,
            cls_doc.get_applied_generalized_force_input_port.doc)
        .def("get_applied_spatial_force_input_port",
            overload_cast_explicit<const systems::InputPort<T>&>(
                &Class::get_applied_spatial_force_input_port),
            py_rvp::reference_internal,
            cls_doc.get_applied_spatial_force_input_port.doc)
        .def("get_body_poses_output_port",
            overload_cast_explicit<const systems::OutputPort<T>&>(
                &Class::get_body_poses_output_port),
            py_rvp::reference_internal, cls_doc.get_body_poses_output_port.doc)
        .def("get_body_spatial_velocities_output_port",
            overload_cast_explicit<const systems::OutputPort<T>&>(
                &Class::get_body_spatial_velocities_output_port),
            py_rvp::reference_internal,
            cls_doc.get_body_spatial_velocities_output_port.doc)
        .def("get_body_spatial_accelerations_output_port",
            overload_cast_explicit<const systems::OutputPort<T>&>(
                &Class::get_body_spatial_accelerations_output_port),
            py_rvp::reference_internal,
            cls_doc.get_body_spatial_accelerations_output_port.doc)
        .def("get_state_output_port",
            overload_cast_explicit<const systems::OutputPort<T>&>(
                &Class::get_state_output_port),
            py_rvp::reference_internal, cls_doc.get_state_output_port.doc_0args)
        .def("get_state_output_port",
            overload_cast_explicit<const systems::OutputPort<T>&,
                ModelInstanceIndex>(&Class::get_state_output_port),
            py::arg("model_instance"), py_rvp::reference_internal,
            cls_doc.get_state_output_port.doc_1args)
        .def("get_generalized_acceleration_output_port",
            overload_cast_explicit<const systems::OutputPort<T>&>(
                &Class::get_generalized_acceleration_output_port),
            py_rvp::reference_internal,
            cls_doc.get_generalized_acceleration_output_port.doc_0args)
        .def("get_generalized_acceleration_output_port",
            overload_cast_explicit<const systems::OutputPort<T>&,
                ModelInstanceIndex>(
                &Class::get_generalized_acceleration_output_port),
            py::arg("model_instance"), py_rvp::reference_internal,
            cls_doc.get_generalized_acceleration_output_port.doc_1args)
        .def("get_reaction_forces_output_port",
            overload_cast_explicit<const systems::OutputPort<T>&>(
                &Class::get_reaction_forces_output_port),
            py_rvp::reference_internal,
            cls_doc.get_reaction_forces_output_port.doc)
        .def("get_contact_results_output_port",
            overload_cast_explicit<const systems::OutputPort<T>&>(
                &Class::get_contact_results_output_port),
            py_rvp::reference_internal,
            cls_doc.get_contact_results_output_port.doc)
        .def("get_generalized_contact_forces_output_port",
            overload_cast_explicit<const systems::OutputPort<T>&,
                ModelInstanceIndex>(
                &Class::get_generalized_contact_forces_output_port),
            py_rvp::reference_internal, py::arg("model_instance"),
            cls_doc.get_generalized_contact_forces_output_port.doc);
    // Property accessors.
    cls  // BR
        .def("world_body", &Class::world_body, py_rvp::reference_internal,
            cls_doc.world_body.doc)
        .def("world_frame", &Class::world_frame, py_rvp::reference_internal,
            cls_doc.world_frame.doc)
        .def("is_finalized", &Class::is_finalized, cls_doc.is_finalized.doc)
        .def("Finalize", py::overload_cast<>(&Class::Finalize),
            cls_doc.Finalize.doc)
        .def("set_contact_model", &Class::set_contact_model, py::arg("model"),
            cls_doc.set_contact_model.doc)
        .def("get_contact_model", &Class::get_contact_model,
            cls_doc.get_contact_model.doc);
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
    cls  // BR
        .def("set_discrete_contact_solver",
            WrapDeprecated(cls_doc.set_discrete_contact_solver.doc_deprecated,
                &Class::set_discrete_contact_solver),
            py::arg("contact_solver"),
            cls_doc.set_discrete_contact_solver.doc_deprecated);
#pragma GCC diagnostic pop
    cls  // BR
        .def("get_discrete_contact_solver", &Class::get_discrete_contact_solver,
            cls_doc.get_discrete_contact_solver.doc)
        .def("set_discrete_contact_approximation",
            &Class::set_discrete_contact_approximation,
            py::arg("approximation"),
            cls_doc.set_discrete_contact_approximation.doc)
        .def("get_discrete_contact_approximation",
            &Class::get_discrete_contact_approximation,
            cls_doc.get_discrete_contact_approximation.doc)
        .def("set_sap_near_rigid_threshold",
            &Class::set_sap_near_rigid_threshold,
            py::arg("near_rigid_threshold") =
                MultibodyPlantConfig{}.sap_near_rigid_threshold,
            cls_doc.set_sap_near_rigid_threshold.doc)
        .def("get_sap_near_rigid_threshold",
            &Class::get_sap_near_rigid_threshold,
            cls_doc.get_sap_near_rigid_threshold.doc)
        .def_static("GetDefaultContactSurfaceRepresentation",
            &Class::GetDefaultContactSurfaceRepresentation,
            py::arg("time_step"),
            cls_doc.GetDefaultContactSurfaceRepresentation.doc)
        .def("set_contact_surface_representation",
            &Class::set_contact_surface_representation,
            py::arg("surface_representation"),
            cls_doc.set_contact_surface_representation.doc)
        .def("get_contact_surface_representation",
            &Class::get_contact_surface_representation,
            cls_doc.get_contact_surface_representation.doc)
        .def("set_adjacent_bodies_collision_filters",
            &Class::set_adjacent_bodies_collision_filters, py::arg("value"),
            cls_doc.set_adjacent_bodies_collision_filters.doc)
        .def("get_adjacent_bodies_collision_filters",
            &Class::get_adjacent_bodies_collision_filters,
            cls_doc.get_adjacent_bodies_collision_filters.doc)
        .def("AddPhysicalModel", &Class::AddPhysicalModel, py::arg("model"),
            cls_doc.AddPhysicalModel.doc)
        .def("physical_models", &Class::physical_models,
            py_rvp::reference_internal, cls_doc.physical_models.doc)
        .def("set_penetration_allowance", &Class::set_penetration_allowance,
            py::arg("penetration_allowance") = 0.001,
            cls_doc.set_penetration_allowance.doc)
        .def("get_contact_penalty_method_time_scale",
            &Class::get_contact_penalty_method_time_scale,
            cls_doc.get_contact_penalty_method_time_scale.doc)
        .def("set_stiction_tolerance", &Class::set_stiction_tolerance,
            py::arg("v_stiction") = 0.001, cls_doc.set_stiction_tolerance.doc)
        .def(
            "GetPositions",
            [](const MultibodyPlant<T>* self, const Context<T>& context)
                -> VectorX<T> { return self->GetPositions(context); },
            py_rvp::reference, py::arg("context"),
            cls_doc.GetPositions.doc_1args)
        .def(
            "GetPositions",
            [](const MultibodyPlant<T>* self, const Context<T>& context,
                ModelInstanceIndex model_instance) -> VectorX<T> {
              return self->GetPositions(context, model_instance);
            },
            py_rvp::reference, py::arg("context"), py::arg("model_instance"),
            cls_doc.GetPositions.doc_2args)
        .def(
            "SetPositions",
            [](const MultibodyPlant<T>* self, Context<T>* context,
                const Eigen::Ref<const VectorX<T>>& q) {
              self->SetPositions(context, q);
            },
            py::arg("context"), py::arg("q"), cls_doc.SetPositions.doc_2args)
        .def(
            "SetPositions",
            [](const MultibodyPlant<T>* self, Context<T>* context,
                ModelInstanceIndex model_instance,
                const Eigen::Ref<const VectorX<T>>& q) {
              self->SetPositions(context, model_instance, q);
            },
            py::arg("context"), py::arg("model_instance"), py::arg("q"),
            cls_doc.SetPositions.doc_2args)
        .def(
            "GetDefaultPositions",
            [](const MultibodyPlant<T>* self) {
              return self->GetDefaultPositions();
            },
            cls_doc.GetDefaultPositions.doc_0args)
        .def(
            "GetDefaultPositions",
            [](const MultibodyPlant<T>* self,
                ModelInstanceIndex model_instance) {
              return self->GetDefaultPositions(model_instance);
            },
            py::arg("model_instance"), cls_doc.GetDefaultPositions.doc_1args)
        .def(
            "SetDefaultPositions",
            [](MultibodyPlant<T>* self,
                const Eigen::Ref<const VectorX<double>>& q) {
              self->SetDefaultPositions(q);
            },
            py::arg("q"), cls_doc.SetDefaultPositions.doc_1args)
        .def(
            "SetDefaultPositions",
            [](MultibodyPlant<T>* self, ModelInstanceIndex model_instance,
                const Eigen::Ref<const VectorX<double>>& q_instance) {
              self->SetDefaultPositions(model_instance, q_instance);
            },
            py::arg("model_instance"), py::arg("q_instance"),
            cls_doc.SetDefaultPositions.doc_2args)
        .def(
            "GetVelocities",
            [](const MultibodyPlant<T>* self, const Context<T>& context)
                -> VectorX<T> { return self->GetVelocities(context); },
            py_rvp::reference, py::arg("context"),
            cls_doc.GetVelocities.doc_1args)
        .def(
            "GetVelocities",
            [](const MultibodyPlant<T>* self, const Context<T>& context,
                ModelInstanceIndex model_instance) -> VectorX<T> {
              return self->GetVelocities(context, model_instance);
            },
            py_rvp::reference, py::arg("context"), py::arg("model_instance"),
            cls_doc.GetVelocities.doc_2args)
        .def(
            "SetVelocities",
            [](const MultibodyPlant<T>* self, Context<T>* context,
                const Eigen::Ref<const VectorX<T>>& v) {
              self->SetVelocities(context, v);
            },
            py_rvp::reference, py::arg("context"), py::arg("v"),
            cls_doc.SetVelocities.doc_2args)
        .def(
            "SetVelocities",
            [](const MultibodyPlant<T>* self, Context<T>* context,
                ModelInstanceIndex model_instance,
                const Eigen::Ref<const VectorX<T>>& v) {
              self->SetVelocities(context, model_instance, v);
            },
            py_rvp::reference, py::arg("context"), py::arg("model_instance"),
            py::arg("v"), cls_doc.SetVelocities.doc_3args)
        .def(
            "GetPositionsAndVelocities",
            [](const MultibodyPlant<T>* self,
                const Context<T>& context) -> VectorX<T> {
              return self->GetPositionsAndVelocities(context);
            },
            py_rvp::reference, py::arg("context"),
            cls_doc.GetPositionsAndVelocities.doc_1args)
        .def(
            "GetPositionsAndVelocities",
            [](const MultibodyPlant<T>* self, const Context<T>& context,
                ModelInstanceIndex model_instance) -> VectorX<T> {
              return self->GetPositionsAndVelocities(context, model_instance);
            },
            py_rvp::reference, py::arg("context"), py::arg("model_instance"),
            cls_doc.GetPositionsAndVelocities.doc_2args)
        .def(
            "SetPositionsAndVelocities",
            [](const MultibodyPlant<T>* self, Context<T>* context,
                const Eigen::Ref<const VectorX<T>>& q_v) {
              self->SetPositionsAndVelocities(context, q_v);
            },
            py_rvp::reference, py::arg("context"), py::arg("q_v"),
            cls_doc.SetPositionsAndVelocities.doc_2args)
        .def(
            "SetPositionsAndVelocities",
            [](const MultibodyPlant<T>* self, Context<T>* context,
                ModelInstanceIndex model_instance,
                const Eigen::Ref<const VectorX<T>>& q_v) {
              self->SetPositionsAndVelocities(context, model_instance, q_v);
            },
            py_rvp::reference, py::arg("context"), py::arg("model_instance"),
            py::arg("q_v"), cls_doc.SetPositionsAndVelocities.doc_3args)
        .def(
            "SetDefaultState",
            [](const Class* self, const Context<T>& context, State<T>* state) {
              self->SetDefaultState(context, state);
            },
            py::arg("context"), py::arg("state"), cls_doc.SetDefaultState.doc)
        .def("GetPositionNames",
            overload_cast_explicit<std::vector<std::string>, bool, bool>(
                &Class::GetPositionNames),
            py::arg("add_model_instance_prefix") = true,
            py::arg("always_add_suffix") = true,
            cls_doc.GetPositionNames.doc_2args)
        .def("GetPositionNames",
            overload_cast_explicit<std::vector<std::string>, ModelInstanceIndex,
                bool, bool>(&Class::GetPositionNames),
            py::arg("model_instance"),
            py::arg("add_model_instance_prefix") = false,
            py::arg("always_add_suffix") = true,
            cls_doc.GetPositionNames.doc_3args)
        .def("GetVelocityNames",
            overload_cast_explicit<std::vector<std::string>, bool, bool>(
                &Class::GetVelocityNames),
            py::arg("add_model_instance_prefix") = true,
            py::arg("always_add_suffix") = true,
            cls_doc.GetVelocityNames.doc_2args)
        .def("GetVelocityNames",
            overload_cast_explicit<std::vector<std::string>, ModelInstanceIndex,
                bool, bool>(&Class::GetVelocityNames),
            py::arg("model_instance"),
            py::arg("add_model_instance_prefix") = false,
            py::arg("always_add_suffix") = true,
            cls_doc.GetVelocityNames.doc_3args)
        .def("GetStateNames",
            overload_cast_explicit<std::vector<std::string>, bool>(
                &Class::GetStateNames),
            py::arg("add_model_instance_prefix") = true,
            cls_doc.GetStateNames.doc_1args)
        .def("GetStateNames",
            overload_cast_explicit<std::vector<std::string>, ModelInstanceIndex,
                bool>(&Class::GetStateNames),
            py::arg("model_instance"),
            py::arg("add_model_instance_prefix") = false,
            cls_doc.GetStateNames.doc_2args)
        .def("GetActuatorNames",
            overload_cast_explicit<std::vector<std::string>, bool>(
                &Class::GetActuatorNames),
            py::arg("add_model_instance_prefix") = true,
            cls_doc.GetActuatorNames.doc_1args)
        .def("GetActuatorNames",
            overload_cast_explicit<std::vector<std::string>, ModelInstanceIndex,
                bool>(&Class::GetActuatorNames),
            py::arg("model_instance"),
            py::arg("add_model_instance_prefix") = false,
            cls_doc.GetActuatorNames.doc_2args);
  }

  {
    // TODO(eric.cousineau): Figure out why we need to use this to explicit
    // keep-alive vs. annotating the return tuple with `py::keep_alive()`.
    // Most likely due to a bug in our fork of pybind11 for handling of
    // unique_ptr<> arguments and keep_alive<> behavior for objects that are
    // not yet registered with pybind11 (#11046).
    auto cast_workaround = [](auto&& nurse, py::object patient_py) {
      // Cast to ensure we have the object registered.
      py::object nurse_py = py::cast(nurse, py_rvp::reference);
      // Directly leverage pybind11's keep alive mechanism.
      py::detail::keep_alive_impl(nurse_py, patient_py);
      return nurse_py;
    };

    auto result_to_tuple =
        [cast_workaround](systems::DiagramBuilder<T>* builder,
            const AddMultibodyPlantSceneGraphResult<T>& pair) {
          py::object builder_py = py::cast(builder, py_rvp::reference);
          // Keep alive, ownership: `plant` keeps `builder` alive.
          py::object plant_py = cast_workaround(pair.plant, builder_py);
          // Keep alive, ownership: `scene_graph` keeps `builder` alive.
          py::object scene_graph_py =
              cast_workaround(pair.scene_graph, builder_py);
          return py::make_tuple(plant_py, scene_graph_py);
        };

    m.def(
        "AddMultibodyPlantSceneGraph",
        [result_to_tuple](systems::DiagramBuilder<T>* builder,
            std::unique_ptr<MultibodyPlant<T>> plant,
            std::unique_ptr<SceneGraph<T>> scene_graph) {
          auto pair = AddMultibodyPlantSceneGraph<T>(
              builder, std::move(plant), std::move(scene_graph));
          return result_to_tuple(builder, pair);
        },
        py::arg("builder"), py::arg("plant"), py::arg("scene_graph") = nullptr,
        doc.AddMultibodyPlantSceneGraph
            .doc_3args_systemsDiagramBuilder_stduniqueptr_stduniqueptr);

    m.def(
        "AddMultibodyPlantSceneGraph",
        [result_to_tuple](systems::DiagramBuilder<T>* builder, double time_step,
            std::unique_ptr<SceneGraph<T>> scene_graph) {
          auto pair = AddMultibodyPlantSceneGraph<T>(
              builder, time_step, std::move(scene_graph));
          return result_to_tuple(builder, pair);
        },
        py::arg("builder"), py::arg("time_step"),
        py::arg("scene_graph") = nullptr,
        doc.AddMultibodyPlantSceneGraph
            .doc_3args_systemsDiagramBuilder_double_stduniqueptr);

    // In C++ these functions are only defined for double, not AutoDiffXd.
    if constexpr (std::is_same_v<T, double>) {
      m.def(
          "AddMultibodyPlant",
          [result_to_tuple](const MultibodyPlantConfig& config,
              systems::DiagramBuilder<T>* builder) {
            auto pair = AddMultibodyPlant(config, builder);
            return result_to_tuple(builder, pair);
          },
          py::arg("config"), py::arg("builder"), doc.AddMultibodyPlant.doc);
      m.def("ApplyMultibodyPlantConfig", &ApplyMultibodyPlantConfig,
          py::arg("config"), py::arg("plant"),
          doc.ApplyMultibodyPlantConfig.doc);
    }
  }

  // ExternallyAppliedSpatialForce
  {
    using Class = ExternallyAppliedSpatialForce<T>;
    constexpr auto& cls_doc = doc.ExternallyAppliedSpatialForce;
    auto cls = DefineTemplateClassWithDefault<Class>(
        m, "ExternallyAppliedSpatialForce", param, cls_doc.doc);
    cls  // BR
        .def(py::init<>())
        .def_readwrite("body_index", &Class::body_index, cls_doc.body_index.doc)
        .def_readwrite("p_BoBq_B", &Class::p_BoBq_B,
            return_value_policy_for_scalar_type<T>(), cls_doc.p_BoBq_B.doc)
        .def_readwrite("F_Bq_W", &Class::F_Bq_W, cls_doc.F_Bq_W.doc);
    DefCopyAndDeepCopy(&cls);
    AddValueInstantiation<Class>(m);
    // Some ports need `Value<std::vector<Class>>`.
    AddValueInstantiation<std::vector<Class>>(m);
  }

  // ExternallyAppliedSpatialForceMultiplexer
  {
    using Class = ExternallyAppliedSpatialForceMultiplexer<T>;
    constexpr auto& cls_doc = doc.ExternallyAppliedSpatialForceMultiplexer;
    auto cls = DefineTemplateClassWithDefault<Class, systems::LeafSystem<T>>(
        m, "ExternallyAppliedSpatialForceMultiplexer", param, cls_doc.doc);
    cls  // BR
        .def(py::init<int>(), py::arg("num_inputs"), cls_doc.ctor.doc);
  }

  // Propeller
  {
    using Class = Propeller<T>;
    constexpr auto& cls_doc = doc.Propeller;
    auto cls = DefineTemplateClassWithDefault<Class, systems::LeafSystem<T>>(
        m, "Propeller", param, cls_doc.doc);
    cls  // BR
        .def(py::init<const BodyIndex&, const math::RigidTransform<double>&,
                 double, double>(),
            py::arg("body_index"),
            py::arg("X_BP") = math::RigidTransform<double>::Identity(),
            py::arg("thrust_ratio") = 1.0, py::arg("moment_ratio") = 0.0,
            cls_doc.ctor.doc_4args)
        .def(py::init<const std::vector<PropellerInfo>&>(),
            py::arg("propeller_info"), cls_doc.ctor.doc_1args)
        .def("num_propellers", &Class::num_propellers,
            cls_doc.num_propellers.doc)
        .def("get_command_input_port", &Class::get_command_input_port,
            py_rvp::reference_internal, cls_doc.get_command_input_port.doc)
        .def("get_body_poses_input_port", &Class::get_body_poses_input_port,
            py_rvp::reference_internal, cls_doc.get_body_poses_input_port.doc)
        .def("get_spatial_forces_output_port",
            &Class::get_spatial_forces_output_port, py_rvp::reference_internal,
            cls_doc.get_spatial_forces_output_port.doc);
  }

  // Wing
  {
    using Class = Wing<T>;
    constexpr auto& cls_doc = doc.Wing;
    auto cls = DefineTemplateClassWithDefault<Class, systems::LeafSystem<T>>(
        m, "Wing", param, cls_doc.doc);
    cls  // BR
        .def(py::init<const BodyIndex&, double,
                 const math::RigidTransform<double>&, double>(),
            py::arg("body_index"), py::arg("surface_area"),
            py::arg("X_BodyWing") = math::RigidTransform<double>::Identity(),
            py::arg("fluid_density") = Wing<T>::kDefaultFluidDensity,
            cls_doc.ctor.doc)
        .def("get_body_poses_input_port", &Class::get_body_poses_input_port,
            py_rvp::reference_internal, cls_doc.get_body_poses_input_port.doc)
        .def("get_body_spatial_velocities_input_port",
            &Class::get_body_spatial_velocities_input_port,
            py_rvp::reference_internal,
            cls_doc.get_body_spatial_velocities_input_port.doc)
        .def("get_wind_velocity_input_port",
            &Class::get_wind_velocity_input_port, py_rvp::reference_internal,
            cls_doc.get_wind_velocity_input_port.doc)
        .def("get_fluid_density_input_port",
            &Class::get_fluid_density_input_port, py_rvp::reference_internal,
            cls_doc.get_fluid_density_input_port.doc)
        .def("get_spatial_force_output_port",
            &Class::get_spatial_force_output_port, py_rvp::reference_internal,
            cls_doc.get_spatial_force_output_port.doc)
        .def("get_aerodynamic_center_output_port",
            &Class::get_aerodynamic_center_output_port,
            py_rvp::reference_internal,
            cls_doc.get_aerodynamic_center_output_port.doc)
        .def_static("AddToBuilder", &Class::AddToBuilder, py::arg("builder"),
            py::arg("plant"), py::arg("body_index"), py::arg("surface_area"),
            py::arg("X_BodyWing") = math::RigidTransform<double>::Identity(),
            py::arg("fluid_density") = Wing<T>::kDefaultFluidDensity,
            // Keep alive, ownership: `return` keeps `builder` alive.
            py::keep_alive<0, 1>(), py_rvp::reference,
            cls_doc.AddToBuilder.doc);
  }

  // NOLINTNEXTLINE(readability/fn_size)
}
}  // namespace

PYBIND11_MODULE(plant, m) {
  PYDRAKE_PREVENT_PYTHON3_MODULE_REIMPORT(m);
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::multibody;

  m.doc() = "Bindings for MultibodyPlant and related classes.";

  py::module::import("pydrake.geometry");
  py::module::import("pydrake.math");
  py::module::import("pydrake.multibody.math");
  py::module::import("pydrake.multibody.tree");
  py::module::import("pydrake.systems.framework");

  constexpr auto& doc = pydrake_doc.drake.multibody;

  using T = double;

  // ContactResultsToLcmSystem
  {
    using Class = ContactResultsToLcmSystem<T>;
    constexpr auto& cls_doc = doc.ContactResultsToLcmSystem;
    py::class_<Class, systems::LeafSystem<T>>(
        m, "ContactResultsToLcmSystem", cls_doc.doc)
        .def(py::init<const MultibodyPlant<T>&>(), py::arg("plant"),
            cls_doc.ctor.doc)
        .def("get_contact_result_input_port",
            &Class::get_contact_result_input_port, py_rvp::reference_internal,
            cls_doc.get_contact_result_input_port.doc)
        .def("get_lcm_message_output_port", &Class::get_lcm_message_output_port,
            py_rvp::reference_internal,
            cls_doc.get_lcm_message_output_port.doc);
  }

  m.def(
      "ConnectContactResultsToDrakeVisualizer",
      [](systems::DiagramBuilder<double>* builder,
          const MultibodyPlant<double>& plant,
          const geometry::SceneGraph<double>& scene_graph,
          lcm::DrakeLcmInterface* lcm, std::optional<double> publish_period) {
        return ConnectContactResultsToDrakeVisualizer(
            builder, plant, scene_graph, lcm, publish_period);
      },
      py::arg("builder"), py::arg("plant"), py::arg("scene_graph"),
      py::arg("lcm") = nullptr, py::arg("publish_period") = std::nullopt,
      py_rvp::reference,
      // Keep alive, ownership: `return` keeps `builder` alive.
      py::keep_alive<0, 1>(),
      // Keep alive, transitive: `plant` keeps `builder` alive.
      py::keep_alive<2, 1>(),
      // Keep alive, transitive: `scene_graph` keeps `builder` alive.
      py::keep_alive<3, 1>(),
      // Keep alive, transitive: `lcm` keeps `builder` alive.
      py::keep_alive<4, 1>(),
      doc.ConnectContactResultsToDrakeVisualizer.doc_5args);

  {
    using Class = PropellerInfo;
    constexpr auto& cls_doc = doc.PropellerInfo;
    py::class_<Class> cls(m, "PropellerInfo", cls_doc.doc);
    cls  // BR
        .def(py::init<const BodyIndex&, const math::RigidTransform<double>&,
                 double, double>(),
            py::arg("body_index"),
            py::arg("X_BP") = math::RigidTransform<double>::Identity(),
            py::arg("thrust_ratio") = 1.0, py::arg("moment_ratio") = 0.0)
        .def_readwrite("body_index", &Class::body_index, cls_doc.body_index.doc)
        .def_readwrite("X_BP", &Class::X_BP, cls_doc.X_BP.doc)
        .def_readwrite(
            "thrust_ratio", &Class::thrust_ratio, cls_doc.thrust_ratio.doc)
        .def_readwrite(
            "moment_ratio", &Class::moment_ratio, cls_doc.moment_ratio.doc);
    DefCopyAndDeepCopy(&cls);
  }

  {
    using Class = ContactModel;
    constexpr auto& cls_doc = doc.ContactModel;
    py::enum_<Class>(m, "ContactModel", cls_doc.doc)
        .value("kHydroelastic", Class::kHydroelastic, cls_doc.kHydroelastic.doc)
        .value("kPoint", Class::kPoint, cls_doc.kPoint.doc)
        .value("kHydroelasticWithFallback", Class::kHydroelasticWithFallback,
            cls_doc.kHydroelasticWithFallback.doc)
        // Legacy alias. TODO(jwnimmer-tri) Deprecate this constant.
        .value("kHydroelasticsOnly", Class::kHydroelasticsOnly,
            cls_doc.kHydroelasticsOnly.doc)
        // Legacy alias. TODO(jwnimmer-tri) Deprecate this constant.
        .value("kPointContactOnly", Class::kPointContactOnly,
            cls_doc.kPointContactOnly.doc);
  }

  {
    using Class = DiscreteContactSolver;
    constexpr auto& cls_doc = doc.DiscreteContactSolver;
    py::enum_<Class>(m, "DiscreteContactSolver", cls_doc.doc)
        .value("kTamsi", Class::kTamsi, cls_doc.kTamsi.doc)
        .value("kSap", Class::kSap, cls_doc.kSap.doc);
  }

  {
    using Class = DiscreteContactApproximation;
    constexpr auto& cls_doc = doc.DiscreteContactApproximation;
    py::enum_<Class>(m, "DiscreteContactApproximation", cls_doc.doc)
        .value("kTamsi", Class::kTamsi, cls_doc.kTamsi.doc)
        .value("kSap", Class::kSap, cls_doc.kSap.doc)
        .value("kSimilar", Class::kSimilar, cls_doc.kSimilar.doc)
        .value("kLagged", Class::kLagged, cls_doc.kLagged.doc);
  }

  {
    using Class = MultibodyPlantConfig;
    constexpr auto& cls_doc = doc.MultibodyPlantConfig;
    py::class_<Class> cls(m, "MultibodyPlantConfig", cls_doc.doc);
    cls  // BR
        .def(ParamInit<Class>());
    DefAttributesUsingSerialize(&cls, cls_doc);
    DefReprUsingSerialize(&cls);
    DefCopyAndDeepCopy(&cls);
  }

  // PhysicalModel
  {
    using Class = PhysicalModel<T>;
    constexpr auto& cls_doc = doc.PhysicalModel;
    auto cls = py::class_<Class>(m, "PhysicalModel", cls_doc.doc);
  }

  // DeformableModel
  {
    using Class = DeformableModel<double>;
    constexpr auto& cls_doc = doc.DeformableModel;
    py::class_<Class, PhysicalModel<T>> cls(m, "DeformableModel", cls_doc.doc);
    cls  // BR
        .def(py::init<MultibodyPlant<T>*>(), cls_doc.ctor.doc)
        .def("num_bodies", &Class::num_bodies, cls_doc.num_bodies.doc)
        .def("RegisterDeformableBody", &Class::RegisterDeformableBody,
            py::arg("geometry_instance"), py::arg("config"),
            py::arg("resolution_hint"), cls_doc.RegisterDeformableBody.doc)
        .def("SetWallBoundaryCondition", &Class::SetWallBoundaryCondition,
            py::arg("id"), py::arg("p_WQ"), py::arg("n_W"),
            cls_doc.SetWallBoundaryCondition.doc)
        .def("GetDiscreteStateIndex", &Class::GetDiscreteStateIndex,
            py::arg("id"), cls_doc.GetDiscreteStateIndex.doc)
        .def("GetReferencePositions", &Class::GetReferencePositions,
            py::arg("id"), py_rvp::reference_internal,
            cls_doc.GetReferencePositions.doc)
        .def("GetGeometryId", &Class::GetGeometryId, py::arg("id"),
            cls_doc.GetGeometryId.doc)
        .def(
            "GetBodyId",
            [](const Class* self, geometry::GeometryId geometry_id) {
              return self->GetBodyId(geometry_id);
            },
            py::arg("geometry_id"), cls_doc.GetBodyId.doc_1args_geometry_id)
        .def("vertex_positions_port", &Class::vertex_positions_port,
            py_rvp::reference_internal, cls_doc.vertex_positions_port.doc);
  }
  // Deformable identifier.
  {
    BindIdentifier<DeformableBodyId>(
        m, "DeformableBodyId", doc.DeformableBodyId.doc);
  }

  type_visit([m](auto dummy) { DoScalarDependentDefinitions(m, dummy); },
      CommonScalarPack{});

  ExecuteExtraPythonCode(m);
}  // NOLINT(readability/fn_size)

}  // namespace pydrake
}  // namespace drake
