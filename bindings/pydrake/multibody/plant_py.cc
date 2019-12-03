#include "pybind11/eigen.h"
#include "pybind11/pybind11.h"
#include "pybind11/stl.h"
#include "pybind11/stl_bind.h"

#include "drake/bindings/pydrake/common/cpp_template_pybind.h"
#include "drake/bindings/pydrake/common/default_scalars_pybind.h"
#include "drake/bindings/pydrake/common/deprecation_pybind.h"
#include "drake/bindings/pydrake/common/eigen_geometry_pybind.h"
#include "drake/bindings/pydrake/common/type_pack.h"
#include "drake/bindings/pydrake/common/value_pybind.h"
#include "drake/bindings/pydrake/documentation_pybind.h"
#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/geometry/query_results/penetration_as_point_pair.h"
#include "drake/geometry/scene_graph.h"
#include "drake/math/rigid_transform.h"
#include "drake/multibody/plant/contact_results.h"
#include "drake/multibody/plant/contact_results_to_lcm.h"
#include "drake/multibody/plant/externally_applied_spatial_force.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/plant/point_pair_contact_info.h"
#include "drake/multibody/tree/spatial_inertia.h"

PYBIND11_MAKE_OPAQUE(
    std::vector<drake::multibody::ExternallyAppliedSpatialForce<double>>);

namespace drake {
namespace pydrake {

using std::string;

using geometry::SceneGraph;
using math::RigidTransform;
using systems::Context;
using systems::State;

namespace {
constexpr char doc_iso3_deprecation[] = R"""(
This API using Isometry3 is / will be deprecated soon with the resolution of
#9865. We only offer it for backwards compatibility. DO NOT USE!.
)""";

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

Eigen::VectorBlock<const VectorX<double>> CopyIfNotPodType(
    Eigen::VectorBlock<const VectorX<double>> x) {
  // N.B. This references the existing vector's data, and does not perform a
  // copy.
  return x;
}

template <typename T>
VectorX<T> CopyIfNotPodType(Eigen::VectorBlock<const VectorX<T>> x) {
  // N.B. This copies the vector's data.
  // TODO(eric.cousineau): Remove this once #8116 is resolved.
  return x;
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
            py::arg("i"), cls_doc.point_pair_contact_info.doc);
    AddValueInstantiation<Class>(m);
  }

  // CoulombFriction
  {
    using Class = CoulombFriction<T>;
    constexpr auto& cls_doc = doc.CoulombFriction;
    auto cls = DefineTemplateClassWithDefault<Class>(
        m, "CoulombFriction", param, cls_doc.doc);
    cls  // BR
        .def(py::init<const T&, const T&>(), py::arg("static_friction"),
            py::arg("dynamic_friction"), cls_doc.ctor.doc_2args)
        .def("static_friction", &Class::static_friction,
            cls_doc.static_friction.doc)
        .def("dynamic_friction", &Class::dynamic_friction,
            cls_doc.dynamic_friction.doc);

    m.def("CalcContactFrictionFromSurfaceProperties",
        [](const multibody::CoulombFriction<T>& surface_properties1,
            const multibody::CoulombFriction<T>& surface_properties2) {
          return drake::multibody::CalcContactFrictionFromSurfaceProperties(
              surface_properties1, surface_properties2);
        },
        py::arg("surface_properties1"), py::arg("surface_properties2"),
        py_reference, doc.CalcContactFrictionFromSurfaceProperties.doc);
  }

  {
    using Class = MultibodyPlant<T>;
    constexpr auto& cls_doc = doc.MultibodyPlant;
    auto cls = DefineTemplateClassWithDefault<Class, systems::LeafSystem<T>>(
        m, "MultibodyPlant", param, cls_doc.doc);
    // N.B. These are defined as they appear in the class declaration.
    // Forwarded methods from `MultibodyTree`.
    cls  // BR
        .def(py::init<double>(), py::arg("time_step") = 0.)
        .def("num_bodies", &Class::num_bodies, cls_doc.num_bodies.doc)
        .def("num_joints", &Class::num_joints, cls_doc.num_joints.doc)
        .def("num_actuators", &Class::num_actuators, cls_doc.num_actuators.doc)
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
        .def("AddJoint",
            [](Class * self, std::unique_ptr<Joint<T>> joint) -> auto& {
              return self->AddJoint(std::move(joint));
            },
            py::arg("joint"), py_reference_internal, cls_doc.AddJoint.doc_1args)
        .def("AddFrame",
            [](Class * self, std::unique_ptr<Frame<T>> frame) -> auto& {
              return self->AddFrame(std::move(frame));
            },
            py_reference_internal, py::arg("frame"), cls_doc.AddFrame.doc)
        .def("AddModelInstance", &Class::AddModelInstance, py::arg("name"),
            cls_doc.AddModelInstance.doc)
        .def("AddRigidBody",
            [](Class * self, const std::string& name,
                const SpatialInertia<double>& s) -> auto& {
              return self->AddRigidBody(name, s);
            },
            py::arg("name"), py::arg("M_BBo_B"), py_reference_internal,
            cls_doc.AddRigidBody.doc_2args)
        .def("AddRigidBody",
            py::overload_cast<const std::string&, ModelInstanceIndex,
                const SpatialInertia<double>&>(&Class::AddRigidBody),
            py::arg("name"), py::arg("model_instance"), py::arg("M_BBo_B"),
            py_reference_internal, cls_doc.AddRigidBody.doc_3args)
        .def("WeldFrames",
            py::overload_cast<const Frame<T>&, const Frame<T>&,
                const RigidTransform<double>&>(&Class::WeldFrames),
            py::arg("A"), py::arg("B"),
            py::arg("X_AB") = RigidTransform<double>::Identity(),
            py_reference_internal, cls_doc.WeldFrames.doc)
        .def("WeldFrames",
            [](Class* self, const Frame<T>& A, const Frame<T>& B,
                const Isometry3<double>& X_AB) -> const WeldJoint<T>& {
              WarnDeprecated(doc_iso3_deprecation);
              return self->WeldFrames(A, B, RigidTransform<double>(X_AB));
            },
            py::arg("A"), py::arg("B"), py::arg("X_AB"), py_reference_internal,
            doc_iso3_deprecation)
        .def("AddForceElement",
            [](Class * self,
                std::unique_ptr<ForceElement<T>> force_element) -> auto& {
              return self->template AddForceElement<ForceElement>(
                  std::move(force_element));
            },
            py::arg("force_element"), py_reference_internal,
            cls_doc.AddForceElement.doc);
    // Mathy bits
    cls  // BR
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
            py::arg("frame_A"), cls_doc.CalcPointsPositions.doc);
    // Bind deprecated overload.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
    cls.def("CalcFrameGeometricJacobianExpressedInWorld",
        WrapDeprecated(
            cls_doc.CalcFrameGeometricJacobianExpressedInWorld.doc_deprecated,
            [](const Class* self, const Context<T>& context,
                const Frame<T>& frame_B, const Vector3<T>& p_BoFo_B) {
              MatrixX<T> Jv_WF(6, self->num_velocities());
              self->CalcFrameGeometricJacobianExpressedInWorld(
                  context, frame_B, p_BoFo_B, &Jv_WF);
              return Jv_WF;
            }),
        py::arg("context"), py::arg("frame_B"),
        py::arg("p_BoFo_B") = Vector3<T>::Zero().eval(),
        cls_doc.CalcFrameGeometricJacobianExpressedInWorld.doc_deprecated);
#pragma GCC diagnostic pop
    cls  // BR
         // TODO(eric.cousineau): Include `CalcInverseDynamics` once there is an
         // overload that (a) services MBP directly and (b) uses body
         // association that is less awkward than implicit BodyNodeIndex.
        .def("SetFreeBodyPose",
            overload_cast_explicit<void, Context<T>*, const Body<T>&,
                const RigidTransform<T>&>(&Class::SetFreeBodyPose),
            py::arg("context"), py::arg("body"), py::arg("X_WB"),
            cls_doc.SetFreeBodyPose.doc_3args)
        .def("SetFreeBodyPose",
            [](const Class* self, Context<T>* context, const Body<T>& body,
                const Isometry3<T>& X_WB) {
              WarnDeprecated(doc_iso3_deprecation);
              return self->SetFreeBodyPose(
                  context, body, RigidTransform<T>(X_WB));
            },
            py::arg("context"), py::arg("body"), py::arg("X_WB"),
            doc_iso3_deprecation)
        .def("SetDefaultFreeBodyPose", &Class::SetDefaultFreeBodyPose,
            py::arg("body"), py::arg("X_WB"),
            cls_doc.SetDefaultFreeBodyPose.doc)
        .def("SetActuationInArray",
            [](const Class* self, multibody::ModelInstanceIndex model_instance,
                const Eigen::Ref<const VectorX<T>> u_instance,
                Eigen::Ref<VectorX<T>> u) -> void {
              self->SetActuationInArray(model_instance, u_instance, &u);
            },
            py::arg("model_instance"), py::arg("u_instance"), py::arg("u"),
            cls_doc.SetActuationInArray.doc)
        .def("GetPositionsFromArray", &Class::GetPositionsFromArray,
            py::arg("model_instance"), py::arg("q"),
            cls_doc.GetPositionsFromArray.doc)
        .def("SetPositionsInArray",
            [](const Class* self, multibody::ModelInstanceIndex model_instance,
                const Eigen::Ref<const VectorX<T>> q_instance,
                Eigen::Ref<VectorX<T>> q) -> void {
              self->SetPositionsInArray(model_instance, q_instance, &q);
            },
            py::arg("model_instance"), py::arg("q_instance"), py::arg("q"),
            cls_doc.SetPositionsInArray.doc)
        .def("GetVelocitiesFromArray", &Class::GetVelocitiesFromArray,
            py::arg("model_instance"), py::arg("v"),
            cls_doc.GetVelocitiesFromArray.doc)
        .def("SetVelocitiesInArray",
            [](const Class* self, multibody::ModelInstanceIndex model_instance,
                const Eigen::Ref<const VectorX<T>> v_instance,
                Eigen::Ref<VectorX<T>> v) -> void {
              self->SetVelocitiesInArray(model_instance, v_instance, &v);
            },
            py::arg("model_instance"), py::arg("v_instance"), py::arg("v"),
            cls_doc.SetVelocitiesInArray.doc)
        // TODO(eric.cousineau): Ensure all of these return either references,
        // or copies, consistently. At present, `GetX(context)` returns a
        // reference, while `GetX(context, model_instance)` returns a copy.
        .def("GetPositions",
            [](const Class* self, const Context<T>& context) {
              // Reference.
              return CopyIfNotPodType(self->GetPositions(context));
            },
            py::arg("context"), return_value_policy_for_scalar_type<T>(),
            // Keep alive, ownership: `return` keeps `context` alive.
            py::keep_alive<0, 2>(), cls_doc.GetPositions.doc_1args)
        .def("GetPositions",
            [](const Class* self, const Context<T>& context,
                ModelInstanceIndex model_instance) {
              // Copy.
              return self->GetPositions(context, model_instance);
            },
            py::arg("context"), py::arg("model_instance"),
            cls_doc.GetPositions.doc_2args)
        .def("GetVelocities",
            [](const Class* self, const Context<T>& context) {
              // Reference.
              return CopyIfNotPodType(self->GetVelocities(context));
            },
            py::arg("context"), return_value_policy_for_scalar_type<T>(),
            // Keep alive, ownership: `return` keeps `context` alive.
            py::keep_alive<0, 2>(), cls_doc.GetVelocities.doc_1args)
        .def("GetVelocities",
            [](const Class* self, const Context<T>& context,
                ModelInstanceIndex model_instance) {
              // Copy.
              return self->GetVelocities(context, model_instance);
            },
            py::arg("context"), py::arg("model_instance"),
            cls_doc.GetVelocities.doc_2args)
        .def("SetFreeBodySpatialVelocity",
            [](const Class* self, const Body<T>& body,
                const SpatialVelocity<T>& V_WB, Context<T>* context) {
              self->SetFreeBodySpatialVelocity(context, body, V_WB);
            },
            py::arg("body"), py::arg("V_WB"), py::arg("context"),
            cls_doc.SetFreeBodySpatialVelocity.doc_3args)
        .def("EvalBodyPoseInWorld",
            [](const Class* self, const Context<T>& context,
                const Body<T>& body_B) {
              return self->EvalBodyPoseInWorld(context, body_B);
            },
            py::arg("context"), py::arg("body"),
            cls_doc.EvalBodyPoseInWorld.doc)
        .def("EvalBodySpatialVelocityInWorld",
            [](const Class* self, const Context<T>& context,
                const Body<T>& body_B) {
              return self->EvalBodySpatialVelocityInWorld(context, body_B);
            },
            py::arg("context"), py::arg("body"),
            cls_doc.EvalBodySpatialVelocityInWorld.doc)
        .def("CalcJacobianSpatialVelocity",
            [](const Class* self, const systems::Context<T>& context,
                JacobianWrtVariable with_respect_to, const Frame<T>& frame_B,
                const Eigen::Ref<const Vector3<T>>& p_BP,
                const Frame<T>& frame_A, const Frame<T>& frame_E) {
              MatrixX<T> Jw_ABp_E(
                  6, GetVariableSize<T>(*self, with_respect_to));
              self->CalcJacobianSpatialVelocity(context, with_respect_to,
                  frame_B, p_BP, frame_A, frame_E, &Jw_ABp_E);
              return Jw_ABp_E;
            },
            py::arg("context"), py::arg("with_respect_to"), py::arg("frame_B"),
            py::arg("p_BP"), py::arg("frame_A"), py::arg("frame_E"),
            cls_doc.CalcJacobianSpatialVelocity.doc)
        .def("CalcJacobianAngularVelocity",
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
        .def("CalcJacobianTranslationalVelocity",
            [](const Class* self, const Context<T>& context,
                JacobianWrtVariable with_respect_to, const Frame<T>& frame_B,
                const Eigen::Ref<const Matrix3X<T>>& p_BoBi_B,
                const Frame<T>& frame_A, const Frame<T>& frame_E) {
              Matrix3X<T> Js_v_ABi_E(
                  3, GetVariableSize<T>(*self, with_respect_to));
              self->CalcJacobianTranslationalVelocity(context, with_respect_to,
                  frame_B, p_BoBi_B, frame_A, frame_E, &Js_v_ABi_E);
              return Js_v_ABi_E;
            },
            py::arg("context"), py::arg("with_respect_to"), py::arg("frame_B"),
            py::arg("p_BoBi_B"), py::arg("frame_A"), py::arg("frame_E"),
            cls_doc.CalcJacobianTranslationalVelocity.doc)
        .def("CalcSpatialAccelerationsFromVdot",
            [](const Class* self, const Context<T>& context,
                const VectorX<T>& known_vdot) {
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
        .def("CalcPotentialEnergy", &Class::CalcPotentialEnergy,
            py::arg("context"), cls_doc.CalcPotentialEnergy.doc)
        .def("CalcConservativePower", &Class::CalcConservativePower,
            py::arg("context"), cls_doc.CalcConservativePower.doc)
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
              VectorX<T> Cv(self->num_velocities());
              self->CalcBiasTerm(context, &Cv);
              return Cv;
            },
            py::arg("context"), cls_doc.CalcBiasTerm.doc)
        .def("CalcGravityGeneralizedForces",
            &Class::CalcGravityGeneralizedForces, py::arg("context"),
            cls_doc.CalcGravityGeneralizedForces.doc)
        .def("MakeActuationMatrix", &Class::MakeActuationMatrix,
            cls_doc.MakeActuationMatrix.doc)
        .def("MapVelocityToQDot",
            [](const Class* self, const Context<T>& context,
                const Eigen::Ref<const VectorX<T>>& v) {
              VectorX<T> qdot(self->num_positions());
              self->MapVelocityToQDot(context, v, &qdot);
              return qdot;
            },
            py::arg("context"), py::arg("v"), cls_doc.MapVelocityToQDot.doc)
        .def("MapQDotToVelocity",
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
    // Topology queries.
    cls  // BR
        .def("num_frames", &Class::num_frames, cls_doc.num_frames.doc)
        .def("get_body", &Class::get_body, py::arg("body_index"),
            py_reference_internal, cls_doc.get_body.doc)
        .def("get_joint", &Class::get_joint, py::arg("joint_index"),
            py_reference_internal, cls_doc.get_joint.doc)
        .def("get_joint_actuator", &Class::get_joint_actuator,
            py::arg("actuator_index"), py_reference_internal,
            cls_doc.get_joint_actuator.doc)
        .def("get_frame", &Class::get_frame, py::arg("frame_index"),
            py_reference_internal, cls_doc.get_frame.doc)
        .def("gravity_field", &Class::gravity_field, py_reference_internal,
            cls_doc.gravity_field.doc)
        .def("mutable_gravity_field", &Class::mutable_gravity_field,
            py_reference_internal, cls_doc.mutable_gravity_field.doc)
        .def("GetModelInstanceName",
            overload_cast_explicit<const string&, ModelInstanceIndex>(
                &Class::GetModelInstanceName),
            py::arg("model_instance"), py_reference_internal,
            cls_doc.GetModelInstanceName.doc)
        .def("HasBodyNamed",
            overload_cast_explicit<bool, const string&>(&Class::HasBodyNamed),
            py::arg("name"), cls_doc.HasBodyNamed.doc_1args)
        .def("HasBodyNamed",
            overload_cast_explicit<bool, const string&, ModelInstanceIndex>(
                &Class::HasBodyNamed),
            py::arg("name"), py::arg("model_instance"),
            cls_doc.HasBodyNamed.doc_2args)
        .def("HasJointNamed",
            overload_cast_explicit<bool, const string&>(&Class::HasJointNamed),
            py::arg("name"), cls_doc.HasJointNamed.doc_1args)
        .def("HasJointNamed",
            overload_cast_explicit<bool, const string&, ModelInstanceIndex>(
                &Class::HasJointNamed),
            py::arg("name"), py::arg("model_instance"),
            cls_doc.HasJointNamed.doc_2args)
        .def("GetFrameByName",
            overload_cast_explicit<const Frame<T>&, const string&>(
                &Class::GetFrameByName),
            py::arg("name"), py_reference_internal,
            cls_doc.GetFrameByName.doc_1args)
        .def("GetFrameByName",
            overload_cast_explicit<const Frame<T>&, const string&,
                ModelInstanceIndex>(&Class::GetFrameByName),
            py::arg("name"), py::arg("model_instance"), py_reference_internal,
            cls_doc.GetFrameByName.doc_2args)
        .def("GetBodyByName",
            overload_cast_explicit<const Body<T>&, const string&>(
                &Class::GetBodyByName),
            py::arg("name"), py_reference_internal,
            cls_doc.GetBodyByName.doc_1args)
        .def("GetBodyByName",
            overload_cast_explicit<const Body<T>&, const string&,
                ModelInstanceIndex>(&Class::GetBodyByName),
            py::arg("name"), py::arg("model_instance"), py_reference_internal,
            cls_doc.GetBodyByName.doc_2args)
        .def("GetBodyFrameIdOrThrow", &Class::GetBodyFrameIdOrThrow,
            py::arg("body_index"), cls_doc.GetBodyFrameIdOrThrow.doc)
        .def("GetBodyIndices", &Class::GetBodyIndices,
            py::arg("model_instance"), cls_doc.GetBodyIndices.doc)
        .def("GetJointByName",
            [](const Class* self, const string& name,
                std::optional<ModelInstanceIndex> model_instance) -> auto& {
              return self->GetJointByName(name, model_instance);
            },
            py::arg("name"), py::arg("model_instance") = std::nullopt,
            py_reference_internal, cls_doc.GetJointByName.doc)
        .def("GetMutableJointByName",
            [](Class * self, const string& name,
                std::optional<ModelInstanceIndex> model_instance) -> auto& {
              return self->GetMutableJointByName(name, model_instance);
            },
            py::arg("name"), py::arg("model_instance") = std::nullopt,
            py_reference_internal, cls_doc.GetJointByName.doc)
        .def("GetJointActuatorByName",
            overload_cast_explicit<const JointActuator<T>&, const string&>(
                &Class::GetJointActuatorByName),
            py::arg("name"), py_reference_internal,
            cls_doc.GetJointActuatorByName.doc_1args)
        .def("GetModelInstanceByName",
            overload_cast_explicit<ModelInstanceIndex, const string&>(
                &Class::GetModelInstanceByName),
            py::arg("name"), py_reference_internal,
            cls_doc.GetModelInstanceByName.doc);
    // Geometry.
    cls  // BR
        .def("RegisterAsSourceForSceneGraph",
            &Class::RegisterAsSourceForSceneGraph, py::arg("scene_graph"),
            cls_doc.RegisterAsSourceForSceneGraph.doc)
        .def("RegisterVisualGeometry",
            py::overload_cast<const Body<T>&, const RigidTransform<double>&,
                const geometry::Shape&, const std::string&,
                const Vector4<double>&>(&Class::RegisterVisualGeometry),
            py::arg("body"), py::arg("X_BG"), py::arg("shape"), py::arg("name"),
            py::arg("diffuse_color"),
            cls_doc.RegisterVisualGeometry
                .doc_5args_body_X_BG_shape_name_diffuse_color)
        .def("RegisterVisualGeometry",
            [](Class* self, const Body<T>& body, const Isometry3<double>& X_BG,
                const geometry::Shape& shape, const std::string& name,
                const Vector4<double>& diffuse_color,
                geometry::SceneGraph<T>* scene_graph) {
              WarnDeprecated(doc_iso3_deprecation);
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
              if (!self->geometry_source_is_registered()) {
                self->RegisterAsSourceForSceneGraph(scene_graph);
              }
              return self->RegisterVisualGeometry(body,
                  RigidTransform<double>(X_BG), shape, name, diffuse_color);
#pragma GCC diagnostic pop
            },
            py::arg("body"), py::arg("X_BG"), py::arg("shape"), py::arg("name"),
            py::arg("diffuse_color"), py::arg("scene_graph") = nullptr,
            doc_iso3_deprecation)
        .def("RegisterCollisionGeometry",
            py::overload_cast<const Body<T>&, const RigidTransform<double>&,
                const geometry::Shape&, const std::string&,
                const CoulombFriction<double>&>(
                &Class::RegisterCollisionGeometry),
            py::arg("body"), py::arg("X_BG"), py::arg("shape"), py::arg("name"),
            py::arg("coulomb_friction"), cls_doc.RegisterCollisionGeometry.doc)
        .def("get_source_id", &Class::get_source_id, cls_doc.get_source_id.doc)
        .def("get_geometry_query_input_port",
            &Class::get_geometry_query_input_port, py_reference_internal,
            cls_doc.get_geometry_query_input_port.doc)
        .def("get_geometry_poses_output_port",
            &Class::get_geometry_poses_output_port, py_reference_internal,
            cls_doc.get_geometry_poses_output_port.doc)
        .def("geometry_source_is_registered",
            &Class::geometry_source_is_registered,
            cls_doc.geometry_source_is_registered.doc)
        .def("GetBodyFromFrameId", &Class::GetBodyFromFrameId,
            py_reference_internal, cls_doc.GetBodyFromFrameId.doc)
        .def("GetBodyFrameIdIfExists", &Class::GetBodyFrameIdIfExists,
            py::arg("body_index"), py_reference_internal,
            cls_doc.GetBodyFrameIdIfExists.doc)
        .def("GetCollisionGeometriesForBody",
            &Class::GetCollisionGeometriesForBody, py::arg("body"),
            py_reference_internal, cls_doc.GetCollisionGeometriesForBody.doc)
        .def("num_collision_geometries", &Class::num_collision_geometries,
            cls_doc.num_collision_geometries.doc)
        .def("default_coulomb_friction", &Class::default_coulomb_friction,
            py::arg("geometry_id"), py_reference_internal,
            cls_doc.default_coulomb_friction.doc);
    // Port accessors.
    cls  // BR
        .def("get_actuation_input_port",
            overload_cast_explicit<const systems::InputPort<T>&>(
                &Class::get_actuation_input_port),
            py_reference_internal, cls_doc.get_actuation_input_port.doc_0args)
        .def("get_actuation_input_port",
            overload_cast_explicit<const systems::InputPort<T>&,
                multibody::ModelInstanceIndex>(
                &Class::get_actuation_input_port),
            py_reference_internal, cls_doc.get_actuation_input_port.doc_1args)
        .def("get_applied_generalized_force_input_port",
            overload_cast_explicit<const systems::InputPort<T>&>(
                &Class::get_applied_generalized_force_input_port),
            py_reference_internal,
            cls_doc.get_applied_generalized_force_input_port.doc)
        .def("get_applied_spatial_force_input_port",
            overload_cast_explicit<const systems::InputPort<T>&>(
                &Class::get_applied_spatial_force_input_port),
            py_reference_internal,
            cls_doc.get_applied_spatial_force_input_port.doc)
        .def("get_state_output_port",
            overload_cast_explicit<const systems::OutputPort<T>&>(
                &Class::get_state_output_port),
            py_reference_internal, cls_doc.get_state_output_port.doc_0args)
        .def("get_state_output_port",
            overload_cast_explicit<const systems::OutputPort<T>&,
                multibody::ModelInstanceIndex>(&Class::get_state_output_port),
            py_reference_internal, cls_doc.get_state_output_port.doc_1args)
        .def("get_contact_results_output_port",
            overload_cast_explicit<const systems::OutputPort<T>&>(
                &Class::get_contact_results_output_port),
            py_reference_internal, cls_doc.get_contact_results_output_port.doc)
        .def("get_generalized_contact_forces_output_port",
            overload_cast_explicit<const systems::OutputPort<T>&,
                multibody::ModelInstanceIndex>(
                &Class::get_generalized_contact_forces_output_port),
            py_reference_internal, py::arg("model_instance"),
            cls_doc.get_generalized_contact_forces_output_port.doc);
    // Property accessors.
    cls  // BR
        .def("world_body", &Class::world_body, py_reference_internal,
            cls_doc.world_body.doc)
        .def("world_frame", &Class::world_frame, py_reference_internal,
            cls_doc.world_frame.doc)
        .def("is_finalized", &Class::is_finalized, cls_doc.is_finalized.doc)
        .def("Finalize", py::overload_cast<>(&Class::Finalize),
            cls_doc.Finalize.doc);
    // Position and velocity accessors and mutators.
    cls  // BR
        .def("GetMutablePositionsAndVelocities",
            [](const MultibodyPlant<T>* self,
                Context<T>* context) -> Eigen::Ref<VectorX<T>> {
              return self->GetMutablePositionsAndVelocities(context);
            },
            py_reference, py::arg("context"),
            // Keep alive, ownership: `return` keeps `context` alive.
            py::keep_alive<0, 2>(),
            cls_doc.GetMutablePositionsAndVelocities.doc)
        .def("GetMutablePositions",
            [](const MultibodyPlant<T>* self,
                Context<T>* context) -> Eigen::Ref<VectorX<T>> {
              return self->GetMutablePositions(context);
            },
            py_reference, py::arg("context"),
            // Keep alive, ownership: `return` keeps `context` alive.
            py::keep_alive<0, 2>(), cls_doc.GetMutablePositions.doc_1args)
        .def("GetMutableVelocities",
            [](const MultibodyPlant<T>* self,
                Context<T>* context) -> Eigen::Ref<VectorX<T>> {
              return self->GetMutableVelocities(context);
            },
            py_reference, py::arg("context"),
            // Keep alive, ownership: `return` keeps `context` alive.
            py::keep_alive<0, 2>(), cls_doc.GetMutableVelocities.doc_1args)
        .def("GetPositions",
            [](const MultibodyPlant<T>* self, const Context<T>& context)
                -> VectorX<T> { return self->GetPositions(context); },
            py_reference, py::arg("context"), cls_doc.GetPositions.doc_1args)
        .def("GetPositions",
            [](const MultibodyPlant<T>* self, const Context<T>& context,
                multibody::ModelInstanceIndex model_instance) -> VectorX<T> {
              return self->GetPositions(context, model_instance);
            },
            py_reference, py::arg("context"), py::arg("model_instance"),
            cls_doc.GetPositions.doc_2args)
        .def("SetPositions",
            [](const MultibodyPlant<T>* self, Context<T>* context,
                const VectorX<T>& q) { self->SetPositions(context, q); },
            py_reference, py::arg("context"), py::arg("q"),
            cls_doc.SetPositions.doc_2args)
        .def("SetPositions",
            [](const MultibodyPlant<T>* self, Context<T>* context,
                multibody::ModelInstanceIndex model_instance,
                const VectorX<T>& q) {
              self->SetPositions(context, model_instance, q);
            },
            py_reference, py::arg("context"), py::arg("model_instance"),
            py::arg("q"), cls_doc.SetPositions.doc_2args)
        .def("GetVelocities",
            [](const MultibodyPlant<T>* self, const Context<T>& context)
                -> VectorX<T> { return self->GetVelocities(context); },
            py_reference, py::arg("context"), cls_doc.GetVelocities.doc_1args)
        .def("GetVelocities",
            [](const MultibodyPlant<T>* self, const Context<T>& context,
                multibody::ModelInstanceIndex model_instance) -> VectorX<T> {
              return self->GetVelocities(context, model_instance);
            },
            py_reference, py::arg("context"), py::arg("model_instance"),
            cls_doc.GetVelocities.doc_2args)
        .def("SetVelocities",
            [](const MultibodyPlant<T>* self, Context<T>* context,
                const VectorX<T>& v) { self->SetVelocities(context, v); },
            py_reference, py::arg("context"), py::arg("v"),
            cls_doc.SetVelocities.doc_2args)
        .def("SetVelocities",
            [](const MultibodyPlant<T>* self, Context<T>* context,
                ModelInstanceIndex model_instance, const VectorX<T>& v) {
              self->SetVelocities(context, model_instance, v);
            },
            py_reference, py::arg("context"), py::arg("model_instance"),
            py::arg("v"), cls_doc.SetVelocities.doc_3args)
        .def("GetPositionsAndVelocities",
            [](const MultibodyPlant<T>* self,
                const Context<T>& context) -> VectorX<T> {
              return self->GetPositionsAndVelocities(context);
            },
            py_reference, py::arg("context"),
            cls_doc.GetPositionsAndVelocities.doc_1args)
        .def("GetPositionsAndVelocities",
            [](const MultibodyPlant<T>* self, const Context<T>& context,
                multibody::ModelInstanceIndex model_instance) -> VectorX<T> {
              return self->GetPositionsAndVelocities(context, model_instance);
            },
            py_reference, py::arg("context"), py::arg("model_instance"),
            cls_doc.GetPositionsAndVelocities.doc_2args)
        .def("SetPositionsAndVelocities",
            [](const MultibodyPlant<T>* self, Context<T>* context,
                const VectorX<T>& q_v) {
              self->SetPositionsAndVelocities(context, q_v);
            },
            py_reference, py::arg("context"), py::arg("q_v"),
            cls_doc.SetPositionsAndVelocities.doc_2args)
        .def("SetPositionsAndVelocities",
            [](const MultibodyPlant<T>* self, Context<T>* context,
                multibody::ModelInstanceIndex model_instance,
                const VectorX<T>& q_v) {
              self->SetPositionsAndVelocities(context, model_instance, q_v);
            },
            py_reference, py::arg("context"), py::arg("model_instance"),
            py::arg("q_v"), cls_doc.SetPositionsAndVelocities.doc_3args)
        .def("SetDefaultState",
            [](const Class* self, const Context<T>& context, State<T>* state) {
              self->SetDefaultState(context, state);
            },
            py::arg("context"), py::arg("state"), cls_doc.SetDefaultState.doc);
  }

  if (!std::is_same<T, symbolic::Expression>::value) {
    m.def("AddMultibodyPlantSceneGraph",
        [](systems::DiagramBuilder<T>* builder,
            std::unique_ptr<MultibodyPlant<T>> plant,
            std::unique_ptr<SceneGraph<T>> scene_graph) {
          auto pair = AddMultibodyPlantSceneGraph<T>(
              builder, std::move(plant), std::move(scene_graph));
          // Must do manual keep alive to dig into tuple.
          py::object builder_py = py::cast(builder, py_reference);
          py::object plant_py = py::cast(pair.plant, py_reference);
          py::object scene_graph_py = py::cast(pair.scene_graph, py_reference);
          return py::make_tuple(
              // Keep alive, ownership: `plant` keeps `builder` alive.
              py_keep_alive(plant_py, builder_py),
              // Keep alive, ownership: `scene_graph` keeps `builder` alive.
              py_keep_alive(scene_graph_py, builder_py));
        },
        py::arg("builder"), py::arg("plant") = nullptr,
        py::arg("scene_graph") = nullptr, doc.AddMultibodyPlantSceneGraph.doc);
  }

  // ExternallyAppliedSpatialForce
  {
    using Class = multibody::ExternallyAppliedSpatialForce<T>;
    constexpr auto& cls_doc = doc.ExternallyAppliedSpatialForce;
    auto cls = DefineTemplateClassWithDefault<Class>(
        m, "ExternallyAppliedSpatialForce", param, cls_doc.doc);
    cls  // BR
        .def(py::init<>())
        .def_readwrite("body_index", &Class::body_index, cls_doc.body_index.doc)
        .def_readwrite("p_BoBq_B", &Class::p_BoBq_B, cls_doc.p_BoBq_B.doc)
        .def_readwrite("F_Bq_W", &Class::F_Bq_W, cls_doc.F_Bq_W.doc);
    AddValueInstantiation<Class>(m);
  }

  // Opaquely bind std::vector<ExternallyAppliedSpatialForce> to enable
  // Python systems to construct AbstractValues of this type with the type
  // being legible for port connections.
  {
    using Class = std::vector<multibody::ExternallyAppliedSpatialForce<T>>;
    // TODO(eric.cousineau): Try to make this specialization for
    // `py::bind_vector` less boiler-platey, like
    // `DefineTemplateClassWithDefault`.
    const std::string default_name = "VectorExternallyAppliedSpatialForced";
    const std::string template_name = default_name + "_";
    auto cls = py::bind_vector<Class>(m, TemporaryClassName<Class>().c_str());
    AddTemplateClass(m, template_name.c_str(), cls, param);
    if (!py::hasattr(m, default_name.c_str())) {
      m.attr(default_name.c_str()) = cls;
    }
    AddValueInstantiation<Class>(m);
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
  py::module::import("pydrake.multibody.math");
  py::module::import("pydrake.multibody.tree");
  py::module::import("pydrake.systems.framework");

  type_visit([m](auto dummy) { DoScalarDependentDefinitions(m, dummy); },
      CommonScalarPack{});
  constexpr auto& doc = pydrake_doc.drake.multibody;

  using T = double;

  // ContactResultsToLcmSystem
  {
    using Class = ContactResultsToLcmSystem<T>;
    constexpr auto& cls_doc = doc.ContactResultsToLcmSystem;
    py::class_<Class, systems::LeafSystem<T>>(
        m, "ContactResultsToLcmSystem", cls_doc.doc)
        .def(py::init<const MultibodyPlant<T>&>(), py::arg("plant"),
            // Keep alive, reference: `self` keeps `plant` alive.
            py::keep_alive<1, 2>(), cls_doc.ctor.doc)
        .def("get_contact_result_input_port",
            &Class::get_contact_result_input_port, py_reference_internal,
            cls_doc.get_contact_result_input_port.doc)
        .def("get_lcm_message_output_port", &Class::get_lcm_message_output_port,
            py_reference_internal, cls_doc.get_lcm_message_output_port.doc);
  }

  m.def("ConnectContactResultsToDrakeVisualizer",
      [](systems::DiagramBuilder<double>* builder,
          const MultibodyPlant<double>& plant, lcm::DrakeLcmInterface* lcm) {
        return drake::multibody::ConnectContactResultsToDrakeVisualizer(
            builder, plant, lcm);
      },
      py::arg("builder"), py::arg("plant"), py::arg("lcm") = nullptr,
      py_reference,
      // Keep alive, ownership: `return` keeps `builder` alive.
      py::keep_alive<0, 1>(),
      // Keep alive, transitive: `plant` keeps `builder` alive.
      py::keep_alive<2, 1>(),
      // Keep alive, transitive: `lcm` keeps `builder` alive.
      py::keep_alive<3, 1>(),
      doc.ConnectContactResultsToDrakeVisualizer.doc_3args);
}  // NOLINT(readability/fn_size)

}  // namespace pydrake
}  // namespace drake
