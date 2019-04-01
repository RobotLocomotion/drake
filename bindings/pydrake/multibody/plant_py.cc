#include "pybind11/eigen.h"
#include "pybind11/pybind11.h"

#include "drake/bindings/pydrake/common/deprecation_pybind.h"
#include "drake/bindings/pydrake/common/drake_optional_pybind.h"
#include "drake/bindings/pydrake/common/eigen_geometry_pybind.h"
#include "drake/bindings/pydrake/documentation_pybind.h"
#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/bindings/pydrake/systems/systems_pybind.h"
#include "drake/geometry/query_results/penetration_as_point_pair.h"
#include "drake/geometry/scene_graph.h"
#include "drake/math/rigid_transform.h"
#include "drake/multibody/plant/contact_info.h"
#include "drake/multibody/plant/contact_results.h"
#include "drake/multibody/plant/contact_results_to_lcm.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/tree/spatial_inertia.h"

namespace drake {
namespace pydrake {

// TODO(eric.cousineau): Expose available scalar types.
using T = double;

using std::string;

using geometry::SceneGraph;
using math::RigidTransform;
using systems::Context;
using systems::State;

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

PYBIND11_MODULE(plant, m) {
  PYDRAKE_PREVENT_PYTHON3_MODULE_REIMPORT(m);

  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::multibody;
  constexpr auto& doc = pydrake_doc.drake.multibody;

  m.doc() = "Bindings for MultibodyPlant and related classes.";

  py::module::import("pydrake.geometry");
  py::module::import("pydrake.multibody.math");
  py::module::import("pydrake.multibody.tree");
  py::module::import("pydrake.systems.framework");

  const char* doc_iso3_deprecation =
      "This API using Isometry3 will be deprecated soon with the resolution of "
      "#9865. We only offer it for backwards compatibility. DO NOT USE!.";

  {
    using Class = MultibodyPlant<T>;
    py::class_<Class, systems::LeafSystem<T>> cls(
        m, "MultibodyPlant", doc.MultibodyPlant.doc);
    // N.B. These are defined as they appear in the class declaration.
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
            doc.MultibodyPlant.AddJoint.doc_1args)
        .def("AddFrame",
            [](Class * self, std::unique_ptr<Frame<double>> frame) -> auto& {
              return self->AddFrame(std::move(frame));
            },
            py_reference_internal, py::arg("frame"),
            doc.MultibodyPlant.AddFrame.doc)
        .def("AddRigidBody",
            py::overload_cast<const std::string&,
                const SpatialInertia<double>&>(&Class::AddRigidBody),
            py::arg("name"), py::arg("M_BBo_B"), py_reference_internal,
            doc.MultibodyPlant.AddRigidBody.doc_2args)
        .def("WeldFrames",
            py::overload_cast<const Frame<T>&, const Frame<T>&,
                const Isometry3<double>&>(&Class::WeldFrames),
            py::arg("A"), py::arg("B"), py::arg("X_AB"), py_reference_internal,
            doc_iso3_deprecation)
        .def("WeldFrames",
            py::overload_cast<const Frame<T>&, const Frame<T>&,
                const RigidTransform<double>&>(&Class::WeldFrames),
            py::arg("A"), py::arg("B"),
            py::arg("X_AB") = RigidTransform<double>::Identity(),
            py_reference_internal, doc.MultibodyPlant.WeldFrames.doc)
        // N.B. This overload of `AddForceElement` is required to precede
        // the generic overload below so we can use our internal specialization
        // to label this as our unique gravity field, mimicking the C++ API.
        .def("AddForceElement",
            [](Class * self,
                std::unique_ptr<UniformGravityFieldElement<T>> force_element)
                -> auto& {
              // N.B. We need to make sure we call the correct specialization in
              // MultibodyPlant for it to take note we are adding gravity to the
              // model. This is ugly API needs to be updated, see #11080.
              return self->AddForceElement<UniformGravityFieldElement>(
                  force_element->gravity_vector());
            },
            py::arg("force_element"), py_reference_internal,
            doc.MultibodyPlant.AddForceElement.doc)
        .def("AddForceElement",
            [](Class * self,
                std::unique_ptr<ForceElement<T>> force_element) -> auto& {
              return self->AddForceElement<ForceElement>(
                  std::move(force_element));
            },
            py::arg("force_element"), py_reference_internal,
            doc.MultibodyPlant.AddForceElement.doc);
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
            py::arg("frame_A"), doc.MultibodyPlant.CalcPointsPositions.doc)
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
            doc.MultibodyPlant.CalcFrameGeometricJacobianExpressedInWorld.doc)
        // TODO(eric.cousineau): Include `CalcInverseDynamics` once there is an
        // overload that (a) services MBP directly and (b) uses body
        // association that is less awkward than implicit BodyNodeIndex.
        .def("SetFreeBodyPose",
            overload_cast_explicit<void, Context<T>*, const Body<T>&,
                const Isometry3<T>&>(&Class::SetFreeBodyPose),
            py::arg("context"), py::arg("body"), py::arg("X_WB"),
            doc_iso3_deprecation)
        .def("SetFreeBodyPose",
            overload_cast_explicit<void, Context<T>*, const Body<T>&,
                const RigidTransform<T>&>(&Class::SetFreeBodyPose),
            py::arg("context"), py::arg("body"), py::arg("X_WB"),
            doc.MultibodyPlant.SetFreeBodyPose.doc_3args)
        .def("SetActuationInArray",
            [](const Class* self, multibody::ModelInstanceIndex model_instance,
                const Eigen::Ref<const VectorX<T>> u_instance,
                Eigen::Ref<VectorX<T>> u) -> void {
              self->SetActuationInArray(model_instance, u_instance, &u);
            },
            py::arg("model_instance"), py::arg("u_instance"), py::arg("u"),
            doc.MultibodyPlant.SetActuationInArray.doc)
        .def("GetPositionsFromArray", &Class::GetPositionsFromArray,
            py::arg("model_instance"), py::arg("q"),
            doc.MultibodyPlant.GetPositionsFromArray.doc)
        .def("SetPositionsInArray",
            [](const Class* self, multibody::ModelInstanceIndex model_instance,
                const Eigen::Ref<const VectorX<T>> q_instance,
                Eigen::Ref<VectorX<T>> q) -> void {
              self->SetPositionsInArray(model_instance, q_instance, &q);
            },
            py::arg("model_instance"), py::arg("q_instance"), py::arg("q"),
            doc.MultibodyPlant.SetPositionsInArray.doc)
        .def("GetVelocitiesFromArray", &Class::GetVelocitiesFromArray,
            py::arg("model_instance"), py::arg("v"),
            doc.MultibodyPlant.GetVelocitiesFromArray.doc)
        .def("SetVelocitiesInArray",
            [](const Class* self, multibody::ModelInstanceIndex model_instance,
                const Eigen::Ref<const VectorX<T>> v_instance,
                Eigen::Ref<VectorX<T>> v) -> void {
              self->SetVelocitiesInArray(model_instance, v_instance, &v);
            },
            py::arg("model_instance"), py::arg("v_instance"), py::arg("v"),
            doc.MultibodyPlant.SetVelocitiesInArray.doc)
        // TODO(eric.cousineau): Ensure all of these return either references,
        // or copies, consistently. At present, `GetX(context)` returns a
        // reference, while `GetX(context, model_instance)` returns a copy.
        .def("GetPositions",
            [](const Class* self, const Context<T>& context) {
              // Reference.
              return self->GetPositions(context);
            },
            py::arg("context"), py_reference_internal,
            // Keep alive, ownership: `return` keeps `context` alive.
            py::keep_alive<0, 2>(), doc.MultibodyPlant.GetPositions.doc_1args)
        .def("GetPositions",
            [](const Class* self, const Context<T>& context,
                ModelInstanceIndex model_instance) {
              // Copy.
              return self->GetPositions(context, model_instance);
            },
            py::arg("context"), py::arg("model_instance"),
            doc.MultibodyPlant.GetPositions.doc_2args)
        .def("GetVelocities",
            [](const Class* self, const Context<T>& context) {
              // Reference.
              return self->GetVelocities(context);
            },
            py::arg("context"), py_reference_internal,
            // Keep alive, ownership: `return` keeps `context` alive.
            py::keep_alive<0, 2>(), doc.MultibodyPlant.GetVelocities.doc_1args)
        .def("GetVelocities",
            [](const Class* self, const Context<T>& context,
                ModelInstanceIndex model_instance) {
              // Copy.
              return self->GetVelocities(context, model_instance);
            },
            py::arg("context"), py::arg("model_instance"),
            doc.MultibodyPlant.GetVelocities.doc_2args)
        .def("SetFreeBodySpatialVelocity",
            [](const Class* self, const Body<T>& body,
                const SpatialVelocity<T>& V_WB, Context<T>* context) {
              self->SetFreeBodySpatialVelocity(context, body, V_WB);
            },
            py::arg("body"), py::arg("V_WB"), py::arg("context"),
            doc.MultibodyPlant.SetFreeBodySpatialVelocity.doc_3args)
        .def("EvalBodyPoseInWorld",
            [](const Class* self, const Context<T>& context,
                const Body<T>& body_B) {
              return self->EvalBodyPoseInWorld(context, body_B);
            },
            py::arg("context"), py::arg("body"),
            doc.MultibodyPlant.EvalBodyPoseInWorld.doc)
        .def("EvalBodySpatialVelocityInWorld",
            [](const Class* self, const Context<T>& context,
                const Body<T>& body_B) {
              return self->EvalBodySpatialVelocityInWorld(context, body_B);
            },
            py::arg("context"), py::arg("body"),
            doc.MultibodyPlant.EvalBodySpatialVelocityInWorld.doc)
        .def("CalcJacobianSpatialVelocity",
            [](const Class* self, const systems::Context<T>& context,
                JacobianWrtVariable with_respect_to, const Frame<T>& frame_B,
                const Eigen::Ref<const Vector3<T>>& p_BP,
                const Frame<T>& frame_A, const Frame<T>& frame_E) {
              MatrixX<T> Jw_ABp_E(6, GetVariableSize(*self, with_respect_to));
              self->CalcJacobianSpatialVelocity(context, with_respect_to,
                  frame_B, p_BP, frame_A, frame_E, &Jw_ABp_E);
              return Jw_ABp_E;
            },
            py::arg("context"), py::arg("with_respect_to"), py::arg("frame_B"),
            py::arg("p_BP"), py::arg("frame_A"), py::arg("frame_E"),
            doc.MultibodyPlant.CalcJacobianSpatialVelocity.doc)
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
            doc.MultibodyPlant.CalcSpatialAccelerationsFromVdot.doc)
        .def("CalcInverseDynamics", &Class::CalcInverseDynamics,
            py::arg("context"), py::arg("known_vdot"),
            py::arg("external_forces"),
            doc.MultibodyPlant.CalcInverseDynamics.doc)
        .def("CalcForceElementsContribution",
            &Class::CalcForceElementsContribution, py::arg("context"),
            py::arg("forces"),
            doc.MultibodyPlant.CalcForceElementsContribution.doc)
        .def("CalcPotentialEnergy", &Class::CalcPotentialEnergy,
            py::arg("context"), doc.MultibodyPlant.CalcPotentialEnergy.doc)
        .def("CalcConservativePower", &Class::CalcConservativePower,
            py::arg("context"), doc.MultibodyPlant.CalcConservativePower.doc)
        .def("GetPositionLowerLimits", &Class::GetPositionLowerLimits,
            doc.MultibodyPlant.GetPositionLowerLimits.doc)
        .def("GetPositionUpperLimits", &Class::GetPositionUpperLimits,
            doc.MultibodyPlant.GetPositionUpperLimits.doc)
        .def("GetVelocityLowerLimits", &Class::GetVelocityLowerLimits,
            doc.MultibodyPlant.GetVelocityLowerLimits.doc)
        .def("GetVelocityUpperLimits", &Class::GetVelocityUpperLimits,
            doc.MultibodyPlant.GetVelocityUpperLimits.doc)
        .def("GetAccelerationLowerLimits", &Class::GetAccelerationLowerLimits,
            doc.MultibodyPlant.GetAccelerationLowerLimits.doc)
        .def("GetAccelerationUpperLimits", &Class::GetAccelerationUpperLimits,
            doc.MultibodyPlant.GetAccelerationUpperLimits.doc)
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
            py::arg("context"), doc.MultibodyPlant.CalcBiasTerm.doc)
        .def("CalcGravityGeneralizedForces",
            &Class::CalcGravityGeneralizedForces, py::arg("context"),
            doc.MultibodyPlant.CalcGravityGeneralizedForces.doc)
        .def("MapVelocityToQDot",
            [](const Class* self, const Context<T>& context,
                const Eigen::Ref<const VectorX<T>>& v) {
              VectorX<T> qdot(self->num_positions());
              self->MapVelocityToQDot(context, v, &qdot);
              return qdot;
            },
            py::arg("context"), py::arg("v"),
            doc.MultibodyPlant.MapVelocityToQDot.doc)
        .def("MapQDotToVelocity",
            [](const Class* self, const Context<T>& context,
                const Eigen::Ref<const VectorX<T>>& qdot) {
              VectorX<T> v(self->num_velocities());
              self->MapQDotToVelocity(context, qdot, &v);
              return v;
            },
            py::arg("context"), py::arg("qdot"),
            doc.MultibodyPlant.MapQDotToVelocity.doc)
        .def("CalcRelativeTransform", &Class::CalcRelativeTransform,
            py::arg("context"), py::arg("frame_A"), py::arg("frame_B"),
            doc.MultibodyPlant.CalcRelativeTransform.doc);
    // Topology queries.
    cls  // BR
        .def(
            "num_frames", &Class::num_frames, doc.MultibodyPlant.num_frames.doc)
        .def("get_body", &Class::get_body, py::arg("body_index"),
            py_reference_internal, doc.MultibodyPlant.get_body.doc)
        .def("get_joint", &Class::get_joint, py::arg("joint_index"),
            py_reference_internal, doc.MultibodyPlant.get_joint.doc)
        .def("get_joint_actuator", &Class::get_joint_actuator,
            py::arg("actuator_index"), py_reference_internal,
            doc.MultibodyPlant.get_joint_actuator.doc)
        .def("get_frame", &Class::get_frame, py::arg("frame_index"),
            py_reference_internal, doc.MultibodyPlant.get_frame.doc)
        .def("GetModelInstanceName",
            overload_cast_explicit<const string&, ModelInstanceIndex>(
                &Class::GetModelInstanceName),
            py::arg("model_instance"), py_reference_internal,
            doc.MultibodyPlant.GetModelInstanceName.doc)
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
        .def("GetBodyIndices", &Class::GetBodyIndices,
            py::arg("model_instance"), doc.MultibodyPlant.GetBodyIndices.doc)
        .def("GetJointByName",
            [](const Class* self, const string& name,
                optional<ModelInstanceIndex> model_instance) -> auto& {
              return self->GetJointByName(name, model_instance);
            },
            py::arg("name"), py::arg("model_instance") = nullopt,
            py_reference_internal, doc.MultibodyPlant.GetJointByName.doc)
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
        .def("RegisterVisualGeometry",
            py::overload_cast<const Body<T>&, const RigidTransform<double>&,
                const geometry::Shape&, const std::string&,
                const Vector4<double>&, geometry::SceneGraph<T>*>(
                &Class::RegisterVisualGeometry),
            py::arg("body"), py::arg("X_BG"), py::arg("shape"), py::arg("name"),
            py::arg("diffuse_color"), py::arg("scene_graph") = nullptr,
            doc.MultibodyPlant.RegisterVisualGeometry
                .doc_6args_body_X_BG_shape_name_diffuse_color_scene_graph)
        .def("RegisterVisualGeometry",
            [doc_iso3_deprecation](Class* self, const Body<T>& body,
                const Isometry3<double>& X_BG, const geometry::Shape& shape,
                const std::string& name, const Vector4<double>& diffuse_color,
                geometry::SceneGraph<T>* scene_graph) {
              WarnDeprecated(doc_iso3_deprecation);
              return self->RegisterVisualGeometry(body,
                  RigidTransform<double>(X_BG), shape, name, diffuse_color,
                  scene_graph);
            },
            py::arg("body"), py::arg("X_BG"), py::arg("shape"), py::arg("name"),
            py::arg("diffuse_color"), py::arg("scene_graph") = nullptr,
            doc_iso3_deprecation)
        .def("RegisterCollisionGeometry",
            py::overload_cast<const Body<T>&, const RigidTransform<double>&,
                const geometry::Shape&, const std::string&,
                const CoulombFriction<double>&, geometry::SceneGraph<T>*>(
                &Class::RegisterCollisionGeometry),
            py::arg("body"), py::arg("X_BG"), py::arg("shape"), py::arg("name"),
            py::arg("coulomb_friction"), py::arg("scene_graph") = nullptr,
            doc.MultibodyPlant.RegisterCollisionGeometry.doc)
        .def("RegisterCollisionGeometry",
            [doc_iso3_deprecation](Class* self, const Body<T>& body,
                const Isometry3<double>& X_BG, const geometry::Shape& shape,
                const std::string& name,
                const CoulombFriction<double>& coulomb_friction,
                geometry::SceneGraph<T>* scene_graph) {
              WarnDeprecated(doc_iso3_deprecation);
              return self->RegisterCollisionGeometry(body,
                  RigidTransform<double>(X_BG), shape, name, coulomb_friction,
                  scene_graph);
            },
            py::arg("body"), py::arg("X_BG"), py::arg("shape"), py::arg("name"),
            py::arg("coulomb_friction"), py::arg("scene_graph") = nullptr,
            doc_iso3_deprecation)
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
            py_reference_internal, doc.MultibodyPlant.GetBodyFromFrameId.doc)
        .def("GetBodyFrameIdIfExists", &Class::GetBodyFrameIdIfExists,
            py::arg("body_index"), py_reference_internal,
            doc.MultibodyPlant.GetBodyFrameIdIfExists.doc);
    // Port accessors.
    cls  // BR
        .def("get_actuation_input_port",
            overload_cast_explicit<const systems::InputPort<T>&>(
                &Class::get_actuation_input_port),
            py_reference_internal,
            doc.MultibodyPlant.get_actuation_input_port.doc_0args)
        .def("get_actuation_input_port",
            overload_cast_explicit<const systems::InputPort<T>&,
                multibody::ModelInstanceIndex>(
                &Class::get_actuation_input_port),
            py_reference_internal,
            doc.MultibodyPlant.get_actuation_input_port.doc_1args)
        .def("get_continuous_state_output_port",
            overload_cast_explicit<const systems::OutputPort<T>&>(
                &Class::get_continuous_state_output_port),
            py_reference_internal,
            doc.MultibodyPlant.get_continuous_state_output_port.doc_0args)
        .def("get_continuous_state_output_port",
            overload_cast_explicit<const systems::OutputPort<T>&,
                multibody::ModelInstanceIndex>(
                &Class::get_continuous_state_output_port),
            py_reference_internal,
            doc.MultibodyPlant.get_continuous_state_output_port.doc_1args)
        .def("get_contact_results_output_port",
            overload_cast_explicit<const systems::OutputPort<T>&>(
                &Class::get_contact_results_output_port),
            py_reference_internal,
            doc.MultibodyPlant.get_contact_results_output_port.doc)
        .def("get_generalized_contact_forces_output_port",
            overload_cast_explicit<const systems::OutputPort<T>&,
                multibody::ModelInstanceIndex>(
                &Class::get_generalized_contact_forces_output_port),
            py_reference_internal, py::arg("model_instance"),
            doc.MultibodyPlant.get_generalized_contact_forces_output_port.doc);
    // Property accessors.
    cls  // BR
        .def("world_body", &Class::world_body, py_reference_internal,
            doc.MultibodyPlant.world_body.doc)
        .def("world_frame", &Class::world_frame, py_reference_internal,
            doc.MultibodyPlant.world_frame.doc)
        .def("is_finalized", &Class::is_finalized,
            doc.MultibodyPlant.is_finalized.doc)
        .def("Finalize", py::overload_cast<SceneGraph<T>*>(&Class::Finalize),
            py::arg("scene_graph") = nullptr, doc.MultibodyPlant.Finalize.doc);
    // Position and velocity accessors and mutators.
    cls  // BR
        .def("GetMutablePositionsAndVelocities",
            [](const MultibodyPlant<T>* self,
                Context<T>* context) -> Eigen::Ref<VectorX<T>> {
              return self->GetMutablePositionsAndVelocities(context);
            },
            py_reference,
            // Keep alive, ownership: `return` keeps `Context` alive.
            py::keep_alive<0, 2>(), py::arg("context"),
            doc.MultibodyPlant.GetMutablePositionsAndVelocities.doc)
        .def("GetMutablePositions",
            [](const MultibodyPlant<T>* self,
                Context<T>* context) -> Eigen::Ref<VectorX<T>> {
              return self->GetMutablePositions(context);
            },
            py_reference,
            // Keep alive, ownership: `return` keeps `Context` alive.
            py::keep_alive<0, 2>(), py::arg("context"),
            doc.MultibodyPlant.GetMutablePositions.doc_1args)
        .def("GetMutableVelocities",
            [](const MultibodyPlant<T>* self,
                Context<T>* context) -> Eigen::Ref<VectorX<T>> {
              return self->GetMutableVelocities(context);
            },
            py_reference,
            // Keep alive, ownership: `return` keeps `Context` alive.
            py::keep_alive<0, 2>(), py::arg("context"),
            doc.MultibodyPlant.GetMutableVelocities.doc_1args)
        .def("GetPositions",
            [](const MultibodyPlant<T>* self, const Context<T>& context)
                -> VectorX<T> { return self->GetPositions(context); },
            py_reference, py::arg("context"),
            doc.MultibodyPlant.GetPositions.doc_1args)
        .def("GetPositions",
            [](const MultibodyPlant<T>* self, const Context<T>& context,
                multibody::ModelInstanceIndex model_instance) -> VectorX<T> {
              return self->GetPositions(context, model_instance);
            },
            py_reference, py::arg("context"), py::arg("model_instance"),
            doc.MultibodyPlant.GetPositions.doc_2args)
        .def("SetPositions",
            [](const MultibodyPlant<T>* self, Context<T>* context,
                const VectorX<T>& q) { self->SetPositions(context, q); },
            py_reference, py::arg("context"), py::arg("q"),
            doc.MultibodyPlant.SetPositions.doc_2args)
        .def("SetPositions",
            [](const MultibodyPlant<T>* self, Context<T>* context,
                multibody::ModelInstanceIndex model_instance,
                const VectorX<T>& q) {
              self->SetPositions(context, model_instance, q);
            },
            py_reference, py::arg("context"), py::arg("model_instance"),
            py::arg("q"), doc.MultibodyPlant.SetPositions.doc_2args)
        .def("GetVelocities",
            [](const MultibodyPlant<T>* self, const Context<T>& context)
                -> VectorX<T> { return self->GetVelocities(context); },
            py_reference, py::arg("context"),
            doc.MultibodyPlant.GetVelocities.doc_1args)
        .def("GetVelocities",
            [](const MultibodyPlant<T>* self, const Context<T>& context,
                multibody::ModelInstanceIndex model_instance) -> VectorX<T> {
              return self->GetVelocities(context, model_instance);
            },
            py_reference, py::arg("context"), py::arg("model_instance"),
            doc.MultibodyPlant.GetVelocities.doc_2args)
        .def("SetVelocities",
            [](const MultibodyPlant<T>* self, Context<T>* context,
                const VectorX<T>& v) { self->SetVelocities(context, v); },
            py_reference, py::arg("context"), py::arg("v"),
            doc.MultibodyPlant.SetVelocities.doc_2args)
        .def("SetVelocities",
            [](const MultibodyPlant<T>* self, Context<T>* context,
                ModelInstanceIndex model_instance, const VectorX<T>& v) {
              self->SetVelocities(context, model_instance, v);
            },
            py_reference, py::arg("context"), py::arg("model_instance"),
            py::arg("v"), doc.MultibodyPlant.SetVelocities.doc_3args)
        .def("GetPositionsAndVelocities",
            [](const MultibodyPlant<T>* self,
                const Context<T>& context) -> VectorX<T> {
              return self->GetPositionsAndVelocities(context);
            },
            py_reference, py::arg("context"),
            doc.MultibodyPlant.GetPositionsAndVelocities.doc_1args)
        .def("GetPositionsAndVelocities",
            [](const MultibodyPlant<T>* self, const Context<T>& context,
                multibody::ModelInstanceIndex model_instance) -> VectorX<T> {
              return self->GetPositionsAndVelocities(context, model_instance);
            },
            py_reference, py::arg("context"), py::arg("model_instance"),
            doc.MultibodyPlant.GetPositionsAndVelocities.doc_2args)
        .def("SetPositionsAndVelocities",
            [](const MultibodyPlant<T>* self, Context<T>* context,
                const VectorX<T>& q_v) {
              self->SetPositionsAndVelocities(context, q_v);
            },
            py_reference, py::arg("context"), py::arg("q_v"),
            doc.MultibodyPlant.SetPositionsAndVelocities.doc_2args)
        .def("SetPositionsAndVelocities",
            [](const MultibodyPlant<T>* self, Context<T>* context,
                multibody::ModelInstanceIndex model_instance,
                const VectorX<T>& q_v) {
              self->SetPositionsAndVelocities(context, model_instance, q_v);
            },
            py_reference, py::arg("context"), py::arg("model_instance"),
            py::arg("q_v"),
            doc.MultibodyPlant.SetPositionsAndVelocities.doc_3args)
        .def("SetDefaultState",
            [](const Class* self, const Context<T>& context, State<T>* state) {
              self->SetDefaultState(context, state);
            },
            py::arg("context"), py::arg("state"),
            doc.MultibodyPlant.SetDefaultState.doc);
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
        .def("separation_speed", &Class::separation_speed)
        .def("point_pair", &Class::point_pair);
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

  // ContactResultsToLcmSystem
  {
    using Class = ContactResultsToLcmSystem<T>;
    py::class_<Class, systems::LeafSystem<T>>(m, "ContactResultsToLcmSystem")
        .def(py::init<const MultibodyPlant<double>&>(), py::arg("plant"),
            // Keep alive, reference: `self` keeps `plant` alive.
            py::keep_alive<1, 2>())
        .def("get_contact_result_input_port",
            &Class::get_contact_result_input_port, py_reference_internal)
        .def("get_lcm_message_output_port", &Class::get_lcm_message_output_port,
            py_reference_internal);
  }

  // CoulombFriction
  {
    using Class = CoulombFriction<T>;
    py::class_<Class>(m, "CoulombFriction")
        .def(py::init<const T&, const T&>(), py::arg("static_friction"),
            py::arg("dynamic_friction"), doc.CoulombFriction.ctor.doc_2args);
  }

  m.def("AddMultibodyPlantSceneGraph",
      [](systems::DiagramBuilder<T>* builder,
          std::unique_ptr<MultibodyPlant<T>> plant,
          std::unique_ptr<SceneGraph<T>> scene_graph) {
        auto pair = AddMultibodyPlantSceneGraph(
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

  m.def("ConnectContactResultsToDrakeVisualizer",
      [](systems::DiagramBuilder<T>* builder,
          const MultibodyPlant<double>& plant, lcm::DrakeLcmInterface* lcm) {
        return drake::multibody::ConnectContactResultsToDrakeVisualizer(
            builder, plant, lcm);
      },
      // Keep alive, ownership: `return` keeps `builder` alive.
      py::keep_alive<0, 1>(),
      // Keep alive, reference transfer: `plant` keeps `builder` alive.
      py::keep_alive<2, 1>(),
      // Keep alive, reference transfer: `lcm` keeps `builder` alive.
      py::keep_alive<3, 1>(), py_reference, py::arg("builder"),
      py::arg("plant"), py::arg("lcm") = nullptr);
}  // NOLINT(readability/fn_size)

}  // namespace pydrake
}  // namespace drake
