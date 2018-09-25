#include "pybind11/eigen.h"
#include "pybind11/eval.h"
#include "pybind11/pybind11.h"

#include "drake/bindings/pydrake/documentation_pybind.h"
#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/bindings/pydrake/util/deprecation_pybind.h"
#include "drake/bindings/pydrake/util/drake_optional_pybind.h"
#include "drake/bindings/pydrake/util/eigen_geometry_pybind.h"
#include "drake/bindings/pydrake/util/type_safe_index_pybind.h"
#include "drake/multibody/multibody_tree/joints/prismatic_joint.h"
#include "drake/multibody/multibody_tree/joints/revolute_joint.h"
#include "drake/multibody/multibody_tree/joints/weld_joint.h"
#include "drake/multibody/multibody_tree/math/spatial_force.h"
#include "drake/multibody/multibody_tree/math/spatial_vector.h"
#include "drake/multibody/multibody_tree/math/spatial_velocity.h"
#include "drake/multibody/multibody_tree/multibody_forces.h"
#include "drake/multibody/multibody_tree/multibody_plant/multibody_plant.h"
#include "drake/multibody/multibody_tree/multibody_tree.h"
#include "drake/multibody/multibody_tree/parsing/multibody_plant_sdf_parser.h"

#define D(...) DOC(drake, multibody, __VA_ARGS__)

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
template <typename PyClass>
void BindMultibodyTreeElementMixin(PyClass* pcls) {
  using Class = typename PyClass::type;
  auto& cls = *pcls;
  cls
      .def("get_parent_tree", &Class::get_parent_tree, py_reference_internal)
      .def("index", &Class::index)
      .def("model_instance", &Class::model_instance);
}

void init_module(py::module m) {
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::multibody;

  using systems::Context;

  // To simplify checking binding coverage, these are defined in the same order
  // as `multibody_tree_indexes.h`.
  BindTypeSafeIndex<FrameIndex>(m, "FrameIndex");
  BindTypeSafeIndex<BodyIndex>(m, "BodyIndex");
  BindTypeSafeIndex<MobilizerIndex>(m, "MobilizerIndex");
  BindTypeSafeIndex<BodyNodeIndex>(m, "BodyNodeIndex");
  BindTypeSafeIndex<ForceElementIndex>(m, "ForceElementIndex");
  BindTypeSafeIndex<JointIndex>(m, "JointIndex");
  BindTypeSafeIndex<JointActuatorIndex>(m, "JointActuatorIndex");
  BindTypeSafeIndex<ModelInstanceIndex>(m, "ModelInstanceIndex");
  m.def("world_index", &world_index, D(world_index));

  // Frames.
  {
    using Class = Frame<T>;
    py::class_<Class> cls(m, "Frame", D(Frame));
    BindMultibodyTreeElementMixin(&cls);
    cls
        .def("name", &Class::name, D(Frame, name))
        .def("body", &Class::body, py_reference_internal,
             D(Frame, body));
  }

  {
    using Class = BodyFrame<T>;
    py::class_<Class, Frame<T>> cls(m, "BodyFrame", D(BodyFrame));
    // No need to re-bind element mixins from `Frame`.
  }

  // Bodies.
  {
    using Class = Body<T>;
    py::class_<Class> cls(m, "Body", D(Body));
    BindMultibodyTreeElementMixin(&cls);
    cls
        .def("name", &Class::name, D(Body, name))
        .def("body_frame", &Class::body_frame, py_reference_internal,
             D(Body, body_frame));
  }

  {
    using Class = RigidBody<T>;
    py::class_<Class, Body<T>> cls(m, "RigidBody", D(RigidBody));
  }

  // Joints.
  {
    using Class = Joint<T>;
    py::class_<Class> cls(m, "Joint", D(Joint));
    BindMultibodyTreeElementMixin(&cls);
    cls
        .def("name", &Class::name, D(Joint, name))
        .def("parent_body", &Class::parent_body, py_reference_internal,
             D(Joint, parent_body))
        .def("child_body", &Class::child_body, py_reference_internal,
             D(Joint, child_body))
        .def("frame_on_parent", &Class::frame_on_parent, py_reference_internal,
             D(Joint, frame_on_parent))
        .def("frame_on_child", &Class::frame_on_child, py_reference_internal,
             D(Joint, frame_on_child))
        .def("position_start", &Class::position_start,
             D(Joint, position_start))
        .def("velocity_start", &Class::velocity_start,
             D(Joint, velocity_start))
        .def("num_positions", &Class::num_positions,
             D(Joint, num_positions))
        .def("num_velocities", &Class::num_velocities,
             D(Joint, num_velocities));

    // Add deprecated methods.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
    cls.def("num_dofs", &Class::num_dofs, D(Joint, num_dofs));
#pragma GCC diagnostic pop  // pop -Wdeprecated-declarations
    cls.attr("message_num_dofs") = "Please use num_velocities().";
    DeprecateAttribute(
        cls, "num_dofs", cls.attr("message_num_dofs"));
  }

  {
    using Class = PrismaticJoint<T>;
    py::class_<Class, Joint<T>> cls(m, "PrismaticJoint");
    cls
        .def("get_translation", &Class::get_translation, py::arg("context"),
        D(PrismaticJoint, get_translation))
        .def("set_translation", &Class::set_translation,
             py::arg("context"), py::arg("translation"),
             D(PrismaticJoint, set_translation))
        .def("get_translation_rate", &Class::get_translation_rate,
             py::arg("context"),
             D(PrismaticJoint, get_translation_rate))
        .def("set_translation_rate", &Class::set_translation_rate,
             py::arg("context"), py::arg("translation_dot"),
             D(PrismaticJoint, set_translation_rate));
  }

  {
    using Class = RevoluteJoint<T>;
    py::class_<Class, Joint<T>> cls(m, "RevoluteJoint");
    cls
        .def(py::init<const string&, const Frame<T>&,
             const Frame<T>&, const Vector3<T>&, double>(),
             py::arg("name"), py::arg("frame_on_parent"),
             py::arg("frame_on_child"), py::arg("axis"),
             py::arg("damping") = 0,
             D(RevoluteJoint, RevoluteJoint, 4))
        .def("get_angle", &Class::get_angle, py::arg("context"),
             D(RevoluteJoint, get_angle))
        .def("set_angle", &Class::set_angle, py::arg("context"),
             py::arg("angle"), D(RevoluteJoint, set_angle));
  }

  {
    using Class = WeldJoint<T>;
    py::class_<Class, Joint<T>> cls(m, "WeldJoint");
    cls
        .def(py::init<const string&, const Frame<T>&,
             const Frame<T>&, const Isometry3<double>&>(),
             py::arg("name"), py::arg("parent_frame_P"),
             py::arg("child_frame_C"), py::arg("X_PC"),
             D(WeldJoint, WeldJoint, 3));
  }

  // Actuators.
  {
    using Class = JointActuator<T>;
    py::class_<Class> cls(m, "JointActuator");
    BindMultibodyTreeElementMixin(&cls);
    cls
        .def("name", &Class::name, D(JointActuator, name))
        .def("joint", &Class::joint, py_reference_internal,
             D(JointActuator, joint));
  }

  // Force Elements.
  {
    using Class = ForceElement<T>;
    py::class_<Class> cls(m, "ForceElement", D(ForceElement));
    BindMultibodyTreeElementMixin(&cls);
  }

  {
    using Class = UniformGravityFieldElement<T>;
    py::class_<Class, ForceElement<T>>(m, "UniformGravityFieldElement")
        .def(py::init<Vector3<double>>(), py::arg("g_W"),
             D(UniformGravityFieldElement, UniformGravityFieldElement));
  }

  // MultibodyForces
  {
    using Class = MultibodyForces<T>;
    py::class_<Class> cls(m, "MultibodyForces", D(MultibodyForces));
    cls
        .def(py::init<MultibodyTree<double>&>(), py::arg("model"),
             D(MultibodyForces, MultibodyForces));
  }

  // Tree.
  {
    // N.B. Pending a concrete direction on #9366, a minimal subset of the
    // `MultibodyTree` API will be exposed.
    using Class = MultibodyTree<T>;
    py::class_<Class>(m, "MultibodyTree")
        .def("CalcRelativeTransform", &Class::CalcRelativeTransform,
             py::arg("context"), py::arg("frame_A"), py::arg("frame_B"),
             D(MultibodyTree, CalcRelativeTransform)
        )
        .def("get_multibody_state_vector",
             [](const MultibodyTree<T>* self,
                const Context<T>& context) -> Eigen::Ref<const VectorX<T>> {
               return self->get_multibody_state_vector(context);
             },
             py_reference,
             // Keep alive, ownership: `return` keeps `Context` alive.
             py::keep_alive<0, 2>(), py::arg("context"),
             D(MultibodyTree, get_multibody_state_vector))
        .def("get_mutable_multibody_state_vector",
             [](const MultibodyTree<T>* self,
                Context<T>* context) -> Eigen::Ref<VectorX<T>> {
               return self->get_mutable_multibody_state_vector(context);
             },
             py_reference,
             // Keep alive, ownership: `return` keeps `Context` alive.
             py::keep_alive<0, 2>(), py::arg("context"),
             D(MultibodyTree, get_mutable_multibody_state_vector)
        )
        .def(
            "CalcPointsPositions",
            [](
                const Class* self,
                const Context<T>& context, const Frame<T>& frame_B,
                const Eigen::Ref<const MatrixX<T>>& p_BQi,
                const Frame<T>& frame_A) {
              MatrixX<T> p_AQi(p_BQi.rows(), p_BQi.cols());
              self->CalcPointsPositions(
                  context, frame_B, p_BQi, frame_A, &p_AQi);
              return p_AQi;
            },
            py::arg("context"), py::arg("frame_B"), py::arg("p_BQi"),
            py::arg("frame_A"),
            D(MultibodyTree, CalcPointsPositions)
        )
        .def(
            "CalcFrameGeometricJacobianExpressedInWorld",
            [](
                const Class* self,
                const Context<T>& context,
                const Frame<T>& frame_B, const Vector3<T>& p_BoFo_B) {
              MatrixX<T> Jv_WF(6, self->num_velocities());
              self->CalcFrameGeometricJacobianExpressedInWorld(
                  context, frame_B, p_BoFo_B, &Jv_WF);
              return Jv_WF;
            },
            py::arg("context"), py::arg("frame_B"),
            py::arg("p_BoFo_B") = Vector3<T>::Zero().eval(),
            D(MultibodyTree, CalcFrameGeometricJacobianExpressedInWorld))
        .def("CalcInverseDynamics",
            overload_cast_explicit<VectorX<T>,
                                  const Context<T>&,
                                  const VectorX<T>&,
                                  const MultibodyForces<T>&>(
               &Class::CalcInverseDynamics),
            py::arg("context"), py::arg("known_vdot"),
            py::arg("external_forces"),
            D(MultibodyTree, CalcInverseDynamics))
        .def("SetFreeBodyPoseOrThrow",
            overload_cast_explicit<void, const Body<T>&, const Isometry3<T>&,
            systems::Context<T>*>(&Class::SetFreeBodyPoseOrThrow),
            py::arg("body"), py::arg("X_WB"), py::arg("context"),
            D(MultibodyTree, SetFreeBodyPoseOrThrow))
        .def("get_positions_from_array",
            &Class::get_positions_from_array,
            py::arg("model_instance"), py::arg("q_array"),
            D(MultibodyTree, get_positions_from_array)
        )
        .def("get_velocities_from_array",
            &Class::get_velocities_from_array,
            py::arg("model_instance"), py::arg("v_array"),
            D(MultibodyTree, get_velocities_from_array));
  }
}

void init_math(py::module m) {
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::multibody;

  m.doc() = "MultibodyTree math functionality.";

  py::class_<SpatialVector<SpatialVelocity, T>>(m, "SpatialVector")
      .def("rotational",
           [](const SpatialVector<SpatialVelocity, T>* self)
               -> const Vector3<T>& { return self->rotational(); },
           py_reference_internal, D(SpatialVector, rotational))
      .def("translational",
           [](const SpatialVector<SpatialVelocity, T>* self)
               -> const Vector3<T>& { return self->translational(); },
           py_reference_internal, D(SpatialVector, translational));

  py::class_<SpatialVelocity<T>, SpatialVector<SpatialVelocity, T>>(
      m, "SpatialVelocity")
      .def(py::init(), D(SpatialVelocity, SpatialVelocity))
      .def(py::init<const Eigen::Ref<const Vector3<T>>&,
                    const Eigen::Ref<const Vector3<T>>&>(),
           py::arg("w"), py::arg("v"), D(SpatialVelocity, SpatialVelocity, 4));
}

void init_multibody_plant(py::module m) {
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::multibody;
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::multibody::multibody_plant;

  py::module::import("pydrake.geometry");
  py::module::import("pydrake.systems.framework");

  {
    using Class = MultibodyPlant<T>;
    py::class_<Class, systems::LeafSystem<T>> cls(m, "MultibodyPlant");
    // N.B. These are defined as they appear in the class declaration.
    // TODO(eric.cousineau): Add model-instance based overloads beyond
    // forwarded methods.
    // Forwarded methods from `MultibodyTree`.
    cls
        .def(py::init<double>(),
             py::arg("time_step") = 0.)
        .def("num_bodies", &Class::num_bodies,
             D(multibody_plant, MultibodyPlant, num_bodies))
        .def("num_joints", &Class::num_joints,
             D(multibody_plant, MultibodyPlant, num_joints))
        .def("num_actuators", &Class::num_actuators,
             D(multibody_plant, MultibodyPlant, num_actuators))
        .def("num_model_instances", &Class::num_model_instances,
             D(multibody_plant, MultibodyPlant, num_model_instances))
        .def("num_positions",
             overload_cast_explicit<int>(&Class::num_positions),
             D(multibody_plant, MultibodyPlant, num_positions))
        .def("num_positions",
             overload_cast_explicit<int, ModelInstanceIndex>(
                &Class::num_positions),
             py::arg("model_instance"),
             D(multibody_plant, MultibodyPlant, num_positions, 2))
        .def("num_velocities",
             overload_cast_explicit<int>(&Class::num_velocities),
             D(multibody_plant, MultibodyPlant, num_velocities))
        .def("num_velocities",
             overload_cast_explicit<int, ModelInstanceIndex>(
                 &Class::num_velocities),
             D(multibody_plant, MultibodyPlant, num_velocities, 2))
        .def("num_multibody_states", &Class::num_multibody_states,
             D(multibody_plant, MultibodyPlant, num_multibody_states))
        .def("num_actuated_dofs",
             overload_cast_explicit<int>(&Class::num_actuated_dofs),
             D(multibody_plant, MultibodyPlant, num_actuated_dofs));
    // Construction.
    cls
        .def("AddJoint",
             [](Class* self, std::unique_ptr<Joint<T>> joint) -> auto& {
               return self->AddJoint(std::move(joint));
             }, py::arg("joint"), py_reference_internal,
             D(multibody_plant, MultibodyPlant, AddJoint))
        .def("WeldFrames", &Class::WeldFrames,
             py::arg("A"), py::arg("B"),
             py::arg("X_AB") = Isometry3<double>::Identity(),
             py_reference_internal,
             D(multibody_plant, MultibodyPlant, WeldFrames))
        .def("AddForceElement",
             [](Class* self,
                std::unique_ptr<ForceElement<T>> force_element) -> auto& {
               return self->AddForceElement<ForceElement>(
                   std::move(force_element));
             }, py::arg("force_element"), py_reference_internal,
             D(multibody_plant, MultibodyPlant, AddForceElement));
    // Topology queries.
    cls
        .def("HasBodyNamed",
             overload_cast_explicit<bool, const string&>(&Class::HasBodyNamed),
             py::arg("name"), D(multibody_plant, MultibodyPlant, HasBodyNamed))
        .def("HasJointNamed",
             overload_cast_explicit<bool, const string&>(
                &Class::HasJointNamed),
             py::arg("name"), D(multibody_plant, MultibodyPlant,
                                HasJointNamed))
        .def("GetFrameByName",
             overload_cast_explicit<const Frame<T>&, const string&>(
                 &Class::GetFrameByName),
             py::arg("name"), py_reference_internal,
             D(multibody_plant, MultibodyPlant, GetFrameByName))
        .def("GetFrameByName",
             overload_cast_explicit<const Frame<T>&, const string&,
                                    ModelInstanceIndex>(
                 &Class::GetFrameByName),
             py::arg("name"), py::arg("model_instance"), py_reference_internal,
             D(multibody_plant, MultibodyPlant, GetFrameByName, 2))
        .def("GetBodyByName",
             overload_cast_explicit<const Body<T>&, const string&>(
                &Class::GetBodyByName),
             py::arg("name"), py_reference_internal,
             D(multibody_plant, MultibodyPlant, GetBodyByName))
        .def("GetBodyByName",
             overload_cast_explicit<const Body<T>&, const string&,
                                    ModelInstanceIndex>(
                &Class::GetBodyByName),
             py::arg("name"), py::arg("model_instance"), py_reference_internal,
             D(multibody_plant, MultibodyPlant, GetBodyByName, 2))
        .def("GetJointByName",
             [](const Class* self, const string& name) -> auto& {
               return self->GetJointByName(name);
             },
             py::arg("name"), py_reference_internal,
             D(multibody_plant, MultibodyPlant, GetJointByName))
        .def("GetJointActuatorByName",
             overload_cast_explicit<const JointActuator<T>&, const string&>(
                &Class::GetJointActuatorByName),
             py::arg("name"), py_reference_internal,
             D(multibody_plant, MultibodyPlant, GetJointActuatorByName));
    // Geometry.
    cls
        .def("get_source_id", &Class::get_source_id,
             D(multibody_plant, MultibodyPlant, get_source_id))
        .def("get_geometry_query_input_port",
             &Class::get_geometry_query_input_port, py_reference_internal,
             D(multibody_plant, MultibodyPlant, get_geometry_query_input_port))
        .def("get_geometry_poses_output_port",
             &Class::get_geometry_poses_output_port, py_reference_internal,
             D(multibody_plant, MultibodyPlant, get_geometry_query_input_port))
        .def("geometry_source_is_registered",
             &Class::geometry_source_is_registered,
             D(multibody_plant, MultibodyPlant,
               geometry_source_is_registered));
    // Port accessors.
    cls
        .def("get_actuation_input_port",
             overload_cast_explicit<const systems::InputPort<T>&>(
                &Class::get_actuation_input_port),
             py_reference_internal,
             D(multibody_plant, MultibodyPlant, get_actuation_input_port))
        .def("get_continuous_state_output_port",
             overload_cast_explicit<const systems::OutputPort<T>&>(
                &Class::get_continuous_state_output_port),
             py_reference_internal,
             D(multibody_plant, MultibodyPlant,
               get_continuous_state_output_port))
        .def("get_contact_results_output_port",
             overload_cast_explicit<const systems::OutputPort<T>&>(
                &Class::get_contact_results_output_port),
             py_reference_internal,
             D(multibody_plant, MultibodyPlant,
               get_contact_results_output_port));
    // Property accessors.
    cls
        .def("world_body", &Class::world_body, py_reference_internal,
             D(multibody_plant, MultibodyPlant, world_body))
        .def("world_frame", &Class::world_frame, py_reference_internal,
             D(multibody_plant, MultibodyPlant, world_frame))
        .def("tree", &Class::tree, py_reference_internal,
             D(multibody_plant, MultibodyPlant, tree))
        .def("is_finalized", &Class::is_finalized,
             D(multibody_plant, MultibodyPlant, is_finalized))
        .def("Finalize", py::overload_cast<SceneGraph<T>*>(&Class::Finalize),
             py::arg("scene_graph") = nullptr,
             D(multibody_plant, MultibodyPlant, Finalize));
    // Add deprecated methods.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
    cls.def("model", &Class::model, py_reference_internal);
#pragma GCC diagnostic pop  // pop -Wdeprecated-declarations
    cls.attr("message_model") = "Please use tree().";
    DeprecateAttribute(
        cls, "model", cls.attr("message_model"));
  }
}

void init_parsing(py::module m) {
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::multibody;
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::multibody::parsing;

  using multibody_plant::MultibodyPlant;

  m.def("AddModelFromSdfFile",
        py::overload_cast<
            const string&, const string&, MultibodyPlant<T>*, SceneGraph<T>*>(
            &AddModelFromSdfFile),
        py::arg("file_name"), py::arg("model_name"), py::arg("plant"),
        py::arg("scene_graph") = nullptr,
        D(parsing, AddModelFromSdfFile));
  m.def("AddModelFromSdfFile",
        py::overload_cast<
            const string&, MultibodyPlant<T>*, SceneGraph<T>*>(
            &AddModelFromSdfFile),
        py::arg("file_name"), py::arg("plant"),
        py::arg("scene_graph") = nullptr,
        D(parsing, AddModelFromSdfFile, 2));
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
