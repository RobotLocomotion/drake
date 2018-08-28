#include "pybind11/eigen.h"
#include "pybind11/eval.h"
#include "pybind11/pybind11.h"

#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/bindings/pydrake/util/eigen_geometry_pybind.h"
#include "drake/bindings/pydrake/util/type_safe_index_pybind.h"
#include "drake/multibody/multibody_tree/math/spatial_force.h"
#include "drake/multibody/multibody_tree/math/spatial_vector.h"
#include "drake/multibody/multibody_tree/math/spatial_velocity.h"
#include "drake/multibody/multibody_tree/multibody_plant/multibody_plant.h"
#include "drake/multibody/multibody_tree/multibody_tree.h"
#include "drake/multibody/multibody_tree/parsing/multibody_plant_sdf_parser.h"

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
  m.def("world_index", &world_index);

  {
    using Class = Frame<T>;
    py::class_<Class> cls(m, "Frame");
    BindMultibodyTreeElementMixin(&cls);
    cls
        .def("body", &Class::body, py_reference_internal);
  }

  {
    using Class = BodyFrame<T>;
    py::class_<Class, Frame<T>> cls(m, "BodyFrame");
    // No need to re-bind element mixins from `Frame`.
  }

  {
    using Class = Body<T>;
    py::class_<Class> cls(m, "Body");
    BindMultibodyTreeElementMixin(&cls);
    cls
        .def("name", &Class::name);
  }

  {
    using Class = Joint<T>;
    py::class_<Class> cls(m, "Joint");
    BindMultibodyTreeElementMixin(&cls);
    cls
        .def("name", &Class::name)
        .def("parent_body", &Class::parent_body, py_reference_internal)
        .def("child_body", &Class::child_body, py_reference_internal)
        .def("frame_on_parent", &Class::frame_on_parent, py_reference_internal)
        .def("frame_on_child", &Class::frame_on_child, py_reference_internal)
        .def("num_dofs", &Class::num_dofs);
  }

  {
    using Class = JointActuator<T>;
    py::class_<Class> cls(m, "JointActuator");
    BindMultibodyTreeElementMixin(&cls);
    cls
        .def("name", &Class::name)
        .def("joint", &Class::joint, py_reference_internal);
  }

  {
    // N.B. We purposely do not expose much functionality, as users should
    // generally be using `MultibodyPlant`. We simply enable passing the object
    // around.
    using Class = MultibodyTree<T>;
    py::class_<Class>(m, "MultibodyTree");
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
           py_reference_internal)
      .def("translational",
           [](const SpatialVector<SpatialVelocity, T>* self)
               -> const Vector3<T>& { return self->translational(); },
           py_reference_internal);

  py::class_<SpatialVelocity<T>, SpatialVector<SpatialVelocity, T>>(
      m, "SpatialVelocity")
      .def(py::init())
      .def(py::init<const Eigen::Ref<const Vector3<T>>&,
                    const Eigen::Ref<const Vector3<T>>&>(),
           py::arg("w"), py::arg("v"));
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
        .def("num_bodies", &Class::num_bodies)
        .def("num_joints", &Class::num_joints)
        .def("num_actuators", &Class::num_actuators)
        .def("num_model_instances", &Class::num_model_instances)
        .def("num_positions",
             overload_cast_explicit<int>(&Class::num_positions))
        .def("num_positions",
             overload_cast_explicit<int, ModelInstanceIndex>(
                &Class::num_positions),
             py::arg("model_instance"))
        .def("num_velocities",
             overload_cast_explicit<int>(&Class::num_velocities))
        .def("num_velocities",
             overload_cast_explicit<int, ModelInstanceIndex>(
                &Class::num_velocities))
        .def("num_multibody_states", &Class::num_multibody_states)
        .def("num_actuated_dofs",
             overload_cast_explicit<int>(&Class::num_actuated_dofs));
    // TODO(eric.cousineau): Add construction methods, `AddRigidBody`, etc.
    // Topology queries.
    cls
        .def("HasBodyNamed",
             overload_cast_explicit<bool, const string&>(&Class::HasBodyNamed),
             py::arg("name"))
        .def("HasJointNamed",
             overload_cast_explicit<bool, const string&>(
                &Class::HasJointNamed),
             py::arg("name"))
        .def("GetBodyByName",
             overload_cast_explicit<const Body<T>&, const string&>(
                &Class::GetBodyByName),
             py::arg("name"), py_reference_internal)
        .def("GetJointByName",
             [](const Class* self, const string& name) -> auto& {
               return self->GetJointByName(name);
             },
             py::arg("name"), py_reference_internal)
        .def("GetJointActuatorByName",
             overload_cast_explicit<const JointActuator<T>&, const string&>(
                &Class::GetJointActuatorByName),
             py::arg("name"), py_reference_internal);
    // Port accessors.
    cls
        .def("get_actuation_input_port",
             overload_cast_explicit<const systems::InputPort<T>&>(
                &Class::get_actuation_input_port),
             py_reference_internal)
        .def("get_continuous_state_output_port",
             overload_cast_explicit<const systems::OutputPort<T>&>(
                &Class::get_continuous_state_output_port),
             py_reference_internal)
        .def("get_contact_results_output_port",
             overload_cast_explicit<const systems::OutputPort<T>&>(
                &Class::get_contact_results_output_port),
             py_reference_internal);
    // Property accessors.
    cls
        .def("world_body", &Class::world_body, py_reference_internal)
        .def("model", &Class::model, py_reference_internal)
        .def("is_finalized", &Class::is_finalized)
        .def("Finalize", py::overload_cast<SceneGraph<T>*>(&Class::Finalize),
             py::arg("scene_graph") = nullptr);
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
        py::arg("scene_graph") = nullptr);
  m.def("AddModelFromSdfFile",
        py::overload_cast<
            const string&, MultibodyPlant<T>*, SceneGraph<T>*>(
            &AddModelFromSdfFile),
        py::arg("file_name"), py::arg("plant"),
        py::arg("scene_graph") = nullptr);
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
