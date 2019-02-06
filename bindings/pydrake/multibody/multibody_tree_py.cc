/// @file
/// Alias and bind deprecated symbols from old `//multibody/multibody_tree`
/// package.

#include "pybind11/eigen.h"
#include "pybind11/eval.h"
#include "pybind11/pybind11.h"

#include "drake/bindings/pydrake/common/deprecation_pybind.h"
#include "drake/bindings/pydrake/common/drake_optional_pybind.h"
#include "drake/bindings/pydrake/common/eigen_geometry_pybind.h"
#include "drake/bindings/pydrake/common/type_safe_index_pybind.h"
#include "drake/bindings/pydrake/documentation_pybind.h"
#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/multibody/parsing/sdf_parser.h"
#include "drake/multibody/tree/multibody_tree.h"
#include "drake/multibody/tree/multibody_tree_indexes.h"

namespace drake {
namespace pydrake {
namespace {

using std::string;

using geometry::SceneGraph;
using systems::Context;

// TODO(eric.cousineau): Expose available scalar types.
using T = double;

void ForwardSymbols(py::module from, py::module m) {
  py::dict vars = m.attr("__dict__");
  py::exec(py::str("from {} import *").format(from.attr("__name__")),
      py::globals(), vars);

  py::str deprecation = py::str(
      "``{}`` is deprecated and will be removed on or around "
      "2019-05-01. Please use ``{}`` instead")
                            .format(m.attr("__name__"), from.attr("__name__"));
  WarnDeprecated(deprecation);
  m.doc() = py::str("Warning:\n    {}").format(deprecation);
}

// Bind deprecated symbols.
void init_module(py::module m) {
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::multibody;
  constexpr auto& doc = pydrake_doc.drake.multibody;

  ForwardSymbols(py::module::import("pydrake.multibody.tree"), m);

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
  {
    auto cls = BindTypeSafeIndex<MobilizerIndex>(
        m, "MobilizerIndex", doc.internal.MobilizerIndex.doc);
    DeprecateAttribute(
        cls, "__init__", "`MobilizerIndex` will soon be internal.");
  }
  {
    auto cls = BindTypeSafeIndex<BodyNodeIndex>(
        m, "BodyNodeIndex", doc.internal.BodyNodeIndex.doc);
    DeprecateAttribute(
        cls, "__init__", "`BodyNodeIndex` will soon be internal.");
  }
#pragma GCC diagnostic pop

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
  // Tree.
  {
    // N.B. MultibodyTree will disappear from public API (#9366).
    // Do NOT add or change ANY methods or constructors.
    using Class = MultibodyTree<T>;
    constexpr auto& cls_doc = doc.internal.MultibodyTree;
    py::class_<Class> cls(m, "MultibodyTree", cls_doc.doc);
    // N.B. Since `MultibodyTree` is only reachable from Python via
    // `MultibodyPlant.tree`, we should not need any deprecation messages.
    cls  // BR
        .def("CalcRelativeTransform", &Class::CalcRelativeTransform,
            py::arg("context"), py::arg("frame_A"), py::arg("frame_B"),
            cls_doc.CalcRelativeTransform.doc)
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
        .def("GetModelInstanceName",
            overload_cast_explicit<const string&, ModelInstanceIndex>(
                &Class::GetModelInstanceName),
            py::arg("model_instance"), py_reference_internal,
            cls_doc.GetModelInstanceName.doc)
        .def("GetPositionsAndVelocities",
            [](const MultibodyTree<T>* self,
                const Context<T>& context) -> Eigen::Ref<const VectorX<T>> {
              return self->GetPositionsAndVelocities(context);
            },
            py_reference,
            // Keep alive, ownership: `return` keeps `Context` alive.
            py::keep_alive<0, 2>(), py::arg("context"),
            cls_doc.GetPositionsAndVelocities.doc_1args)
        .def("GetMutablePositionsAndVelocities",
            [](const MultibodyTree<T>* self,
                Context<T>* context) -> Eigen::Ref<VectorX<T>> {
              return self->GetMutablePositionsAndVelocities(context);
            },
            py_reference,
            // Keep alive, ownership: `return` keeps `Context` alive.
            py::keep_alive<0, 2>(), py::arg("context"),
            cls_doc.GetMutablePositionsAndVelocities.doc_1args)
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
            py::arg("frame_A"), cls_doc.CalcPointsPositions.doc)
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
            cls_doc.CalcFrameGeometricJacobianExpressedInWorld.doc)
        .def("CalcInverseDynamics",
            overload_cast_explicit<VectorX<T>, const Context<T>&,
                const VectorX<T>&, const MultibodyForces<T>&>(
                &Class::CalcInverseDynamics),
            py::arg("context"), py::arg("known_vdot"),
            py::arg("external_forces"), cls_doc.CalcInverseDynamics.doc_3args)
        .def("SetFreeBodyPoseOrThrow",
            overload_cast_explicit<void, const Body<T>&, const Isometry3<T>&,
                Context<T>*>(&Class::SetFreeBodyPoseOrThrow),
            py::arg("body"), py::arg("X_WB"), py::arg("context"),
            cls_doc.SetFreeBodyPoseOrThrow.doc)
        .def("GetPositionsFromArray", &Class::GetPositionsFromArray,
            py::arg("model_instance"), py::arg("q_array"),
            cls_doc.get_positions_from_array.doc)
        .def("GetVelocitiesFromArray", &Class::GetVelocitiesFromArray,
            py::arg("model_instance"), py::arg("v_array"),
            cls_doc.get_velocities_from_array.doc)
        .def("SetFreeBodySpatialVelocityOrThrow",
            [](const Class* self, const Body<T>& body,
                const SpatialVelocity<T>& V_WB, Context<T>* context) {
              self->SetFreeBodySpatialVelocityOrThrow(body, V_WB, context);
            },
            py::arg("body"), py::arg("V_WB"), py::arg("context"),
            cls_doc.SetFreeBodySpatialVelocityOrThrow.doc)
        .def("CalcAllBodySpatialVelocitiesInWorld",
            [](const Class* self, const Context<T>& context) {
              std::vector<SpatialVelocity<T>> V_WB;
              self->CalcAllBodySpatialVelocitiesInWorld(context, &V_WB);
              return V_WB;
            },
            py::arg("context"), cls_doc.CalcAllBodySpatialVelocitiesInWorld.doc)
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
        .def("CalcAllBodyPosesInWorld",
            [](const Class* self, const Context<T>& context) {
              std::vector<Isometry3<T>> X_WB;
              self->CalcAllBodyPosesInWorld(context, &X_WB);
              return X_WB;
            },
            py::arg("context"), cls_doc.CalcAllBodyPosesInWorld.doc)
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
            py::arg("context"), cls_doc.CalcBiasTerm.doc);
  }
#pragma GCC diagnostic pop
}

void init_math(py::module m) {
  ForwardSymbols(py::module::import("pydrake.multibody.math"), m);
}

void init_multibody_plant(py::module m) {
  ForwardSymbols(py::module::import("pydrake.multibody.plant"), m);
}

void init_parsing_deprecated(py::module m) {
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::multibody;
  constexpr auto& doc = pydrake_doc.drake.multibody.parsing;

  m.doc() =
      "Multibody parsing functionality.\n\n"
      "Warning:\n   This module is deprecated, and will be removed on or "
      "around 2019-05-01. Please use ``pydrake.multibody.parsing`` instead.";

  // N.B. This module is deprecated; add all new methods to `parsing_py.cc`.

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
      py::arg("scene_graph") = nullptr, doc.AddModelFromSdfFile.doc);
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
      doc.AddModelFromSdfFile.doc);
}

void init_all(py::module m) {
  py::dict vars = m.attr("__dict__");
  py::exec(
      "from pydrake.multibody.multibody_tree import *\n"
      "from pydrake.multibody.multibody_tree.math import *\n"
      "from pydrake.multibody.multibody_tree.multibody_plant import *\n"
      "from pydrake.multibody.multibody_tree.parsing import *\n",
      py::globals(), vars);
  // N.B. Deprecation will have been emitted by `multibody_tree` module already.
  m.doc() =
      "Warning:\n   ``pydrake.multibody.multibody_tree.all`` is deprecated "
      "and will be removed on or around 2019-05-01.";
}

}  // namespace

PYBIND11_MODULE(multibody_tree, m) {
  PYDRAKE_PREVENT_PYTHON3_MODULE_REIMPORT(m);

  init_module(m);
  init_math(m.def_submodule("math"));
  init_multibody_plant(m.def_submodule("multibody_plant"));
  init_parsing_deprecated(m.def_submodule("parsing"));
  init_all(m.def_submodule("all"));
}

}  // namespace pydrake
}  // namespace drake
