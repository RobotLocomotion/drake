#include "drake/bindings/pydrake/visualization/visualization_py.h"

#include "drake/bindings/pydrake/common/default_scalars_pybind.h"
#include "drake/bindings/pydrake/documentation_pybind.h"
#include "drake/visualization/meshcat_pose_sliders.h"

namespace drake {
namespace pydrake {
namespace {

template <typename T>
void DoScalarDependentDefinitions(py::module m, T) {
  py::tuple param = GetPyParam<T>();

  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::visualization;
  constexpr auto& doc = pydrake_doc.drake.visualization;

  // MeshcatPoseSliders
  {
    using Class = MeshcatPoseSliders<T>;
    constexpr auto& cls_doc = doc.MeshcatPoseSliders;
    DefineTemplateClassWithDefault<MeshcatPoseSliders<T>,
        systems::LeafSystem<T>>(
        m, "MeshcatPoseSliders", param, doc.MeshcatPoseSliders.doc)
        .def(
            py::init<std::shared_ptr<geometry::Meshcat>,
                const math::RigidTransformd&, const Eigen::Ref<const Vector6d>&,
                const Eigen::Ref<const Vector6d>&,
                const Eigen::Ref<const Vector6d>&, std::vector<std::string>,
                std::vector<std::string>, std::string,
                const Eigen::Ref<const Vector6<bool>>&>(),
            py::arg("meshcat"),
            py::arg("initial_pose") = math::RigidTransformd(),
            py::arg("lower_limit") =
                (Vector6d() << -M_PI, -M_PI, -M_PI, -1, -1, -1).finished(),
            py::arg("upper_limit") =
                (Vector6d() << M_PI, M_PI, M_PI, 1, 1, 1).finished(),
            py::arg("step") = Vector6d::Constant(0.01),
            py::arg("decrement_keycodes") = std::vector<std::string>(
                {"KeyQ", "KeyS", "KeyA", "KeyJ", "KeyK", "KeyU"}),
            py::arg("increment_keycodes") = std::vector<std::string>(
                {"KeyE", "KeyW", "KeyD", "KeyL", "KeyI", "KeyO"}),
            py::arg("prefix") = "",
            py::arg("visible") = Vector6<bool>::Constant(true),
            cls_doc.ctor.doc)
        .def("Delete", &Class::Delete, cls_doc.Delete.doc)
        .def("Run", &Class::Run, py::arg("system"), py::arg("context"),
            py::arg("timeout") = py::none(),
            py::arg("stop_button_keycode") = "Escape", cls_doc.Run.doc)
        .def("SetPose", &Class::SetPose, py::arg("pose"), cls_doc.SetPose.doc);
  }
}

}  // namespace

PYBIND11_MODULE(visualization, m) {
  PYDRAKE_PREVENT_PYTHON3_MODULE_REIMPORT(m);

  m.doc() = R"""(
Bindings for Visualization.
)""";

  py::module::import("pydrake.geometry");
  py::module::import("pydrake.multibody");
  py::module::import("pydrake.systems");

  // The order of these calls matters. Some modules rely on prior definitions.
  internal::DefineVisualizationConfig(m);

  type_visit([m](auto dummy) { DoScalarDependentDefinitions(m, dummy); },
      NonSymbolicScalarPack{});

  py::module::import("pydrake.visualization.meldis");
  py::module::import("pydrake.visualization.model_visualizer");
  ExecuteExtraPythonCode(m, true);
}

}  // namespace pydrake
}  // namespace drake
