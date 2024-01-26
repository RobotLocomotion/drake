#include "drake/bindings/pydrake/common/cpp_template_pybind.h"
#include "drake/bindings/pydrake/common/default_scalars_pybind.h"
#include "drake/bindings/pydrake/common/serialize_pybind.h"
#include "drake/bindings/pydrake/common/type_pack.h"
#include "drake/bindings/pydrake/documentation_pybind.h"
#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/multibody/meshcat/contact_visualizer.h"
#include "drake/multibody/meshcat/contact_visualizer_params.h"
#include "drake/multibody/meshcat/hydroelastic_contact_visualizer.h"
#include "drake/multibody/meshcat/joint_sliders.h"
#include "drake/multibody/meshcat/point_contact_visualizer.h"

using drake::multibody::MultibodyPlant;
using drake::systems::LeafSystem;

namespace drake {
namespace pydrake {
namespace {

void DoScalarIndependentDefinitions(py::module m) {
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::multibody::meshcat;
  constexpr auto& doc = pydrake_doc.drake.multibody.meshcat;

  // ContactVisualizerParams
  {
    using Class = ContactVisualizerParams;
    constexpr auto& cls_doc = doc.ContactVisualizerParams;
    py::class_<Class> cls(
        m, "ContactVisualizerParams", py::dynamic_attr(), cls_doc.doc);
    cls  // BR
        .def(ParamInit<Class>());
    DefAttributesUsingSerialize(&cls, cls_doc);
    DefReprUsingSerialize(&cls);
    DefCopyAndDeepCopy(&cls);
  }

  // PointContactVisualizerItem (internal)
  {
    using Class = multibody::meshcat::internal::PointContactVisualizerItem;
    constexpr char doc_internal[] = "(internal use only)";
    py::class_<Class>(
        m, "_PointContactVisualizerItem", py::dynamic_attr(), doc_internal)
        .def(py::init<std::string, std::string, Eigen::Vector3d,
                 Eigen::Vector3d>(),
            py::arg("body_A"), py::arg("body_B"), py::arg("contact_force"),
            py::arg("contact_point"), doc_internal)
        .def_readwrite("body_A", &Class::body_A, doc_internal)
        .def_readwrite("body_B", &Class::body_B, doc_internal)
        .def_readwrite("contact_force", &Class::contact_force, doc_internal)
        .def_readwrite("contact_point", &Class::contact_point, doc_internal);
  }

  // PointContactVisualizer (internal)
  {
    using Class = multibody::meshcat::internal::PointContactVisualizer;
    constexpr char doc_internal[] = "(internal use only)";
    py::class_<Class>(m, "_PointContactVisualizer", doc_internal)
        .def(py::init<std::shared_ptr<geometry::Meshcat>,
                 ContactVisualizerParams>(),
            py::arg("meshcat"), py::arg("params"), doc_internal)
        .def("Update", &Class::Update, py::arg("time"), py::arg("items"));
  }

  // HydroelasticContactVisualizerItem (internal)
  {
    using Class =
        multibody::meshcat::internal::HydroelasticContactVisualizerItem;
    constexpr char doc_internal[] = "(internal use only)";
    py::class_<Class>(m, "_HydroelasticContactVisualizerItem",
        py::dynamic_attr(), doc_internal)
        .def(py::init<std::string, std::string, Eigen::Vector3d,
                 Eigen::Vector3d, Eigen::Vector3d, Eigen::Matrix3Xd,
                 Eigen::Matrix3Xi, Eigen::VectorXd>(),
            py::arg("body_A"), py::arg("body_B"), py::arg("centroid_W"),
            py::arg("force_C_W"), py::arg("moment_C_W"), py::arg("p_WV"),
            py::arg("faces"), py::arg("pressure"), doc_internal)
        .def_readwrite("body_A", &Class::body_A, doc_internal)
        .def_readwrite("body_B", &Class::body_B, doc_internal)
        .def_readwrite("centroid_W", &Class::centroid_W, doc_internal)
        .def_readwrite("force_C_W", &Class::force_C_W, doc_internal)
        .def_readwrite("moment_C_W", &Class::moment_C_W, doc_internal)
        .def_readwrite("p_WV", &Class::p_WV, doc_internal)
        .def_readwrite("faces", &Class::faces, doc_internal)
        .def_readwrite("pressure", &Class::pressure, doc_internal);
  }

  // HydroelasticContactVisualizer (internal)
  {
    using Class = multibody::meshcat::internal::HydroelasticContactVisualizer;
    constexpr char doc_internal[] = "(internal use only)";
    py::class_<Class>(m, "_HydroelasticContactVisualizer", doc_internal)
        .def(py::init<std::shared_ptr<geometry::Meshcat>,
                 ContactVisualizerParams>(),
            py::arg("meshcat"), py::arg("params"), doc_internal)
        .def("Update", &Class::Update, py::arg("time"), py::arg("items"))
        .def("Delete", &Class::Delete);
  }
}

template <typename T>
void DoScalarDependentDefinitions(py::module m, T) {
  py::tuple param = GetPyParam<T>();

  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::multibody::meshcat;
  constexpr auto& doc = pydrake_doc.drake.multibody.meshcat;

  // ContactVisualizer
  {
    using Class = ContactVisualizer<T>;
    constexpr auto& cls_doc = doc.ContactVisualizer;
    auto cls = DefineTemplateClassWithDefault<Class, systems::LeafSystem<T>>(
        m, "ContactVisualizer", param, cls_doc.doc);
    cls  // BR
        .def(py::init<std::shared_ptr<geometry::Meshcat>,
                 ContactVisualizerParams>(),
            py::arg("meshcat"), py::arg("params") = ContactVisualizerParams{},
            // `meshcat` is a shared_ptr, so does not need a keep_alive.
            cls_doc.ctor.doc)
        .def("Delete", &Class::Delete, cls_doc.Delete.doc)
        .def("contact_results_input_port", &Class::contact_results_input_port,
            py_rvp::reference_internal, cls_doc.contact_results_input_port.doc)
        .def("query_object_input_port", &Class::query_object_input_port,
            py_rvp::reference_internal, cls_doc.query_object_input_port.doc)
        .def_static("AddToBuilder",
            py::overload_cast<systems::DiagramBuilder<T>*,
                const MultibodyPlant<T>&, std::shared_ptr<geometry::Meshcat>,
                ContactVisualizerParams>(&ContactVisualizer<T>::AddToBuilder),
            py::arg("builder"), py::arg("plant"), py::arg("meshcat"),
            py::arg("params") = ContactVisualizerParams{},
            // Keep alive, ownership: `return` keeps `builder` alive.
            py::keep_alive<0, 1>(),
            // `meshcat` is a shared_ptr, so does not need a keep_alive.
            py_rvp::reference,
            cls_doc.AddToBuilder.doc_4args_builder_plant_meshcat_params)
        .def_static("AddToBuilder",
            py::overload_cast<systems::DiagramBuilder<T>*,
                const systems::OutputPort<T>&, const systems::OutputPort<T>&,
                std::shared_ptr<geometry::Meshcat>, ContactVisualizerParams>(
                &ContactVisualizer<T>::AddToBuilder),
            py::arg("builder"), py::arg("contact_results_port"),
            py::arg("query_object_port"), py::arg("meshcat"),
            py::arg("params") = ContactVisualizerParams{},
            // Keep alive, ownership: `return` keeps `builder` alive.
            py::keep_alive<0, 1>(),
            // `meshcat` is a shared_ptr, so does not need a keep_alive.
            py_rvp::reference,
            cls_doc.AddToBuilder
                .doc_5args_builder_contact_results_port_query_object_port_meshcat_params)
        .def_static("AddToBuilder",
            py::overload_cast<systems::DiagramBuilder<T>*,
                const systems::OutputPort<T>&,
                std::shared_ptr<geometry::Meshcat>, ContactVisualizerParams>(
                &ContactVisualizer<T>::AddToBuilder),
            py::arg("builder"), py::arg("contact_results_port"),
            py::arg("meshcat"), py::arg("params") = ContactVisualizerParams{},
            // Keep alive, ownership: `return` keeps `builder` alive.
            py::keep_alive<0, 1>(),
            // `meshcat` is a shared_ptr, so does not need a keep_alive.
            py_rvp::reference,
            cls_doc.AddToBuilder
                .doc_4args_builder_contact_results_port_meshcat_params);
  }

  // JointSliders
  {
    using Class = JointSliders<T>;
    constexpr auto& cls_doc = doc.JointSliders;
    DefineTemplateClassWithDefault<JointSliders<T>, LeafSystem<T>>(
        m, "JointSliders", param, doc.JointSliders.doc)
        .def(py::init<std::shared_ptr<geometry::Meshcat>,
                 const MultibodyPlant<T>*, std::optional<Eigen::VectorXd>,
                 std::variant<std::monostate, double, Eigen::VectorXd>,
                 std::variant<std::monostate, double, Eigen::VectorXd>,
                 std::variant<std::monostate, double, Eigen::VectorXd>,
                 std::vector<std::string>, std::vector<std::string>>(),
            py::arg("meshcat"), py::arg("plant"),
            py::arg("initial_value") = py::none(),
            py::arg("lower_limit") = py::none(),
            py::arg("upper_limit") = py::none(), py::arg("step") = py::none(),
            py::arg("decrement_keycodes") = std::vector<std::string>(),
            py::arg("increment_keycodes") = std::vector<std::string>(),
            // Keep alive, reference: `self` keeps `plant` alive.
            py::keep_alive<1, 3>(),  // BR
            cls_doc.ctor.doc)
        .def("Delete", &Class::Delete, cls_doc.Delete.doc)
        .def("Run", &Class::Run, py::arg("diagram"),
            py::arg("timeout") = py::none(),
            py::arg("stop_button_keycode") = "Escape", cls_doc.Run.doc)
        .def("SetPositions", &Class::SetPositions, py::arg("q"),
            cls_doc.SetPositions.doc);
  }
}
}  // namespace

PYBIND11_MODULE(meshcat, m) {
  PYDRAKE_PREVENT_PYTHON3_MODULE_REIMPORT(m);
  m.doc() = "Interface code for Meshcat-based visualization";

  py::module::import("pydrake.multibody.plant");

  DoScalarIndependentDefinitions(m);
  type_visit([m](auto dummy) { DoScalarDependentDefinitions(m, dummy); },
      NonSymbolicScalarPack{});
}

}  // namespace pydrake
}  // namespace drake
