#include "drake/bindings/generated_docstrings/planning.h"
#include "drake/bindings/pydrake/common/cpp_template_pybind.h"
#include "drake/bindings/pydrake/common/default_scalars_pybind.h"
#include "drake/bindings/pydrake/common/ref_cycle_pybind.h"
#include "drake/bindings/pydrake/planning/planning_py.h"
#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/bindings/pydrake/systems/builder_life_support_pybind.h"
#include "drake/planning/robot_diagram.h"
#include "drake/planning/robot_diagram_builder.h"

namespace drake {
namespace pydrake {
namespace internal {

void DefinePlanningRobotDiagram(py::module m) {
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::planning;
  constexpr auto& doc = pydrake_doc_planning.drake.planning;

  auto bind_common_scalar_types = [&m, &doc](auto dummy) {
    using T = decltype(dummy);

    {
      using Class = RobotDiagram<T>;
      constexpr auto& cls_doc = doc.RobotDiagram;
      auto cls = DefineTemplateClassWithDefault<Class, systems::Diagram<T>>(
          m, "RobotDiagram", GetPyParam<T>(), cls_doc.doc);
      cls  // BR
          .def("plant", &Class::plant, py_rvp::reference_internal,
              cls_doc.plant.doc)
          .def("mutable_scene_graph", &Class::mutable_scene_graph,
              py_rvp::reference_internal, cls_doc.mutable_scene_graph.doc)
          .def("scene_graph", &Class::scene_graph, py_rvp::reference_internal,
              cls_doc.scene_graph.doc)
          .def("mutable_plant_context", &Class::mutable_plant_context,
              py::arg("root_context"), py_rvp::reference,
              // Keep alive, ownership: `return` keeps `root_context` alive.
              py::keep_alive<0, 2>(), cls_doc.mutable_plant_context.doc)
          .def("plant_context", &Class::plant_context, py::arg("root_context"),
              py_rvp::reference,
              // Keep alive, ownership: `return` keeps `root_context` alive.
              py::keep_alive<0, 2>(), cls_doc.plant_context.doc)
          .def("mutable_scene_graph_context",
              &Class::mutable_scene_graph_context, py::arg("root_context"),
              py_rvp::reference,
              // Keep alive, ownership: `return` keeps `root_context` alive.
              py::keep_alive<0, 2>(), cls_doc.mutable_scene_graph_context.doc)
          .def("scene_graph_context", &Class::scene_graph_context,
              py::arg("root_context"), py_rvp::reference,
              // Keep alive, ownership: `return` keeps `root_context` alive.
              py::keep_alive<0, 2>(), cls_doc.scene_graph_context.doc);
    }

    {
      using internal::BuilderLifeSupport;
      using Class = RobotDiagramBuilder<T>;
      constexpr auto& cls_doc = doc.RobotDiagramBuilder;
      auto cls = DefineTemplateClassWithDefault<Class>(
          m, "RobotDiagramBuilder", GetPyParam<T>(), cls_doc.doc);
      cls  // BR
          .def(py::init<double>(), py::arg("time_step") = 0.001,
              cls_doc.ctor.doc)
          .def("builder",
              overload_cast_explicit<systems::DiagramBuilder<T>&>(
                  &Class::builder),
              py_rvp::reference_internal, cls_doc.builder.doc_0args_nonconst);
      if constexpr (std::is_same_v<T, double>) {
        cls  // BR
            .def(
                "parser", [](Class& self) { return &self.parser(); },
                py_rvp::reference_internal, cls_doc.parser.doc_0args_nonconst);
      }
      cls  // BR
          .def("plant",
              overload_cast_explicit<multibody::MultibodyPlant<T>&>(
                  &Class::plant),
              py_rvp::reference_internal, cls_doc.plant.doc_0args_nonconst)
          .def("scene_graph",
              overload_cast_explicit<geometry::SceneGraph<T>&>(
                  &Class::scene_graph),
              py_rvp::reference_internal,
              cls_doc.scene_graph.doc_0args_nonconst)
          .def("IsDiagramBuilt", &Class::IsDiagramBuilt,
              cls_doc.IsDiagramBuilt.doc)
          .def(
              "Build",
              [](RobotDiagramBuilder<T>* self) {
                // Construct a ref_cycle between the DiagramBuilder python
                // wrapper and the RobotDiagram python wrapper, to maintain the
                // lifetime semantics attached to the DiagramBuilder python
                // wrapper. To avoid leaks, also abandon the builder's
                // BuilderLifeSupport. This is a very delicate dance.
                systems::DiagramBuilder<T>& cpp_builder = self->builder();
                // We *must* take a non-owning py::object *before* we delete
                // the one stashed in BuilderLifeSupport. This prevents
                // grievous errors.
                py::object py_builder =
                    py::cast(&cpp_builder, py_rvp::reference);
                // Abandon life support before Build(), after which it would
                // be too late.
                BuilderLifeSupport<T>::abandon(&cpp_builder);
                // Build the diagram.
                planning::internal::BuildResultForPython<T> result =
                    self->BuildForPython();
                // Transfer ownership of the diagram to a py::object.
                py::object py_diagram =
                    py::cast(result.diagram.release(), py_rvp::take_ownership);
                // Transfer ownership of the builder to a respun
                // py::object. Pydrake diagram/builder ref-cycle memory
                // management requires the internal builder to outlive the (now
                // irrelevant) RobotDiagramBuilder. In order to avoid
                // documented UB hazards of `py_rvp::reference` used above, the
                // *exact* C++ instance must also be owned by the cycle. See
                // also issue #23161.
                py_builder =
                    py::cast(result.builder.release(), py_rvp::take_ownership);
                // Make the ref_cycle.
                internal::make_arbitrary_ref_cycle(
                    py_builder, py_diagram, "RobotDiagramBuilder::Build");
                // Return the py::object that owns the diagram.
                return py_diagram;
              },
              cls_doc.Build.doc);
    }
  };
  type_visit(bind_common_scalar_types, CommonScalarPack{});
}

}  // namespace internal
}  // namespace pydrake
}  // namespace drake
