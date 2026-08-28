#include <utility>

#include "drake/bindings/generated_docstrings/planning_continuous_collision.h"
#include "drake/bindings/pydrake/planning/planning_py.h"
#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/planning/continuous_collision/continuous_collision_checker.h"
#include "drake/planning/robot_diagram.h"

namespace drake {
namespace pydrake {
namespace internal {

void DefinePlanningContinuousCollision(py::module_ m) {
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::planning::continuous_collision;
  constexpr auto& doc = pydrake_doc_planning_continuous_collision.drake.planning
                            .continuous_collision;

  using drake::planning::RobotDiagram;

  m.doc() = R"""(
Certified continuous collision checking: proves that a trajectory is
collision-free over its entire continuous time domain, rather than sampling it.
)""";

  {
    using Class = Verdict;
    constexpr auto& cls_doc = doc.Verdict;
    py::enum_<Class>(m, "Verdict", cls_doc.doc)
        .value(
            "kCertifiedFree", Class::kCertifiedFree, cls_doc.kCertifiedFree.doc)
        .value("kViolationFound", Class::kViolationFound,
            cls_doc.kViolationFound.doc)
        .value(
            "kInconclusive", Class::kInconclusive, cls_doc.kInconclusive.doc);
  }

  {
    using Class = Finding;
    constexpr auto& cls_doc = doc.Finding;
    class_<Class> cls(m, "Finding", cls_doc.doc);
    cls  // BR
        .def(py::init<>())
        .def(ParamInit<Class>())
        .def_rw("time", &Class::time, cls_doc.time.doc)
        .def_rw("q", &Class::q, cls_doc.q.doc)
        .def_rw("geometry_a", &Class::geometry_a, cls_doc.geometry_a.doc)
        .def_rw("geometry_b", &Class::geometry_b, cls_doc.geometry_b.doc)
        .def_rw("body_a", &Class::body_a, cls_doc.body_a.doc)
        .def_rw("body_b", &Class::body_b, cls_doc.body_b.doc)
        .def_rw("distance", &Class::distance, cls_doc.distance.doc)
        .def_rw("nearest_a_W", &Class::nearest_a_W, cls_doc.nearest_a_W.doc)
        .def_rw("nearest_b_W", &Class::nearest_b_W, cls_doc.nearest_b_W.doc);
    DefCopyAndDeepCopy(&cls);
  }

  {
    using Class = Options;
    constexpr auto& cls_doc = doc.Options;
    class_<Class> cls(m, "Options", cls_doc.doc);
    cls  // BR
        .def(py::init<>())
        .def(ParamInit<Class>())
        .def_rw("margin", &Class::margin, cls_doc.margin.doc)
        .def_rw("min_interval", &Class::min_interval, cls_doc.min_interval.doc)
        .def_rw("continuous_revolute_indices",
            &Class::continuous_revolute_indices,
            cls_doc.continuous_revolute_indices.doc)
        .def_rw("parallelism", &Class::parallelism, cls_doc.parallelism.doc);
    DefCopyAndDeepCopy(&cls);
  }

  {
    using Class = Result;
    constexpr auto& cls_doc = doc.Result;
    class_<Class> cls(m, "Result", cls_doc.doc);
    cls  // BR
        .def(py::init<>())
        .def(ParamInit<Class>())
        .def_rw("verdict", &Class::verdict, cls_doc.verdict.doc)
        .def_rw("finding", &Class::finding, cls_doc.finding.doc)
        .def_rw("num_nodes", &Class::num_nodes, cls_doc.num_nodes.doc);
    DefCopyAndDeepCopy(&cls);
  }

  {
    using Class = ContinuousCollisionChecker;
    constexpr auto& cls_doc = doc.ContinuousCollisionChecker;
    class_<Class> cls(m, "ContinuousCollisionChecker", cls_doc.doc);
    cls  // BR
        .def(
            "__init__",
            [](Class* self, py::object model, const Options& default_options) {
              // For lifetime management, add a python reference to model
              // (owned by the shared pointer) and transfer that to the c++
              // checker.
              new (self) Class(
                  make_shared_ptr_from_py_object<RobotDiagram<double>>(model),
                  default_options);
            },
            py::kw_only(), py::arg("model"),
            py::arg("default_options") = Options{}, cls_doc.ctor.doc)
        .def("CheckTrajectory", &Class::CheckTrajectory, py::arg("trajectory"),
            py::arg("options") = std::nullopt,
            py::call_guard<py::gil_scoped_release>(),
            cls_doc.CheckTrajectory.doc)
        .def("CheckPath", &Class::CheckPath, py::arg("waypoints"),
            py::arg("options") = std::nullopt,
            py::call_guard<py::gil_scoped_release>(), cls_doc.CheckPath.doc)
        .def("CheckEdge", &Class::CheckEdge, py::arg("q1"), py::arg("q2"),
            py::arg("options") = std::nullopt,
            py::call_guard<py::gil_scoped_release>(), cls_doc.CheckEdge.doc)
        .def("model", &Class::model, py_rvp::reference_internal,
            cls_doc.model.doc);
  }
}

}  // namespace internal
}  // namespace pydrake
}  // namespace drake
