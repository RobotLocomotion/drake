/* @file The contains all of the public entities found in the
 drake::geometry::optimization namespace. They can be found in the
 pydrake.geometry.optimization module. */

#include "drake/bindings/pydrake/common/default_scalars_pybind.h"
#include "drake/bindings/pydrake/common/deprecation_pybind.h"
#include "drake/bindings/pydrake/common/identifier_pybind.h"
#include "drake/bindings/pydrake/documentation_pybind.h"
#include "drake/bindings/pydrake/geometry_py.h"
#include "drake/geometry/optimization/cartesian_product.h"
#include "drake/geometry/optimization/graph_of_convex_sets.h"
#include "drake/geometry/optimization/hpolyhedron.h"
#include "drake/geometry/optimization/hyperellipsoid.h"
#include "drake/geometry/optimization/iris.h"
#include "drake/geometry/optimization/minkowski_sum.h"
#include "drake/geometry/optimization/point.h"
#include "drake/geometry/optimization/vpolytope.h"

namespace drake {
namespace pydrake {

void DefineGeometryOptimization(py::module m) {
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake;
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::geometry;
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::geometry::optimization;
  m.doc() = "Local bindings for `drake::geometry::optimization`";
  constexpr auto& doc = pydrake_doc.drake.geometry.optimization;

  // ConvexSet
  {
    const auto& cls_doc = doc.ConvexSet;
    py::class_<ConvexSet>(m, "ConvexSet", cls_doc.doc)
        .def("Clone",
            static_cast<::std::unique_ptr<ConvexSet> (ConvexSet::*)() const>(
                &ConvexSet::Clone),
            cls_doc.Clone.doc)
        .def("ambient_dimension", &ConvexSet::ambient_dimension,
            cls_doc.ambient_dimension.doc)
        .def("IntersectsWith", &ConvexSet::IntersectsWith, py::arg("other"),
            cls_doc.IntersectsWith.doc)
        .def("IsBounded", &ConvexSet::IsBounded, cls_doc.IsBounded.doc)
        .def("PointInSet", &ConvexSet::PointInSet, py::arg("x"),
            py::arg("tol") = 1e-8, cls_doc.PointInSet.doc)
        .def("AddPointInSetConstraints", &ConvexSet::AddPointInSetConstraints,
            py::arg("prog"), py::arg("vars"),
            cls_doc.AddPointInSetConstraints.doc)
        .def("AddPointInNonnegativeScalingConstraints",
            &ConvexSet::AddPointInNonnegativeScalingConstraints,
            py::arg("prog"), py::arg("x"), py::arg("t"),
            cls_doc.AddPointInNonnegativeScalingConstraints.doc)
        .def("ToShapeWithPose", &ConvexSet::ToShapeWithPose,
            cls_doc.ToShapeWithPose.doc);
    // Note: We use the copyable_unique_ptr constructor which calls Clone() on
    // the set, so that the new object is never an alias to the old.
    py::class_<copyable_unique_ptr<ConvexSet>>(m, "CopyableUniquePtrConvexSet")
        .def(py::init([](const ConvexSet& s) {
          return copyable_unique_ptr<ConvexSet>(s);
        }));
  }

  // CartesianProduct
  {
    const auto& cls_doc = doc.CartesianProduct;
    py::class_<CartesianProduct, ConvexSet>(m, "CartesianProduct", cls_doc.doc)
        .def(py::init<const ConvexSets&>(), py::arg("sets"),
            cls_doc.ctor.doc_1args_sets)
        .def(py::init<const ConvexSet&, const ConvexSet&>(), py::arg("setA"),
            py::arg("setB"), cls_doc.ctor.doc_2args_setA_setB)
        .def(py::init<const ConvexSets&,
                 const Eigen::Ref<const Eigen::MatrixXd>&,
                 const Eigen::Ref<const Eigen::VectorXd>&>(),
            py::arg("sets"), py::arg("A"), py::arg("b"),
            cls_doc.ctor.doc_3args_sets_A_b)
        .def(py::init<const QueryObject<double>&, GeometryId,
                 std::optional<FrameId>>(),
            py::arg("query_object"), py::arg("geometry_id"),
            py::arg("reference_frame") = std::nullopt,
            cls_doc.ctor.doc_3args_query_object_geometry_id_reference_frame)
        .def("num_factors", &CartesianProduct::num_factors,
            cls_doc.num_factors.doc)
        .def("factor", &CartesianProduct::factor, py_rvp::reference_internal,
            py::arg("index"), cls_doc.factor.doc);
    py::implicitly_convertible<CartesianProduct,
        copyable_unique_ptr<ConvexSet>>();
  }

  // HPolyhedron
  {
    const auto& cls_doc = doc.HPolyhedron;
    py::class_<HPolyhedron, ConvexSet>(m, "HPolyhedron", cls_doc.doc)
        .def(py::init<const Eigen::Ref<const Eigen::MatrixXd>&,
                 const Eigen::Ref<const Eigen::VectorXd>&>(),
            py::arg("A"), py::arg("b"), cls_doc.ctor.doc_2args)
        .def(py::init<const QueryObject<double>&, GeometryId,
                 std::optional<FrameId>>(),
            py::arg("query_object"), py::arg("geometry_id"),
            py::arg("reference_frame") = std::nullopt, cls_doc.ctor.doc_3args)
        .def("A", &HPolyhedron::A, cls_doc.A.doc)
        .def("b", &HPolyhedron::b, cls_doc.b.doc)
        .def("MaximumVolumeInscribedEllipsoid",
            &HPolyhedron::MaximumVolumeInscribedEllipsoid,
            cls_doc.MaximumVolumeInscribedEllipsoid.doc)
        .def("ChebyshevCenter", &HPolyhedron::ChebyshevCenter,
            cls_doc.ChebyshevCenter.doc)
        .def("CartesianProduct", &HPolyhedron::CartesianProduct,
            py::arg("other"), cls_doc.CartesianProduct.doc)
        .def("CartesianPower", &HPolyhedron::CartesianPower, py::arg("n"),
            cls_doc.CartesianPower.doc)
        .def_static("MakeBox", &HPolyhedron::MakeBox, py::arg("lb"),
            py::arg("ub"), cls_doc.MakeBox.doc)
        .def_static("MakeUnitBox", &HPolyhedron::MakeUnitBox, py::arg("dim"),
            cls_doc.MakeUnitBox.doc);
    py::implicitly_convertible<HPolyhedron, copyable_unique_ptr<ConvexSet>>();
  }

  // Hyperellipsoid
  {
    const auto& cls_doc = doc.Hyperellipsoid;
    py::class_<Hyperellipsoid, ConvexSet>(m, "Hyperellipsoid", cls_doc.doc)
        .def(py::init<const Eigen::Ref<const Eigen::MatrixXd>&,
                 const Eigen::Ref<const Eigen::VectorXd>&>(),
            py::arg("A"), py::arg("center"), cls_doc.ctor.doc_2args)
        .def(py::init<const QueryObject<double>&, GeometryId,
                 std::optional<FrameId>>(),
            py::arg("query_object"), py::arg("geometry_id"),
            py::arg("reference_frame") = std::nullopt, cls_doc.ctor.doc_3args)
        .def("A", &Hyperellipsoid::A, cls_doc.A.doc)
        .def("center", &Hyperellipsoid::center, cls_doc.center.doc)
        .def("Volume", &Hyperellipsoid::Volume, cls_doc.Volume.doc)
        .def("MinimumUniformScalingToTouch",
            &Hyperellipsoid::MinimumUniformScalingToTouch, py::arg("other"),
            cls_doc.MinimumUniformScalingToTouch.doc)
        .def_static("MakeAxisAligned", &Hyperellipsoid::MakeAxisAligned,
            py::arg("radius"), py::arg("center"), cls_doc.MakeAxisAligned.doc)
        .def_static("MakeHypersphere", &Hyperellipsoid::MakeHypersphere,
            py::arg("radius"), py::arg("center"), cls_doc.MakeHypersphere.doc)
        .def_static("MakeUnitBall", &Hyperellipsoid::MakeUnitBall,
            py::arg("dim"), cls_doc.MakeUnitBall.doc);
    py::implicitly_convertible<Hyperellipsoid,
        copyable_unique_ptr<ConvexSet>>();
  }

  // MinkowskiSum
  {
    const auto& cls_doc = doc.MinkowskiSum;
    py::class_<MinkowskiSum, ConvexSet>(m, "MinkowskiSum", cls_doc.doc)
        .def(py::init<const ConvexSets&>(), py::arg("sets"),
            cls_doc.ctor.doc_1args)
        .def(py::init<const ConvexSet&, const ConvexSet&>(), py::arg("setA"),
            py::arg("setB"), cls_doc.ctor.doc_2args)
        .def(py::init<const QueryObject<double>&, GeometryId,
                 std::optional<FrameId>>(),
            py::arg("query_object"), py::arg("geometry_id"),
            py::arg("reference_frame") = std::nullopt, cls_doc.ctor.doc_3args)
        .def("num_terms", &MinkowskiSum::num_terms, cls_doc.num_terms.doc)
        .def("term", &MinkowskiSum::term, py_rvp::reference_internal,
            py::arg("index"), cls_doc.term.doc);
    py::implicitly_convertible<MinkowskiSum, copyable_unique_ptr<ConvexSet>>();
  }

  // Point
  {
    const auto& cls_doc = doc.Point;
    py::class_<Point, ConvexSet>(m, "Point", cls_doc.doc)
        .def(py::init<const Eigen::Ref<const Eigen::VectorXd>&>(), py::arg("x"),
            cls_doc.ctor.doc_1args)
        .def(py::init<const QueryObject<double>&, GeometryId,
                 std::optional<FrameId>, double>(),
            py::arg("query_object"), py::arg("geometry_id"),
            py::arg("reference_frame") = std::nullopt,
            py::arg("maximum_allowable_radius") = 0.0, cls_doc.ctor.doc_4args)
        .def("x", &Point::x, cls_doc.x.doc)
        .def("set_x", &Point::set_x, py::arg("x"), cls_doc.set_x.doc);
    py::implicitly_convertible<Point, copyable_unique_ptr<ConvexSet>>();
  }

  // VPolytope
  {
    const auto& cls_doc = doc.VPolytope;
    py::class_<VPolytope, ConvexSet>(m, "VPolytope", cls_doc.doc)
        .def(py::init<const Eigen::Ref<const Eigen::MatrixXd>&>(),
            py::arg("vertices"), cls_doc.ctor.doc_1args)
        .def(py::init<const QueryObject<double>&, GeometryId,
                 std::optional<FrameId>>(),
            py::arg("query_object"), py::arg("geometry_id"),
            py::arg("reference_frame") = std::nullopt, cls_doc.ctor.doc_3args)
        .def("vertices", &VPolytope::vertices, cls_doc.vertices.doc)
        .def_static("MakeBox", &VPolytope::MakeBox, py::arg("lb"),
            py::arg("ub"), cls_doc.MakeBox.doc)
        .def_static("MakeUnitBox", &VPolytope::MakeUnitBox, py::arg("dim"),
            cls_doc.MakeUnitBox.doc);
    py::implicitly_convertible<VPolytope, copyable_unique_ptr<ConvexSet>>();
  }

  py::class_<IrisOptions>(m, "IrisOptions", doc.IrisOptions.doc)
      .def(py::init<>(), doc.IrisOptions.ctor.doc)
      .def_readwrite("require_sample_point_is_contained",
          &IrisOptions::require_sample_point_is_contained,
          doc.IrisOptions.require_sample_point_is_contained.doc)
      .def_readwrite("iteration_limit", &IrisOptions::iteration_limit,
          doc.IrisOptions.iteration_limit.doc)
      .def_readwrite("termination_threshold",
          &IrisOptions::termination_threshold,
          doc.IrisOptions.termination_threshold.doc)
      .def_readwrite("relative_termination_threshold",
          &IrisOptions::relative_termination_threshold,
          doc.IrisOptions.relative_termination_threshold.doc)
      .def_readwrite("configuration_space_margin",
          &IrisOptions::configuration_space_margin,
          doc.IrisOptions.configuration_space_margin.doc)
      .def_readwrite("enable_ibex", &IrisOptions::enable_ibex,
          doc.IrisOptions.enable_ibex.doc)
      .def("__repr__", [](const IrisOptions& self) {
        return py::str(
            "IrisOptions("
            "require_sample_point_is_contained={}, "
            "iteration_limit={}, "
            "termination_threshold={}, "
            "relative_termination_threshold={}, "
            "configuration_space_margin={}, "
            "enable_ibex={}"
            ")")
            .format(self.require_sample_point_is_contained,
                self.iteration_limit, self.termination_threshold,
                self.relative_termination_threshold,
                self.configuration_space_margin, self.enable_ibex);
      });

  m.def("Iris", &Iris, py::arg("obstacles"), py::arg("sample"),
      py::arg("domain"), py::arg("options") = IrisOptions(), doc.Iris.doc);

  m.def("MakeIrisObstacles", &MakeIrisObstacles, py::arg("query_object"),
      py::arg("reference_frame") = std::nullopt, doc.MakeIrisObstacles.doc);

  m.def("IrisInConfigurationSpace",
      py::overload_cast<const multibody::MultibodyPlant<double>&,
          const systems::Context<double>&, const IrisOptions&>(
          &IrisInConfigurationSpace),
      py::arg("plant"), py::arg("context"), py::arg("options") = IrisOptions(),
      doc.IrisInConfigurationSpace.doc);

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
  m.def("IrisInConfigurationSpace",
      WrapDeprecated(doc.IrisInConfigurationSpace.doc_deprecated,
          [](const multibody::MultibodyPlant<double>& plant,
              const systems::Context<double>& context,
              const Eigen::Ref<const Eigen::VectorXd>& sample,
              const IrisOptions& options) {
            return IrisInConfigurationSpace(plant, context, sample, options);
          }),
      py::arg("plant"), py::arg("context"), py::arg("sample"),
      py::arg("options") = IrisOptions(),
      doc.IrisInConfigurationSpace.doc_deprecated);
#pragma GCC diagnostic pop

  // GraphOfConvexSets
  {
    const auto& cls_doc = doc.GraphOfConvexSets;
    auto graph_of_convex_sets =
        py::class_<GraphOfConvexSets>(m, "GraphOfConvexSets", cls_doc.doc)
            .def(py::init<>(), cls_doc.ctor.doc)
            .def("AddVertex", &GraphOfConvexSets::AddVertex, py::arg("set"),
                py::arg("name") = "", py_rvp::reference_internal,
                cls_doc.AddVertex.doc)
            .def("AddEdge",
                py::overload_cast<const GraphOfConvexSets::VertexId&,
                    const GraphOfConvexSets::VertexId&, std::string>(
                    &GraphOfConvexSets::AddEdge),
                py::arg("u_id"), py::arg("v_id"), py::arg("name") = "",
                py_rvp::reference_internal, cls_doc.AddEdge.doc_by_id)
            .def("AddEdge",
                py::overload_cast<const GraphOfConvexSets::Vertex&,
                    const GraphOfConvexSets::Vertex&, std::string>(
                    &GraphOfConvexSets::AddEdge),
                py::arg("u"), py::arg("v"), py::arg("name") = "",
                py_rvp::reference_internal, cls_doc.AddEdge.doc_by_reference)
            .def(
                "Vertices",
                [](GraphOfConvexSets* self) {
                  py::list out;
                  py::object self_py = py::cast(self, py_rvp::reference);
                  for (auto* vertex : self->Vertices()) {
                    py::object vertex_py = py::cast(vertex, py_rvp::reference);
                    // Keep alive, ownership: `vertex` keeps `self` alive.
                    py_keep_alive(vertex_py, self_py);
                    out.append(vertex_py);
                  }
                  return out;
                },
                cls_doc.Vertices.doc)
            .def(
                "Edges",
                [](GraphOfConvexSets* self) {
                  py::list out;
                  py::object self_py = py::cast(self, py_rvp::reference);
                  for (auto* edge : self->Edges()) {
                    py::object edge_py = py::cast(edge, py_rvp::reference);
                    // Keep alive, ownership: `edge` keeps `self` alive.
                    py_keep_alive(edge_py, self_py);
                    out.append(edge_py);
                  }
                  return out;
                },
                cls_doc.Edges.doc)
            .def("GetGraphvizString", &GraphOfConvexSets::GetGraphvizString,
                py::arg("result"), py::arg("show_slacks") = true,
                py::arg("precision") = 3, py::arg("scientific") = false,
                cls_doc.GetGraphvizString.doc)
            .def("SolveShortestPath",
                overload_cast_explicit<solvers::MathematicalProgramResult,
                    const GraphOfConvexSets::VertexId&,
                    const GraphOfConvexSets::VertexId&, bool>(
                    &GraphOfConvexSets::SolveShortestPath),
                py::arg("source_id"), py::arg("target_id"),
                py::arg("convex_relaxation") = false,
                cls_doc.SolveShortestPath.doc_by_id)
            .def("SolveShortestPath",
                overload_cast_explicit<solvers::MathematicalProgramResult,
                    const GraphOfConvexSets::Vertex&,
                    const GraphOfConvexSets::Vertex&, bool>(
                    &GraphOfConvexSets::SolveShortestPath),
                py::arg("source"), py::arg("target"),
                py::arg("convex_relaxation") = false,
                cls_doc.SolveShortestPath.doc_by_reference);

    BindIdentifier<GraphOfConvexSets::VertexId>(
        graph_of_convex_sets, "VertexId", doc.GraphOfConvexSets.VertexId.doc);
    BindIdentifier<GraphOfConvexSets::EdgeId>(
        graph_of_convex_sets, "EdgeId", doc.GraphOfConvexSets.EdgeId.doc);

    // Vertex
    const auto& vertex_doc = doc.GraphOfConvexSets.Vertex;
    py::class_<GraphOfConvexSets::Vertex>(
        graph_of_convex_sets, "Vertex", vertex_doc.doc)
        .def("id", &GraphOfConvexSets::Vertex::id, vertex_doc.id.doc)
        .def("ambient_dimension", &GraphOfConvexSets::Vertex::ambient_dimension,
            vertex_doc.ambient_dimension.doc)
        .def("name", &GraphOfConvexSets::Vertex::name, vertex_doc.name.doc)
        // As in trajectory_optimization_py.cc, we use a lambda to *copy*
        // the decision variables; otherwise we get dtype=object arrays
        // cannot be referenced.
        .def(
            "x",
            [](const GraphOfConvexSets::Vertex& self)
                -> const VectorX<symbolic::Variable> { return self.x(); },
            vertex_doc.x.doc)
        .def("set", &GraphOfConvexSets::Vertex::set, py_rvp::reference_internal,
            vertex_doc.set.doc)
        .def("GetSolution", &GraphOfConvexSets::Vertex::GetSolution,
            py::arg("result"), vertex_doc.GetSolution.doc);

    // Edge
    const auto& edge_doc = doc.GraphOfConvexSets.Edge;
    py::class_<GraphOfConvexSets::Edge>(
        graph_of_convex_sets, "Edge", edge_doc.doc)
        .def("id", &GraphOfConvexSets::Edge::id, edge_doc.id.doc)
        .def("name", &GraphOfConvexSets::Edge::name, edge_doc.name.doc)
        .def("u", &GraphOfConvexSets::Edge::u, py_rvp::reference_internal,
            edge_doc.u.doc)
        .def("v", &GraphOfConvexSets::Edge::v, py_rvp::reference_internal,
            edge_doc.v.doc)
        .def("phi", &GraphOfConvexSets::Edge::phi, py_rvp::reference_internal,
            edge_doc.phi.doc)
        .def(
            "xu",
            [](const GraphOfConvexSets::Edge& self)
                -> const VectorX<symbolic::Variable> { return self.xu(); },
            edge_doc.xu.doc)
        .def(
            "xv",
            [](const GraphOfConvexSets::Edge& self)
                -> const VectorX<symbolic::Variable> { return self.xv(); },
            edge_doc.xv.doc)
        .def("AddCost",
            py::overload_cast<const symbolic::Expression&>(
                &GraphOfConvexSets::Edge::AddCost),
            py::arg("e"), edge_doc.AddCost.doc_expression)
        .def("AddCost",
            py::overload_cast<const solvers::Binding<solvers::Cost>&>(
                &GraphOfConvexSets::Edge::AddCost),
            py::arg("binding"), edge_doc.AddCost.doc_binding)
        .def("AddConstraint",
            overload_cast_explicit<solvers::Binding<solvers::Constraint>,
                const symbolic::Formula&>(
                &GraphOfConvexSets::Edge::AddConstraint),
            py::arg("f"), edge_doc.AddConstraint.doc_formula)
        .def("AddConstraint",
            overload_cast_explicit<solvers::Binding<solvers::Constraint>,
                const solvers::Binding<solvers::Constraint>&>(
                &GraphOfConvexSets::Edge::AddConstraint),
            py::arg("binding"), edge_doc.AddCost.doc_binding)
        .def("AddPhiConstraint", &GraphOfConvexSets::Edge::AddPhiConstraint,
            py::arg("phi_value"), edge_doc.AddPhiConstraint.doc)
        .def("ClearPhiConstraints",
            &GraphOfConvexSets::Edge::ClearPhiConstraints,
            edge_doc.ClearPhiConstraints.doc)
        .def("GetSolutionCost", &GraphOfConvexSets::Edge::GetSolutionCost,
            py::arg("result"), edge_doc.GetSolutionCost.doc)
        .def("GetSolutionPhiXu", &GraphOfConvexSets::Edge::GetSolutionPhiXu,
            py::arg("result"), edge_doc.GetSolutionPhiXu.doc)
        .def("GetSolutionPhiXv", &GraphOfConvexSets::Edge::GetSolutionPhiXv,
            py::arg("result"), edge_doc.GetSolutionPhiXv.doc);
  }
}

}  // namespace pydrake
}  // namespace drake
