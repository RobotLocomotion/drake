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
#include "drake/geometry/optimization/intersection.h"
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

  py::module::import("pydrake.solvers");

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
            overload_cast_explicit<
                std::vector<solvers::Binding<solvers::Constraint>>,
                solvers::MathematicalProgram*,
                const Eigen::Ref<const solvers::VectorXDecisionVariable>&,
                const symbolic::Variable&>(
                &ConvexSet::AddPointInNonnegativeScalingConstraints),
            py::arg("prog"), py::arg("x"), py::arg("t"),
            cls_doc.AddPointInNonnegativeScalingConstraints.doc_3args)
        .def("AddPointInNonnegativeScalingConstraints",
            overload_cast_explicit<
                std::vector<solvers::Binding<solvers::Constraint>>,
                solvers::MathematicalProgram*,
                const Eigen::Ref<const Eigen::MatrixXd>&,
                const Eigen::Ref<const Eigen::VectorXd>&,
                const Eigen::Ref<const Eigen::VectorXd>&, double,
                const Eigen::Ref<const solvers::VectorXDecisionVariable>&,
                const Eigen::Ref<const solvers::VectorXDecisionVariable>&>(
                &ConvexSet::AddPointInNonnegativeScalingConstraints),
            py::arg("prog"), py::arg("A"), py::arg("b"), py::arg("c"),
            py::arg("d"), py::arg("x"), py::arg("t"),
            cls_doc.AddPointInNonnegativeScalingConstraints.doc_7args)
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
        .def("ContainedIn", &HPolyhedron::ContainedIn, py::arg("other"),
            py::arg("tol") = 1E-9, cls_doc.ContainedIn.doc)
        .def("Intersection", &HPolyhedron::Intersection, py::arg("other"),
            py::arg("check_for_redundancy") = false, py::arg("tol") = 1E-9,
            cls_doc.Intersection.doc)
        .def("ReduceInequalities", &HPolyhedron::ReduceInequalities,
            py::arg("tol") = 1E-9, cls_doc.ReduceInequalities.doc)
        .def("IsEmpty", &HPolyhedron::IsEmpty, cls_doc.IsEmpty.doc)
        .def("MaximumVolumeInscribedEllipsoid",
            &HPolyhedron::MaximumVolumeInscribedEllipsoid,
            cls_doc.MaximumVolumeInscribedEllipsoid.doc)
        .def("ChebyshevCenter", &HPolyhedron::ChebyshevCenter,
            cls_doc.ChebyshevCenter.doc)
        .def("CartesianProduct", &HPolyhedron::CartesianProduct,
            py::arg("other"), cls_doc.CartesianProduct.doc)
        .def("CartesianPower", &HPolyhedron::CartesianPower, py::arg("n"),
            cls_doc.CartesianPower.doc)
        .def("PontryaginDifference", &HPolyhedron::PontryaginDifference,
            py::arg("other"), cls_doc.PontryaginDifference.doc)
        .def("UniformSample",
            overload_cast_explicit<Eigen::VectorXd, RandomGenerator*,
                const Eigen::Ref<const Eigen::VectorXd>&>(
                &HPolyhedron::UniformSample),
            py::arg("generator"), py::arg("previous_sample"),
            cls_doc.UniformSample.doc_2args)
        .def("UniformSample",
            overload_cast_explicit<Eigen::VectorXd, RandomGenerator*>(
                &HPolyhedron::UniformSample),
            py::arg("generator"), cls_doc.UniformSample.doc_1args)
        .def_static("MakeBox", &HPolyhedron::MakeBox, py::arg("lb"),
            py::arg("ub"), cls_doc.MakeBox.doc)
        .def_static("MakeUnitBox", &HPolyhedron::MakeUnitBox, py::arg("dim"),
            cls_doc.MakeUnitBox.doc)
        .def_static("MakeL1Ball", &HPolyhedron::MakeL1Ball, py::arg("dim"),
            cls_doc.MakeL1Ball.doc)
        .def(py::pickle(
            [](const HPolyhedron& self) {
              return std::make_pair(self.A(), self.b());
            },
            [](std::pair<Eigen::MatrixXd, Eigen::VectorXd> args) {
              return HPolyhedron(std::get<0>(args), std::get<1>(args));
            }));
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
            py::arg("dim"), cls_doc.MakeUnitBall.doc)
        .def(py::pickle(
            [](const Hyperellipsoid& self) {
              return std::make_pair(self.A(), self.center());
            },
            [](std::pair<Eigen::MatrixXd, Eigen::VectorXd> args) {
              return Hyperellipsoid(std::get<0>(args), std::get<1>(args));
            }));
    py::implicitly_convertible<Hyperellipsoid,
        copyable_unique_ptr<ConvexSet>>();
  }

  // Intersection
  {
    const auto& cls_doc = doc.Intersection;
    py::class_<Intersection, ConvexSet>(m, "Intersection", cls_doc.doc)
        .def(py::init<const ConvexSets&>(), py::arg("sets"),
            cls_doc.ctor.doc_1args)
        .def(py::init<const ConvexSet&, const ConvexSet&>(), py::arg("setA"),
            py::arg("setB"), cls_doc.ctor.doc_2args)
        .def("num_elements", &Intersection::num_elements,
            cls_doc.num_elements.doc)
        .def("element", &Intersection::element, py_rvp::reference_internal,
            py::arg("index"), cls_doc.element.doc);
    py::implicitly_convertible<Intersection, copyable_unique_ptr<ConvexSet>>();
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
        .def("set_x", &Point::set_x, py::arg("x"), cls_doc.set_x.doc)
        .def(py::pickle([](const Point& self) { return self.x(); },
            [](Eigen::VectorXd arg) { return Point(arg); }));
    py::implicitly_convertible<Point, copyable_unique_ptr<ConvexSet>>();
  }

  // VPolytope
  {
    const auto& cls_doc = doc.VPolytope;
    py::class_<VPolytope, ConvexSet>(m, "VPolytope", cls_doc.doc)
        .def(py::init<const Eigen::Ref<const Eigen::MatrixXd>&>(),
            py::arg("vertices"), cls_doc.ctor.doc_vertices)
        .def(py::init<const HPolyhedron&>(), py::arg("H"),
            cls_doc.ctor.doc_hpolyhedron)
        .def(py::init<const QueryObject<double>&, GeometryId,
                 std::optional<FrameId>>(),
            py::arg("query_object"), py::arg("geometry_id"),
            py::arg("reference_frame") = std::nullopt,
            cls_doc.ctor.doc_scenegraph)
        .def("GetMinimalRepresentation", &VPolytope::GetMinimalRepresentation,
            cls_doc.GetMinimalRepresentation.doc)
        .def("vertices", &VPolytope::vertices, cls_doc.vertices.doc)
        .def_static("MakeBox", &VPolytope::MakeBox, py::arg("lb"),
            py::arg("ub"), cls_doc.MakeBox.doc)
        .def_static("MakeUnitBox", &VPolytope::MakeUnitBox, py::arg("dim"),
            cls_doc.MakeUnitBox.doc)
        .def("CalcVolume", &VPolytope::CalcVolume, cls_doc.CalcVolume.doc)
        .def(py::pickle([](const VPolytope& self) { return self.vertices(); },
            [](Eigen::MatrixXd arg) { return VPolytope(arg); }));
    py::implicitly_convertible<VPolytope, copyable_unique_ptr<ConvexSet>>();
  }

  {
    const auto& cls_doc = doc.IrisOptions;
    py::class_<IrisOptions> iris_options(m, "IrisOptions", cls_doc.doc);
    iris_options.def(py::init<>(), cls_doc.ctor.doc)
        .def_readwrite("require_sample_point_is_contained",
            &IrisOptions::require_sample_point_is_contained,
            cls_doc.require_sample_point_is_contained.doc)
        .def_readwrite("iteration_limit", &IrisOptions::iteration_limit,
            cls_doc.iteration_limit.doc)
        .def_readwrite("termination_threshold",
            &IrisOptions::termination_threshold,
            cls_doc.termination_threshold.doc)
        .def_readwrite("relative_termination_threshold",
            &IrisOptions::relative_termination_threshold,
            cls_doc.relative_termination_threshold.doc)
        .def_readwrite("configuration_space_margin",
            &IrisOptions::configuration_space_margin,
            cls_doc.configuration_space_margin.doc)
        .def_readwrite("num_collision_infeasible_samples",
            &IrisOptions::num_collision_infeasible_samples,
            cls_doc.num_collision_infeasible_samples.doc)
        .def_readwrite("configuration_obstacles",
            &IrisOptions::configuration_obstacles,
            cls_doc.configuration_obstacles.doc)
        .def_readwrite("num_additional_constraint_infeasible_samples",
            &IrisOptions::num_additional_constraint_infeasible_samples,
            cls_doc.num_additional_constraint_infeasible_samples.doc)
        .def_readwrite(
            "random_seed", &IrisOptions::random_seed, cls_doc.random_seed.doc)
        .def("__repr__", [](const IrisOptions& self) {
          return py::str(
              "IrisOptions("
              "require_sample_point_is_contained={}, "
              "iteration_limit={}, "
              "termination_threshold={}, "
              "relative_termination_threshold={}, "
              "configuration_space_margin={}, "
              "num_collision_infeasible_samples={}, "
              "configuration_obstacles {}, "
              "prog_with_additional_constraints {}, "
              "num_additional_constraint_infeasible_samples={}, "
              "random_seed={}"
              ")")
              .format(self.require_sample_point_is_contained,
                  self.iteration_limit, self.termination_threshold,
                  self.relative_termination_threshold,
                  self.configuration_space_margin,
                  self.num_collision_infeasible_samples,
                  self.configuration_obstacles,
                  self.prog_with_additional_constraints ? "is set"
                                                        : "is not set",
                  self.num_additional_constraint_infeasible_samples,
                  self.random_seed);
        });

    DefReadWriteKeepAlive(&iris_options, "prog_with_additional_constraints",
        &IrisOptions::prog_with_additional_constraints,
        cls_doc.prog_with_additional_constraints.doc);
  }

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

  // GraphOfConvexSetsOptions
  {
    const auto& cls_doc = doc.GraphOfConvexSetsOptions;
    py::class_<GraphOfConvexSetsOptions> gcs_options(
        m, "GraphOfConvexSetsOptions", cls_doc.doc);
    gcs_options.def(py::init<>(), cls_doc.ctor.doc)
        .def_readwrite("convex_relaxation",
            &GraphOfConvexSetsOptions::convex_relaxation,
            cls_doc.convex_relaxation.doc)
        .def_readwrite("preprocessing",
            &GraphOfConvexSetsOptions::preprocessing, cls_doc.preprocessing.doc)
        .def_readwrite("max_rounded_paths",
            &GraphOfConvexSetsOptions::max_rounded_paths,
            cls_doc.max_rounded_paths.doc)
        .def_readwrite("max_rounding_trials",
            &GraphOfConvexSetsOptions::max_rounding_trials,
            cls_doc.max_rounding_trials.doc)
        .def_readwrite("flow_tolerance",
            &GraphOfConvexSetsOptions::flow_tolerance,
            cls_doc.flow_tolerance.doc)
        .def_readwrite("rounding_seed",
            &GraphOfConvexSetsOptions::rounding_seed, cls_doc.rounding_seed.doc)
        .def_property("solver_options",
            py::cpp_function(
                [](GraphOfConvexSetsOptions& self) {
                  return &(self.solver_options);
                },
                py_rvp::reference_internal),
            py::cpp_function([](GraphOfConvexSetsOptions& self,
                                 solvers::SolverOptions solver_options) {
              self.solver_options = std::move(solver_options);
            }),
            cls_doc.solver_options.doc)
        .def("__repr__", [](const GraphOfConvexSetsOptions& self) {
          return py::str(
              "GraphOfConvexSetsOptions("
              "convex_relaxation={}, "
              "preprocessing={}, "
              "max_rounded_paths={}, "
              "max_rounding_trials={}, "
              "flow_tolerance={}, "
              "rounding_seed={}, "
              "solver={}, "
              "solver_options={}, "
              ")")
              .format(self.convex_relaxation, self.preprocessing,
                  self.max_rounded_paths, self.max_rounding_trials,
                  self.flow_tolerance, self.rounding_seed, self.solver,
                  self.solver_options);
        });

    DefReadWriteKeepAlive(&gcs_options, "solver",
        &GraphOfConvexSetsOptions::solver, cls_doc.solver.doc);
  }

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
                py::overload_cast<GraphOfConvexSets::VertexId,
                    GraphOfConvexSets::VertexId, std::string>(
                    &GraphOfConvexSets::AddEdge),
                py::arg("u_id"), py::arg("v_id"), py::arg("name") = "",
                py_rvp::reference_internal, cls_doc.AddEdge.doc_by_id)
            .def("AddEdge",
                py::overload_cast<const GraphOfConvexSets::Vertex&,
                    const GraphOfConvexSets::Vertex&, std::string>(
                    &GraphOfConvexSets::AddEdge),
                py::arg("u"), py::arg("v"), py::arg("name") = "",
                py_rvp::reference_internal, cls_doc.AddEdge.doc_by_reference)
            .def("RemoveVertex",
                py::overload_cast<GraphOfConvexSets::VertexId>(
                    &GraphOfConvexSets::RemoveVertex),
                py::arg("vertex_id"), cls_doc.RemoveVertex.doc_by_id)
            .def("RemoveVertex",
                py::overload_cast<const GraphOfConvexSets::Vertex&>(
                    &GraphOfConvexSets::RemoveVertex),
                py::arg("vertex"), cls_doc.RemoveVertex.doc_by_reference)
            .def("RemoveEdge",
                py::overload_cast<GraphOfConvexSets::EdgeId>(
                    &GraphOfConvexSets::RemoveEdge),
                py::arg("edge_id"), cls_doc.RemoveEdge.doc_by_id)
            .def("RemoveEdge",
                py::overload_cast<const GraphOfConvexSets::Edge&>(
                    &GraphOfConvexSets::RemoveEdge),
                py::arg("edge"), cls_doc.RemoveEdge.doc_by_reference)
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
            .def("ClearAllPhiConstraints",
                &GraphOfConvexSets::ClearAllPhiConstraints,
                cls_doc.ClearAllPhiConstraints.doc)
            .def("GetGraphvizString", &GraphOfConvexSets::GetGraphvizString,
                py::arg("result") = std::nullopt, py::arg("show_slacks") = true,
                py::arg("precision") = 3, py::arg("scientific") = false,
                cls_doc.GetGraphvizString.doc)
            .def("SolveShortestPath",
                overload_cast_explicit<solvers::MathematicalProgramResult,
                    GraphOfConvexSets::VertexId, GraphOfConvexSets::VertexId,
                    const GraphOfConvexSetsOptions&>(
                    &GraphOfConvexSets::SolveShortestPath),
                py::arg("source_id"), py::arg("target_id"),
                py::arg("options") = GraphOfConvexSetsOptions(),
                cls_doc.SolveShortestPath.doc_by_id)
            .def("SolveShortestPath",
                overload_cast_explicit<solvers::MathematicalProgramResult,
                    const GraphOfConvexSets::Vertex&,
                    const GraphOfConvexSets::Vertex&,
                    const GraphOfConvexSetsOptions&>(
                    &GraphOfConvexSets::SolveShortestPath),
                py::arg("source"), py::arg("target"),
                py::arg("options") = GraphOfConvexSetsOptions(),
                cls_doc.SolveShortestPath.doc_by_reference);
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
    graph_of_convex_sets
        .def("SolveShortestPath",
            WrapDeprecated(cls_doc.SolveShortestPath.doc_deprecated_by_id,
                overload_cast_explicit<solvers::MathematicalProgramResult,
                    GraphOfConvexSets::VertexId, GraphOfConvexSets::VertexId,
                    bool, const solvers::SolverInterface*,
                    const std::optional<solvers::SolverOptions>&>(
                    &GraphOfConvexSets::SolveShortestPath)),
            py::arg("source_id"), py::arg("target_id"),
            py::arg("convex_relaxation"), py::arg("solver") = nullptr,
            py::arg("solver_options") = std::nullopt,
            cls_doc.SolveShortestPath.doc_deprecated_by_id)
        .def("SolveShortestPath",
            WrapDeprecated(
                cls_doc.SolveShortestPath.doc_deprecated_by_reference,
                overload_cast_explicit<solvers::MathematicalProgramResult,
                    const GraphOfConvexSets::Vertex&,
                    const GraphOfConvexSets::Vertex&, bool,
                    const solvers::SolverInterface*,
                    const std::optional<solvers::SolverOptions>&>(
                    &GraphOfConvexSets::SolveShortestPath)),
            py::arg("source"), py::arg("target"), py::arg("convex_relaxation"),
            py::arg("solver") = nullptr,
            py::arg("solver_options") = std::nullopt,
            cls_doc.SolveShortestPath.doc_deprecated_by_reference);
#pragma GCC diagnostic pop

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
        .def("AddCost",
            py::overload_cast<const symbolic::Expression&>(
                &GraphOfConvexSets::Vertex::AddCost),
            py::arg("e"), vertex_doc.AddCost.doc_expression)
        .def("AddCost",
            py::overload_cast<const solvers::Binding<solvers::Cost>&>(
                &GraphOfConvexSets::Vertex::AddCost),
            py::arg("binding"), vertex_doc.AddCost.doc_binding)
        .def("AddConstraint",
            overload_cast_explicit<solvers::Binding<solvers::Constraint>,
                const symbolic::Formula&>(
                &GraphOfConvexSets::Vertex::AddConstraint),
            py::arg("f"), vertex_doc.AddConstraint.doc_formula)
        .def("AddConstraint",
            overload_cast_explicit<solvers::Binding<solvers::Constraint>,
                const solvers::Binding<solvers::Constraint>&>(
                &GraphOfConvexSets::Vertex::AddConstraint),
            py::arg("binding"), vertex_doc.AddCost.doc_binding)
        .def("GetCosts", &GraphOfConvexSets::Vertex::GetCosts,
            vertex_doc.GetCosts.doc)
        .def("GetConstraints", &GraphOfConvexSets::Vertex::GetConstraints,
            vertex_doc.GetConstraints.doc)
        .def("GetSolutionCost", &GraphOfConvexSets::Vertex::GetSolutionCost,
            py::arg("result"), vertex_doc.GetSolutionCost.doc)
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
        .def("GetCosts", &GraphOfConvexSets::Edge::GetCosts,
            edge_doc.GetCosts.doc)
        .def("GetConstraints", &GraphOfConvexSets::Edge::GetConstraints,
            edge_doc.GetConstraints.doc)
        .def("GetSolutionCost", &GraphOfConvexSets::Edge::GetSolutionCost,
            py::arg("result"), edge_doc.GetSolutionCost.doc)
        .def("GetSolutionPhiXu", &GraphOfConvexSets::Edge::GetSolutionPhiXu,
            py::arg("result"), edge_doc.GetSolutionPhiXu.doc)
        .def("GetSolutionPhiXv", &GraphOfConvexSets::Edge::GetSolutionPhiXv,
            py::arg("result"), edge_doc.GetSolutionPhiXv.doc);
  }

  // NOLINTNEXTLINE(readability/fn_size)
}

}  // namespace pydrake
}  // namespace drake
