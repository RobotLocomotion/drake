#include <memory>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include "drake/bindings/generated_docstrings/planning_continuous_collision.h"
#include "drake/bindings/pydrake/planning/planning_py.h"
#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/planning/continuous_collision/bounding_sphere.h"
#include "drake/planning/continuous_collision/certificate.h"
#include "drake/planning/continuous_collision/continuous_collision_checker.h"
#include "drake/planning/continuous_collision/distance_oracle.h"
#include "drake/planning/continuous_collision/motion_bound_table.h"
#include "drake/planning/continuous_collision/numerics.h"
#include "drake/planning/continuous_collision/options.h"
#include "drake/planning/continuous_collision/piecewise_bezier_path.h"
#include "drake/planning/continuous_collision/vpolytope_ingestion.h"
#include "drake/planning/robot_diagram.h"

namespace drake {
namespace pydrake {
namespace internal {

void DefinePlanningContinuousCollision(py::module m) {
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::planning::continuous_collision;
  constexpr auto& doc = pydrake_doc_planning_continuous_collision.drake.planning
                            .continuous_collision;

  using drake::planning::RobotDiagram;

  m.doc() = R"""(
Certified continuous collision checking: proves that a trajectory is
collision-free over its entire continuous time domain, rather than sampling it.
)""";

  // options.h
  {
    using Class = SearchMode;
    constexpr auto& cls_doc = doc.SearchMode;
    py::enum_<Class>(m, "SearchMode", cls_doc.doc)
        .value("kFindFirstViolation", Class::kFindFirstViolation,
            cls_doc.kFindFirstViolation.doc)
        .value("kCertifyAll", Class::kCertifyAll, cls_doc.kCertifyAll.doc);
  }

  {
    using Class = Verdict;
    constexpr auto& cls_doc = doc.Verdict;
    py::enum_<Class>(m, "Verdict", cls_doc.doc)
        .value(
            "kCertifiedFree", Class::kCertifiedFree, cls_doc.kCertifiedFree.doc)
        .value("kViolationFound", Class::kViolationFound,
            cls_doc.kViolationFound.doc)
        .value("kInconclusive", Class::kInconclusive, cls_doc.kInconclusive.doc)
        .value("kBudgetExhausted", Class::kBudgetExhausted,
            cls_doc.kBudgetExhausted.doc);
  }

  {
    using Class = Options;
    constexpr auto& cls_doc = doc.Options;
    py::class_<Class> cls(m, "Options", cls_doc.doc);
    cls  // BR
        .def(py::init<>())
        .def(ParamInit<Class>())
        .def_readwrite("margin", &Class::margin, cls_doc.margin.doc)
        .def_readwrite("continuity_tolerance", &Class::continuity_tolerance,
            cls_doc.continuity_tolerance.doc)
        .def_readwrite("query_tolerance", &Class::query_tolerance,
            cls_doc.query_tolerance.doc)
        .def_readwrite("certificate_slack", &Class::certificate_slack,
            cls_doc.certificate_slack.doc)
        .def_readwrite(
            "min_interval", &Class::min_interval, cls_doc.min_interval.doc)
        .def_readwrite("continuous_revolute_indices",
            &Class::continuous_revolute_indices,
            cls_doc.continuous_revolute_indices.doc)
        .def_readwrite("max_conversion_degree", &Class::max_conversion_degree,
            cls_doc.max_conversion_degree.doc)
        .def_readwrite("mode", &Class::mode, cls_doc.mode.doc)
        .def_readwrite("max_reported_findings", &Class::max_reported_findings,
            cls_doc.max_reported_findings.doc)
        .def_readwrite("max_nodes", &Class::max_nodes, cls_doc.max_nodes.doc)
        .def_readwrite("emit_certificate", &Class::emit_certificate,
            cls_doc.emit_certificate.doc)
        .def_readwrite(
            "parallelism", &Class::parallelism, cls_doc.parallelism.doc);
    DefCopyAndDeepCopy(&cls);
  }

  {
    using Class = PaddingSpec;
    constexpr auto& cls_doc = doc.PaddingSpec;
    py::class_<Class> cls(m, "PaddingSpec", cls_doc.doc);
    cls  // BR
        .def(py::init<>())
        .def(ParamInit<Class>())
        .def_readwrite(
            "env_padding", &Class::env_padding, cls_doc.env_padding.doc)
        .def_readwrite(
            "self_padding", &Class::self_padding, cls_doc.self_padding.doc)
        .def_readwrite(
            "per_body_pair", &Class::per_body_pair, cls_doc.per_body_pair.doc);
    DefCopyAndDeepCopy(&cls);
  }

  {
    using Class = PairId;
    constexpr auto& cls_doc = doc.PairId;
    py::class_<Class> cls(m, "PairId", cls_doc.doc);
    cls  // BR
        .def(py::init<>())
        .def(ParamInit<Class>())
        .def_readwrite("a", &Class::a, cls_doc.a.doc)
        .def_readwrite("b", &Class::b, cls_doc.b.doc)
        .def_readwrite("body_a", &Class::body_a, cls_doc.body_a.doc)
        .def_readwrite("body_b", &Class::body_b, cls_doc.body_b.doc);
    DefCopyAndDeepCopy(&cls);
  }

  {
    using Class = Finding;
    constexpr auto& cls_doc = doc.Finding;
    py::class_<Class> cls(m, "Finding", cls_doc.doc);
    cls  // BR
        .def(py::init<>())
        .def(ParamInit<Class>())
        .def_readwrite("time", &Class::time, cls_doc.time.doc)
        .def_readwrite("q", &Class::q, cls_doc.q.doc)
        .def_readwrite("pair", &Class::pair, cls_doc.pair.doc)
        .def_readwrite("distance", &Class::distance, cls_doc.distance.doc)
        .def_readwrite(
            "motion_bound", &Class::motion_bound, cls_doc.motion_bound.doc)
        .def_readwrite("definite", &Class::definite, cls_doc.definite.doc)
        .def_readwrite(
            "nearest_a_W", &Class::nearest_a_W, cls_doc.nearest_a_W.doc)
        .def_readwrite(
            "nearest_b_W", &Class::nearest_b_W, cls_doc.nearest_b_W.doc);
    DefCopyAndDeepCopy(&cls);
  }

  {
    using Class = Statistics;
    constexpr auto& cls_doc = doc.Statistics;
    py::class_<Class> cls(m, "Statistics", cls_doc.doc);
    cls  // BR
        .def(py::init<>())
        .def(ParamInit<Class>())
        .def_readwrite("nodes", &Class::nodes, cls_doc.nodes.doc)
        .def_readwrite("narrowphase_queries", &Class::narrowphase_queries,
            cls_doc.narrowphase_queries.doc)
        .def_readwrite("sphere_certifications", &Class::sphere_certifications,
            cls_doc.sphere_certifications.doc)
        .def_readwrite("max_depth", &Class::max_depth, cls_doc.max_depth.doc)
        .def_readwrite(
            "wall_time_s", &Class::wall_time_s, cls_doc.wall_time_s.doc);
    DefCopyAndDeepCopy(&cls);
  }

  // bounding_sphere.h
  {
    using Class = BoundingSphere;
    constexpr auto& cls_doc = doc.BoundingSphere;
    py::class_<Class> cls(m, "BoundingSphere", cls_doc.doc);
    cls  // BR
        .def(py::init<>())
        .def(ParamInit<Class>())
        .def_readwrite("center_L", &Class::center_L, cls_doc.center_L.doc)
        .def_readwrite("radius", &Class::radius, cls_doc.radius.doc);
    DefCopyAndDeepCopy(&cls);
  }

  m.def("ComputeBoundingSphere", &ComputeBoundingSphere, py::arg("shape"),
      py::arg("X_LG"), doc.ComputeBoundingSphere.doc);

  // piecewise_bezier_path.h
  {
    using Class = BezierSegment;
    constexpr auto& cls_doc = doc.BezierSegment;
    py::class_<Class> cls(m, "BezierSegment", cls_doc.doc);
    cls  // BR
        .def(py::init<>())
        .def(ParamInit<Class>())
        .def_readwrite("t_start", &Class::t_start, cls_doc.t_start.doc)
        .def_readwrite("t_end", &Class::t_end, cls_doc.t_end.doc)
        .def_readwrite("control_points", &Class::control_points,
            cls_doc.control_points.doc);
    DefCopyAndDeepCopy(&cls);
  }

  {
    using Class = PiecewiseBezierPath;
    constexpr auto& cls_doc = doc.PiecewiseBezierPath;
    py::class_<Class> cls(m, "PiecewiseBezierPath", cls_doc.doc);
    cls  // BR
        .def_static("FromTrajectory", &Class::FromTrajectory,
            py::arg("trajectory"), py::arg("options"),
            cls_doc.FromTrajectory.doc)
        .def_static("FromWaypoints", &Class::FromWaypoints,
            py::arg("waypoints"), py::arg("options"), cls_doc.FromWaypoints.doc)
        .def("num_positions", &Class::num_positions, cls_doc.num_positions.doc)
        .def("segments", &Class::segments, cls_doc.segments.doc)
        .def("start_time", &Class::start_time, cls_doc.start_time.doc)
        .def("end_time", &Class::end_time, cls_doc.end_time.doc)
        .def("global_lower_bound", &Class::global_lower_bound,
            cls_doc.global_lower_bound.doc)
        .def("global_upper_bound", &Class::global_upper_bound,
            cls_doc.global_upper_bound.doc)
        .def("constant_coordinates", &Class::constant_coordinates,
            cls_doc.constant_coordinates.doc)
        .def("Value", &Class::Value, py::arg("t"), cls_doc.Value.doc)
        .def("EvaluateSegment", &Class::EvaluateSegment,
            py::arg("segment_index"), py::arg("s"),
            cls_doc.EvaluateSegment.doc);
    DefCopyAndDeepCopy(&cls);
  }

  m.def(
      "DeCasteljauSplitAtHalf",
      [](const Eigen::MatrixXd& cps) {
        Eigen::MatrixXd left;
        Eigen::MatrixXd right;
        Eigen::VectorXd mid;
        DeCasteljauSplitAtHalf(cps, &left, &right, &mid);
        return std::make_tuple(
            std::move(left), std::move(right), std::move(mid));
      },
      py::arg("cps"),
      (std::string(doc.DeCasteljauSplitAtHalf.doc) +
          "\n\n"
          "Note:\n"
          "    Unlike the C++ signature, which writes through output "
          "pointers, this returns a tuple ``(left, right, mid)``.")
          .c_str());

  // motion_bound_table.h
  {
    using Class = MotionBoundTable;
    constexpr auto& cls_doc = doc.MotionBoundTable;
    py::class_<Class> cls(m, "MotionBoundTable", cls_doc.doc);
    cls  // BR
        .def(py::init<>(), cls_doc.ctor.doc_0args)
        .def(py::init<std::vector<int>, std::vector<int>, std::vector<double>,
                 std::vector<double>>(),
            py::arg("row_start"), py::arg("coord"), py::arg("lambda"),
            py::arg("carveout_slack"), cls_doc.ctor.doc_4args)
        .def("num_pairs", &Class::num_pairs, cls_doc.num_pairs.doc)
        .def("pair_is_static", &Class::pair_is_static, py::arg("pair_index"),
            cls_doc.pair_is_static.doc)
        .def("MotionBound", &Class::MotionBound, py::arg("pair_index"),
            py::arg("w"), cls_doc.MotionBound.doc)
        .def("carveout_slack", &Class::carveout_slack, py::arg("pair_index"),
            cls_doc.carveout_slack.doc)
        .def("GetEntries", &Class::GetEntries, py::arg("pair_index"),
            cls_doc.GetEntries.doc)
        .def("num_entries", &Class::num_entries, cls_doc.num_entries.doc);
    DefCopyAndDeepCopy(&cls);
  }

  {
    using Class = KinematicsEngine;
    constexpr auto& cls_doc = doc.KinematicsEngine;
    py::class_<Class> cls(m, "KinematicsEngine", cls_doc.doc);
    cls  // BR
        .def(py::init<const RobotDiagram<double>&>(), py::arg("model"),
            // Keep the model alive as long as the engine: the C++ object
            // aliases it (see the constructor's documentation).
            py::keep_alive<1, 2>(), cls_doc.ctor.doc)
        .def("CoordinatesAffectingPair", &Class::CoordinatesAffectingPair,
            py::arg("body_a"), py::arg("body_b"),
            cls_doc.CoordinatesAffectingPair.doc)
        .def("ComputeMotionBoundTable",
            overload_cast_explicit<MotionBoundTable, const PiecewiseBezierPath&,
                const std::vector<PairId>&>(&Class::ComputeMotionBoundTable),
            py::arg("path"), py::arg("pairs"),
            cls_doc.ComputeMotionBoundTable.doc_2args)
        .def("ComputeMotionBoundTable",
            overload_cast_explicit<MotionBoundTable, const Eigen::VectorXd&,
                const Eigen::VectorXd&, const std::vector<bool>&,
                const std::vector<PairId>&>(&Class::ComputeMotionBoundTable),
            py::arg("lower"), py::arg("upper"), py::arg("constant_coordinates"),
            py::arg("pairs"), cls_doc.ComputeMotionBoundTable.doc_4args)
        .def("body_spheres", &Class::body_spheres, py::arg("body"),
            cls_doc.body_spheres.doc)
        .def("body_sphere_geometries", &Class::body_sphere_geometries,
            py::arg("body"), cls_doc.body_sphere_geometries.doc)
        .def("geometry_sphere", &Class::geometry_sphere, py::arg("id"),
            cls_doc.geometry_sphere.doc)
        .def("body_has_halfspace", &Class::body_has_halfspace, py::arg("body"),
            cls_doc.body_has_halfspace.doc)
        .def("body_radius", &Class::body_radius, py::arg("body"),
            cls_doc.body_radius.doc)
        .def("num_positions", &Class::num_positions, cls_doc.num_positions.doc)
        .def("plant", &Class::plant, py_rvp::reference_internal,
            cls_doc.plant.doc);
  }

  // distance_oracle.h
  {
    using Class = DistanceRoute;
    constexpr auto& cls_doc = doc.DistanceRoute;
    py::enum_<Class>(m, "DistanceRoute", cls_doc.doc)
        .value("kNative", Class::kNative, cls_doc.kNative.doc)
        .value("kHalfSpaceA", Class::kHalfSpaceA, cls_doc.kHalfSpaceA.doc)
        .value("kHalfSpaceB", Class::kHalfSpaceB, cls_doc.kHalfSpaceB.doc);
  }

  {
    using Class = PairRecord;
    constexpr auto& cls_doc = doc.PairRecord;
    py::class_<Class> cls(m, "PairRecord", cls_doc.doc);
    cls  // BR
        .def(py::init<>())
        .def(ParamInit<Class>())
        .def_readwrite("id", &Class::id, cls_doc.id.doc)
        .def_readwrite("route", &Class::route, cls_doc.route.doc)
        .def_readwrite("threshold", &Class::threshold, cls_doc.threshold.doc);
    DefCopyAndDeepCopy(&cls);
  }

  {
    using Class = DistanceOracle;
    constexpr auto& cls_doc = doc.DistanceOracle;
    py::class_<Class> cls(m, "DistanceOracle", cls_doc.doc);
    cls  // BR
        .def(py::init<const RobotDiagram<double>&, double>(), py::arg("model"),
            py::arg("query_tolerance"), cls_doc.ctor.doc)
        .def("pairs", &Class::pairs, cls_doc.pairs.doc)
        .def(
            "SignedDistance",
            [](const Class& self,
                const geometry::QueryObject<double>& query_object,
                const PairRecord& pair) {
              Eigen::Vector3d nearest_a_W = Eigen::Vector3d::Zero();
              Eigen::Vector3d nearest_b_W = Eigen::Vector3d::Zero();
              const double distance = self.SignedDistance(
                  query_object, pair, &nearest_a_W, &nearest_b_W);
              return std::make_tuple(distance, nearest_a_W, nearest_b_W);
            },
            py::arg("query_object"), py::arg("pair"),
            (std::string(cls_doc.SignedDistance.doc) +
                "\n\n"
                "Note:\n"
                "    Unlike the C++ signature, which reports the closest "
                "points through optional output pointers, this returns a "
                "tuple ``(distance, nearest_a_W, nearest_b_W)``.")
                .c_str())
        .def("tolerance", &Class::tolerance, cls_doc.tolerance.doc)
        .def("support_report", &Class::support_report,
            cls_doc.support_report.doc);
    DefCopyAndDeepCopy(&cls);
  }

  // certificate.h
  {
    using Class = CertificateRecord;
    constexpr auto& cls_doc = doc.CertificateRecord;
    py::class_<Class> cls(m, "CertificateRecord", cls_doc.doc);
    cls  // BR
        .def(py::init<>())
        .def(ParamInit<Class>())
        .def_readwrite("segment", &Class::segment, cls_doc.segment.doc)
        .def_readwrite("s_start", &Class::s_start, cls_doc.s_start.doc)
        .def_readwrite("s_end", &Class::s_end, cls_doc.s_end.doc)
        .def_readwrite("pair_index", &Class::pair_index, cls_doc.pair_index.doc)
        .def_readwrite("qc", &Class::qc, cls_doc.qc.doc)
        .def_readwrite("phi_hat", &Class::phi_hat, cls_doc.phi_hat.doc)
        .def_readwrite(
            "motion_bound", &Class::motion_bound, cls_doc.motion_bound.doc)
        .def_readwrite("threshold", &Class::threshold, cls_doc.threshold.doc);
    DefCopyAndDeepCopy(&cls);
  }

  {
    using Class = Certificate;
    constexpr auto& cls_doc = doc.Certificate;
    py::class_<Class> cls(m, "Certificate", cls_doc.doc);
    cls  // BR
        .def(py::init<>())
        .def(ParamInit<Class>())
        .def_readwrite("records", &Class::records, cls_doc.records.doc)
        .def_readwrite("pairs", &Class::pairs, cls_doc.pairs.doc);
    DefCopyAndDeepCopy(&cls);
  }

  // continuous_collision_checker.h
  {
    using Class = CertificationResult;
    constexpr auto& cls_doc = doc.CertificationResult;
    py::class_<Class> cls(m, "CertificationResult", cls_doc.doc);
    cls  // BR
        .def(py::init<>())
        .def(ParamInit<Class>())
        .def_readwrite("verdict", &Class::verdict, cls_doc.verdict.doc)
        .def_readwrite("findings", &Class::findings, cls_doc.findings.doc)
        .def_readwrite("stats", &Class::stats, cls_doc.stats.doc)
        .def_readwrite(
            "certificate", &Class::certificate, cls_doc.certificate.doc);
    DefCopyAndDeepCopy(&cls);
  }

  {
    using Class = ContinuousCollisionChecker;
    constexpr auto& cls_doc = doc.ContinuousCollisionChecker;
    py::class_<Class> cls(m, "ContinuousCollisionChecker", cls_doc.doc);

    {
      using Nested = Class::Params;
      constexpr auto& nested_doc = cls_doc.Params;
      py::class_<Nested> nested_cls(cls, "Params", nested_doc.doc);
      nested_cls  // BR
          .def(py::init<>())
          .def(ParamInit<Nested>())
          .def_property(
              "model",
              [](const Nested& self) -> const RobotDiagram<double>* {
                return self.model.get();
              },
              [](Nested& self, py::object model) {
                // Add a python reference to model (owned by the shared
                // pointer), and transfer that to the c++ params struct.
                self.model =
                    make_shared_ptr_from_py_object<RobotDiagram<double>>(model);
              },
              nested_doc.model.doc)
          .def_readwrite("padding", &Nested::padding, nested_doc.padding.doc)
          .def_readwrite("default_options", &Nested::default_options,
              nested_doc.default_options.doc);
    }

    py::object params_ctor = cls.attr("Params");
    cls  // BR
        .def(
            py::init([params_ctor](py::object model, const py::kwargs& kwargs) {
              // For lifetime management, we need to treat pointer-like
              // arguments separately. Start by creating a Params object in
              // Python with all of the other non-pointer kwargs.
              py::object params_py = params_ctor(**kwargs);
              auto* params = params_py.cast<Class::Params*>();
              DRAKE_DEMAND(params != nullptr);
              // Now, add a python reference to model (owned by the shared
              // pointer), and transfer that to the c++ checker.
              params->model =
                  make_shared_ptr_from_py_object<RobotDiagram<double>>(model);
              return std::make_unique<Class>(std::move(*params));
            }),
            py::kw_only(), py::arg("model"),
            (std::string(cls_doc.ctor.doc) +
                "\n\n"
                "See :class:`pydrake.planning.continuous_collision"
                ".ContinuousCollisionChecker.Params` for the list of "
                "properties available here as kwargs.")
                .c_str())
        .def(py::init<Class::Params>(), py::arg("params"), cls_doc.ctor.doc)
        .def("CheckTrajectory", &Class::CheckTrajectory, py::arg("trajectory"),
            py::arg("options") = std::nullopt, cls_doc.CheckTrajectory.doc)
        .def("CheckPath", &Class::CheckPath, py::arg("waypoints"),
            py::arg("options") = std::nullopt, cls_doc.CheckPath.doc)
        .def("CheckEdge", &Class::CheckEdge, py::arg("q1"), py::arg("q2"),
            py::arg("options") = std::nullopt, cls_doc.CheckEdge.doc)
        .def("Normalize", &Class::Normalize, py::arg("trajectory"),
            py::arg("options") = std::nullopt, cls_doc.Normalize.doc)
        .def("ComputeMotionBounds", &Class::ComputeMotionBounds,
            py::arg("path"), cls_doc.ComputeMotionBounds.doc)
        .def("distance_oracle", &Class::distance_oracle,
            py_rvp::reference_internal, cls_doc.distance_oracle.doc)
        .def("kinematics_engine", &Class::kinematics_engine,
            py_rvp::reference_internal, cls_doc.kinematics_engine.doc)
        .def("pairs", &Class::pairs, cls_doc.pairs.doc)
        .def("model", &Class::model, py_rvp::reference_internal,
            cls_doc.model.doc);
  }

  m.def("VerifyCertificate", &VerifyCertificate, py::arg("checker"),
      py::arg("path"), py::arg("certificate"), doc.VerifyCertificate.doc);

  // vpolytope_ingestion.h
  m.def("AddVPolytopeObstacle", &AddVPolytopeObstacle, py::arg("plant"),
      py::arg("vpoly"), py::arg("X_WG"), py::arg("name"),
      doc.AddVPolytopeObstacle.doc);

  // numerics.h
  m.def("IsCertified", &IsCertified, py::arg("phi_hat"), py::arg("tau"),
      py::arg("motion_bound"), py::arg("threshold"), py::arg("slack"),
      doc.IsCertified.doc);

  m.def("IsDefiniteViolation", &IsDefiniteViolation, py::arg("phi_hat"),
      py::arg("tau"), py::arg("threshold"), doc.IsDefiniteViolation.doc);
}

}  // namespace internal
}  // namespace pydrake
}  // namespace drake
