/* @file The contains all of the public entities found in the
 drake::geometry::optimization namespace. They can be found in the
 pydrake.geometry.optimization module. */

#include "drake/bindings/pydrake/common/cpp_template_pybind.h"
#include "drake/bindings/pydrake/common/default_scalars_pybind.h"
#include "drake/bindings/pydrake/common/deprecation_pybind.h"
#include "drake/bindings/pydrake/common/identifier_pybind.h"
#include "drake/bindings/pydrake/common/serialize_pybind.h"
#include "drake/bindings/pydrake/common/sorted_pair_pybind.h"
#include "drake/bindings/pydrake/common/value_pybind.h"
#include "drake/bindings/pydrake/documentation_pybind.h"
#include "drake/bindings/pydrake/geometry/geometry_py.h"
#include "drake/bindings/pydrake/geometry/optimization_pybind.h"
#include "drake/bindings/pydrake/polynomial_types_pybind.h"
#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/common/yaml/yaml_io.h"
#include "drake/geometry/optimization/affine_ball.h"
#include "drake/geometry/optimization/affine_subspace.h"
#include "drake/geometry/optimization/c_iris_collision_geometry.h"
#include "drake/geometry/optimization/cartesian_product.h"
#include "drake/geometry/optimization/cspace_free_polytope.h"
#include "drake/geometry/optimization/cspace_free_polytope_base.h"
#include "drake/geometry/optimization/cspace_free_structs.h"
#include "drake/geometry/optimization/cspace_separating_plane.h"
#include "drake/geometry/optimization/geodesic_convexity.h"
#include "drake/geometry/optimization/graph_of_convex_sets.h"
#include "drake/geometry/optimization/hpolyhedron.h"
#include "drake/geometry/optimization/hyperellipsoid.h"
#include "drake/geometry/optimization/hyperrectangle.h"
#include "drake/geometry/optimization/intersection.h"
#include "drake/geometry/optimization/iris.h"
#include "drake/geometry/optimization/minkowski_sum.h"
#include "drake/geometry/optimization/point.h"
#include "drake/geometry/optimization/spectrahedron.h"
#include "drake/geometry/optimization/vpolytope.h"

namespace drake {
namespace pydrake {

// CSpaceSeparatingPlane, CIrisSeparatingPlane
template <typename T>
void DoSeparatingPlaneDeclaration(py::module m, T) {
  constexpr auto& doc = pydrake_doc.drake.geometry.optimization;
  py::tuple param = GetPyParam<T>();
  using Class = geometry::optimization::CSpaceSeparatingPlane<T>;
  constexpr auto& base_cls_doc = doc.CSpaceSeparatingPlane;
  {
    auto cls =
        DefineTemplateClassWithDefault<Class>(
            m, "CSpaceSeparatingPlane", param, base_cls_doc.doc)
            // Use py_rvp::copy here because numpy.ndarray with dtype=object
            // arrays must be copied, and cannot be referenced.
            .def_readonly("a", &Class::a, py_rvp::copy, base_cls_doc.a.doc)
            .def_readonly("b", &Class::b, base_cls_doc.b.doc)
            .def_readonly("positive_side_geometry",
                &Class::positive_side_geometry,
                base_cls_doc.positive_side_geometry.doc)
            .def_readonly("negative_side_geometry",
                &Class::negative_side_geometry,
                base_cls_doc.negative_side_geometry.doc)
            .def_readonly("expressed_body", &Class::expressed_body,
                base_cls_doc.expressed_body.doc)
            .def_readonly("plane_degree", &Class::plane_degree)
            // Use py_rvp::copy here because numpy.ndarray with dtype=object
            // arrays must be copied, and cannot be referenced.
            .def_readonly("decision_variables", &Class::decision_variables,
                py_rvp::copy, base_cls_doc.a.doc);
    DefCopyAndDeepCopy(&cls);
    AddValueInstantiation<Class>(m);
  }
}

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

  // SampledVolume. This struct must be declared before ConvexSet as methods in
  // ConvexSet depend on this struct.
  {
    py::class_<SampledVolume>(m, "SampledVolume", doc.SampledVolume.doc)
        .def_readwrite(
            "volume", &SampledVolume::volume, doc.SampledVolume.volume.doc)
        .def_readwrite("rel_accuracy", &SampledVolume::rel_accuracy,
            doc.SampledVolume.rel_accuracy.doc)
        .def_readwrite("num_samples", &SampledVolume::num_samples,
            doc.SampledVolume.num_samples.doc);
  }
  // ConvexSet
  {
    const auto& cls_doc = doc.ConvexSet;
    py::class_<ConvexSet>(m, "ConvexSet", cls_doc.doc)
        .def("Clone", &ConvexSet::Clone, cls_doc.Clone.doc)
        .def("ambient_dimension", &ConvexSet::ambient_dimension,
            cls_doc.ambient_dimension.doc)
        .def("IntersectsWith", &ConvexSet::IntersectsWith, py::arg("other"),
            cls_doc.IntersectsWith.doc)
        .def("IsBounded", &ConvexSet::IsBounded, cls_doc.IsBounded.doc)
        .def("IsEmpty", &ConvexSet::IsEmpty, cls_doc.IsEmpty.doc)
        .def("MaybeGetPoint", &ConvexSet::MaybeGetPoint,
            cls_doc.MaybeGetPoint.doc)
        .def("MaybeGetFeasiblePoint", &ConvexSet::MaybeGetFeasiblePoint,
            cls_doc.MaybeGetFeasiblePoint.doc)
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
            cls_doc.ToShapeWithPose.doc)
        .def("CalcVolume", &ConvexSet::CalcVolume, cls_doc.CalcVolume.doc)
        .def("CalcVolumeViaSampling", &ConvexSet::CalcVolumeViaSampling,
            py::arg("generator"), py::arg("desired_rel_accuracy") = 1e-2,
            py::arg("max_num_samples") = 1e4,
            cls_doc.CalcVolumeViaSampling.doc);
  }
  // There is a dependency cycle between Hyperellipsoid and AffineBall, so we
  // need to "forward declare" the Hyperellipsoid class here.
  py::class_<Hyperellipsoid, ConvexSet> hyperellipsoid_cls(
      m, "Hyperellipsoid", doc.Hyperellipsoid.doc);

  // AffineBall
  {
    const auto& cls_doc = doc.AffineBall;
    py::class_<AffineBall, ConvexSet> cls(m, "AffineBall", cls_doc.doc);
    cls  // BR
        .def(py::init<>(), cls_doc.ctor.doc_0args)
        .def(py::init<const Eigen::Ref<const Eigen::MatrixXd>&,
                 const Eigen::Ref<const Eigen::VectorXd>&>(),
            py::arg("B"), py::arg("center"), cls_doc.ctor.doc_2args)
        .def(py::init<const Hyperellipsoid&>(), py::arg("ellipsoid"),
            cls_doc.ctor.doc_1args)
        .def("B", &AffineBall::B, py_rvp::reference_internal, cls_doc.B.doc)
        .def("center", &AffineBall::center, py_rvp::reference_internal,
            cls_doc.center.doc)
        .def_static("MinimumVolumeCircumscribedEllipsoid",
            &AffineBall::MinimumVolumeCircumscribedEllipsoid, py::arg("points"),
            py::arg("rank_tol") = 1e-6,
            cls_doc.MinimumVolumeCircumscribedEllipsoid.doc)
        .def_static("MakeAxisAligned", &AffineBall::MakeAxisAligned,
            py::arg("radius"), py::arg("center"), cls_doc.MakeAxisAligned.doc)
        .def_static("MakeHypersphere", &AffineBall::MakeHypersphere,
            py::arg("radius"), py::arg("center"), cls_doc.MakeHypersphere.doc)
        .def_static("MakeUnitBall", &AffineBall::MakeUnitBall, py::arg("dim"),
            cls_doc.MakeUnitBall.doc);
    DefClone(&cls);
  }

  // AffineSubspace
  {
    const auto& cls_doc = doc.AffineSubspace;
    py::class_<AffineSubspace, ConvexSet> cls(m, "AffineSubspace", cls_doc.doc);
    cls  // BR
        .def(py::init<>(), cls_doc.ctor.doc_0args)
        .def(py::init<const Eigen::Ref<const Eigen::MatrixXd>&,
                 const Eigen::Ref<const Eigen::VectorXd>&>(),
            py::arg("basis"), py::arg("translation"),
            cls_doc.ctor.doc_2args_basis_translation)
        .def(py::init<const ConvexSet&, double>(), py::arg("set"),
            py::arg("tol") = 1E-12, cls_doc.ctor.doc_2args_set_tol)
        .def("basis", &AffineSubspace::basis, py_rvp::reference_internal,
            cls_doc.basis.doc)
        .def("translation", &AffineSubspace::translation,
            py_rvp::reference_internal, cls_doc.translation.doc)
        .def("AffineDimension", &AffineSubspace::AffineDimension,
            cls_doc.AffineDimension.doc)
        .def("Project", &AffineSubspace::Project, py::arg("x"),
            cls_doc.Project.doc)
        .def("ToLocalCoordinates", &AffineSubspace::ToLocalCoordinates,
            py::arg("x"), cls_doc.ToLocalCoordinates.doc)
        .def("ToGlobalCoordinates", &AffineSubspace::ToGlobalCoordinates,
            py::arg("y"), cls_doc.ToGlobalCoordinates.doc)
        .def("ContainedIn", &AffineSubspace::ContainedIn, py::arg("other"),
            py::arg("tol") = 1e-15, cls_doc.ContainedIn.doc)
        .def("IsNearlyEqualTo", &AffineSubspace::IsNearlyEqualTo,
            py::arg("other"), py::arg("tol") = 1e-15, cls_doc.ContainedIn.doc)
        .def("OrthogonalComplementBasis",
            &AffineSubspace::OrthogonalComplementBasis,
            cls_doc.OrthogonalComplementBasis.doc);
    DefClone(&cls);
  }

  // CartesianProduct
  {
    const auto& cls_doc = doc.CartesianProduct;
    py::class_<CartesianProduct, ConvexSet>(m, "CartesianProduct", cls_doc.doc)
        .def(py::init<>(), cls_doc.ctor.doc_0args)
        .def(py::init([](const std::vector<ConvexSet*>& sets) {
          return std::make_unique<CartesianProduct>(CloneConvexSets(sets));
        }),
            py::arg("sets"), cls_doc.ctor.doc_1args_sets)
        .def(py::init<const ConvexSet&, const ConvexSet&>(), py::arg("setA"),
            py::arg("setB"), cls_doc.ctor.doc_2args_setA_setB)
        .def(py::init([](const std::vector<ConvexSet*>& sets,
                          const Eigen::Ref<const Eigen::MatrixXd>& A,
                          const Eigen::Ref<const Eigen::VectorXd>& b) {
          return std::make_unique<CartesianProduct>(
              CloneConvexSets(sets), A, b);
        }),
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
  }

  // HPolyhedron
  {
    const auto& cls_doc = doc.HPolyhedron;
    py::class_<HPolyhedron, ConvexSet>(m, "HPolyhedron", cls_doc.doc)
        .def(py::init<>(), cls_doc.ctor.doc_0args)
        .def(py::init<const Eigen::Ref<const Eigen::MatrixXd>&,
                 const Eigen::Ref<const Eigen::VectorXd>&>(),
            py::arg("A"), py::arg("b"), cls_doc.ctor.doc_2args_A_b)
        .def(py::init<const QueryObject<double>&, GeometryId,
                 std::optional<FrameId>>(),
            py::arg("query_object"), py::arg("geometry_id"),
            py::arg("reference_frame") = std::nullopt,
            cls_doc.ctor.doc_3args_query_object_geometry_id_reference_frame)
        .def(py::init<const VPolytope&, double>(), py::arg("vpoly"),
            py::arg("tol") = 1E-9, cls_doc.ctor.doc_2args_vpoly_tol)
        .def(py::init<const solvers::MathematicalProgram&>(), py::arg("prog"),
            cls_doc.ctor.doc_1args_prog)
        .def("A", &HPolyhedron::A, cls_doc.A.doc)
        .def("b", &HPolyhedron::b, cls_doc.b.doc)
        .def("ContainedIn", &HPolyhedron::ContainedIn, py::arg("other"),
            py::arg("tol") = 1E-9, cls_doc.ContainedIn.doc)
        .def("Intersection", &HPolyhedron::Intersection, py::arg("other"),
            py::arg("check_for_redundancy") = false, py::arg("tol") = 1E-9,
            cls_doc.Intersection.doc)
        .def("ReduceInequalities", &HPolyhedron::ReduceInequalities,
            py::arg("tol") = 1E-9, cls_doc.ReduceInequalities.doc)
        .def("FindRedundant", &HPolyhedron::FindRedundant,
            py::arg("tol") = 1E-9, cls_doc.FindRedundant.doc)
        .def("MaximumVolumeInscribedEllipsoid",
            &HPolyhedron::MaximumVolumeInscribedEllipsoid,
            cls_doc.MaximumVolumeInscribedEllipsoid.doc)
        .def("ChebyshevCenter", &HPolyhedron::ChebyshevCenter,
            cls_doc.ChebyshevCenter.doc)
        .def("Scale", &HPolyhedron::Scale, py::arg("scale"),
            py::arg("center") = std::nullopt, cls_doc.Scale.doc)
        .def("CartesianProduct", &HPolyhedron::CartesianProduct,
            py::arg("other"), cls_doc.CartesianProduct.doc)
        .def("CartesianPower", &HPolyhedron::CartesianPower, py::arg("n"),
            cls_doc.CartesianPower.doc)
        .def("PontryaginDifference", &HPolyhedron::PontryaginDifference,
            py::arg("other"), cls_doc.PontryaginDifference.doc)
        .def("UniformSample",
            overload_cast_explicit<Eigen::VectorXd, RandomGenerator*,
                const Eigen::Ref<const Eigen::VectorXd>&, int>(
                &HPolyhedron::UniformSample),
            py::arg("generator"), py::arg("previous_sample"),
            py::arg("mixing_steps") = 10, cls_doc.UniformSample.doc_3args)
        .def("UniformSample",
            overload_cast_explicit<Eigen::VectorXd, RandomGenerator*, int>(
                &HPolyhedron::UniformSample),
            py::arg("generator"), py::arg("mixing_steps") = 10,
            cls_doc.UniformSample.doc_2args)
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
  }

  // Hyperellipsoid
  {
    const auto& cls_doc = doc.Hyperellipsoid;
    hyperellipsoid_cls  // BR
        .def(py::init<>(), cls_doc.ctor.doc_0args)
        .def(py::init<const Eigen::Ref<const Eigen::MatrixXd>&,
                 const Eigen::Ref<const Eigen::VectorXd>&>(),
            py::arg("A"), py::arg("center"), cls_doc.ctor.doc_2args)
        .def(py::init<const QueryObject<double>&, GeometryId,
                 std::optional<FrameId>>(),
            py::arg("query_object"), py::arg("geometry_id"),
            py::arg("reference_frame") = std::nullopt, cls_doc.ctor.doc_3args)
        .def(py::init<const AffineBall&>(), py::arg("ellipsoid"),
            cls_doc.ctor.doc_1args)
        .def("A", &Hyperellipsoid::A, cls_doc.A.doc)
        .def("center", &Hyperellipsoid::center, cls_doc.center.doc)
        .def("MinimumUniformScalingToTouch",
            &Hyperellipsoid::MinimumUniformScalingToTouch, py::arg("other"),
            cls_doc.MinimumUniformScalingToTouch.doc)
        .def_static("MakeAxisAligned", &Hyperellipsoid::MakeAxisAligned,
            py::arg("radius"), py::arg("center"), cls_doc.MakeAxisAligned.doc)
        .def_static("MakeHypersphere", &Hyperellipsoid::MakeHypersphere,
            py::arg("radius"), py::arg("center"), cls_doc.MakeHypersphere.doc)
        .def_static("MakeUnitBall", &Hyperellipsoid::MakeUnitBall,
            py::arg("dim"), cls_doc.MakeUnitBall.doc)
        .def_static("MinimumVolumeCircumscribedEllipsoid",
            &Hyperellipsoid::MinimumVolumeCircumscribedEllipsoid,
            py::arg("points"), py::arg("rank_tol") = 1e-6,
            cls_doc.MinimumVolumeCircumscribedEllipsoid.doc)
        .def(py::pickle(
            [](const Hyperellipsoid& self) {
              return std::make_pair(self.A(), self.center());
            },
            [](std::pair<Eigen::MatrixXd, Eigen::VectorXd> args) {
              return Hyperellipsoid(std::get<0>(args), std::get<1>(args));
            }));
  }

  // Hyperrectangle
  {
    const auto& cls_doc = doc.Hyperrectangle;
    py::class_<Hyperrectangle, ConvexSet>(m, "Hyperrectangle", cls_doc.doc)
        .def(py::init<>(), cls_doc.ctor.doc_0args)
        .def(py::init<const Eigen::Ref<const Eigen::VectorXd>&,
                 const Eigen::Ref<const Eigen::VectorXd>&>(),
            py::arg("lb"), py::arg("ub"), cls_doc.ctor.doc_2args)
        .def("lb", &Hyperrectangle::lb, cls_doc.lb.doc)
        .def("ub", &Hyperrectangle::ub, cls_doc.ub.doc)
        .def("UniformSample", &Hyperrectangle::UniformSample,
            py::arg("generator"), cls_doc.UniformSample.doc)
        .def("Center", &Hyperrectangle::Center, cls_doc.Center.doc)
        .def("MakeHPolyhedron", &Hyperrectangle::MakeHPolyhedron,
            cls_doc.MakeHPolyhedron.doc)
        .def(py::pickle(
            [](const Hyperrectangle& self) {
              return std::make_pair(self.lb(), self.ub());
            },
            [](std::pair<Eigen::VectorXd, Eigen::VectorXd> args) {
              return Hyperrectangle(std::get<0>(args), std::get<1>(args));
            }))
        .def_static("MaybeCalcAxisAlignedBoundingBox",
            &Hyperrectangle::MaybeCalcAxisAlignedBoundingBox, py::arg("set"),
            cls_doc.MaybeCalcAxisAlignedBoundingBox.doc);
  }

  // Intersection
  {
    const auto& cls_doc = doc.Intersection;
    py::class_<Intersection, ConvexSet>(m, "Intersection", cls_doc.doc)
        .def(py::init<>(), cls_doc.ctor.doc_0args)
        .def(py::init([](const std::vector<ConvexSet*>& sets) {
          return std::make_unique<Intersection>(CloneConvexSets(sets));
        }),
            py::arg("sets"), cls_doc.ctor.doc_1args)
        .def(py::init<const ConvexSet&, const ConvexSet&>(), py::arg("setA"),
            py::arg("setB"), cls_doc.ctor.doc_2args)
        .def("num_elements", &Intersection::num_elements,
            cls_doc.num_elements.doc)
        .def("element", &Intersection::element, py_rvp::reference_internal,
            py::arg("index"), cls_doc.element.doc);
  }

  // MinkowskiSum
  {
    const auto& cls_doc = doc.MinkowskiSum;
    py::class_<MinkowskiSum, ConvexSet>(m, "MinkowskiSum", cls_doc.doc)
        .def(py::init<>(), cls_doc.ctor.doc_0args)
        .def(py::init([](const std::vector<ConvexSet*>& sets) {
          return std::make_unique<MinkowskiSum>(CloneConvexSets(sets));
        }),
            py::arg("sets"), cls_doc.ctor.doc_1args)
        .def(py::init<const ConvexSet&, const ConvexSet&>(), py::arg("setA"),
            py::arg("setB"), cls_doc.ctor.doc_2args)
        .def(py::init<const QueryObject<double>&, GeometryId,
                 std::optional<FrameId>>(),
            py::arg("query_object"), py::arg("geometry_id"),
            py::arg("reference_frame") = std::nullopt, cls_doc.ctor.doc_3args)
        .def("num_terms", &MinkowskiSum::num_terms, cls_doc.num_terms.doc)
        .def("term", &MinkowskiSum::term, py_rvp::reference_internal,
            py::arg("index"), cls_doc.term.doc);
  }

  // Point
  {
    const auto& cls_doc = doc.Point;
    py::class_<Point, ConvexSet>(m, "Point", cls_doc.doc)
        .def(py::init<>(), cls_doc.ctor.doc_0args)
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
  }

  // Spectrahedron
  {
    const auto& cls_doc = doc.Spectrahedron;
    py::class_<Spectrahedron, ConvexSet>(m, "Spectrahedron", cls_doc.doc)
        .def(py::init<>(), cls_doc.ctor.doc_0args)
        .def(py::init<const solvers::MathematicalProgram&>(), py::arg("prog"),
            cls_doc.ctor.doc_1args);
  }

  // VPolytope
  {
    const auto& cls_doc = doc.VPolytope;
    py::class_<VPolytope, ConvexSet>(m, "VPolytope", cls_doc.doc)
        .def(py::init<>(), cls_doc.ctor.doc)
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
        .def("WriteObj", &VPolytope::WriteObj, py::arg("filename"),
            cls_doc.WriteObj.doc)
        .def(py::pickle([](const VPolytope& self) { return self.vertices(); },
            [](Eigen::MatrixXd arg) { return VPolytope(arg); }));
  }

  {
    const auto& cls_doc = doc.IrisOptions;
    py::class_<IrisOptions> iris_options(m, "IrisOptions", cls_doc.doc);
    iris_options.def(ParamInit<IrisOptions>())
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
        .def_property(
            "configuration_obstacles",
            [](const IrisOptions& self) {
              std::vector<const ConvexSet*> convex_sets;
              for (const copyable_unique_ptr<ConvexSet>& convex_set :
                  self.configuration_obstacles) {
                convex_sets.push_back(convex_set.get());
              }
              py::object self_py = py::cast(self, py_rvp::reference);
              // Keep alive, ownership: each item in `convex_sets` keeps `self`
              // alive.
              return py::cast(convex_sets, py_rvp::reference_internal, self_py);
            },
            [](IrisOptions& self, const std::vector<ConvexSet*>& sets) {
              self.configuration_obstacles = CloneConvexSets(sets);
            },
            cls_doc.configuration_obstacles.doc)
        .def_readwrite("starting_ellipse", &IrisOptions::starting_ellipse,
            cls_doc.starting_ellipse.doc)
        .def_readwrite("bounding_region", &IrisOptions::bounding_region,
            cls_doc.bounding_region.doc)
        .def_readwrite("num_additional_constraint_infeasible_samples",
            &IrisOptions::num_additional_constraint_infeasible_samples,
            cls_doc.num_additional_constraint_infeasible_samples.doc)
        .def_readwrite(
            "random_seed", &IrisOptions::random_seed, cls_doc.random_seed.doc)
        .def_readwrite("mixing_steps", &IrisOptions::mixing_steps,
            cls_doc.mixing_steps.doc)
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
              "random_seed={}, "
              "mixing_steps={}"
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
                  self.random_seed, self.mixing_steps);
        });

    DefReadWriteKeepAlive(&iris_options, "prog_with_additional_constraints",
        &IrisOptions::prog_with_additional_constraints,
        cls_doc.prog_with_additional_constraints.doc);
  }

  m.def(
      "Iris",
      [](const std::vector<ConvexSet*>& obstacles,
          const Eigen::Ref<const Eigen::VectorXd>& sample,
          const HPolyhedron& domain, const IrisOptions& options) {
        return Iris(CloneConvexSets(obstacles), sample, domain, options);
      },
      py::arg("obstacles"), py::arg("sample"), py::arg("domain"),
      py::arg("options") = IrisOptions(), doc.Iris.doc);

  m.def(
      "MakeIrisObstacles",
      [](const QueryObject<double>& query_object,
          std::optional<FrameId> reference_frame) {
        std::vector<copyable_unique_ptr<ConvexSet>> copyable_result =
            MakeIrisObstacles(query_object, reference_frame);
        std::vector<std::unique_ptr<ConvexSet>> result(
            std::make_move_iterator(copyable_result.begin()),
            std::make_move_iterator(copyable_result.end()));
        return result;
      },
      py::arg("query_object"), py::arg("reference_frame") = std::nullopt,
      doc.MakeIrisObstacles.doc);

  m.def("IrisInConfigurationSpace",
      py::overload_cast<const multibody::MultibodyPlant<double>&,
          const systems::Context<double>&, const IrisOptions&>(
          &IrisInConfigurationSpace),
      py::arg("plant"), py::arg("context"), py::arg("options") = IrisOptions(),
      doc.IrisInConfigurationSpace.doc);

  // TODO(#19597) Deprecate and remove these functions once Python
  // can natively handle the file I/O.
  m.def(
      "SaveIrisRegionsYamlFile",
      [](const std::filesystem::path& filename, const IrisRegions& regions,
          const std::optional<std::string>& child_name) {
        yaml::SaveYamlFile(filename, regions, child_name);
      },
      py::arg("filename"), py::arg("regions"),
      py::arg("child_name") = std::nullopt,
      "Calls SaveYamlFile() to serialize an IrisRegions object.");

  m.def(
      "LoadIrisRegionsYamlFile",
      [](const std::filesystem::path& filename,
          const std::optional<std::string>& child_name) {
        return yaml::LoadYamlFile<IrisRegions>(filename, child_name);
      },
      py::arg("filename"), py::arg("child_name") = std::nullopt,
      "Calls LoadYamlFile() to deserialize an IrisRegions object.");

  // GraphOfConvexSetsOptions
  {
    const auto& cls_doc = doc.GraphOfConvexSetsOptions;
    py::class_<GraphOfConvexSetsOptions> gcs_options(
        m, "GraphOfConvexSetsOptions", cls_doc.doc);
    gcs_options.def(py::init<>())
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
        .def_property("rounding_solver_options",
            py::cpp_function(
                [](GraphOfConvexSetsOptions& self) {
                  return &(self.rounding_solver_options);
                },
                py_rvp::reference_internal),
            py::cpp_function(
                [](GraphOfConvexSetsOptions& self,
                    solvers::SolverOptions rounding_solver_options) {
                  self.rounding_solver_options =
                      std::move(rounding_solver_options);
                }),
            cls_doc.rounding_solver_options.doc)
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
              "rounding_solver_options={}, "
              ")")
              .format(self.convex_relaxation, self.preprocessing,
                  self.max_rounded_paths, self.max_rounding_trials,
                  self.flow_tolerance, self.rounding_seed, self.solver,
                  self.solver_options, self.rounding_solver_options);
        });

    DefReadWriteKeepAlive(&gcs_options, "solver",
        &GraphOfConvexSetsOptions::solver, cls_doc.solver.doc);
  }

  // GraphOfConvexSets
  {
    const auto& cls_doc = doc.GraphOfConvexSets;
    py::class_<GraphOfConvexSets> graph_of_convex_sets(
        m, "GraphOfConvexSets", cls_doc.doc);

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
            py::arg("result"), vertex_doc.GetSolution.doc)
        .def("incoming_edges", &GraphOfConvexSets::Vertex::incoming_edges,
            py_rvp::reference_internal, vertex_doc.incoming_edges.doc)
        .def("outgoing_edges", &GraphOfConvexSets::Vertex::outgoing_edges,
            py_rvp::reference_internal, vertex_doc.outgoing_edges.doc);

    // Edge
    const auto& edge_doc = doc.GraphOfConvexSets.Edge;
    py::class_<GraphOfConvexSets::Edge>(
        graph_of_convex_sets, "Edge", edge_doc.doc)
        .def("id", &GraphOfConvexSets::Edge::id, edge_doc.id.doc)
        .def("name", &GraphOfConvexSets::Edge::name, edge_doc.name.doc)
        .def("u",
            overload_cast_explicit<GraphOfConvexSets::Vertex&>(
                &GraphOfConvexSets::Edge::u),
            py_rvp::reference_internal, edge_doc.u.doc_0args_nonconst)
        .def("v",
            overload_cast_explicit<GraphOfConvexSets::Vertex&>(
                &GraphOfConvexSets::Edge::v),
            py_rvp::reference_internal, edge_doc.v.doc_0args_nonconst)
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

    graph_of_convex_sets  // BR
        .def(py::init<>(), cls_doc.ctor.doc)
        .def("AddVertex", &GraphOfConvexSets::AddVertex, py::arg("set"),
            py::arg("name") = "", py_rvp::reference_internal,
            cls_doc.AddVertex.doc)
        .def("AddEdge",
            py::overload_cast<GraphOfConvexSets::Vertex*,
                GraphOfConvexSets::Vertex*, std::string>(
                &GraphOfConvexSets::AddEdge),
            py::arg("u"), py::arg("v"), py::arg("name") = "",
            py_rvp::reference_internal, cls_doc.AddEdge.doc)
        .def("RemoveVertex",
            py::overload_cast<GraphOfConvexSets::Vertex*>(
                &GraphOfConvexSets::RemoveVertex),
            py::arg("vertex"), cls_doc.RemoveVertex.doc)
        .def("RemoveEdge",
            py::overload_cast<GraphOfConvexSets::Edge*>(
                &GraphOfConvexSets::RemoveEdge),
            py::arg("edge"), cls_doc.RemoveEdge.doc)
        .def("Vertices",
            overload_cast_explicit<std::vector<GraphOfConvexSets::Vertex*>>(
                &GraphOfConvexSets::Vertices),
            py_rvp::reference_internal, cls_doc.Vertices.doc)
        .def("Edges",
            overload_cast_explicit<std::vector<GraphOfConvexSets::Edge*>>(
                &GraphOfConvexSets::Edges),
            py_rvp::reference_internal, cls_doc.Edges.doc)
        .def("ClearAllPhiConstraints",
            &GraphOfConvexSets::ClearAllPhiConstraints,
            cls_doc.ClearAllPhiConstraints.doc)
        .def("GetGraphvizString", &GraphOfConvexSets::GetGraphvizString,
            py::arg("result") = std::nullopt, py::arg("show_slacks") = true,
            py::arg("precision") = 3, py::arg("scientific") = false,
            cls_doc.GetGraphvizString.doc)
        .def("SolveShortestPath",
            overload_cast_explicit<solvers::MathematicalProgramResult,
                const GraphOfConvexSets::Vertex&,
                const GraphOfConvexSets::Vertex&,
                const GraphOfConvexSetsOptions&>(
                &GraphOfConvexSets::SolveShortestPath),
            py::arg("source"), py::arg("target"),
            py::arg("options") = GraphOfConvexSetsOptions(),
            cls_doc.SolveShortestPath.doc)
        .def("GetSolutionPath", &GraphOfConvexSets::GetSolutionPath,
            py::arg("source"), py::arg("target"), py::arg("result"),
            py::arg("tolerance") = 1e-3, py_rvp::reference_internal,
            cls_doc.GetSolutionPath.doc)
        .def("SolveConvexRestriction",
            &GraphOfConvexSets::SolveConvexRestriction, py::arg("active_edges"),
            py::arg("options") = GraphOfConvexSetsOptions(),
            cls_doc.SolveConvexRestriction.doc);
  }
  {
    // Definitions for c_iris_collision_geometry.h/cc
    py::enum_<PlaneSide>(m, "PlaneSide", doc.PlaneSide.doc)
        .value("kPositive", PlaneSide::kPositive)
        .value("kNegative", PlaneSide::kNegative);

    py::enum_<CIrisGeometryType>(
        m, "CIrisGeometryType", doc.CIrisGeometryType.doc)
        .value("kPolytope", CIrisGeometryType::kPolytope,
            doc.CIrisGeometryType.kPolytope.doc)
        .value("kSphere", CIrisGeometryType::kSphere,
            doc.CIrisGeometryType.kSphere.doc)
        .value("kCylinder", CIrisGeometryType::kCylinder,
            doc.CIrisGeometryType.kCylinder.doc)
        .value("kCapsule", CIrisGeometryType::kCapsule,
            doc.CIrisGeometryType.kCapsule.doc);

    py::class_<CIrisCollisionGeometry>(
        m, "CIrisCollisionGeometry", doc.CIrisCollisionGeometry.doc)
        .def("type", &CIrisCollisionGeometry::type,
            doc.CIrisCollisionGeometry.type.doc)
        .def("geometry", &CIrisCollisionGeometry::geometry,
            py_rvp::reference_internal, doc.CIrisCollisionGeometry.geometry.doc)
        .def("body_index", &CIrisCollisionGeometry::body_index,
            doc.CIrisCollisionGeometry.body_index.doc)
        .def("id", &CIrisCollisionGeometry::id,
            doc.CIrisCollisionGeometry.id.doc)
        .def("X_BG", &CIrisCollisionGeometry::X_BG,
            doc.CIrisCollisionGeometry.X_BG.doc)
        .def("num_rationals", &CIrisCollisionGeometry::num_rationals,
            doc.CIrisCollisionGeometry.num_rationals.doc);
  }
  {
    py::enum_<SeparatingPlaneOrder>(
        m, "SeparatingPlaneOrder", doc.SeparatingPlaneOrder.doc)
        .value("kAffine", SeparatingPlaneOrder::kAffine,
            doc.SeparatingPlaneOrder.kAffine.doc);
    type_visit([m](auto dummy) { DoSeparatingPlaneDeclaration(m, dummy); },
        type_pack<double, symbolic::Variable>());
  }
  {
    // Definitions for cpsace_free_structs.h/cc
    constexpr auto& prog_doc = doc.SeparationCertificateProgramBase;
    auto prog_cls = py::class_<SeparationCertificateProgramBase>(
        m, "SeparationCertificateProgramBase", prog_doc.doc)
                        .def(
                            "prog",
                            [](const SeparationCertificateProgramBase* self) {
                              return self->prog.get();
                            },
                            py_rvp::reference_internal)
                        .def_readonly("plane_index",
                            &SeparationCertificateProgramBase::plane_index);

    constexpr auto& result_doc = doc.SeparationCertificateResultBase;
    auto result_cls =
        py::class_<SeparationCertificateResultBase>(
            m, "SeparationCertificateResultBase", result_doc.doc)
            .def_readonly("a", &SeparationCertificateResultBase::a)
            .def_readonly("b", &SeparationCertificateResultBase::b)
            .def_readonly("plane_decision_var_vals",
                &SeparationCertificateResultBase::plane_decision_var_vals)
            .def_readonly("result", &SeparationCertificateResultBase::result);

    constexpr auto& find_options_doc = doc.FindSeparationCertificateOptions;
    auto find_options_cls =
        py::class_<FindSeparationCertificateOptions>(
            m, "FindSeparationCertificateOptions", find_options_doc.doc)
            .def(py::init<>())
            .def_readwrite(
                "parallelism", &FindSeparationCertificateOptions::parallelism)
            .def_readwrite(
                "verbose", &FindSeparationCertificateOptions::verbose)
            .def_readwrite(
                "solver_id", &FindSeparationCertificateOptions::solver_id)
            .def_readwrite("terminate_at_failure",
                &FindSeparationCertificateOptions::terminate_at_failure)
            .def_readwrite("solver_options",
                &FindSeparationCertificateOptions::solver_options);
    constexpr char kNumThreadsDeprecated[] =
        "FindSeparationCertificateOptions.num_threads is deprecated and will "
        "be removed on or after 2024-05-01. Use options.parallelism instead.";
    find_options_cls.def_property("num_threads",
        WrapDeprecated(kNumThreadsDeprecated,
            [](const FindSeparationCertificateOptions& self) {
              return self.parallelism.num_threads();
            }),
        WrapDeprecated(kNumThreadsDeprecated,
            [](FindSeparationCertificateOptions& self, int num_threads) {
              self.parallelism =
                  (num_threads > 0) ? num_threads : Parallelism::Max();
            }));
  }
  {
    using BaseClass = CspaceFreePolytopeBase;
    const auto& base_cls_doc = doc.CspaceFreePolytopeBase;
    py::class_<BaseClass> cspace_free_polytope_base_cls(
        m, "CspaceFreePolytopeBase", base_cls_doc.doc);
    cspace_free_polytope_base_cls
        // TODO(Alexandre.Amice): Bind rational_forward_kinematics to resolve
        // #20025.
        .def("map_geometries_to_separating_planes",
            &BaseClass::map_geometries_to_separating_planes,
            base_cls_doc.map_geometries_to_separating_planes.doc)
        .def("separating_planes", &BaseClass::separating_planes,
            base_cls_doc.separating_planes.doc)
        .def("y_slack", &BaseClass::y_slack, base_cls_doc.y_slack.doc);

    {
      const auto& options_cls_doc = base_cls_doc.Options;
      py::class_<BaseClass::Options> options_cls(
          cspace_free_polytope_base_cls, "Options", options_cls_doc.doc);
      options_cls  // BR
          .def(py::init<>(), options_cls_doc.ctor.doc)
          .def_readwrite("with_cross_y", &BaseClass::Options::with_cross_y,
              options_cls_doc.with_cross_y.doc);
      DefReprUsingSerialize(&options_cls);
    }

    using Class = CspaceFreePolytope;
    const auto& cls_doc = doc.CspaceFreePolytope;
    py::class_<Class, BaseClass> cspace_free_polytope_cls(
        m, "CspaceFreePolytope", cls_doc.doc);
    cspace_free_polytope_cls
        .def(py::init<const multibody::MultibodyPlant<double>*,
                 const geometry::SceneGraph<double>*, SeparatingPlaneOrder,
                 const Eigen::Ref<const Eigen::VectorXd>&,
                 const Class::Options&>(),
            py::arg("plant"), py::arg("scene_graph"), py::arg("plane_order"),
            py::arg("q_star"), py::arg("options") = Class::Options(),
            // Keep alive, reference: `self` keeps `plant` alive.
            py::keep_alive<1, 2>(),
            // Keep alive, reference: `self` keeps `scene_graph` alive.
            py::keep_alive<1, 3>(), cls_doc.ctor.doc)
        .def(
            "FindSeparationCertificateGivenPolytope",
            [](const CspaceFreePolytope* self,
                const Eigen::Ref<const Eigen::MatrixXd>& C,
                const Eigen::Ref<const Eigen::VectorXd>& d,
                const CspaceFreePolytope::IgnoredCollisionPairs&
                    ignored_collision_pairs,
                const CspaceFreePolytope::
                    FindSeparationCertificateGivenPolytopeOptions& options) {
              std::unordered_map<SortedPair<geometry::GeometryId>,
                  CspaceFreePolytope::SeparationCertificateResult>
                  certificates;
              bool success = self->FindSeparationCertificateGivenPolytope(
                  C, d, ignored_collision_pairs, options, &certificates);
              return std::pair(success, certificates);
            },
            py::arg("C"), py::arg("d"), py::arg("ignored_collision_pairs"),
            py::arg("options"),
            // The `options` contains a `Parallelism`; we must release the GIL.
            py::call_guard<py::gil_scoped_release>(),
            cls_doc.FindSeparationCertificateGivenPolytope.doc)
        .def("SearchWithBilinearAlternation",
            &Class::SearchWithBilinearAlternation,
            py::arg("ignored_collision_pairs"), py::arg("C_init"),
            py::arg("d_init"), py::arg("options"),
            cls_doc.SearchWithBilinearAlternation.doc)
        .def("BinarySearch", &Class::BinarySearch,
            py::arg("ignored_collision_pairs"), py::arg("C"), py::arg("d"),
            py::arg("s_center"), py::arg("options"), cls_doc.BinarySearch.doc)
        .def("MakeIsGeometrySeparableProgram",
            &Class::MakeIsGeometrySeparableProgram, py::arg("geometry_pair"),
            py::arg("C"), py::arg("d"),
            cls_doc.MakeIsGeometrySeparableProgram.doc)
        .def("SolveSeparationCertificateProgram",
            &Class::SolveSeparationCertificateProgram,
            py::arg("certificate_program"), py::arg("options"),
            // The `options` contains a `Parallelism`; we must release the GIL.
            py::call_guard<py::gil_scoped_release>(),
            cls_doc.SolveSeparationCertificateProgram.doc);
    py::class_<Class::SeparatingPlaneLagrangians>(cspace_free_polytope_cls,
        "SeparatingPlaneLagrangians", cls_doc.SeparatingPlaneLagrangians.doc)
        .def(py::init<int, int>(), py::arg("C_rows"), py::arg("s_size"),
            cls_doc.SeparatingPlaneLagrangians.ctor.doc)
        .def("GetSolution", &Class::SeparatingPlaneLagrangians::GetSolution,
            py::arg("result"),
            cls_doc.SeparatingPlaneLagrangians.GetSolution.doc)
        // Use py_rvp::copy here because numpy.ndarray with dtype=object arrays
        // must be copied, and cannot be referenced.
        .def("polytope", &Class::SeparatingPlaneLagrangians::mutable_polytope,
            py_rvp::copy)
        .def("s_lower", &Class::SeparatingPlaneLagrangians::mutable_s_lower)
        .def("s_upper", &Class::SeparatingPlaneLagrangians::mutable_s_upper);

    using SepCertClass = Class::SeparationCertificateResult;
    py::class_<SepCertClass>(cspace_free_polytope_cls,
        "SeparationCertificateResult", cls_doc.SeparationCertificateResult.doc)
        .def_readonly("plane_index", &SepCertClass::plane_index)
        .def_readonly("positive_side_rational_lagrangians",
            &Class::SeparationCertificateResult::
                positive_side_rational_lagrangians,
            cls_doc.SeparationCertificateResult
                .positive_side_rational_lagrangians.doc)
        .def_readonly("negative_side_rational_lagrangians",
            &Class::SeparationCertificateResult::
                negative_side_rational_lagrangians,
            cls_doc.SeparationCertificateResult
                .negative_side_rational_lagrangians.doc)
        // Use py_rvp::copy here because numpy.ndarray with dtype=object
        // arrays must be copied, and cannot be referenced.
        .def_readonly("a", &SepCertClass::a, py_rvp::copy,
            doc.SeparationCertificateResultBase.a.doc)
        .def_readonly(
            "b", &SepCertClass::b, doc.SeparationCertificateResultBase.b.doc)
        .def_readonly("result", &SepCertClass::result)
        // Use py_rvp::copy here because numpy.ndarray with dtype=object
        // arrays must be copied, and cannot be referenced.
        .def_readonly("plane_decision_var_vals",
            &SepCertClass::plane_decision_var_vals, py_rvp::copy);

    py::class_<Class::SeparationCertificate>(cspace_free_polytope_cls,
        "SeparationCertificate", cls_doc.SeparationCertificate.doc)
        .def("GetSolution", &Class::SeparationCertificate::GetSolution,
            py::arg("plane_index"), py::arg("a"), py::arg("b"),
            py::arg("plane_decision_vars"), py::arg("result"),
            cls_doc.SeparationCertificate.GetSolution.doc)
        .def_readwrite("positive_side_rational_lagrangians",
            &Class::SeparationCertificate::positive_side_rational_lagrangians)
        .def_readwrite("negative_side_rational_lagrangians",
            &Class::SeparationCertificate::negative_side_rational_lagrangians);

    py::class_<Class::SeparationCertificateProgram,
        SeparationCertificateProgramBase>(cspace_free_polytope_cls,
        "SeparationCertificateProgram",
        cls_doc.SeparationCertificateProgram.doc)
        .def(py::init<>())
        .def_readonly(
            "plane_index", &Class::SeparationCertificateProgram::plane_index)
        .def_readonly(
            "certificate", &Class::SeparationCertificateProgram::certificate);

    py::class_<Class::FindSeparationCertificateGivenPolytopeOptions,
        FindSeparationCertificateOptions>(cspace_free_polytope_cls,
        "FindSeparationCertificateGivenPolytopeOptions",
        cls_doc.FindSeparationCertificateGivenPolytopeOptions.doc)
        .def(py::init<>())
        .def_readwrite("ignore_redundant_C",
            &Class::FindSeparationCertificateGivenPolytopeOptions::
                ignore_redundant_C);

    py::enum_<Class::EllipsoidMarginCost>(cspace_free_polytope_cls,
        "EllipsoidMarginCost", cls_doc.EllipsoidMarginCost.doc)
        .value("kSum", Class::EllipsoidMarginCost::kSum)
        .value("kGeometricMean", Class::EllipsoidMarginCost::kGeometricMean);

    py::class_<Class::FindPolytopeGivenLagrangianOptions>(
        cspace_free_polytope_cls, "FindPolytopeGivenLagrangianOptions",
        cls_doc.FindPolytopeGivenLagrangianOptions.doc)
        .def(py::init<>())
        .def_readwrite("backoff_scale",
            &Class::FindPolytopeGivenLagrangianOptions::backoff_scale)
        .def_readwrite("ellipsoid_margin_epsilon",
            &Class::FindPolytopeGivenLagrangianOptions::
                ellipsoid_margin_epsilon)
        .def_readwrite(
            "solver_id", &Class::FindPolytopeGivenLagrangianOptions::solver_id)
        .def_readwrite("solver_options",
            &Class::FindPolytopeGivenLagrangianOptions::solver_options)
        .def_readwrite("s_inner_pts",
            &Class::FindPolytopeGivenLagrangianOptions::s_inner_pts)
        .def_readwrite("search_s_bounds_lagrangians",
            &Class::FindPolytopeGivenLagrangianOptions::
                search_s_bounds_lagrangians)
        .def_readwrite("ellipsoid_margin_cost",
            &Class::FindPolytopeGivenLagrangianOptions::ellipsoid_margin_cost);

    py::class_<Class::SearchResult>(
        cspace_free_polytope_cls, "SearchResult", cls_doc.SearchResult.doc)
        .def(py::init<>())
        .def("C", &Class::SearchResult::C)
        .def("d", &Class::SearchResult::d)
        // Use py_rvp::copy here because numpy.ndarray with dtype=object arrays
        // must be copied, and cannot be referenced.
        .def("a", &Class::SearchResult::a, py_rvp::copy)
        .def("b", &Class::SearchResult::b)
        .def("num_iter", &Class::SearchResult::num_iter)
        .def("certified_polytope", &Class::SearchResult::certified_polytope);

    py::class_<Class::BilinearAlternationOptions>(cspace_free_polytope_cls,
        "BilinearAlternationOptions", cls_doc.BilinearAlternationOptions.doc)
        .def(py::init<>())
        .def_readwrite("max_iter", &Class::BilinearAlternationOptions::max_iter,
            cls_doc.BilinearAlternationOptions.max_iter.doc)
        .def_readwrite("convergence_tol",
            &Class::BilinearAlternationOptions::convergence_tol,
            cls_doc.BilinearAlternationOptions.convergence_tol.doc)
        .def_readwrite("find_polytope_options",
            &Class::BilinearAlternationOptions::find_polytope_options,
            cls_doc.BilinearAlternationOptions.find_polytope_options.doc)
        .def_readonly("find_lagrangian_options",
            &Class::BilinearAlternationOptions::find_lagrangian_options,
            cls_doc.BilinearAlternationOptions.find_lagrangian_options.doc)
        .def_readwrite("ellipsoid_scaling",
            &Class::BilinearAlternationOptions::ellipsoid_scaling,
            cls_doc.BilinearAlternationOptions.ellipsoid_scaling.doc);

    py::class_<Class::BinarySearchOptions>(cspace_free_polytope_cls,
        "BinarySearchOptions", cls_doc.BinarySearchOptions.doc)
        .def(py::init<>())
        .def_readwrite("scale_max", &Class::BinarySearchOptions::scale_max)
        .def_readwrite("scale_min", &Class::BinarySearchOptions::scale_min)
        .def_readwrite("max_iter", &Class::BinarySearchOptions::max_iter)
        .def_readwrite(
            "convergence_tol", &Class::BinarySearchOptions::convergence_tol)
        .def_readonly("find_lagrangian_options",
            &Class::BinarySearchOptions::find_lagrangian_options);
  }

  using drake::geometry::optimization::ConvexSet;
  m.def("CheckIfSatisfiesConvexityRadius", &CheckIfSatisfiesConvexityRadius,
      py::arg("convex_set"), py::arg("continuous_revolute_joints"),
      doc.CheckIfSatisfiesConvexityRadius.doc);
  m.def(
      "PartitionConvexSet",
      [](const ConvexSet& convex_set,
          const std::vector<int>& continuous_revolute_joints,
          const double epsilon) {
        std::vector<copyable_unique_ptr<ConvexSet>> copyable_result =
            PartitionConvexSet(convex_set, continuous_revolute_joints, epsilon);
        std::vector<std::unique_ptr<ConvexSet>> result(
            std::make_move_iterator(copyable_result.begin()),
            std::make_move_iterator(copyable_result.end()));
        return result;
      },
      py::arg("convex_set"), py::arg("continuous_revolute_joints"),
      py::arg("epsilon") = 1e-5,
      doc.PartitionConvexSet
          .doc_3args_convex_set_continuous_revolute_joints_epsilon);
  m.def(
      "PartitionConvexSet",
      [](const std::vector<ConvexSet*>& convex_sets,
          const std::vector<int>& continuous_revolute_joints,
          const double epsilon = 1e-5) {
        std::vector<copyable_unique_ptr<ConvexSet>> copyable_result =
            PartitionConvexSet(CloneConvexSets(convex_sets),
                continuous_revolute_joints, epsilon);
        std::vector<std::unique_ptr<ConvexSet>> result(
            std::make_move_iterator(copyable_result.begin()),
            std::make_move_iterator(copyable_result.end()));
        return result;
      },
      py::arg("convex_sets"), py::arg("continuous_revolute_joints"),
      py::arg("epsilon") = 1e-5,
      doc.PartitionConvexSet
          .doc_3args_convex_sets_continuous_revolute_joints_epsilon);
  m.def(
      "CalcPairwiseIntersections",
      [](const std::vector<ConvexSet*>& convex_sets_A,
          const std::vector<ConvexSet*>& convex_sets_B,
          const std::vector<int>& continuous_revolute_joints) {
        return CalcPairwiseIntersections(CloneConvexSets(convex_sets_A),
            CloneConvexSets(convex_sets_B), continuous_revolute_joints);
      },
      py::arg("convex_sets_A"), py::arg("convex_sets_B"),
      py::arg("continuous_revolute_joints"),
      doc.CalcPairwiseIntersections.doc_3args);
  m.def(
      "CalcPairwiseIntersections",
      [](const std::vector<ConvexSet*>& convex_sets,
          const std::vector<int>& continuous_revolute_joints) {
        return CalcPairwiseIntersections(
            CloneConvexSets(convex_sets), continuous_revolute_joints);
      },
      py::arg("convex_sets"), py::arg("continuous_revolute_joints"),
      doc.CalcPairwiseIntersections.doc_2args);
  // NOLINTNEXTLINE(readability/fn_size)
}

}  // namespace pydrake
}  // namespace drake
