/* @file The contains all of the public entities found in the
 drake::geometry::optimization namespace. They can be found in the
 pydrake.geometry.optimization module. */

#include "drake/bindings/pydrake/common/default_scalars_pybind.h"
#include "drake/bindings/pydrake/documentation_pybind.h"
#include "drake/bindings/pydrake/geometry_py.h"
#include "drake/geometry/optimization/cartesian_product.h"
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
          doc.IrisOptions.termination_threshold.doc);

  m.def("Iris", &Iris, py::arg("obstacles"), py::arg("sample"),
      py::arg("domain"), py::arg("options") = IrisOptions(), doc.Iris.doc);

  m.def("MakeIrisObstacles", &MakeIrisObstacles, py::arg("query_object"),
      py::arg("reference_frame") = std::nullopt, doc.MakeIrisObstacles.doc);
}

}  // namespace pydrake
}  // namespace drake
