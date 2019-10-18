#include "pybind11/eigen.h"
#include "pybind11/operators.h"
#include "pybind11/pybind11.h"
#include "pybind11/stl.h"

#include "drake/bindings/pydrake/documentation_pybind.h"
#include "drake/bindings/pydrake/polynomial_types_pybind.h"
#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/common/polynomial.h"
#include "drake/common/trajectories/piecewise_polynomial.h"
#include "drake/common/trajectories/trajectory.h"

namespace drake {
namespace pydrake {

PYBIND11_MODULE(trajectories, m) {
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::trajectories;
  constexpr auto& doc = pydrake_doc.drake.trajectories;
  py::module::import("pydrake.polynomial");

  using T = double;

  py::class_<Trajectory<T>>(m, "Trajectory", doc.Trajectory.doc);

  py::class_<PiecewiseTrajectory<T>, Trajectory<T>>(
      m, "PiecewiseTrajectory", doc.PiecewiseTrajectory.doc)
      .def("get_number_of_segments",
          &PiecewiseTrajectory<T>::get_number_of_segments,
          doc.PiecewiseTrajectory.get_number_of_segments.doc)
      .def("start_time",
          overload_cast_explicit<double, int>(
              &PiecewiseTrajectory<T>::start_time),
          py::arg("segment_index"), doc.PiecewiseTrajectory.start_time.doc)
      .def("end_time",
          overload_cast_explicit<double, int>(
              &PiecewiseTrajectory<T>::end_time),
          py::arg("segment_index"), doc.PiecewiseTrajectory.end_time.doc)
      .def("duration", &PiecewiseTrajectory<T>::duration,
          py::arg("segment_index"), doc.PiecewiseTrajectory.duration.doc)
      .def("start_time",
          overload_cast_explicit<double>(&PiecewiseTrajectory<T>::start_time),
          doc.PiecewiseTrajectory.start_time.doc)
      .def("end_time",
          overload_cast_explicit<double>(&PiecewiseTrajectory<T>::end_time),
          doc.PiecewiseTrajectory.end_time.doc)
      .def("is_time_in_range", &PiecewiseTrajectory<T>::is_time_in_range,
          py::arg("t"), doc.PiecewiseTrajectory.is_time_in_range.doc)
      .def("get_segment_index", &PiecewiseTrajectory<T>::get_segment_index,
          py::arg("t"), doc.PiecewiseTrajectory.get_segment_index.doc)
      .def("get_segment_times", &PiecewiseTrajectory<T>::get_segment_times,
          doc.PiecewiseTrajectory.get_segment_times.doc);

  py::class_<PiecewisePolynomial<T>, PiecewiseTrajectory<T>>(
      m, "PiecewisePolynomial", doc.PiecewisePolynomial.doc)
      .def(py::init<>(), doc.PiecewisePolynomial.ctor.doc_0args)
      .def(py::init<const Eigen::Ref<const MatrixX<T>>&>(),
          doc.PiecewisePolynomial.ctor.doc_1args_constEigenMatrixBase)
      .def(py::init<std::vector<MatrixX<Polynomial<T>>> const&,
               std::vector<double> const&>(),
          doc.PiecewisePolynomial.ctor.doc_2args_polynomials_matrix_breaks)
      .def(py::init<std::vector<Polynomial<T>> const&,
               std::vector<double> const&>(),
          doc.PiecewisePolynomial.ctor.doc_2args_polynomials_breaks)
      .def_static("ZeroOrderHold",
          py::overload_cast<const Eigen::Ref<const Eigen::VectorXd>&,
              const Eigen::Ref<const MatrixX<T>>&>(
              &PiecewisePolynomial<T>::ZeroOrderHold),
          doc.PiecewisePolynomial.ZeroOrderHold.doc)
      .def_static("FirstOrderHold",
          py::overload_cast<const Eigen::Ref<const Eigen::VectorXd>&,
              const Eigen::Ref<const MatrixX<T>>&>(
              &PiecewisePolynomial<T>::FirstOrderHold),
          doc.PiecewisePolynomial.FirstOrderHold.doc)
      .def_static("Pchip",
          py::overload_cast<const Eigen::Ref<const Eigen::VectorXd>&,
              const Eigen::Ref<const MatrixX<T>>&, bool>(
              &PiecewisePolynomial<T>::Pchip),
          py::arg("breaks"), py::arg("knots"),
          py::arg("zero_end_point_derivatives") = false,
          doc.PiecewisePolynomial.Pchip.doc)
      .def_static("Cubic",
          py::overload_cast<const Eigen::Ref<const Eigen::VectorXd>&,
              const Eigen::Ref<const MatrixX<T>>&,
              const Eigen::Ref<const VectorX<T>>&,
              const Eigen::Ref<const VectorX<T>>&>(
              &PiecewisePolynomial<T>::Cubic),
          py::arg("breaks"), py::arg("knots"), py::arg("knots_dot_start"),
          py::arg("knots_dot_end"),
          doc.PiecewisePolynomial.Cubic
              .doc_4args_breaks_knots_knots_dot_start_knots_dot_end)
      .def_static("Cubic",
          py::overload_cast<const Eigen::Ref<const Eigen::VectorXd>&,
              const Eigen::Ref<const MatrixX<T>>&,
              const Eigen::Ref<const MatrixX<T>>&>(
              &PiecewisePolynomial<T>::Cubic),
          py::arg("breaks"), py::arg("knots"), py::arg("knots_dot"),
          doc.PiecewisePolynomial.Cubic.doc_3args_breaks_knots_knots_dot)
      .def_static("Cubic",
          py::overload_cast<const Eigen::Ref<const Eigen::VectorXd>&,
              const Eigen::Ref<const MatrixX<T>>&, bool>(
              &PiecewisePolynomial<T>::Cubic),
          py::arg("breaks"), py::arg("knots"), py::arg("periodic_end"),
          doc.PiecewisePolynomial.Cubic
              .doc_3args_breaks_knots_periodic_end_condition)
      .def("value", &PiecewisePolynomial<T>::value, py::arg("t"),
          doc.PiecewisePolynomial.value.doc)
      .def("derivative", &PiecewisePolynomial<T>::derivative,
          py::arg("derivative_order") = 1,
          doc.PiecewisePolynomial.derivative.doc)
      .def("getPolynomialMatrix", &PiecewisePolynomial<T>::getPolynomialMatrix,
          py::arg("segment_index"),
          doc.PiecewisePolynomial.getPolynomialMatrix.doc)
      .def("getPolynomial", &PiecewisePolynomial<T>::getPolynomial,
          py::arg("segment_index"), py::arg("row") = 0, py::arg("col") = 0,
          doc.PiecewisePolynomial.getPolynomial.doc)
      .def("getSegmentPolynomialDegree",
          &PiecewisePolynomial<T>::getSegmentPolynomialDegree,
          py::arg("segment_index"), py::arg("row") = 0, py::arg("col") = 0,
          doc.PiecewisePolynomial.getSegmentPolynomialDegree.doc)
      .def("rows", &PiecewisePolynomial<T>::rows,
          doc.PiecewisePolynomial.rows.doc)
      .def("cols", &PiecewisePolynomial<T>::cols,
          doc.PiecewisePolynomial.cols.doc)
      .def("isApprox", &PiecewisePolynomial<T>::isApprox, py::arg("other"),
          py::arg("tol"), doc.PiecewisePolynomial.isApprox.doc)
      .def("ConcatenateInTime", &PiecewisePolynomial<T>::ConcatenateInTime,
          py::arg("other"), doc.PiecewisePolynomial.ConcatenateInTime.doc)
      .def("slice", &PiecewisePolynomial<T>::slice,
          py::arg("start_segment_index"), py::arg("num_segments"),
          doc.PiecewisePolynomial.slice.doc)
      .def("shiftRight", &PiecewisePolynomial<T>::shiftRight, py::arg("offset"),
          doc.PiecewisePolynomial.shiftRight.doc)
      .def("setPolynomialMatrixBlock",
          &PiecewisePolynomial<T>::setPolynomialMatrixBlock,
          py::arg("replacement"), py::arg("segment_index"),
          py::arg("row_start") = 0, py::arg("col_start") = 0,
          doc.PiecewisePolynomial.setPolynomialMatrixBlock.doc);
}

}  // namespace pydrake
}  // namespace drake
