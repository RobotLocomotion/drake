#include "pybind11/eigen.h"
#include "pybind11/operators.h"
#include "pybind11/pybind11.h"
#include "pybind11/stl.h"

#include "drake/bindings/pydrake/documentation_pybind.h"
#include "drake/bindings/pydrake/polynomial_types_pybind.h"
#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/common/polynomial.h"
#include "drake/common/trajectories/bspline_trajectory.h"
#include "drake/common/trajectories/piecewise_polynomial.h"
#include "drake/common/trajectories/piecewise_pose.h"
#include "drake/common/trajectories/piecewise_quaternion.h"
#include "drake/common/trajectories/trajectory.h"

namespace drake {
namespace pydrake {

namespace {
// For bindings that want a `const vector<VectorX<T>>&` but are bound
// via a `const vector<vector<T>>&` for overloading priority,
// this function converts the input.  A numpy matrix is row-major, so a 2x3
// samples numpy array (unfortunately) turns into a std::vector with 2 elements,
// each a vector of 3 elements; we'll transpose that.
template <typename T>
std::vector<MatrixX<T>> MakeEigenFromRowMajorVectors(
    const std::vector<std::vector<T>>& in) {
  if (in.size() == 0) {
    return std::vector<MatrixX<T>>();
  }
  std::vector<MatrixX<T>> vec(in[0].size(), Eigen::VectorXd(in.size()));
  for (int row = 0; row < static_cast<int>(in.size()); ++row) {
    DRAKE_THROW_UNLESS(in[row].size() == in[0].size());
    for (int col = 0; col < static_cast<int>(in[row].size()); ++col) {
      vec[col](row, 0) = in[row][col];
    }
  }
  return vec;
}

}  // namespace

PYBIND11_MODULE(trajectories, m) {
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::trajectories;
  constexpr auto& doc = pydrake_doc.drake.trajectories;

  py::module::import("pydrake.common");
  py::module::import("pydrake.polynomial");

  using T = double;

  py::class_<Trajectory<T>>(m, "Trajectory", doc.Trajectory.doc)
      .def("value", &Trajectory<T>::value, py::arg("t"),
          doc.Trajectory.value.doc)
      .def("vector_values", &Trajectory<T>::vector_values,
          doc.Trajectory.vector_values.doc)
      .def("EvalDerivative", &Trajectory<T>::EvalDerivative, py::arg("t"),
          py::arg("derivative_order") = 1, doc.Trajectory.EvalDerivative.doc)
      .def("MakeDerivative", &Trajectory<T>::MakeDerivative,
          py::arg("derivative_order") = 1, doc.Trajectory.MakeDerivative.doc)
      .def("start_time", &Trajectory<T>::start_time,
          doc.Trajectory.start_time.doc)
      .def("end_time", &Trajectory<T>::end_time, doc.Trajectory.end_time.doc)
      .def("rows", &Trajectory<T>::rows, doc.Trajectory.rows.doc)
      .def("cols", &Trajectory<T>::cols, doc.Trajectory.cols.doc);

  py::class_<BsplineTrajectory<T>, Trajectory<T>>(
      m, "BsplineTrajectory", doc.BsplineTrajectory.doc)
      .def(py::init<>())
      .def(py::init<math::BsplineBasis<T>, std::vector<MatrixX<T>>>(),
          py::arg("basis"), py::arg("control_points"),
          doc.BsplineTrajectory.ctor.doc)
      .def("Clone", &BsplineTrajectory<T>::Clone,
          doc.BsplineTrajectory.Clone.doc)
      .def("num_control_points", &BsplineTrajectory<T>::num_control_points,
          doc.BsplineTrajectory.num_control_points.doc)
      .def("control_points", &BsplineTrajectory<T>::control_points,
          doc.BsplineTrajectory.control_points.doc)
      .def("InitialValue", &BsplineTrajectory<T>::InitialValue,
          doc.BsplineTrajectory.InitialValue.doc)
      .def("FinalValue", &BsplineTrajectory<T>::FinalValue,
          doc.BsplineTrajectory.FinalValue.doc)
      .def("basis", &BsplineTrajectory<T>::basis,
          doc.BsplineTrajectory.basis.doc)
      .def("InsertKnots", &BsplineTrajectory<T>::InsertKnots,
          py::arg("additional_knots"), doc.BsplineTrajectory.InsertKnots.doc)
      .def("CopyBlock", &BsplineTrajectory<T>::CopyBlock, py::arg("start_row"),
          py::arg("start_col"), py::arg("block_rows"), py::arg("block_cols"),
          doc.BsplineTrajectory.CopyBlock.doc)
      .def("CopyHead", &BsplineTrajectory<T>::CopyHead, py::arg("n"),
          doc.BsplineTrajectory.CopyHead.doc);

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
      // N.B. We must redefine these two overloads, as we cannot use the base
      // classes' overloads. See: https://github.com/pybind/pybind11/issues/974
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
      .def_static(
          "ZeroOrderHold",
          // This serves the same purpose as the C++ ZeroOrderHold(VectorX<T>,
          // MatrixX<T>) method.  For 2d numpy arrays, pybind apparently matches
          // vector<vector<T>> then vector<MatrixX<T>> then MatrixX<T>.
          [](const std::vector<T>& breaks,
              const std::vector<std::vector<T>>& samples) {
            return PiecewisePolynomial<T>::ZeroOrderHold(
                breaks, MakeEigenFromRowMajorVectors(samples));
          },
          py::arg("breaks"), py::arg("samples"),
          doc.PiecewisePolynomial.ZeroOrderHold.doc_vector)
      .def_static("ZeroOrderHold",
          py::overload_cast<const std::vector<T>&,
              const std::vector<MatrixX<T>>&>(
              &PiecewisePolynomial<T>::ZeroOrderHold),
          py::arg("breaks"), py::arg("samples"),
          doc.PiecewisePolynomial.ZeroOrderHold.doc_matrix)
      .def_static(
          "FirstOrderHold",
          [](const std::vector<T>& breaks,
              const std::vector<std::vector<T>>& samples) {
            return PiecewisePolynomial<T>::FirstOrderHold(
                breaks, MakeEigenFromRowMajorVectors(samples));
          },
          py::arg("breaks"), py::arg("samples"),
          doc.PiecewisePolynomial.FirstOrderHold.doc_vector)
      .def_static("FirstOrderHold",
          py::overload_cast<const std::vector<T>&,
              const std::vector<MatrixX<T>>&>(
              &PiecewisePolynomial<T>::FirstOrderHold),
          py::arg("breaks"), py::arg("samples"),
          doc.PiecewisePolynomial.FirstOrderHold.doc_matrix)
      .def_static(
          "CubicShapePreserving",
          [](const std::vector<T>& breaks,
              const std::vector<std::vector<T>>& samples,
              bool zero_end_point_derivatives) {
            return PiecewisePolynomial<T>::CubicShapePreserving(breaks,
                MakeEigenFromRowMajorVectors(samples),
                zero_end_point_derivatives);
          },
          py::arg("breaks"), py::arg("samples"),
          py::arg("zero_end_point_derivatives") = false,
          doc.PiecewisePolynomial.CubicShapePreserving.doc_vector)
      .def_static("CubicShapePreserving",
          py::overload_cast<const std::vector<T>&,
              const std::vector<MatrixX<T>>&, bool>(
              &PiecewisePolynomial<T>::CubicShapePreserving),
          py::arg("breaks"), py::arg("samples"),
          py::arg("zero_end_point_derivatives") = false,
          doc.PiecewisePolynomial.CubicShapePreserving.doc_matrix)
      .def_static(
          "CubicWithContinuousSecondDerivatives",
          [](const std::vector<T>& breaks,
              const std::vector<std::vector<T>>& samples,
              const MatrixX<T>& sample_dot_at_start,
              const MatrixX<T>& sample_dot_at_end) {
            return PiecewisePolynomial<T>::CubicWithContinuousSecondDerivatives(
                breaks, MakeEigenFromRowMajorVectors(samples),
                sample_dot_at_start, sample_dot_at_end);
          },
          py::arg("breaks"), py::arg("samples"), py::arg("sample_dot_at_start"),
          py::arg("sample_dot_at_end"),
          doc.PiecewisePolynomial.CubicWithContinuousSecondDerivatives
              .doc_4args_vector)
      .def_static("CubicWithContinuousSecondDerivatives",
          py::overload_cast<const std::vector<T>&,
              const std::vector<MatrixX<T>>&, const MatrixX<T>&,
              const MatrixX<T>&>(
              &PiecewisePolynomial<T>::CubicWithContinuousSecondDerivatives),
          py::arg("breaks"), py::arg("samples"), py::arg("sample_dot_at_start"),
          py::arg("sample_dot_at_end"),
          doc.PiecewisePolynomial.CubicWithContinuousSecondDerivatives
              .doc_4args_matrix)
      .def_static(
          "CubicHermite",
          [](const std::vector<T>& breaks,
              const std::vector<std::vector<T>>& samples,
              const std::vector<std::vector<T>>& samples_dot) {
            return PiecewisePolynomial<T>::CubicHermite(breaks,
                MakeEigenFromRowMajorVectors(samples),
                MakeEigenFromRowMajorVectors(samples_dot));
          },
          py::arg("breaks"), py::arg("samples"), py::arg("samples_dot"),
          doc.PiecewisePolynomial.CubicHermite.doc_vector)
      .def_static("CubicHermite",
          py::overload_cast<const std::vector<T>&,
              const std::vector<MatrixX<T>>&, const std::vector<MatrixX<T>>&>(
              &PiecewisePolynomial<T>::CubicHermite),
          py::arg("breaks"), py::arg("samples"), py::arg("samples_dot"),
          doc.PiecewisePolynomial.CubicHermite.doc_matrix)
      .def_static(
          "CubicWithContinuousSecondDerivatives",
          [](const std::vector<T>& breaks,
              const std::vector<std::vector<T>>& samples,
              bool periodic_end_condition) {
            return PiecewisePolynomial<T>::CubicWithContinuousSecondDerivatives(
                breaks, MakeEigenFromRowMajorVectors(samples),
                periodic_end_condition);
          },
          py::arg("breaks"), py::arg("samples"),
          py::arg("periodic_end_condition") = false,
          doc.PiecewisePolynomial.CubicWithContinuousSecondDerivatives
              .doc_3args_vector)
      .def_static("CubicWithContinuousSecondDerivatives",
          py::overload_cast<const std::vector<T>&,
              const std::vector<MatrixX<T>>&, bool>(
              &PiecewisePolynomial<T>::CubicWithContinuousSecondDerivatives),
          py::arg("breaks"), py::arg("samples"), py::arg("periodic_end"),
          doc.PiecewisePolynomial.CubicWithContinuousSecondDerivatives
              .doc_3args_matrix)
      .def_static(
          "LagrangeInterpolatingPolynomial",
          [](const std::vector<T>& breaks,
              const std::vector<std::vector<T>>& samples) {
            return PiecewisePolynomial<T>::LagrangeInterpolatingPolynomial(
                breaks, MakeEigenFromRowMajorVectors(samples));
          },
          py::arg("times"), py::arg("samples"),
          doc.PiecewisePolynomial.LagrangeInterpolatingPolynomial.doc_vector)
      .def_static("LagrangeInterpolatingPolynomial",
          py::overload_cast<const std::vector<T>&,
              const std::vector<MatrixX<T>>&>(
              &PiecewisePolynomial<T>::LagrangeInterpolatingPolynomial),
          py::arg("times"), py::arg("samples"),
          doc.PiecewisePolynomial.LagrangeInterpolatingPolynomial.doc_matrix)
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
      .def("isApprox", &PiecewisePolynomial<T>::isApprox, py::arg("other"),
          py::arg("tol"), py::arg("tol_type") = drake::ToleranceType::kRelative,
          doc.PiecewisePolynomial.isApprox.doc)
      .def("Reshape", &PiecewisePolynomial<T>::Reshape, py::arg("rows"),
          py::arg("cols"), doc.PiecewisePolynomial.Reshape.doc)
      .def("Block", &PiecewisePolynomial<T>::Block, py::arg("start_row"),
          py::arg("start_col"), py::arg("block_rows"), py::arg("block_cols"),
          doc.PiecewisePolynomial.Block.doc)
      .def("ConcatenateInTime", &PiecewisePolynomial<T>::ConcatenateInTime,
          py::arg("other"), doc.PiecewisePolynomial.ConcatenateInTime.doc)
      .def("AppendCubicHermiteSegment",
          &PiecewisePolynomial<T>::AppendCubicHermiteSegment, py::arg("time"),
          py::arg("sample"), py::arg("sample_dot"),
          doc.PiecewisePolynomial.AppendCubicHermiteSegment.doc)
      .def("AppendFirstOrderSegment",
          &PiecewisePolynomial<T>::AppendFirstOrderSegment, py::arg("time"),
          py::arg("sample"),
          doc.PiecewisePolynomial.AppendFirstOrderSegment.doc)
      .def("RemoveFinalSegment", &PiecewisePolynomial<T>::RemoveFinalSegment,
          doc.PiecewisePolynomial.RemoveFinalSegment.doc)
      .def("ReverseTime", &PiecewisePolynomial<T>::ReverseTime,
          doc.PiecewisePolynomial.ReverseTime.doc)
      .def("ScaleTime", &PiecewisePolynomial<T>::ScaleTime, py::arg("scale"),
          doc.PiecewisePolynomial.ScaleTime.doc)
      .def("slice", &PiecewisePolynomial<T>::slice,
          py::arg("start_segment_index"), py::arg("num_segments"),
          doc.PiecewisePolynomial.slice.doc)
      .def("shiftRight", &PiecewisePolynomial<T>::shiftRight, py::arg("offset"),
          doc.PiecewisePolynomial.shiftRight.doc)
      .def(py::self + py::self)
      .def("setPolynomialMatrixBlock",
          &PiecewisePolynomial<T>::setPolynomialMatrixBlock,
          py::arg("replacement"), py::arg("segment_index"),
          py::arg("row_start") = 0, py::arg("col_start") = 0,
          doc.PiecewisePolynomial.setPolynomialMatrixBlock.doc);

  py::class_<PiecewiseQuaternionSlerp<T>, PiecewiseTrajectory<T>>(
      m, "PiecewiseQuaternionSlerp", doc.PiecewiseQuaternionSlerp.doc)
      .def(py::init<>(), doc.PiecewiseQuaternionSlerp.ctor.doc_0args)
      .def(py::init<const std::vector<double>&,
               const std::vector<Quaternion<T>>&>(),
          py::arg("breaks"), py::arg("quaternions"),
          doc.PiecewiseQuaternionSlerp.ctor.doc_2args_breaks_quaternions)
      .def(py::init<const std::vector<double>&,
               const std::vector<Matrix3<T>>&>(),
          py::arg("breaks"), py::arg("rotation_matrices"),
          doc.PiecewiseQuaternionSlerp.ctor.doc_2args_breaks_rotation_matrices)
      .def(py::init<const std::vector<double>&,
               const std::vector<math::RotationMatrix<T>>&>(),
          py::arg("breaks"), py::arg("rotation_matrices"),
          doc.PiecewiseQuaternionSlerp.ctor.doc_2args_breaks_rotation_matrices)
      .def(py::init<const std::vector<double>&,
               const std::vector<AngleAxis<T>>&>(),
          py::arg("breaks"), py::arg("angle_axes"),
          doc.PiecewiseQuaternionSlerp.ctor.doc_2args_breaks_angle_axes)
      .def("Append",
          py::overload_cast<const T&, const Quaternion<T>&>(
              &PiecewiseQuaternionSlerp<T>::Append),
          py::arg("time"), py::arg("quaternion"),
          doc.PiecewiseQuaternionSlerp.Append.doc_2args_time_quaternion)
      .def("Append",
          py::overload_cast<const T&, const math::RotationMatrix<T>&>(
              &PiecewiseQuaternionSlerp<T>::Append),
          py::arg("time"), py::arg("rotation_matrix"),
          doc.PiecewiseQuaternionSlerp.Append.doc_2args_time_rotation_matrix)
      .def("Append",
          py::overload_cast<const T&, const AngleAxis<T>&>(
              &PiecewiseQuaternionSlerp<T>::Append),
          py::arg("time"), py::arg("angle_axis"),
          doc.PiecewiseQuaternionSlerp.Append.doc_2args_time_angle_axis)
      .def("orientation", &PiecewiseQuaternionSlerp<T>::orientation,
          py::arg("time"), doc.PiecewiseQuaternionSlerp.orientation.doc)
      .def("angular_velocity", &PiecewiseQuaternionSlerp<T>::angular_velocity,
          py::arg("time"), doc.PiecewiseQuaternionSlerp.angular_velocity.doc)
      .def("angular_acceleration",
          &PiecewiseQuaternionSlerp<T>::angular_acceleration, py::arg("time"),
          doc.PiecewiseQuaternionSlerp.angular_acceleration.doc);

  py::class_<PiecewisePose<T>, PiecewiseTrajectory<T>>(
      m, "PiecewisePose", doc.PiecewisePose.doc)
      .def(py::init<>(), doc.PiecewisePose.ctor.doc_0args)
      .def(py::init<const PiecewisePolynomial<T>&,
               const PiecewiseQuaternionSlerp<T>&>(),
          py::arg("position_trajectory"), py::arg("orientation_trajectory"),
          doc.PiecewisePose.ctor.doc_2args)
      .def_static("MakeLinear", &PiecewisePose<T>::MakeLinear, py::arg("times"),
          py::arg("poses"), doc.PiecewisePose.MakeLinear.doc)
      .def_static("MakeCubicLinearWithEndLinearVelocity",
          &PiecewisePose<T>::MakeCubicLinearWithEndLinearVelocity,
          py::arg("times"), py::arg("poses"),
          py::arg("start_vel") = Vector3<T>::Zero(),
          py::arg("end_vel") = Vector3<T>::Zero(),
          doc.PiecewisePose.MakeCubicLinearWithEndLinearVelocity.doc)
      .def("GetPose", &PiecewisePose<T>::GetPose, py::arg("time"),
          doc.PiecewisePose.GetPose.doc)
      .def("GetVelocity", &PiecewisePose<T>::GetVelocity, py::arg("time"),
          doc.PiecewisePose.GetVelocity.doc)
      .def("GetAcceleration", &PiecewisePose<T>::GetAcceleration,
          py::arg("time"), doc.PiecewisePose.GetAcceleration.doc)
      .def("IsApprox", &PiecewisePose<T>::IsApprox, py::arg("other"),
          py::arg("tol"), doc.PiecewisePose.IsApprox.doc)
      .def("get_position_trajectory",
          &PiecewisePose<T>::get_position_trajectory,
          doc.PiecewisePose.get_position_trajectory.doc)
      .def("get_orientation_trajectory",
          &PiecewisePose<T>::get_orientation_trajectory,
          doc.PiecewisePose.get_orientation_trajectory.doc);
}

}  // namespace pydrake
}  // namespace drake
