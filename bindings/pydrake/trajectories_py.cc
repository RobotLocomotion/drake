#include "pybind11/eigen.h"
#include "pybind11/operators.h"
#include "pybind11/pybind11.h"
#include "pybind11/stl.h"

#include "drake/bindings/pydrake/common/default_scalars_pybind.h"
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

template <typename T>
void DoScalarDependentDefinitions(py::module m, T) {
  py::tuple param = GetPyParam<T>();

  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::trajectories;
  constexpr auto& doc = pydrake_doc.drake.trajectories;

  {
    using Class = Trajectory<T>;
    constexpr auto& cls_doc = doc.Trajectory;
    auto cls = DefineTemplateClassWithDefault<Class>(
        m, "Trajectory", param, cls_doc.doc);
    cls  // BR
        .def("value", &Class::value, py::arg("t"), cls_doc.value.doc)
        .def("vector_values", &Class::vector_values, cls_doc.vector_values.doc)
        .def("EvalDerivative", &Class::EvalDerivative, py::arg("t"),
            py::arg("derivative_order") = 1, cls_doc.EvalDerivative.doc)
        .def("MakeDerivative", &Class::MakeDerivative,
            py::arg("derivative_order") = 1, cls_doc.MakeDerivative.doc)
        .def("start_time", &Class::start_time, cls_doc.start_time.doc)
        .def("end_time", &Class::end_time, cls_doc.end_time.doc)
        .def("rows", &Class::rows, cls_doc.rows.doc)
        .def("cols", &Class::cols, cls_doc.cols.doc);
  }

  {
    using Class = BsplineTrajectory<T>;
    constexpr auto& cls_doc = doc.BsplineTrajectory;
    auto cls = DefineTemplateClassWithDefault<Class, Trajectory<T>>(
        m, "BsplineTrajectory", param, cls_doc.doc);
    cls  // BR
        .def(py::init<>())
        .def(py::init<math::BsplineBasis<T>, std::vector<MatrixX<T>>>(),
            py::arg("basis"), py::arg("control_points"), cls_doc.ctor.doc)
        .def("Clone", &Class::Clone, cls_doc.Clone.doc)
        .def("num_control_points", &Class::num_control_points,
            cls_doc.num_control_points.doc)
        .def("control_points", &Class::control_points,
            cls_doc.control_points.doc)
        .def("InitialValue", &Class::InitialValue, cls_doc.InitialValue.doc)
        .def("FinalValue", &Class::FinalValue, cls_doc.FinalValue.doc)
        .def("basis", &Class::basis, cls_doc.basis.doc)
        .def("InsertKnots", &Class::InsertKnots, py::arg("additional_knots"),
            cls_doc.InsertKnots.doc)
        .def("CopyBlock", &Class::CopyBlock, py::arg("start_row"),
            py::arg("start_col"), py::arg("block_rows"), py::arg("block_cols"),
            cls_doc.CopyBlock.doc)
        .def("CopyHead", &Class::CopyHead, py::arg("n"), cls_doc.CopyHead.doc);
    DefCopyAndDeepCopy(&cls);
  }

  {
    using Class = PiecewiseTrajectory<T>;
    constexpr auto& cls_doc = doc.PiecewiseTrajectory;
    auto cls = DefineTemplateClassWithDefault<Class, Trajectory<T>>(
        m, "PiecewiseTrajectory", param, cls_doc.doc);
    cls  // BR
        .def("get_number_of_segments", &Class::get_number_of_segments,
            cls_doc.get_number_of_segments.doc)
        .def("start_time", overload_cast_explicit<T, int>(&Class::start_time),
            py::arg("segment_index"), cls_doc.start_time.doc)
        .def("end_time", overload_cast_explicit<T, int>(&Class::end_time),
            py::arg("segment_index"), cls_doc.end_time.doc)
        .def("duration", &Class::duration, py::arg("segment_index"),
            cls_doc.duration.doc)
        // N.B. We must redefine these two overloads, as we cannot use the base
        // classes' overloads. See:
        // https://github.com/pybind/pybind11/issues/974
        .def("start_time", overload_cast_explicit<T>(&Class::start_time),
            cls_doc.start_time.doc)
        .def("end_time", overload_cast_explicit<T>(&Class::end_time),
            cls_doc.end_time.doc)
        .def("is_time_in_range", &Class::is_time_in_range, py::arg("t"),
            cls_doc.is_time_in_range.doc)
        .def("get_segment_index", &Class::get_segment_index, py::arg("t"),
            cls_doc.get_segment_index.doc)
        .def("get_segment_times", &Class::get_segment_times,
            cls_doc.get_segment_times.doc);
  }

  {
    using Class = PiecewisePolynomial<T>;
    constexpr auto& cls_doc = doc.PiecewisePolynomial;
    auto cls = DefineTemplateClassWithDefault<Class, PiecewiseTrajectory<T>>(
        m, "PiecewisePolynomial", param, cls_doc.doc);
    cls  // BR
        .def(py::init<>(), cls_doc.ctor.doc_0args)
        .def(py::init<const Eigen::Ref<const MatrixX<T>>&>(),
            cls_doc.ctor.doc_1args_constEigenMatrixBase)
        .def(py::init<std::vector<MatrixX<Polynomial<T>>> const&,
                 std::vector<T> const&>(),
            cls_doc.ctor.doc_2args_polynomials_matrix_breaks)
        .def(py::init<std::vector<Polynomial<T>> const&,
                 std::vector<T> const&>(),
            cls_doc.ctor.doc_2args_polynomials_breaks)
        .def("Clone", &Class::Clone, cls_doc.Clone.doc)
        .def_static(
            "ZeroOrderHold",
            // This serves the same purpose as the C++ ZeroOrderHold(VectorX<T>,
            // MatrixX<T>) method.  For 2d numpy arrays, pybind apparently
            // matches vector<vector<T>> then vector<MatrixX<T>> then
            // MatrixX<T>.
            [](const std::vector<T>& breaks,
                const std::vector<std::vector<T>>& samples) {
              return Class::ZeroOrderHold(
                  breaks, MakeEigenFromRowMajorVectors(samples));
            },
            py::arg("breaks"), py::arg("samples"),
            cls_doc.ZeroOrderHold.doc_vector)
        .def_static("ZeroOrderHold",
            py::overload_cast<const std::vector<T>&,
                const std::vector<MatrixX<T>>&>(&Class::ZeroOrderHold),
            py::arg("breaks"), py::arg("samples"),
            cls_doc.ZeroOrderHold.doc_matrix)
        .def_static(
            "FirstOrderHold",
            [](const std::vector<T>& breaks,
                const std::vector<std::vector<T>>& samples) {
              return Class::FirstOrderHold(
                  breaks, MakeEigenFromRowMajorVectors(samples));
            },
            py::arg("breaks"), py::arg("samples"),
            cls_doc.FirstOrderHold.doc_vector)
        .def_static("FirstOrderHold",
            py::overload_cast<const std::vector<T>&,
                const std::vector<MatrixX<T>>&>(&Class::FirstOrderHold),
            py::arg("breaks"), py::arg("samples"),
            cls_doc.FirstOrderHold.doc_matrix)
        .def_static(
            "CubicShapePreserving",
            [](const std::vector<T>& breaks,
                const std::vector<std::vector<T>>& samples,
                bool zero_end_point_derivatives) {
              return Class::CubicShapePreserving(breaks,
                  MakeEigenFromRowMajorVectors(samples),
                  zero_end_point_derivatives);
            },
            py::arg("breaks"), py::arg("samples"),
            py::arg("zero_end_point_derivatives") = false,
            cls_doc.CubicShapePreserving.doc_vector)
        .def_static("CubicShapePreserving",
            py::overload_cast<const std::vector<T>&,
                const std::vector<MatrixX<T>>&, bool>(
                &Class::CubicShapePreserving),
            py::arg("breaks"), py::arg("samples"),
            py::arg("zero_end_point_derivatives") = false,
            cls_doc.CubicShapePreserving.doc_matrix)
        .def_static(
            "CubicWithContinuousSecondDerivatives",
            [](const std::vector<T>& breaks,
                const std::vector<std::vector<T>>& samples,
                const MatrixX<T>& sample_dot_at_start,
                const MatrixX<T>& sample_dot_at_end) {
              return PiecewisePolynomial<
                  T>::CubicWithContinuousSecondDerivatives(breaks,
                  MakeEigenFromRowMajorVectors(samples), sample_dot_at_start,
                  sample_dot_at_end);
            },
            py::arg("breaks"), py::arg("samples"),
            py::arg("sample_dot_at_start"), py::arg("sample_dot_at_end"),
            cls_doc.CubicWithContinuousSecondDerivatives.doc_4args_vector)
        .def_static("CubicWithContinuousSecondDerivatives",
            py::overload_cast<const std::vector<T>&,
                const std::vector<MatrixX<T>>&, const MatrixX<T>&,
                const MatrixX<T>&>(
                &Class::CubicWithContinuousSecondDerivatives),
            py::arg("breaks"), py::arg("samples"),
            py::arg("sample_dot_at_start"), py::arg("sample_dot_at_end"),
            cls_doc.CubicWithContinuousSecondDerivatives.doc_4args_matrix)
        .def_static(
            "CubicHermite",
            [](const std::vector<T>& breaks,
                const std::vector<std::vector<T>>& samples,
                const std::vector<std::vector<T>>& samples_dot) {
              return Class::CubicHermite(breaks,
                  MakeEigenFromRowMajorVectors(samples),
                  MakeEigenFromRowMajorVectors(samples_dot));
            },
            py::arg("breaks"), py::arg("samples"), py::arg("samples_dot"),
            cls_doc.CubicHermite.doc_vector)
        .def_static("CubicHermite",
            py::overload_cast<const std::vector<T>&,
                const std::vector<MatrixX<T>>&, const std::vector<MatrixX<T>>&>(
                &Class::CubicHermite),
            py::arg("breaks"), py::arg("samples"), py::arg("samples_dot"),
            cls_doc.CubicHermite.doc_matrix)
        .def_static(
            "CubicWithContinuousSecondDerivatives",
            [](const std::vector<T>& breaks,
                const std::vector<std::vector<T>>& samples,
                bool periodic_end_condition) {
              return PiecewisePolynomial<
                  T>::CubicWithContinuousSecondDerivatives(breaks,
                  MakeEigenFromRowMajorVectors(samples),
                  periodic_end_condition);
            },
            py::arg("breaks"), py::arg("samples"),
            py::arg("periodic_end_condition") = false,
            cls_doc.CubicWithContinuousSecondDerivatives.doc_3args_vector)
        .def_static("CubicWithContinuousSecondDerivatives",
            py::overload_cast<const std::vector<T>&,
                const std::vector<MatrixX<T>>&, bool>(
                &Class::CubicWithContinuousSecondDerivatives),
            py::arg("breaks"), py::arg("samples"), py::arg("periodic_end"),
            cls_doc.CubicWithContinuousSecondDerivatives.doc_3args_matrix)
        .def_static(
            "LagrangeInterpolatingPolynomial",
            [](const std::vector<T>& breaks,
                const std::vector<std::vector<T>>& samples) {
              return Class::LagrangeInterpolatingPolynomial(
                  breaks, MakeEigenFromRowMajorVectors(samples));
            },
            py::arg("times"), py::arg("samples"),
            cls_doc.LagrangeInterpolatingPolynomial.doc_vector)
        .def_static("LagrangeInterpolatingPolynomial",
            py::overload_cast<const std::vector<T>&,
                const std::vector<MatrixX<T>>&>(
                &Class::LagrangeInterpolatingPolynomial),
            py::arg("times"), py::arg("samples"),
            cls_doc.LagrangeInterpolatingPolynomial.doc_matrix)
        .def("derivative", &Class::derivative, py::arg("derivative_order") = 1,
            cls_doc.derivative.doc)
        .def("getPolynomialMatrix", &Class::getPolynomialMatrix,
            py::arg("segment_index"), cls_doc.getPolynomialMatrix.doc)
        .def("getPolynomial", &Class::getPolynomial, py::arg("segment_index"),
            py::arg("row") = 0, py::arg("col") = 0, cls_doc.getPolynomial.doc)
        .def("getSegmentPolynomialDegree", &Class::getSegmentPolynomialDegree,
            py::arg("segment_index"), py::arg("row") = 0, py::arg("col") = 0,
            cls_doc.getSegmentPolynomialDegree.doc)
        .def("isApprox", &Class::isApprox, py::arg("other"), py::arg("tol"),
            py::arg("tol_type") = drake::ToleranceType::kRelative,
            cls_doc.isApprox.doc)
        .def("Reshape", &Class::Reshape, py::arg("rows"), py::arg("cols"),
            cls_doc.Reshape.doc)
        .def("Block", &Class::Block, py::arg("start_row"), py::arg("start_col"),
            py::arg("block_rows"), py::arg("block_cols"), cls_doc.Block.doc)
        .def("ConcatenateInTime", &Class::ConcatenateInTime, py::arg("other"),
            cls_doc.ConcatenateInTime.doc)
        .def("AppendCubicHermiteSegment", &Class::AppendCubicHermiteSegment,
            py::arg("time"), py::arg("sample"), py::arg("sample_dot"),
            cls_doc.AppendCubicHermiteSegment.doc)
        .def("AppendFirstOrderSegment", &Class::AppendFirstOrderSegment,
            py::arg("time"), py::arg("sample"),
            cls_doc.AppendFirstOrderSegment.doc)
        .def("RemoveFinalSegment", &Class::RemoveFinalSegment,
            cls_doc.RemoveFinalSegment.doc)
        .def("ReverseTime", &Class::ReverseTime, cls_doc.ReverseTime.doc)
        .def("ScaleTime", &Class::ScaleTime, py::arg("scale"),
            cls_doc.ScaleTime.doc)
        .def("slice", &Class::slice, py::arg("start_segment_index"),
            py::arg("num_segments"), cls_doc.slice.doc)
        .def("shiftRight", &Class::shiftRight, py::arg("offset"),
            cls_doc.shiftRight.doc)
        .def(py::self + py::self)
        .def("setPolynomialMatrixBlock", &Class::setPolynomialMatrixBlock,
            py::arg("replacement"), py::arg("segment_index"),
            py::arg("row_start") = 0, py::arg("col_start") = 0,
            cls_doc.setPolynomialMatrixBlock.doc);
    DefCopyAndDeepCopy(&cls);
  }

  {
    using Class = PiecewiseQuaternionSlerp<T>;
    constexpr auto& cls_doc = doc.PiecewiseQuaternionSlerp;
    auto cls = DefineTemplateClassWithDefault<Class, PiecewiseTrajectory<T>>(
        m, "PiecewiseQuaternionSlerp", param, cls_doc.doc);
    cls  // BR
        .def(py::init<>(), cls_doc.ctor.doc_0args)
        .def(py::init<const std::vector<T>&,
                 const std::vector<Quaternion<T>>&>(),
            py::arg("breaks"), py::arg("quaternions"),
            cls_doc.ctor.doc_2args_breaks_quaternions)
        .def(py::init<const std::vector<T>&, const std::vector<Matrix3<T>>&>(),
            py::arg("breaks"), py::arg("rotation_matrices"),
            cls_doc.ctor.doc_2args_breaks_rotation_matrices)
        .def(py::init<const std::vector<T>&,
                 const std::vector<math::RotationMatrix<T>>&>(),
            py::arg("breaks"), py::arg("rotation_matrices"),
            cls_doc.ctor.doc_2args_breaks_rotation_matrices)
        .def(
            py::init<const std::vector<T>&, const std::vector<AngleAxis<T>>&>(),
            py::arg("breaks"), py::arg("angle_axes"),
            cls_doc.ctor.doc_2args_breaks_angle_axes)
        .def("Append",
            py::overload_cast<const T&, const Quaternion<T>&>(&Class::Append),
            py::arg("time"), py::arg("quaternion"),
            cls_doc.Append.doc_2args_time_quaternion)
        .def("Append",
            py::overload_cast<const T&, const math::RotationMatrix<T>&>(
                &Class::Append),
            py::arg("time"), py::arg("rotation_matrix"),
            cls_doc.Append.doc_2args_time_rotation_matrix)
        .def("Append",
            py::overload_cast<const T&, const AngleAxis<T>&>(&Class::Append),
            py::arg("time"), py::arg("angle_axis"),
            cls_doc.Append.doc_2args_time_angle_axis)
        .def("orientation", &Class::orientation, py::arg("time"),
            cls_doc.orientation.doc)
        .def("angular_velocity", &Class::angular_velocity, py::arg("time"),
            cls_doc.angular_velocity.doc)
        .def("angular_acceleration", &Class::angular_acceleration,
            py::arg("time"), cls_doc.angular_acceleration.doc);
    DefCopyAndDeepCopy(&cls);
  }

  {
    using Class = PiecewisePose<T>;
    constexpr auto& cls_doc = doc.PiecewisePose;
    auto cls = DefineTemplateClassWithDefault<Class, PiecewiseTrajectory<T>>(
        m, "PiecewisePose", param, cls_doc.doc);
    cls  // BR
        .def(py::init<>(), cls_doc.ctor.doc_0args)
        .def(py::init<const PiecewisePolynomial<T>&,
                 const PiecewiseQuaternionSlerp<T>&>(),
            py::arg("position_trajectory"), py::arg("orientation_trajectory"),
            cls_doc.ctor.doc_2args)
        .def_static("MakeLinear", &Class::MakeLinear, py::arg("times"),
            py::arg("poses"), cls_doc.MakeLinear.doc)
        .def_static("MakeCubicLinearWithEndLinearVelocity",
            &Class::MakeCubicLinearWithEndLinearVelocity, py::arg("times"),
            py::arg("poses"), py::arg("start_vel") = Vector3<T>::Zero().eval(),
            py::arg("end_vel") = Vector3<T>::Zero().eval(),
            cls_doc.MakeCubicLinearWithEndLinearVelocity.doc)
        .def("GetPose", &Class::GetPose, py::arg("time"), cls_doc.GetPose.doc)
        .def("GetVelocity", &Class::GetVelocity, py::arg("time"),
            cls_doc.GetVelocity.doc)
        .def("GetAcceleration", &Class::GetAcceleration, py::arg("time"),
            cls_doc.GetAcceleration.doc)
        .def("IsApprox", &Class::IsApprox, py::arg("other"), py::arg("tol"),
            cls_doc.IsApprox.doc)
        .def("get_position_trajectory", &Class::get_position_trajectory,
            cls_doc.get_position_trajectory.doc)
        .def("get_orientation_trajectory", &Class::get_orientation_trajectory,
            cls_doc.get_orientation_trajectory.doc);
    DefCopyAndDeepCopy(&cls);
  }
}
}  // namespace

PYBIND11_MODULE(trajectories, m) {
  py::module::import("pydrake.autodiffutils");
  py::module::import("pydrake.common");
  py::module::import("pydrake.polynomial");
  py::module::import("pydrake.symbolic");

  type_visit([m](auto dummy) { DoScalarDependentDefinitions(m, dummy); },
      CommonScalarPack{});
}
}  // namespace pydrake
}  // namespace drake
