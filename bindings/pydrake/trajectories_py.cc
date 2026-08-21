#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "pybind11/eval.h"

#include "drake/bindings/generated_docstrings/common_trajectories.h"
#include "drake/bindings/pydrake/common/default_scalars_pybind.h"
#include "drake/bindings/pydrake/common/deprecation_pybind.h"
#include "drake/bindings/pydrake/polynomial_types_pybind.h"
#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/common/polynomial.h"
#include "drake/common/scope_exit.h"
#include "drake/common/trajectories/bezier_curve.h"
#include "drake/common/trajectories/bspline_trajectory.h"
#include "drake/common/trajectories/composite_trajectory.h"
#include "drake/common/trajectories/derivative_trajectory.h"
#include "drake/common/trajectories/discrete_time_trajectory.h"
#include "drake/common/trajectories/exponential_plus_piecewise_polynomial.h"
#include "drake/common/trajectories/function_handle_trajectory.h"
#include "drake/common/trajectories/path_parameterized_trajectory.h"
#include "drake/common/trajectories/piecewise_polynomial.h"
#include "drake/common/trajectories/piecewise_pose.h"
#include "drake/common/trajectories/piecewise_quaternion.h"
#include "drake/common/trajectories/stacked_trajectory.h"
#include "drake/common/trajectories/trajectory.h"
#include "drake/common/trajectories/wrapped_trajectory.h"

namespace drake {
namespace pydrake {

namespace {

using trajectories::Trajectory;

// Binds PiecewisePolynomial<double>::Serialize().
// We cannot use DefAttributesUsingSerialize() for this because the Serialize
// function transforms the data (rather than providing pointers to the member
// fields as would be typical), so we must bind the attributes manually.
template <typename PyClass>
void BindPiecewisePolynomialSerialize(PyClass* cls) {
  using Class = trajectories::PiecewisePolynomial<double>;
  // The C++ types of the serialized fields.
  using Breaks = std::vector<double>;
  using Polynomials = std::vector<MatrixX<Eigen::VectorXd>>;
  // An adapter to get and/or set the serialized fields.
  struct Archive {
    void Visit(const NameValue<Breaks>& nv) {
      if (set_breaks) {
        *nv.value() = std::move(breaks);
      } else {
        breaks = *nv.value();
      }
    }
    void Visit(const NameValue<Polynomials>& nv) {
      if (set_polynomials) {
        *nv.value() = std::move(polynomials);
      } else {
        polynomials = *nv.value();
      }
    }
    bool set_breaks{false};
    Breaks breaks;
    bool set_polynomials{false};
    Polynomials polynomials;
  };
  // Add the same __fields__ that DefAttributesUsingSerialize would have added.
  cls->def_property_readonly_static("__fields__", [](py::object /* cls */) {
    auto ndarray = py::module::import("numpy").attr("ndarray");
    auto make_namespace = py::module::import("types").attr("SimpleNamespace");
    auto breaks = make_namespace();
    py::setattr(breaks, "name", py::str("breaks"));
    py::setattr(breaks, "type", ndarray);
    auto polynomials = make_namespace();
    py::setattr(polynomials, "name", py::str("polynomials"));
    py::setattr(polynomials, "type", ndarray);
    py::list fields;
    fields.append(breaks);
    fields.append(polynomials);
    return py::tuple(fields);
  });
  // Given the __fields__ above, yaml_dump_typed (and yaml_load_typed) will try
  // to getattr (and setattr) on "breaks" and "polynomials". However, we don't
  // want to expose those properties to users so we'll respell the name to add a
  // leading underscore, and bind the properties using the private name.
  cls->def("__getattr__", [](Class& self, py::str name) -> py::object {
    py::object self_py = py::cast(self, py_rvp::reference);
    if (std::string(name) == "breaks") {
      return self_py.attr("_breaks");
    } else if (std::string(name) == "polynomials") {
      return self_py.attr("_polynomials");
    }
    throw py::attribute_error(fmt::format(
        "PiecewisePolynomial has no attribute '{}'", std::string{name}));
  });
  cls->def("__setattr__", [](Class& self, py::str name, py::object value) {
    if (std::string(name) == "breaks") {
      name = py::str("_breaks");
    } else if (std::string(name) == "polynomials") {
      name = py::str("_polynomials");
    }
    py::eval("object.__setattr__")(self, name, value);
  });
  // Define a private property for "_breaks". Setting the breaks resets all of
  // the polynomials; this is fine because deserialization matches __fields__
  // order, which has "breaks" come first followed by setting the "polynomials"
  // afterward.
  cls->def_property(
      "_breaks",
      [](const Class& self) -> Eigen::VectorXd {
        const int num_poly = self.get_number_of_segments();
        if (num_poly == 0) {
          return Eigen::VectorXd();
        }
        Eigen::VectorXd breaks(num_poly + 1);
        breaks[0] = self.start_time(0);
        for (int i = 0; i < num_poly; ++i) {
          breaks[i + 1] = self.end_time(i);
        }
        return breaks;
      },
      [](Class& self, const Breaks& breaks) {
        const size_t num_poly = breaks.empty() ? 0 : breaks.size() - 1;
        const MatrixX<Eigen::VectorXd> empty_poly;
        Archive archive{.set_breaks = true,
            .breaks = breaks,
            .set_polynomials = true,
            .polynomials = Polynomials(num_poly, empty_poly)};
        self.Serialize(&archive);
      });
  // Define a private property for "_polynomials". The property is a 4D ndarray
  // that we biject to C++'s convention of vector-of-matrix-of-coeffs storage.
  cls->def_property(
      "_polynomials",
      [](const Class& self) -> py::array_t<double> {
        Archive archive;
        const_cast<Class&>(self).Serialize(&archive);
        const Polynomials& polynomials = archive.polynomials;
        const int num_poly = ssize(polynomials);
        const int num_rows = num_poly == 0 ? 0 : polynomials.at(0).rows();
        const int num_cols = num_poly == 0 ? 0 : polynomials.at(0).cols();
        const int num_coeffs = (num_rows == 0 || num_cols == 0)
                                   ? 0
                                   : polynomials.at(0)(0, 0).size();
        const std::vector<int> shape{num_poly, num_rows, num_cols, num_coeffs};
        std::vector<double> buffer;
        for (int i = 0; i < num_poly; ++i) {
          for (int j = 0; j < num_rows; ++j) {
            for (int k = 0; k < num_cols; ++k) {
              for (int c = 0; c < num_coeffs; ++c) {
                buffer.push_back(polynomials[i](j, k)(c));
              }
            }
          }
        }
        return py::array_t<double>(shape, buffer.data());
      },
      [](Class& self, const py::array_t<double>& polynomials) {
        Polynomials cxx_poly;
        if (polynomials.size() > 0) {
          DRAKE_THROW_UNLESS(polynomials.ndim() == 4);
          const int num_poly = polynomials.shape(0);
          const int num_rows = polynomials.shape(1);
          const int num_cols = polynomials.shape(2);
          const int num_coeffs = polynomials.shape(3);
          cxx_poly.resize(num_poly);
          for (int i = 0; i < num_poly; ++i) {
            cxx_poly[i].resize(num_rows, num_cols);
            for (int j = 0; j < num_rows; ++j) {
              for (int k = 0; k < num_cols; ++k) {
                cxx_poly[i](j, k).resize(num_coeffs);
                for (int c = 0; c < num_coeffs; ++c) {
                  cxx_poly[i](j, k)(c) = polynomials.at(i, j, k, c);
                }
              }
            }
          }
        }
        Archive archive{
            .set_polynomials = true, .polynomials = std::move(cxx_poly)};
        self.Serialize(&archive);
      });
}

// Provides a templated 'namespace'.
template <typename T>
struct Impl {
  // For bindings that want a `const vector<VectorX<T>>&` but are bound
  // via a `const vector<vector<T>>&` for overloading priority,
  // this function converts the input.  A numpy matrix is row-major, so a 2x3
  // samples numpy array (unfortunately) turns into a std::vector with 2
  // elements, each a vector of 3 elements; we'll transpose that.
  static std::vector<MatrixX<T>> MakeEigenFromRowMajorVectors(
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

  class TrajectoryPublic : public Trajectory<T> {
   public:
    using Base = Trajectory<T>;

    TrajectoryPublic() = default;

    // Virtual methods, for explicit bindings.
    using Base::do_cols;
    using Base::do_end_time;
    using Base::do_has_derivative;
    using Base::do_rows;
    using Base::do_start_time;
    using Base::do_value;
    using Base::DoClone;
  };

  class PyTrajectory : public TrajectoryPublic {
   public:
    using Base = TrajectoryPublic;
    using Base::Base;

    // Utility function that takes a Python object which is-a Trajectory and
    // wraps it in a unique_ptr that manages object lifetime when returned back
    // to C++.
    static std::unique_ptr<Trajectory<T>> WrapPyTrajectory(py::object py_traj) {
      DRAKE_THROW_UNLESS(!py_traj.is_none());
      // Convert py_traj to a shared_ptr<Trajectory<T>> whose C++ lifetime keeps
      // the python object alive.
      Trajectory<T>* cpp_traj = py::cast<Trajectory<T>*>(py_traj);
      DRAKE_THROW_UNLESS(cpp_traj != nullptr);
      std::shared_ptr<Trajectory<T>> shared_cpp_traj(
          /* stored pointer = */ cpp_traj,
          /* deleter = */ [captured_py_traj = std::move(py_traj)](  // BR
                              void*) mutable {
            py::gil_scoped_acquire deleter_guard;
            captured_py_traj = py::none();
          });
      // Wrap the shared_ptr inside a WrappedTrajectory and return that via
      // unique_ptr to meet our required return signature.
      return std::make_unique<trajectories::internal::WrappedTrajectory<T>>(
          std::move(shared_cpp_traj));
    }

    // Trampoline virtual methods.

    std::unique_ptr<Trajectory<T>> DoClone() const final {
      py::gil_scoped_acquire guard;
      // Trajectory subclasses in Python must implement cloning by defining
      // a __deepcopy__ method.
      auto deepcopy = py::module_::import("copy").attr("deepcopy");
      return WrapPyTrajectory(deepcopy(this));
    }

    MatrixX<T> do_value(const T& t) const final {
      PYBIND11_OVERRIDE_PURE(MatrixX<T>, Trajectory<T>, do_value, t);
    }

    bool do_has_derivative() const final {
      PYBIND11_OVERRIDE_PURE(bool, Trajectory<T>, do_has_derivative);
    }

    MatrixX<T> DoEvalDerivative(const T& t, int derivative_order) const final {
      PYBIND11_OVERRIDE_PURE(
          MatrixX<T>, Trajectory<T>, DoEvalDerivative, t, derivative_order);
    }

    std::unique_ptr<Trajectory<T>> DoMakeDerivative(
        int derivative_order) const final {
      py::gil_scoped_acquire guard;
      // Because the PYBIND11_OVERRIDE_PURE macro embeds a `return ...;`
      // statement, we must wrap it in lambda so that we can post-process the
      // return value.
      auto make_python_derivative = [&]() -> py::object {
        PYBIND11_OVERRIDE_PURE(
            py::object, Trajectory<T>, DoMakeDerivative, derivative_order);
      };
      return WrapPyTrajectory(make_python_derivative());
    }

    T do_start_time() const final {
      PYBIND11_OVERRIDE_PURE(T, Trajectory<T>, do_start_time);
    }

    T do_end_time() const final {
      PYBIND11_OVERRIDE_PURE(T, Trajectory<T>, do_end_time);
    }

    Eigen::Index do_rows() const final {
      PYBIND11_OVERRIDE_PURE(Eigen::Index, Trajectory<T>, do_rows);
    }

    Eigen::Index do_cols() const final {
      PYBIND11_OVERRIDE_PURE(Eigen::Index, Trajectory<T>, do_cols);
    }
  };

  static void DoScalarDependentDefinitions(py::module m) {
    py::tuple param = GetPyParam<T>();

    // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
    using namespace drake::trajectories;
    constexpr auto& doc = pydrake_doc_common_trajectories.drake.trajectories;

    {
      using Class = Trajectory<T>;
      constexpr auto& cls_doc = doc.Trajectory;
      auto cls = DefineTemplateClassWithDefault<Class, PyTrajectory>(
          m, "Trajectory", param, cls_doc.doc);
      cls  // BR
          .def(py::init<>())
          .def("value", &Class::value, py::arg("t"), cls_doc.value.doc)
          .def("vector_values",
              overload_cast_explicit<MatrixX<T>, const std::vector<T>&>(
                  &Class::vector_values),
              py::arg("t"), cls_doc.vector_values.doc)
          .def("has_derivative", &Class::has_derivative,
              cls_doc.has_derivative.doc)
          .def("EvalDerivative", &Class::EvalDerivative, py::arg("t"),
              py::arg("derivative_order") = 1, cls_doc.EvalDerivative.doc)
          .def("MakeDerivative", &Class::MakeDerivative,
              py::arg("derivative_order") = 1, cls_doc.MakeDerivative.doc)
          .def("start_time", &Class::start_time, cls_doc.start_time.doc)
          .def("end_time", &Class::end_time, cls_doc.end_time.doc)
          .def("rows", &Class::rows, cls_doc.rows.doc)
          .def("cols", &Class::cols, cls_doc.cols.doc);
      // This binds Clone() and __copy__ and __deepcopy__. For final subclasses
      // of Trajectory that offer a public copy constructor, in their bindings
      // we also call DefCopyAndDeepCopy to override __copy__ and __deepcopy__
      // with a more efficient implementation.
      DefClone(&cls);
    }

    {
      using Class = BezierCurve<T>;
      constexpr auto& cls_doc = doc.BezierCurve;
      auto cls = DefineTemplateClassWithDefault<Class, Trajectory<T>>(
          m, "BezierCurve", param, cls_doc.doc);
      cls  // BR
          .def(py::init<>(), cls_doc.ctor.doc_0args)
          .def(py::init<double, double, const Eigen::Ref<const MatrixX<T>>&>(),
              py::arg("start_time"), py::arg("end_time"),
              py::arg("control_points"), cls_doc.ctor.doc_3args)
          .def("order", &Class::order, cls_doc.order.doc)
          .def("BernsteinBasis", &Class::BernsteinBasis, py::arg("i"),
              py::arg("time"), py::arg("order") = std::nullopt,
              cls_doc.BernsteinBasis.doc)
          .def("control_points", &Class::control_points,
              cls_doc.control_points.doc)
          .def("GetExpression", &Class::GetExpression,
              py::arg("time") = symbolic::Variable("t"),
              cls_doc.GetExpression.doc)
          .def("ElevateOrder", &Class::ElevateOrder, cls_doc.ElevateOrder.doc);
      if constexpr (std::is_same_v<T, double>) {  // #19712
        cls.def("AsLinearInControlPoints", &Class::AsLinearInControlPoints,
            py::arg("derivative_order") = 1,
            cls_doc.AsLinearInControlPoints.doc);
      }
      DefCopyAndDeepCopy(&cls);
    }

    {
      using Class = BsplineTrajectory<T>;
      constexpr auto& cls_doc = doc.BsplineTrajectory;
      auto cls = DefineTemplateClassWithDefault<Class, Trajectory<T>>(
          m, "BsplineTrajectory", param, cls_doc.doc);
      cls  // BR
          .def(py::init<>())
          // This overload will match 2d numpy arrays before
          // std::vector<MatrixX<T>>. We want each column of the numpy array as
          // a MatrixX of control points, but the std::vectors here are
          // associated with the rows in numpy.
          .def(py::init([](math::BsplineBasis<T> basis,
                            std::vector<std::vector<T>> control_points) {
            return Class(basis, MakeEigenFromRowMajorVectors(control_points));
          }),
              py::arg("basis"), py::arg("control_points"), cls_doc.ctor.doc)
          .def(py::init<math::BsplineBasis<T>, std::vector<MatrixX<T>>>(),
              py::arg("basis"), py::arg("control_points"), cls_doc.ctor.doc)
          .def("EvaluateLinearInControlPoints",
              &Class::EvaluateLinearInControlPoints, py::arg("parameter_value"),
              py::arg("derivative_order") = 0,
              cls_doc.EvaluateLinearInControlPoints.doc)
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
              py::arg("start_col"), py::arg("block_rows"),
              py::arg("block_cols"), cls_doc.CopyBlock.doc)
          .def("CopyHead", &Class::CopyHead, py::arg("n"), cls_doc.CopyHead.doc)
          .def(py::pickle(
              [](const Class& self) {
                return std::make_pair(self.basis(), self.control_points());
              },
              [](std::pair<math::BsplineBasis<T>, std::vector<MatrixX<T>>>
                      args) {
                return Class(std::get<0>(args), std::get<1>(args));
              }));
      if constexpr (std::is_same_v<T, double>) {  // #19712
        cls.def("AsLinearInControlPoints", &Class::AsLinearInControlPoints,
            py::arg("derivative_order") = 1,
            cls_doc.AsLinearInControlPoints.doc);
      }
      DefCopyAndDeepCopy(&cls);
    }

    {
      using Class = DerivativeTrajectory<T>;
      constexpr auto& cls_doc = doc.DerivativeTrajectory;
      auto cls = DefineTemplateClassWithDefault<Class, Trajectory<T>>(
          m, "DerivativeTrajectory", param, cls_doc.doc);
      cls  // BR
          .def(py::init<const Trajectory<T>&, int>(), py::arg("nominal"),
              py::arg("derivative_order") = 1, cls_doc.ctor.doc);
      DefCopyAndDeepCopy(&cls);
    }

    {
      using Class = FunctionHandleTrajectory<T>;
      constexpr auto& cls_doc = doc.FunctionHandleTrajectory;
      auto cls = DefineTemplateClassWithDefault<Class, Trajectory<T>>(
          m, "FunctionHandleTrajectory", param, cls_doc.doc);
      cls  // BR
          .def(py::init<const std::function<MatrixX<T>(const T&)>&, int, int,
                   double, double>(),
              py::arg("func"), py::arg("rows"), py::arg("cols") = 1,
              py::arg("start_time") = -std::numeric_limits<double>::infinity(),
              py::arg("end_time") = std::numeric_limits<double>::infinity(),
              cls_doc.ctor.doc)
          .def("set_derivative", &Class::set_derivative, py::arg("func"),
              cls_doc.set_derivative.doc);
      DefCopyAndDeepCopy(&cls);
    }

    {
      using Class = PathParameterizedTrajectory<T>;
      constexpr auto& cls_doc = doc.PathParameterizedTrajectory;
      auto cls = DefineTemplateClassWithDefault<Class, Trajectory<T>>(
          m, "PathParameterizedTrajectory", param, cls_doc.doc);
      cls  // BR
          .def(py::init<const Trajectory<T>&, const Trajectory<T>&>(),
              py::arg("path"), py::arg("time_scaling"), cls_doc.ctor.doc)
          .def("path", &Class::path, py_rvp::reference_internal,
              cls_doc.path.doc)
          .def("time_scaling", &Class::time_scaling, py_rvp::reference_internal,
              cls_doc.time_scaling.doc);
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
          // N.B. We must redefine these two overloads, as we cannot use the
          // base classes' overloads. See:
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
          .def_static(
              "ZeroOrderHold",
              // This serves the same purpose as the C++
              // ZeroOrderHold(VectorX<T>, MatrixX<T>) method.  For 2d numpy
              // arrays, pybind apparently matches vector<vector<T>> then
              // vector<MatrixX<T>> then MatrixX<T>.
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
                  const std::vector<MatrixX<T>>&,
                  const std::vector<MatrixX<T>>&>(&Class::CubicHermite),
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
          .def("derivative", &Class::derivative,
              py::arg("derivative_order") = 1, cls_doc.derivative.doc)
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
          .def("Transpose", &Class::Transpose, cls_doc.Transpose.doc)
          .def("Block", &Class::Block, py::arg("start_row"),
              py::arg("start_col"), py::arg("block_rows"),
              py::arg("block_cols"), cls_doc.Block.doc)
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
      if constexpr (std::is_same_v<T, double>) {
        BindPiecewisePolynomialSerialize(&cls);
      }
    }

    {
      using Class = CompositeTrajectory<T>;
      constexpr auto& cls_doc = doc.CompositeTrajectory;
      auto cls = DefineTemplateClassWithDefault<Class, PiecewiseTrajectory<T>>(
          m, "CompositeTrajectory", param, cls_doc.doc);
      cls  // BR
          .def(py::init([](std::vector<const Trajectory<T>*> py_segments) {
            std::vector<copyable_unique_ptr<Trajectory<T>>> segments;
            segments.reserve(py_segments.size());
            for (const Trajectory<T>* py_segment : py_segments) {
              segments.emplace_back(py_segment ? py_segment->Clone() : nullptr);
            }
            return std::make_unique<CompositeTrajectory<T>>(
                std::move(segments));
          }),
              py::arg("segments"), cls_doc.ctor.doc)
          .def("segment", &Class::segment, py::arg("segment_index"),
              py_rvp::reference_internal, cls_doc.segment.doc)
          .def_static(
              "AlignAndConcatenate",
              [](std::vector<const Trajectory<T>*> py_segments) {
                std::vector<copyable_unique_ptr<Trajectory<T>>> segments;
                segments.reserve(py_segments.size());
                for (const Trajectory<T>* py_segment : py_segments) {
                  segments.emplace_back(
                      py_segment ? py_segment->Clone() : nullptr);
                }
                return CompositeTrajectory<T>::AlignAndConcatenate(segments);
              },
              py::arg("segments"), cls_doc.AlignAndConcatenate.doc);
      DefCopyAndDeepCopy(&cls);
    }

    {
      using Class = DiscreteTimeTrajectory<T>;
      constexpr auto& cls_doc = doc.DiscreteTimeTrajectory;
      auto cls = DefineTemplateClassWithDefault<Class, Trajectory<T>>(
          m, "DiscreteTimeTrajectory", param, cls_doc.doc);
      cls  // BR
          .def(py::init<std::vector<T>, std::vector<MatrixX<T>>, double>(),
              py::arg("times"), py::arg("values"),
              py::arg("time_comparison_tolerance") =
                  std::numeric_limits<double>::epsilon(),
              cls_doc.ctor.doc_stdvector)
          .def("ToZeroOrderHold", &Class::ToZeroOrderHold,
              cls_doc.ToZeroOrderHold.doc)
          .def("time_comparison_tolerance", &Class::time_comparison_tolerance,
              cls_doc.time_comparison_tolerance.doc)
          .def("num_times", &Class::num_times, cls_doc.num_times.doc)
          .def("get_times", &Class::get_times, cls_doc.get_times.doc);
      DefCopyAndDeepCopy(&cls);
    }

    if constexpr (std::is_same_v<T, double>) {
      using Class = ExponentialPlusPiecewisePolynomial<T>;
      constexpr auto& cls_doc = doc.ExponentialPlusPiecewisePolynomial;
      auto cls = DefineTemplateClassWithDefault<Class, PiecewiseTrajectory<T>>(
          m, "ExponentialPlusPiecewisePolynomial", param, cls_doc.doc);
      cls  // BR
          .def(
              py::init(
                  [](const Eigen::MatrixX<T>& K, const Eigen::MatrixX<T>& A,
                      const Eigen::MatrixX<T>& alpha,
                      const PiecewisePolynomial<T>& piecewise_polynomial_part) {
                    return Class(K, A, alpha, piecewise_polynomial_part);
                  }),
              py::arg("K"), py::arg("A"), py::arg("alpha"),
              py::arg("piecewise_polynomial_part"), cls_doc.ctor.doc)
          .def("shiftRight", &Class::shiftRight, py::arg("offset"),
              cls_doc.shiftRight.doc);
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
          .def(
              py::init<const std::vector<T>&, const std::vector<Matrix3<T>>&>(),
              py::arg("breaks"), py::arg("rotation_matrices"),
              cls_doc.ctor.doc_2args_breaks_rotation_matrices)
          .def(py::init<const std::vector<T>&,
                   const std::vector<math::RotationMatrix<T>>&>(),
              py::arg("breaks"), py::arg("rotation_matrices"),
              cls_doc.ctor.doc_2args_breaks_rotation_matrices)
          .def(py::init<const std::vector<T>&,
                   const std::vector<AngleAxis<T>>&>(),
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
              py::arg("poses"),
              py::arg("start_vel") = Vector3<T>::Zero().eval(),
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

    {
      using Class = StackedTrajectory<T>;
      constexpr auto& cls_doc = doc.StackedTrajectory;
      auto cls = DefineTemplateClassWithDefault<Class, Trajectory<T>>(
          m, "StackedTrajectory", param, cls_doc.doc);
      cls  // BR
          .def(py::init<bool>(), py::arg("rowwise") = true, cls_doc.ctor.doc)
          .def("Append",
              py::overload_cast<const Trajectory<T>&>(&Class::Append),
              /* N.B. We choose to omit any py::arg name here. */
              cls_doc.Append.doc);
      DefCopyAndDeepCopy(&cls);
    }

    {
      using Class = trajectories::internal::WrappedTrajectory<T>;
      auto cls = DefineTemplateClassWithDefault<Class, Trajectory<T>>(
          m, "_WrappedTrajectory", param, "(Internal use only)");
      cls  // BR
          .def(py::init([](const Trajectory<T>& trajectory) {
            // The keep_alive is responsible for object lifetime, so we'll give
            // the constructor an unowned pointer.
            return std::make_unique<Class>(
                make_unowned_shared_ptr_from_raw(&trajectory));
          }),
              py::arg("trajectory"),
              // Keep alive, ownership: `return` keeps `trajectory` alive.
              py::keep_alive<0, 1>())
          .def("unwrap", &Class::unwrap, py_rvp::reference_internal);
    }
  }
};
}  // namespace

PYBIND11_MODULE(trajectories, m) {
  py::module::import("pydrake.autodiffutils");
  py::module::import("pydrake.common");
  py::module::import("pydrake.polynomial");
  py::module::import("pydrake.symbolic");

  // Do templated instantiations of system types.
  auto bind_common_scalar_types = [m](auto dummy) {
    using T = decltype(dummy);
    Impl<T>::DoScalarDependentDefinitions(m);
  };
  type_visit(bind_common_scalar_types, CommonScalarPack{});
  ExecuteExtraPythonCode(m);
}
}  // namespace pydrake
}  // namespace drake
