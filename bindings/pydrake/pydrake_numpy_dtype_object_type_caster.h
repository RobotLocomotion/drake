#pragma once

#include "nanobind/eigen/dense.h"

#ifndef PYDRAKE_USE_NANOBIND
#error "Should only be used when the binder is nanobind!"
#endif

#include <iostream>

namespace nanobind {
namespace detail {

template <typename T>
constexpr bool is_pydrake_numpy_dtype_object_castable =
    is_eigen_plain_v<T> || is_eigen_xpr_v<T>;

template <typename T>
struct pydrake_numpy_dtype_object_type_caster {
  using Scalar = typename T::Scalar;
  using PlainScalar = std::remove_cv_t<Scalar>;
  using PlainScalarCaster = make_caster<PlainScalar>;
  static constexpr bool kCompileTime1D =
      (T::RowsAtCompileTime == 1 || T::ColsAtCompileTime == 1);

  NB_TYPE_CASTER(
      T, const_name("numpy.ndarray[") +
             concat_maybe(const_name("dtype=") + PlainScalarCaster::Name,
                 // Config::Shape::name,
                 // Config::Order::name,
                 dtype_const_name<Scalar>::name) +
             const_name("]"))

  bool from_python(handle src, uint8_t flags, cleanup_list* /* cleanup */) {
    auto numpy = module_::import_("numpy");

    if (src.is_none()) {
      return false;
    }

    // Avoid converting np.array(dtype=np.float64) to AutoDiff prematurely.
    // Only accept autodiff conversions as a last resort.
    const bool convert = flags & (uint8_t)cast_flags::convert;
    if (!convert) {
      return false;
    }

    // TODO(jwnimmer-tri) This implementation is probably terribly inefficient.
    // Using the NumPy C API (like pybind11 does) would probably be faster.

    auto array = numpy.attr("asarray")(src, arg("dtype") = "object");
    auto shape = cast<nanobind::tuple>(array.attr("shape"));

    int rows = 0;
    int cols = 0;
    if constexpr (kCompileTime1D) {
      if (shape.size() != 1) {
        std::cerr << "from_python wrong shape 1d\n";
        return false;
      }
      rows = cast<int>(shape[0]);
      cols = 1;
    } else {
      if (shape.size() == 1) {
        // Promote from 1d array to 2d array (as column vector).
        array = array.attr("reshape")(-1, 1);
        rows = cast<int>(shape[0]);
        cols = 1;
      } else if (shape.size() == 2) {
        rows = cast<int>(shape[0]);
        cols = cast<int>(shape[1]);
      } else {
        std::cerr << "from_python wrong shape 2d\n";
        return false;
      }
    }

    try {
      if constexpr (T::RowsAtCompileTime != Eigen::Dynamic) {
        if (rows != T::RowsAtCompileTime) {
          return false;
        }
      }
      if constexpr (T::ColsAtCompileTime != Eigen::Dynamic) {
        if (cols != T::ColsAtCompileTime) {
          return false;
        }
      }
      value.resize(rows, cols);
      if constexpr (kCompileTime1D) {
        for (Eigen::Index i = 0; i < rows; ++i) {
          value(i) = cast<PlainScalar>(array[i]);
        }
      } else {
        for (Eigen::Index i = 0; i < rows; ++i) {
          for (Eigen::Index j = 0; j < cols; ++j) {
            list ij;
            ij.append(i);
            ij.append(j);
            value(i, j) = cast<PlainScalar>(array[nanobind::tuple(ij)]);
          }
        }
      }
    } catch (const cast_error&) {
      // XXX cleanup?
      return false;
    }

    return true;
  }

  static handle from_cpp(
      const T& src, rv_policy /* policy */, cleanup_list* /* cleanup */) {
    auto numpy = module_::import_("numpy");

    // Construct an empty numpy.ndarray with the desired shape.
    list shape;
    if constexpr (kCompileTime1D) {
      shape.append(src.size());
    } else {
      shape.append(src.rows());
      shape.append(src.cols());
    }
    auto result = numpy.attr("empty")(shape, arg("dtype") = "object");

    // Fill in the array elements, converting to Python objects one at a time.
    if constexpr (kCompileTime1D) {
      for (Eigen::Index i = 0; i < src.size(); ++i) {
        result[i] = src[i];
      }
    } else {
      for (Eigen::Index i = 0; i < src.rows(); ++i) {
        for (Eigen::Index j = 0; j < src.cols(); ++j) {
          list ij;
          ij.append(i);
          ij.append(j);
          result[nanobind::tuple(ij)] = src(i, j);
        }
      }
    }

    return result.release();
  }
};

}  // namespace detail
}  // namespace nanobind
