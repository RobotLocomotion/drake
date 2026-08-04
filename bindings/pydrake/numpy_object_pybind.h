#pragma once

#include "drake_nanobind/eigen/dense.h"

#ifndef PYDRAKE_USE_NANOBIND
#error "Should only be used when the binder is nanobind!"
#endif

/* @file
This file provides the PYDRAKE_NUMPY_OBJECT_DTYPE macro. */

namespace nanobind {
namespace detail {

/* Base class for PYDRAKE_NUMPY_OBJECT_DTYPE casters.
@tparam T is the Eigen matrix type (e.g., MatrixX<AutoDiffXd>)*/
template <typename T>
struct pydrake_numpy_dtype_object_type_caster {
  using Scalar = typename T::Scalar;
  using PlainScalar = std::remove_cv_t<Scalar>;
  using PlainScalarCaster = make_caster<PlainScalar>;
  static constexpr bool kCompileTime1D =
      (T::RowsAtCompileTime == 1 || T::ColsAtCompileTime == 1);
  static constexpr int kCompileTime1DShape =
      (T::RowsAtCompileTime == 1)   ? T::ColsAtCompileTime
      : (T::ColsAtCompileTime == 1) ? T::RowsAtCompileTime
                                    : Eigen::Dynamic;

  NB_TYPE_CASTER(
      T, const_name("numpy.ndarray[") +
             concat_maybe(const_name("dtype=") + PlainScalarCaster::Name,
                 const_name<kCompileTime1D>(shape<kCompileTime1DShape>::name,
                     shape<T::RowsAtCompileTime, T::ColsAtCompileTime>::name),
                 dtype_const_name<Scalar>::name) +
             const_name("]"))

  bool from_python(
      handle src, uint8_t flags, cleanup_list* /* cleanup */) noexcept {
    auto numpy = module_::import_("numpy");

    if (src.is_none()) {
      return false;
    }

    // Avoid converting np.array(dtype=np.float64) to AutoDiffXd prematurely.
    // Only accept autodiff conversions as a last resort.
    const bool convert = flags & (uint8_t)cast_flags::convert;
    if (!convert) {
      return false;
    }

    // TODO(jwnimmer-tri) This implementation is probably terribly inefficient.
    // Using the NumPy C API (like pybind11 does) would probably be faster.

    auto array = numpy.attr("asarray")(src, arg("dtype") = "object");
    auto shape = cast<nanobind::tuple>(array.attr("shape"));

    int rows{};
    int cols{};
    bool np_1d{};
    if (shape.size() == 1) {
      if constexpr (T::ColsAtCompileTime == 1) {
        rows = cast<int>(shape[0]);
        cols = 1;
        np_1d = true;
      } else if constexpr (T::RowsAtCompileTime == 1) {
        rows = 1;
        cols = cast<int>(shape[0]);
        np_1d = true;
      } else {
        // Promote from 1d array to 2d array (as column vector).
        array = array.attr("reshape")(-1, 1);
        rows = cast<int>(shape[0]);
        cols = 1;
        np_1d = false;
      }
    } else if (shape.size() == 2) {
      rows = cast<int>(shape[0]);
      cols = cast<int>(shape[1]);
      np_1d = false;
    } else {
      return false;
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
      for (Eigen::Index i = 0; i < rows; ++i) {
        for (Eigen::Index j = 0; j < cols; ++j) {
          value(i, j) = cast<PlainScalar>(
              array[np_1d ? nanobind::object(nanobind::int_(i + j))
                          : nanobind::object(nanobind::make_tuple(i, j))]);
        }
      }
    } catch (const cast_error&) {
      return false;
    }

    return true;
  }

  static handle from_cpp(const T& src, rv_policy /* policy */,
      cleanup_list* /* cleanup */) noexcept {
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
          result[nanobind::make_tuple(i, j)] = src(i, j);
        }
      }
    }

    return result.release();
  }
};

}  // namespace detail
}  // namespace nanobind

/* The PYDRAKE_NUMPY_OBJECT_DTYPE macro defines a type_caster<> specialization
between C++ Eigen matrices and Python NumPy arrays in cases where array's
dtype=object (i.e., the Eigen::Scalar type is not a primitive type).

@tparam Type the Eigen::Scalar type to declare casters for (e.g., AutoDiffXd).

The first type_caster partial template specialization handles plain (dense)
matrices and expression trees that can be evaluated to same.

The second and third specializations handle Eigen:Ref and Eigen::Map,
respectively.

TODO(#24749) The Ref and Map casters should disallow mutable matrices. */
#define PYDRAKE_NUMPY_OBJECT_DTYPE(Type)                                      \
  namespace nanobind {                                                        \
  namespace detail {                                                          \
  template <typename MatrixType>                                              \
  struct type_caster<MatrixType,                                              \
      enable_if_t<                                                            \
          (is_eigen_plain_v<MatrixType> || is_eigen_xpr_v<MatrixType>) &&     \
          std::is_same_v<std::remove_cv_t<typename MatrixType::Scalar>,       \
              Type>>>                                                         \
      : public pydrake_numpy_dtype_object_type_caster<MatrixType> {};         \
  template <typename PlainObjectType, int Options, typename StrideType>       \
  struct type_caster<Eigen::Ref<PlainObjectType, Options, StrideType>,        \
      enable_if_t<std::is_same_v<                                             \
          std::remove_cv_t<typename PlainObjectType::Scalar>, Type>>>         \
      : public pydrake_numpy_dtype_object_type_caster<                        \
            Eigen::Matrix<typename PlainObjectType::Scalar,                   \
                PlainObjectType::RowsAtCompileTime,                           \
                PlainObjectType::ColsAtCompileTime,                           \
                (PlainObjectType::RowsAtCompileTime == 1 &&                   \
                    PlainObjectType::ColsAtCompileTime != 1)                  \
                    ? Eigen::RowMajor                                         \
                    : 0,                                                      \
                PlainObjectType::MaxRowsAtCompileTime,                        \
                PlainObjectType::MaxColsAtCompileTime>> {                     \
    using _matrix = Eigen::Matrix<typename PlainObjectType::Scalar,           \
        PlainObjectType::RowsAtCompileTime,                                   \
        PlainObjectType::ColsAtCompileTime,                                   \
        (PlainObjectType::RowsAtCompileTime == 1 &&                           \
            PlainObjectType::ColsAtCompileTime != 1)                          \
            ? Eigen::RowMajor                                                 \
            : 0,                                                              \
        PlainObjectType::MaxRowsAtCompileTime,                                \
        PlainObjectType::MaxColsAtCompileTime>;                               \
    template <typename T>                                                     \
    using Cast = _matrix;                                                     \
    explicit operator _matrix() const { return this->value; }                 \
  };                                                                          \
  template <typename MatrixType, int MapOptions, typename StrideType>         \
  struct type_caster<Eigen::Map<MatrixType, MapOptions, StrideType>,          \
      enable_if_t<std::is_same_v<                                             \
          std::remove_cv_t<typename MatrixType::Scalar>, Type>>>              \
      : public pydrake_numpy_dtype_object_type_caster<                        \
            Eigen::Matrix<typename MatrixType::Scalar,                        \
                MatrixType::RowsAtCompileTime, MatrixType::ColsAtCompileTime, \
                (MatrixType::RowsAtCompileTime == 1 &&                        \
                    MatrixType::ColsAtCompileTime != 1)                       \
                    ? Eigen::RowMajor                                         \
                    : 0,                                                      \
                MatrixType::MaxRowsAtCompileTime,                             \
                MatrixType::MaxColsAtCompileTime>> {                          \
    using _matrix = Eigen::Matrix<typename MatrixType::Scalar,                \
        MatrixType::RowsAtCompileTime, MatrixType::ColsAtCompileTime,         \
        (MatrixType::RowsAtCompileTime == 1 &&                                \
            MatrixType::ColsAtCompileTime != 1)                               \
            ? Eigen::RowMajor                                                 \
            : 0,                                                              \
        MatrixType::MaxRowsAtCompileTime, MatrixType::MaxColsAtCompileTime>;  \
    template <typename T>                                                     \
    using Cast = _matrix;                                                     \
    explicit operator _matrix() const { return this->value; }                 \
  };                                                                          \
  } /* namespace detail */                                                    \
  } /* namespace nanobind */
