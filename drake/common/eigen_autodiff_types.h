#pragma once

/// @file
/// This file contains abbreviated definitions for certain uses of
/// AutoDiffScalar that are commonly used in Drake.
/// @see also eigen_types.h

#include <type_traits>

#include <Eigen/Dense>
#include <unsupported/Eigen/AutoDiff>

#include "drake/common/eigen_types.h"

namespace drake {

/// An autodiff variable with a dynamic number of partials, up to 73 maximum.
using AutoDiffUpTo73d = Eigen::AutoDiffScalar<VectorUpTo73d>;

/// An autodiff variable with a dynamic number of partials.
using AutoDiffXd = Eigen::AutoDiffScalar<Eigen::VectorXd>;

// todo: recursive template to get arbitrary gradient order

// note: tried using template default values (e.g. Eigen::Dynamic), but they
// didn't seem to work on my mac clang
template <int num_vars>
using TaylorVard = Eigen::AutoDiffScalar<Eigen::Matrix<double, num_vars, 1> >;
template <int num_vars, int rows>
using TaylorVecd = Eigen::Matrix<TaylorVard<num_vars>, rows, 1>;
template <int num_vars, int rows, int cols>
using TaylorMatd = Eigen::Matrix<TaylorVard<num_vars>, rows, cols>;

typedef TaylorVard<Eigen::Dynamic> TaylorVarXd;
typedef TaylorVecd<Eigen::Dynamic, Eigen::Dynamic> TaylorVecXd;
typedef TaylorMatd<Eigen::Dynamic, Eigen::Dynamic, Eigen::Dynamic> TaylorMatXd;

static_assert(std::is_same<AutoDiffXd, TaylorVarXd>::value,
              "AutoDiffXd and TaylorVarXd should be two different names "
              "for the same type.");

}  // namespace drake
