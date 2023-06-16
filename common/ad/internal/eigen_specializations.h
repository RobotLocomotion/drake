#pragma once

#include <limits>

#include <Eigen/Core>

/* This file contains Eigen-related specializations for Drake's AutoDiff
scalar type (plus the intimately related std::numeric_limits specialization).

NOTE: This file should never be included directly, rather only from
auto_diff.h in a very specific order. */

#ifndef DRAKE_DOXYGEN_CXX

namespace std {
template <>
class numeric_limits<drake::ad::AutoDiff> : public numeric_limits<double> {};
}  // namespace std

namespace Eigen {

// === See Eigen/src/Core/NumTraits.h ===

// https://eigen.tuxfamily.org/dox/structEigen_1_1NumTraits.html
template <>
struct NumTraits<drake::ad::AutoDiff> : public NumTraits<double> {
  // Inherit the constants from `double`, but be sure to fatten a few types up
  // to full AutoDiff where necessary.
  using Real = drake::ad::AutoDiff;
  using NonInteger = drake::ad::AutoDiff;
  using Nested = drake::ad::AutoDiff;
  enum { RequireInitialization = 1 };
};

// Computing "ADS [op] double" yields an ADS.
template <typename BinOp>
struct ScalarBinaryOpTraits<drake::ad::AutoDiff, double, BinOp> {
  using ReturnType = drake::ad::AutoDiff;
};

// Computing "double [op] ADS" yields an ADS.
template <typename BinOp>
struct ScalarBinaryOpTraits<double, drake::ad::AutoDiff, BinOp> {
  using ReturnType = drake::ad::AutoDiff;
};

}  // namespace Eigen

#endif  // DRAKE_DOXYGEN_CXX
