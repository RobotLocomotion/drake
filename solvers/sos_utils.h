#pragma once

#include <vector>

#include <Eigen/Core>

#include "drake/common/symbolic.h"

typedef Eigen::MatrixXi Exponent;
typedef Eigen::MatrixXi ExponentList;
typedef Eigen::Matrix<drake::symbolic::Monomial, Eigen::Dynamic, 1>
        MonomialVector;

ExponentList ConstructMonomialBasis(const ExponentList & M);
MonomialVector ConstructMonomialBasis(const drake::symbolic::Polynomial & p);

