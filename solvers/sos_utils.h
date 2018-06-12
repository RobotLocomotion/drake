#pragma once
#include <Eigen/Core>

typedef Eigen::MatrixXi Exponent;
typedef Eigen::MatrixXi ExponentList;
ExponentList ConstructMonomialBasis(const ExponentList & M);
