#pragma once

#include "drake/common/eigen_types.h"
#include "drake/multibody/fem/matrix_utilities.h"

namespace drake {
namespace multibody {
namespace mpm {

// A implementation of Fixed Corotated Model (Constitutive Model)
class CorotatedModel {
 public:
    // Constructor uses Young's modulus E and Poisson's ratio nu
    CorotatedModel(double E, double nu);

    // First Piola Kirchhoff stress density: P = dpsi/dF
    void CalcFirstPiolaKirchhoffStress(
        const Matrix3<double>& F, EigenPtr<Matrix3<double>> P);

    // Kirchhoff stress density: tau = P F^T = dpsi/dF F^T
    void CalcKirchhoffStress(const Matrix3<double>& F,
                             EigenPtr<Matrix3<double>> tau);
    void CalcFirstPiolaKirchhoffStressAndKirchhoffStress(
        const Matrix3<double>& F, EigenPtr<Matrix3<double>> P,
        EigenPtr<Matrix3<double>> tau);

 private:
    double mu_;                         // Parameters in defining
    double lambda_;                     // the energy density function
};  // class CorotatedModel

}  // namespace mpm
}  // namespace multibody
}  // namespace drake
