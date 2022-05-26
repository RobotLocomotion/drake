#pragma once

#include <utility>

#include "drake/common/eigen_types.h"

namespace drake {
namespace multibody {
namespace mpm {

// A implementation of 3D quadratic B spline
class BSpline {
 public:
    BSpline();
    BSpline(const double h, const Vector3<double>& pos);

    // Check whether the input point is inside the support of this Bspline
    bool InSupport(const Vector3<double>& x) const;

    // Evaluation of Bspline basis on a particular position
    double EvalBasis(const Vector3<double>& x) const;
    // TODO(yiminlin.tri): Pass in pointer to avoid allocations
    Vector3<double> EvalGradientBasis(const Vector3<double>& x) const;
    std::pair<double, Vector3<double>>
      EvalBasisAndGradient(const Vector3<double> & x) const;

    // Helper function
    double get_h() const;
    Vector3<double> get_position() const;

 private:
    // Helper function. Evaluate the values and the gradients of 1D quadratic
    // Bspline on the reference 1D domain r, note that the basis has compact
    // support in [-1.5, 1.5]
    double Eval1DBasis(double r) const;
    double EvalGradient1DBasis(double r) const;

    double h_{};                        // The scaling of the reference domain.
                                        // Since the class is for the usage of
                                        // MPM, we assume we are on the uniform
                                        // grid, and the physical domain (x, y,
                                        // z) can be transformed to the
                                        // reference domain (r, s, t) through a
                                        // affine transformation \phi^-1(x, y,
                                        // z) = (r, s, t) = 1/h*(x-xc, y-yc,
                                        // z-zc). The basis on physical domain
                                        // will have support [-1.5h, 1.5h]
    Vector3<double> position_{};        // The "center" of the Bspline, or (xc,
                                        // yc, zc) in above comment
};  // class BSpline

}  // namespace mpm
}  // namespace multibody
}  // namespace drake
