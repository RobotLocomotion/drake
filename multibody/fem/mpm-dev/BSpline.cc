#include "drake/multibody/fem/mpm-dev/BSpline.h"

namespace drake {
namespace multibody {
namespace mpm {

BSpline::BSpline(): h_(1.0), position_(0.0, 0.0, 0.0) {}

BSpline::BSpline(const double h, const Vector3<double>& pos) {
    DRAKE_DEMAND(h > 0.0);
    h_        = h;
    position_ = pos;
}

// TODO(yiminlin.tri): For below routines, After determining the implementations
// of particles and grid points, we can pass these objects as arguments for
// convinience.
bool BSpline::InSupport(const Vector3<double>& x) {
    return (std::abs(x(0)-position_(0))/h_ < 1.5
         && std::abs(x(1)-position_(1))/h_ < 1.5
         && std::abs(x(2)-position_(2))/h_ < 1.5);
}

double BSpline::EvalBasis(const Vector3<double>& x) {
    // If the basis is not in the support, we simply return 0.0
    if (!InSupport(x)) {
        return 0.0;
    } else {
        return Eval1DBasis((x(0)-position_(0))/h_)
              *Eval1DBasis((x(1)-position_(1))/h_)
              *Eval1DBasis((x(2)-position_(2))/h_);
    }
}

double BSpline::get_h() const {
    return h_;
}

Vector3<double> BSpline::get_position() const {
    return position_;
}

// TODO(yiminlin.tri): Potential speedup in the arithmetic, is std::pow
// efficient in this case?
double BSpline::Eval1DBasis(double r) {
    double r_abs    = std::abs(r);
    if (r_abs >= 1.5) {
        return 0.0;
    } else if (r_abs < 1.5 && r_abs >= 0.5) {
        return .5*(1.5-r_abs)*(1.5-r_abs);
    } else {
        return 0.75-r_abs*r_abs;
    }
}


}  // namespace mpm
}  // namespace multibody
}  // namespace drake
