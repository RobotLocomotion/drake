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
bool BSpline::InSupport(const Vector3<double>& x) const {
    return (std::abs(x(0)-position_(0))/h_ < 1.5
         && std::abs(x(1)-position_(1))/h_ < 1.5
         && std::abs(x(2)-position_(2))/h_ < 1.5);
}

double BSpline::EvalBasis(const Vector3<double>& x) const {
    // If the basis is not in the support, we simply return 0.0
    if (!InSupport(x)) {
        return 0.0;
    } else {
        return Eval1DBasis((x(0)-position_(0))/h_)
              *Eval1DBasis((x(1)-position_(1))/h_)
              *Eval1DBasis((x(2)-position_(2))/h_);
    }
}

Vector3<double> BSpline::EvalGradientBasis(const Vector3<double>& x) const {
    if (!InSupport(x)) {
        return Vector3<double>(0.0, 0.0, 0.0);
    } else {
        double scale = 1.0/h_;
        Vector3<double> coordinate = Vector3<double>(scale*(x(0)-position_(0)),
                                                     scale*(x(1)-position_(1)),
                                                     scale*(x(2)-position_(2)));
        Vector3<double> basis_val = Vector3<double>(Eval1DBasis(coordinate(0)),
                                                    Eval1DBasis(coordinate(1)),
                                                    Eval1DBasis(coordinate(2)));
        return Vector3<double>(scale*(EvalGradient1DBasis(coordinate(0)))
                              *basis_val(1)*basis_val(2),
                               scale*(EvalGradient1DBasis(coordinate(1)))
                              *basis_val(0)*basis_val(2),
                               scale*(EvalGradient1DBasis(coordinate(2)))
                              *basis_val(0)*basis_val(1));
    }
}

std::pair<double, Vector3<double>>
    BSpline::EvalBasisAndGradient(const Vector3<double>& x) const {
    if (!InSupport(x)) {
        return std::pair<double, Vector3<double>>(0.0,
                                                Vector3<double>(0.0, 0.0, 0.0));
    } else {
        double scale = 1.0/h_;
        Vector3<double> coordinate = Vector3<double>(scale*(x(0)-position_(0)),
                                                     scale*(x(1)-position_(1)),
                                                     scale*(x(2)-position_(2)));
        Vector3<double> basis_val = Vector3<double>(Eval1DBasis(coordinate(0)),
                                                    Eval1DBasis(coordinate(1)),
                                                    Eval1DBasis(coordinate(2)));
        return std::pair<double, Vector3<double>>(
              basis_val(0)*basis_val(1)*basis_val(2),
              Vector3<double>(scale*(EvalGradient1DBasis(coordinate(0)))
                             *basis_val(1)*basis_val(2),
                              scale*(EvalGradient1DBasis(coordinate(1)))
                             *basis_val(0)*basis_val(2),
                              scale*(EvalGradient1DBasis(coordinate(2)))
                             *basis_val(0)*basis_val(1)));
    }
}

double BSpline::get_h() const {
    return h_;
}

Vector3<double> BSpline::get_position() const {
    return position_;
}

double BSpline::Eval1DBasis(double r) const {
    double r_abs = std::abs(r);
    if (r_abs >= 1.5) {
        return 0.0;
    } else if (r_abs < 1.5 && r_abs >= 0.5) {
        return .5*(1.5-r_abs)*(1.5-r_abs);
    } else {
        return 0.75-r_abs*r_abs;
    }
}

double BSpline::EvalGradient1DBasis(double r) const {
    if (r <= 0.5 && r >= -0.5) {
        return -2.0*r;
    } else if (r >= 0.5 && r < 1.5) {
        return -1.5 + r;
    } else if (r <= -0.5 && r > -1.5) {
        return 1.5 + r;
    } else {
        return 0.0;
    }
}

}  // namespace mpm
}  // namespace multibody
}  // namespace drake
