#pragma once

#include <cmath>

#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace systems {
namespace analysis {
namespace test {

using Vector2d = Vector2<double>;

class PleidesSystem : public LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(PleidesSystem)
  PleidesSystem() {
    this->DeclareContinuousState(14 /* nq */, 14 /* nv */, 0 /* nz */);

    // Set masses.
    for (int i = 0; i < num_particles(); ++i) m_.push_back(i + 1);
  }

  static int num_particles() { return 7; }

  void SetDefaultState(
      const Context<double>& context, State<double>* state) const override {
    VectorX<double> q(num_particles() * 2), v(num_particles() * 2);

    // Set the initial positions.
    q.head(num_particles())[0] =  3.0;   q.tail(num_particles())[0] =  3.0;
    q.head(num_particles())[1] =  3.0;   q.tail(num_particles())[1] = -3.0;
    q.head(num_particles())[2] = -1.0;   q.tail(num_particles())[2] =  2.0;
    q.head(num_particles())[3] = -3.0;   q.tail(num_particles())[3] =  0.0;
    q.head(num_particles())[4] =  2.0;   q.tail(num_particles())[4] =  0.0;
    q.head(num_particles())[5] = -2.0;   q.tail(num_particles())[5] = -4.0;
    q.head(num_particles())[6] =  2.0;   q.tail(num_particles())[6] =  4.0;

    // Set the initial velocities.
    v.head(num_particles())[0] =  0.0;   v.tail(num_particles())[0] =  0.0;
    v.head(num_particles())[1] =  0.0;   v.tail(num_particles())[1] =  0.0;
    v.head(num_particles())[2] =  0.0;   v.tail(num_particles())[2] =  0.0;
    v.head(num_particles())[3] =  0.0;   v.tail(num_particles())[3] = -1.25;
    v.head(num_particles())[4] =  0.0;   v.tail(num_particles())[4] =  1.0;
    v.head(num_particles())[5] =  1.75;  v.tail(num_particles())[5] =  0.0;
    v.head(num_particles())[6] = -1.5;   v.tail(num_particles())[6] =  0.0;

    state->get_mutable_continuous_state().
        get_mutable_generalized_position().SetFromVector(q);
    state->get_mutable_continuous_state().
        get_mutable_generalized_velocity().SetFromVector(v);
  }

  void DoCalcTimeDerivatives(const Context<double>& context,
                             ContinuousState<double>* deriv) const override {
    const VectorBase<double>& q = context.get_continuous_state().
        get_generalized_position();
    const VectorBase<double>& v = context.get_continuous_state().
        get_generalized_velocity();

    // Get the positions of each particle.
    const VectorX<double> x = q.CopyToVector().head(num_particles());
    const VectorX<double> y = q.CopyToVector().tail(num_particles());

    // Set the derivatives of the positions.
    deriv->get_mutable_generalized_position().SetFrom(v);

    // Set the derivatives of the velocities.
    VectorX<double> vdot(v.size());
    for (int i = 0; i < num_particles(); ++i) {
      Vector2d Fi(0.0, 0.0);

      // Accumulate the forces due to gravitational interaction with every other
      // particle.
      for (int j = 0; j < num_particles(); ++j) {
        if (i == j) continue;
        const Vector2d dij(x[j] - x[i], y[j] - y[i]);
        const double distance = dij.norm();
        Fi += g() * (m_[i] * m_[j]) * dij / (distance * distance * distance);
      }

      vdot.head(num_particles())[i] = Fi[0] / m_[i];
      vdot.tail(num_particles())[i] = Fi[1] / m_[i];
      deriv->get_mutable_generalized_velocity().SetFromVector(vdot);
    }
  }

  // Gets the end time for integration.
  double get_end_time() const { return 3.0; }

  // The gravitational constant.
  double g() const { return 1.0; }

  // Gets the system solution *for the positions only*. Only valid for
  // time=3.0.
  static VectorX<double> GetSolution(double t) {
    DRAKE_DEMAND(t == 3.0);
    VectorX<double> sol(num_particles() * 2);
    sol(0) =   0.3706139143970502;
    sol(1) =   0.3237284092057233 * 10.0;
    sol(2) =  -0.3222559032418324 * 10.0;
    sol(3) =   0.6597091455775310;
    sol(4) =   0.3425581707156584;
    sol(5) =   0.1562172101400631 * 10.0;
    sol(6) =  -0.7003092922212495;
    sol(7) =  -0.3943437585517392 * 10.0;
    sol(8) =  -0.3271380973972550 * 10.0;
    sol(9) =   0.5225081843456543 * 10.0;
    sol(10) = -0.2590612434977470 * 10.0;
    sol(11) =  0.1198213693392275 * 10.0;
    sol(12) = -0.2429682344935824;
    sol(13) =  0.1091449240428980 * 10.0;
    return sol;
  }

 private:
  std::vector<double> m_;
};

}  // namespace test
}  // namespace analysis
}  // namespace systems
}  // namespace drake
