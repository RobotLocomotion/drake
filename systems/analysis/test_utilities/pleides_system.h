#pragma once

#include <cmath>
#include <vector>

#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace systems {
namespace analysis {
namespace test {

using Vector2d = Vector2<double>;

/// A system of ODEs that can be used to test performance of the initial value
/// problem (IVP) solvers. This problem corresponds to the "n-body-problem"
/// (where n = 7); in short, given initial positions and velocities of seven
/// particles that move according to Newtonian Mechanics (F=ma) and subject to
/// inverse-square gravitational forces, compute their positions and velocities
/// at a given time in the future. One potentially useful aspect of this problem
/// is that it is energy conserving. The problem setup and data are taken from:
/// https://archimede.dm.uniba.it/~testset/report/plei.pdf, which is part of the
/// IVP benchmark suite described in:
///
/// F.Mazzia and C.Magherini. Test Set for Initial Value Problem Solvers,
/// release 2.4. Department of Mathematics, University of Bari and INdAM,
/// Research Unit of Bari, February 2008. http://www.dm.uniba.it/~testset.
class PleidesSystem : public LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(PleidesSystem)
  PleidesSystem() {
    const int nq = 14;  // Number of generalized positions.
    const int nv = 14;  // Number of generalized velocities.
    const int nz = 0;   // No additional state variables.

    this->DeclareContinuousState(nq, nv, nz);

    // Set masses (as defined in plei.pdf).
    for (int i = 0; i < kNumParticles; ++i) mass_[i] = i + 1;
  }

  // Number of particles being simulated.
  constexpr static int kNumParticles = 7;

  void SetDefaultState(
      const Context<double>& context, State<double>* state) const override {
    VectorX<double> q(kNumParticles * 2), v(kNumParticles * 2);

    // Set the initial positions. Note that these are all representable in
    // IEEE 754 without any representation error. Each pair of initial positions
    // corresponds to the (x, y) location of a particle. From plei.pdf,
    // the first particle's initial position is x0 = q[0] = 3, y0 = q[7] = 3.
    q.head(kNumParticles)[0] =  3.0;   q.tail(kNumParticles)[0] =  3.0;
    q.head(kNumParticles)[1] =  3.0;   q.tail(kNumParticles)[1] = -3.0;
    q.head(kNumParticles)[2] = -1.0;   q.tail(kNumParticles)[2] =  2.0;
    q.head(kNumParticles)[3] = -3.0;   q.tail(kNumParticles)[3] =  0.0;
    q.head(kNumParticles)[4] =  2.0;   q.tail(kNumParticles)[4] =  0.0;
    q.head(kNumParticles)[5] = -2.0;   q.tail(kNumParticles)[5] = -4.0;
    q.head(kNumParticles)[6] =  2.0;   q.tail(kNumParticles)[6] =  4.0;

    // Set the initial velocities. Uses the same representation layout as with
    // the positions.
    v.head(kNumParticles)[0] =  0.0;   v.tail(kNumParticles)[0] =  0.0;
    v.head(kNumParticles)[1] =  0.0;   v.tail(kNumParticles)[1] =  0.0;
    v.head(kNumParticles)[2] =  0.0;   v.tail(kNumParticles)[2] =  0.0;
    v.head(kNumParticles)[3] =  0.0;   v.tail(kNumParticles)[3] = -1.25;
    v.head(kNumParticles)[4] =  0.0;   v.tail(kNumParticles)[4] =  1.0;
    v.head(kNumParticles)[5] =  1.75;  v.tail(kNumParticles)[5] =  0.0;
    v.head(kNumParticles)[6] = -1.5;   v.tail(kNumParticles)[6] =  0.0;

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
    const VectorX<double> x = q.CopyToVector().head(kNumParticles);
    const VectorX<double> y = q.CopyToVector().tail(kNumParticles);

    // Set the derivatives of the positions.
    deriv->get_mutable_generalized_position().SetFrom(v);

    // Set the time derivatives of the velocities. The time derivatives are just
    // the (gravitational) forces acting on each particle divided by the mass of
    // each particle.
    VectorX<double> vdot(v.size());
    for (int i = 0; i < kNumParticles; ++i) {
      Vector2d Fi(0.0, 0.0);

      // Accumulate the net forces due to inverse-square law gravitational
      // interaction with every other particle. See (II.6.5) in plei.pdf.
      for (int j = 0; j < kNumParticles; ++j) {
        if (i == j) continue;
        const Vector2d rij(x[j] - x[i], y[j] - y[i]);
        const double distance = rij.norm();
        Fi += G() * (mass_[i] * mass_[j]) * rij /
            (distance * distance * distance);
      }

      vdot.head(kNumParticles)[i] = Fi[0] / mass_[i];
      vdot.tail(kNumParticles)[i] = Fi[1] / mass_[i];
      deriv->get_mutable_generalized_velocity().SetFromVector(vdot);
    }
  }

  // Gets the end time for integration (to be consistent with plei.pdf).
  double get_end_time() const { return 3.0; }

  // Non-physical inverse-square law gravitational constant (defined in
  // plei.pdf).
  double G() const { return 1.0; }

  // Gets the system solution *for the positions only*. Only valid for
  // time=3.0. Solutions are provided to sixteen decimal digits in plei.pdf.
  static VectorX<double> GetSolution(double t) {
    DRAKE_DEMAND(t == 3.0);
    VectorX<double> sol(kNumParticles * 2);
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
  // Mass of each particle.
  double mass_[kNumParticles];
};

}  // namespace test
}  // namespace analysis
}  // namespace systems
}  // namespace drake
