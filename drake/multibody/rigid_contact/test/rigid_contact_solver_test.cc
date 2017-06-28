#include "drake/multibody/rigid_contact/rigid_contact_solver.h"
#include "drake/examples/rod2d/rod2d.h"

#include <memory>

#include <gtest/gtest.h>

#include "drake/common/drake_assert.h"

using drake::systems::ContinuousState;
using drake::systems::Context;
using drake::examples::rod2d::Rod2D;

namespace drake {
namespace multibody {
namespace rigid_contact {
namespace {

class RigidContact2DSolverTest : public ::testing::Test {
 protected:
  void SetUp() override {
    rod_ = std::make_unique<Rod2D<double>>(
        Rod2D<double>::SimulationType::kPiecewiseDAE, 0);
    context_ = rod_->CreateDefaultContext();
  }

  Vector3<double> get_rod_config() const {
    return context_->get_state().
        get_continuous_state()->get_generalized_position().CopyToVector();
  }

  Vector3<double> get_rod_velocity() const {
    return context_->get_state().
        get_continuous_state()->get_generalized_velocity().CopyToVector();
  }

  // Construct the inertia solve function.
  std::function<MatrixX<double>(const MatrixX<double>&)> get_inertia_solve() {
    return [this](const MatrixX<double>& B) {
      return this->solve_inertia(*this->rod_, B);
    };
  }

  // Sets the rod to a horizontal, resting configuration.
  void SetRestingHorizontal() {
    ContinuousState<double>& xc = *context_->
        get_mutable_continuous_state();
    for (int i = 0; i < xc.size(); ++i)
      xc[i] = 0.0;
  }

  // Gets the gravitational force on the rod.
  MatrixX<double> GetRodGravitationalForce() const {
    const double mg = rod_->get_rod_mass() *
        rod_->get_gravitational_acceleration();
    MatrixX<double> f(3, 1);
    f << 0, mg, 0;
    return f;
  }

  // Initializes the contact data for the rod *based purely upon its current
  // continuous state*.
  void SetProblemData(RigidContactAccelProblemData<double>* data, double mu) {
    const int ngc = 3;

    // Set the inertia solver.
    data->solve_inertia = get_inertia_solve();

    // There is always one normal and one tangent direction.
    const Vector2<double> contact_normal(0.0, 1.0);
    const Vector2<double> contact_tan(1.0, 0.0);

    // Get the set of contact points.
    const std::vector<Vector2<double>> points = GetContacts();
    const int nc = points.size();

    // Get the set of tangent velocities.
    const double sliding_vel_tol = 100 * std::numeric_limits<double>::epsilon();
    const std::vector<double> vels = GetContactPointsTangentVelocities(points);

    // Set sliding and non-sliding contacts.
    for (int i = 0; i < nc; ++i) {
      if (std::fabs(vels[i]) < sliding_vel_tol) {
        data->non_sliding_contacts.push_back(i);
      } else {
        data->sliding_contacts.push_back(i);
      }
    }

    // Designate sliding and non-sliding contacts.
    const int num_sliding = data->sliding_contacts.size();
    const int num_non_sliding = data->non_sliding_contacts.size();
    EXPECT_EQ(num_sliding + num_non_sliding, nc);

    // Set sliding and non-sliding friction coefficients.
    data->mu_sliding.setOnes(num_sliding) *= mu;
    data->mu_non_sliding.setOnes(num_non_sliding) *= mu;

    // Set spanning friction cone directions (set to unity, because rod is 2D).
    data->r.resize(num_non_sliding);
    for (int i = 0; i < num_non_sliding; ++i)
      data->r[i] = 1;

    // Form the normal contact Jacobian (N).
    data->N.resize(nc, ngc);
    for (int i = 0; i < nc; ++i)
      data->N.row(i) =  GetJacobianRow(points[i], contact_normal);

    // Form Ndot (time derivative of N) and compute Ndot * v.
    MatrixX<double> Ndot(nc, ngc);
    for (int i = 0; i < nc; ++i)
      Ndot.row(i) =  GetJacobianDotRow(points[i], contact_normal);
    const VectorX<double> v = context_->get_state().get_continuous_state()->
        get_generalized_velocity().CopyToVector();
    EXPECT_EQ(v.size(), ngc);
    data->Ndot_x_v = Ndot * v;

    // Form the tangent directions contact Jacobian (F), its time derivative
    // (Fdot), and compute Fdot * v.
    const int nr = std::accumulate(data->r.begin(), data->r.end(), 0);
    EXPECT_EQ(nr, num_non_sliding);
    data->F.resize(nr, ngc);
    MatrixX<double> Fdot(nr, ngc);
    for (int i = 0, j = 0; i < nc; ++i) {
      if (std::binary_search(data->sliding_contacts.begin(),
                             data->sliding_contacts.end(), i))
        continue;
      data->F.row(j) = GetJacobianRow(points[i], contact_tan);
      Fdot.row(j) = GetJacobianDotRow(points[i], contact_tan);
      ++j;
    }
    data->Fdot_x_v = Fdot * v;

    // Form N - mu*Q (Q is sliding contact direction Jacobian).
    data->N_minus_mu_Q = data->N;
    VectorX<double> Qrow;
    for (int i = 0, j = 0; i < nc; ++i) {
      if (std::binary_search(data->non_sliding_contacts.begin(),
                             data->non_sliding_contacts.end(), i))
        continue;
      if (vels[i] > 0) {
        Qrow = GetJacobianRow(points[i], contact_tan);
      } else {
        Qrow = GetJacobianRow(points[i], -contact_tan);
      }
      data->N.row(i) -= data->mu_sliding[j] * Qrow;
      ++j;
    }

    // Set external force vector.
    data->f = GetRodGravitationalForce();
  }

  double cfm_{1e-8};   // Regularization parameter.
  RigidContactSolver<double> solver_;
  std::unique_ptr<Rod2D<double>> rod_;
  std::unique_ptr<Context<double>> context_;
  RigidContactAccelProblemData<double> data_;

 private:
  // Gets the point(s) of contact for the 2D rod.
  std::vector<Vector2<double>> GetContacts() {
    std::vector<Vector2<double>> points;

    // Get the rod configuration.
    const Vector3<double> q = get_rod_config();
    const double x = q[0], y = q[1], cth = std::cos(q[2]), sth = std::sin(q[2]);

    // Get the two rod endpoint locations.
    const double half_len = rod_->get_rod_half_length();
    Vector2<double> pa = rod_->CalcRodEndpoint(x, y, -1, cth, sth, half_len);
    Vector2<double> pb = rod_->CalcRodEndpoint(x, y, +1, cth, sth, half_len);

    // If an endpoint touches the ground, add it as a contact point.
    if (pa[1] <= 0)
      points.push_back(pa);
    if (pb[1] <= 0)
      points.push_back(pb);

    return points;
  }

  // Gets the tangent velocities for all contact points.
  std::vector<double> GetContactPointsTangentVelocities(
      const std::vector<Vector2<double>>& points) {
    const Vector3<double> q = get_rod_config();
    const Vector3<double> v = get_rod_velocity();

    // Get necessary quantities.
    const Vector2<double> p_WRo = q.segment(0, 2);
    const Vector2<double> v_WRo = v.segment(0, 2);
    const double w_WR = q[2];

    std::vector<double> vels(points.size());
    for (size_t i = 0; i < points.size(); ++i) {
      vels[i] = rod_->CalcCoincidentRodPointVelocity(p_WRo, v_WRo, w_WR,
                                                     points[i])[0];
    }

    return vels;
  }

};

// Tests the rod in a ballistic configuration (non-contacting) configuration.
TEST_F(RigidContact2DSolverTest, NonContacting) {
  // Set the state of the rod to reseting on its side with no velocity.
  SetRestingHorizontal();

  // Set the vertical position to strictly positive.
  ContinuousState<double>& xc = *context_->
      get_mutable_continuous_state();
  xc[1] = 1.0;

  // Compute the rigid contact data.
  SetProblemData(&data_, 1.0 /* Coulomb friction coefficient */);

  // Verify there are no contacts.
  EXPECT_TRUE(data_.sliding_contacts.empty());
  EXPECT_TRUE(data_.non_sliding_contacts.empty());

  // Compute the contact forces.
  VectorX<double> cf;
  solver_.SolveContactProblem(cfm_, data_, &cf);

  // Verify that the contact forces are zero.
  EXPECT_LT(cf.norm(), cfm_);
}

// Tests the rod in a two-point non-sliding configuration.
TEST_F(RigidContact2DSolverTest, TwoPointNonSliding) {
  // Set the state of the rod to reseting on its side with no velocity.
  SetRestingHorizontal();

  // Compute the rigid contact data.
  SetProblemData(&data_, 1.0 /* Coulomb friction coefficient */);

  // Compute the contact forces.
  VectorX<double> cf;
  solver_.SolveContactProblem(cfm_, data_, &cf);

  // Verify that there are no frictional forces.
  EXPECT_TRUE(data_.sliding_contacts.empty());
  const int nc = data_.non_sliding_contacts.size();
  EXPECT_LT(cf.segment(nc, cf.size() - nc).norm(), 10 * cfm_);

  // Verify that the normal contact forces exactly oppose gravity (there should
  // be no frictional forces).
  const double mg = GetRodGravitationalForce().norm();
  EXPECT_NEAR(cf.segment(0, nc).lpNorm<1>(), mg, 10 * cfm_);
}

// Tests the rod in a two-point sliding configuration.
TEST_F(RigidContact2DSolverTest, TwoPointSliding) {
  // Set the state of the rod to reseting on its side with horizontal velocity.
  SetRestingHorizontal();
  ContinuousState<double>& xc = *context_->
      get_mutable_continuous_state();
  xc[3] = 1.0;

  // Compute the rigid contact data.
  SetProblemData(&data_, 0.0 /* frictionless contact */);

  // Compute the contact forces.
  VectorX<double> cf;
  solver_.SolveContactProblem(cfm_, data_, &cf);

  // Verify that there are no frictional forces.
  EXPECT_TRUE(data_.non_sliding_contacts.empty());
  const int nc = data_.sliding_contacts.size();
  EXPECT_LT(cf.segment(nc, cf.size() - nc).norm(), 10 * cfm_);

  // Verify that the normal contact forces exactly oppose gravity (there should
  // be no frictional forces).
  const double mg = GetRodGravitationalForce().norm();
  EXPECT_NEAR(cf.segment(0, nc).lpNorm<1>(), mg, 10 * cfm_);
}

}  // namespace
}  // namespace rigid_contact
}  // namespace multibody
}  // namespace drake
