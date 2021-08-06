#include "drake/multibody/constraint/constraint_solver.h"

#include <cmath>
#include <memory>

#include <gtest/gtest.h>

#include "drake/common/drake_assert.h"
#include "drake/examples/rod2d/rod2d.h"
#include "drake/solvers/unrevised_lemke_solver.h"

using drake::systems::ContinuousState;
using drake::systems::Context;
using drake::examples::rod2d::Rod2D;
using drake::systems::BasicVector;
using Vector2d = Eigen::Vector2d;

namespace drake {
namespace multibody {
namespace constraint {
namespace {

class Constraint2DSolverTest : public ::testing::Test {
 protected:
  void SetUp() override {
    rod_ = std::make_unique<Rod2D<double>>(
        Rod2D<double>::SystemType::kPiecewiseDAE, 0);
    context_ = rod_->CreateDefaultContext();

    // Use a non-unit mass.
    rod_->set_rod_mass(2.0);

    // Set a zero input force (this is the default).
    const Vector3<double> ext_input(0.0, 0.0, 0.0);
    rod_->get_input_port(0).FixValue(context_.get(), ext_input);

    // Construct the problem data for the 2D rod.
    const int num_velocities = 3;
    accel_data_ = std::make_unique<ConstraintAccelProblemData<double>>(
      num_velocities);
    vel_data_ = std::make_unique<ConstraintVelProblemData<double>>(
      num_velocities);

    // Set epsilon for floating point tolerance testing; due to rounding error
    // in the LCP solver, this is approximately the tightest tolerance with
    // which the tests will still pass.
    eps_ = 100 * std::numeric_limits<double>::epsilon();
  }

  // Zero tolerance for results depending on LCP solve (< 0 indicates not set).
  double eps_{-1};
  ConstraintSolver<double> solver_;
  std::unique_ptr<Rod2D<double>> rod_;
  std::unique_ptr<Context<double>> context_;
  std::unique_ptr<ConstraintAccelProblemData<double>> accel_data_;
  std::unique_ptr<ConstraintVelProblemData<double>> vel_data_;
  const bool kForceAppliedToLeft = false;
  const bool kForceAppliedToRight = true;
  const bool kSlideLeft = false;
  const bool kSlideRight = true;
  const bool kLinearSystemSolver = false;
  const bool kLCPSolver = true;

  // Gets the frame for a sliding contact.
  Matrix2<double> GetSlidingContactFrameToWorldTransform(
      double xaxis_velocity) const {
    const double dir = (xaxis_velocity > 0) ? 1 : -1;
    const Matrix2<double> R =
        rod_->GetSlidingContactFrameToWorldTransform(xaxis_velocity);
    // Verify that the x-axis of the contact frame, which corresponds to
    // the contact normal, points along the world y-axis, and the y-axis
    // of the contact frame, which corresponds to the contact sliding direction
    // vector, along the x-axis or negative x-axis.
    EXPECT_LT(
        std::fabs(R.col(0).dot(Vector2<double>::UnitY()) - 1.0),
        std::numeric_limits<double>::epsilon());
    EXPECT_LT(
        std::fabs(R.col(1).dot(Vector2<double>::UnitX()) - dir),
        std::numeric_limits<double>::epsilon());
    return R;
  }

  // Gets the frame for a non-sliding contact.
  Matrix2<double> GetNonSlidingContactFrameToWorldTransform() const {
    const Matrix2<double> R = rod_->GetNonSlidingContactFrameToWorldTransform();
    // Verify that the x-axis of the contact frame, which corresponds to
    // the contact normal, points along the world y-axis, and the y-axis
    // of the contact frame, which corresponds to a contact tangent
    // vector, along the world x-axis.
    EXPECT_LT(
        std::fabs(R.col(0).dot(Vector2<double>::UnitY()) - 1.0),
        std::numeric_limits<double>::epsilon());
    EXPECT_LT(
        std::fabs(R.col(1).dot(Vector2<double>::UnitX()) - 1.0),
        std::numeric_limits<double>::epsilon());
    return R;
  }

  // Sets the rod to a resting horizontal configuration without modifying the
  // rod's mode variables.
  void SetRodToRestingHorizontalConfig() {
    ContinuousState<double>& xc = context_->get_mutable_continuous_state();
    // Configuration has the rod on its side.
    xc[0] = 0.0;     // com horizontal position
    xc[1] = 0.0;     // com vertical position
    xc[2] = 0.0;     // rod rotation
    xc[3] = xc[4] = xc[5] = 0.0;   // velocity variables
  }

  // Sets the rod to an upward moving horizontal configuration without modifying
  // the rod's mode variables.
  void SetRodToUpwardMovingHorizontalConfig() {
    ContinuousState<double>& xc = context_->get_mutable_continuous_state();
    // Configuration has the rod on its side.
    xc[0] = 0.0;     // com horizontal position
    xc[1] = 0.0;     // com vertical position
    xc[2] = 0.0;     // rod rotation
    xc[3] = 0.0;     // no horizontal velocity.
    xc[4] = 1.0;     // upward velocity.
    xc[5] = 0.0;     // no angular velocity.
  }

  // Sets the rod to an impacting, sliding velocity with the rod
  // configured to lie upon its side and without modifying the rod's mode
  // variables.
  void SetRodToSlidingImpactingHorizontalConfig(bool sliding_to_right) {
    ContinuousState<double>& xc = context_->get_mutable_continuous_state();
    // Configuration has the rod on its side.
    xc[0] = 0.0;                          // com horizontal position
    xc[1] = 0.0;                          // com vertical position
    xc[2] = 0.0;                          // rod rotation
    xc[3] = (sliding_to_right) ? 1 : -1;  // sliding horizontal velocity.
    xc[4] = -1.0;                         // impacting velocity.
    xc[5] = 0.0;                          // no angular velocity.
  }

  // Sets the rod to a sliding velocity with the rod configured to impact
  // vertically and without modifying the rod's mode variables.
  void SetRodToSlidingImpactingVerticalConfig(bool sliding_to_right) {
    ContinuousState<double>& xc = context_->get_mutable_continuous_state();
    // Configuration has the rod on its side.
    xc[0] = 0.0;                          // com horizontal position
    xc[1] = 0.0;                          // com vertical position
    xc[2] = M_PI_2;                       // rod rotation
    xc[3] = (sliding_to_right) ? 1 : -1;  // sliding horizontal velocity.
    xc[4] = -1.0;                         // impacting velocity.
    xc[5] = 0.0;                          // no angular velocity.
  }

  // Sets the rod to a resting vertical configuration without modifying the
  // mode variables.
  void SetRodToRestingVerticalConfig() {
    ContinuousState<double>& xc = context_->get_mutable_continuous_state();
    xc[0] = 0.0;                             // com horizontal position
    xc[1] = rod_->get_rod_half_length();     // com vertical position
    xc[2] = M_PI_2;                          // rod rotation
    xc[3] = xc[4] = xc[5] = 0.0;             // velocity variables
  }

  // Computes rigid contact data, duplicating contact points and friction
  // directions, as specified.
  // @param contact_points_dup the number of times (>= 0) each contact point
  //        should be duplicated in the contact data.
  // @param friction_directions_dup the number of times (>= 0) that each
  //        friction basis direction should be duplicated. Since the 2D tests
  //        only use one friction direction, duplication ensures that the
  //        algorithms are able to handle duplicated directions without error.
  void CalcConstraintAccelProblemData(
      ConstraintAccelProblemData<double>* data,
      int contact_points_dup,
      int friction_directions_dup) {
    DRAKE_DEMAND(contact_points_dup >= 0);
    DRAKE_DEMAND(friction_directions_dup >= 0);

    // Reset constraint acceleration data.
    const int num_velocities = 3;
    *data = ConstraintAccelProblemData<double>(num_velocities);

    // Get the points of contact from Rod2D.
    std::vector<Vector2d> contacts;
    std::vector<double> tangent_vels;
    rod_->GetContactPoints(*context_, &contacts);

    // Duplicate the contact points.
    std::vector<Vector2d> contacts_dup;
    for (int i = 0; i < static_cast<int>(contacts.size()); ++i) {
      for (int j = 0; j < contact_points_dup; ++j)
        contacts_dup.push_back(contacts[i]);
    }
    contacts.insert(contacts.end(), contacts_dup.begin(), contacts_dup.end());

    // Get the contact tangent velocities.
    rod_->GetContactPointsTangentVelocities(*context_, contacts, &tangent_vels);

    // Compute the problem data.
    rod_->CalcConstraintProblemData(*context_, contacts, tangent_vels, data);

    // Get old F.
    const int num_fdir = std::accumulate(data->r.begin(), data->r.end(), 0);
    const int ngc = get_rod_num_coordinates();
    MatrixX<double> Fold = MatrixX<double>::Zero(num_fdir, ngc);
    for (int j = 0; j < ngc; ++j)
      Fold.col(j) = data->F_mult(VectorX<double>::Unit(ngc, j));

    // Determine new F by duplicating each row the specified number of times.
    const int new_friction_directions = friction_directions_dup + 1;
    MatrixX<double> F(new_friction_directions *
                      data->non_sliding_contacts.size(), ngc);
    for (int i = 0, k = 0; i < Fold.rows(); ++i)
      for (int j = 0; j < new_friction_directions; ++j)
        F.row(k++) = Fold.row(i);

    // Redetermine F_mult and F_transpose_mult operator lambdas.
    data->F_mult = [F](const VectorX<double>& w) -> VectorX<double> {
      return F * w;
    };
    data->F_transpose_mult = [F](const VectorX<double>& w) -> VectorX<double> {
      return F.transpose() * w;
    };

    // Update r with the new friction directions.
    for (int i = 0; i < static_cast<int>(data->r.size()); ++i)
      data->r[i] = new_friction_directions;

    // Resize kF (recall the vector always is zero for this 2D problem), gammaF,
    // and gammaE.
    data->kF.setZero(data->non_sliding_contacts.size() *
                               new_friction_directions);

    // Add in empty rows to G, by default, allowing us to verify that no
    // constraint forces are added (and that solution method is robust to
    // unnecessary constraints).
    data->G_mult = [](const VectorX<double>& w) -> VectorX<double> {
      return VectorX<double>::Zero(1);
    };
    data->G_transpose_mult = [ngc](const VectorX<double>& w)
        -> VectorX<double> {
      return VectorX<double>::Zero(ngc);
    };
    data->kG.setZero(0);

    // Check the consistency of the data.
    CheckProblemConsistency(*data, contacts.size());
  }

  // Computes velocity data for solving acceleration problems. This allows us
  // to compare the first-order discretized solutions (with rigid contact) and
  // force/velocity constraints against the force/acceleration solutions using
  // force/acceleration constraints.
  // @param contact_points_dup the number of times (>= 0) each contact point
  //        should be duplicated in the contact data.
  // @param friction_directions_dup the number of times (>= 0) that each
  //        friction basis direction should be duplicated. Since the 2D tests
  //        only use one friction direction, duplication ensures that the
  //        algorithms are able to handle duplicated directions without error.
  void CalcDiscreteTimeProblemData(
      double dt,
      ConstraintVelProblemData<double>* data,
      int contact_points_dup,
      int friction_directions_dup) {
    // Use the constraint velocity data to do most of the work.
    CalcConstraintProblemDataForImpact(data, contact_points_dup,
                                       friction_directions_dup);

    // Set the normal compliance and damping to yield truly rigid contact.
    const double stiffness = std::numeric_limits<double>::infinity();
    const double damping = 0.0;

    // Compute the softening coefficients.
    const double denom = dt * stiffness + damping;
    const double cfm = 1.0 / denom;
    data->gammaN.setOnes() *= cfm;

    // Add in gravitational forces.
    const double grav_accel = rod_->get_gravitational_acceleration();
    data->Mv += Vector3<double>(0, grav_accel * rod_->get_rod_mass(), 0) * dt;
  }

  void SolveDiscretizationProblem(
      const ConstraintVelProblemData<double>& problem_data, double dt,
      VectorX<double>* cf) {
    DRAKE_DEMAND(cf != nullptr);

    VectorX<double> qq, a;
    MatrixX<double> MM;
    ConstraintSolver<double>::MlcpToLcpData mlcp_to_lcp_data;
    solver_.ConstructBaseDiscretizedTimeLcp(
        problem_data, &mlcp_to_lcp_data, &MM, &qq);
    solver_.UpdateDiscretizedTimeLcp(
        problem_data, dt, &mlcp_to_lcp_data, &a, &MM, &qq);

    // Determine the zero tolerance.
    solvers::UnrevisedLemkeSolver<double> lcp;
    const double zero_tol = lcp.ComputeZeroTolerance(MM);

    // Attempt to solve the linear complementarity problem.
    int num_pivots;
    VectorX<double> zz;
    const bool success = lcp.SolveLcpLemke(MM, qq, &zz, &num_pivots);
    VectorX<double> ww = MM*zz + qq;
    double max_dot = (zz.size() > 0) ?
                     (zz.array() * ww.array()).abs().maxCoeff() : 0.0;

    // Check for success.
    const int num_vars = qq.size();
    ASSERT_TRUE(success);
    ASSERT_TRUE(zz.size() == 0 || zz.minCoeff() > -num_vars * zero_tol);
    ASSERT_TRUE(ww.size() == 0 || ww.minCoeff() > -num_vars * zero_tol);
    ASSERT_TRUE(zz.size() == 0 || max_dot < std::max(1.0, zz.maxCoeff()) *
        std::max(1.0, ww.maxCoeff()) * num_vars * zero_tol);

    // Compute the packed constraint forces.
    solver_.PopulatePackedConstraintForcesFromLcpSolution(
        problem_data, mlcp_to_lcp_data, zz, a, dt, cf);
  }

  // Computes rigid contact data without any duplication of contact points or
  // friction directions.
  void CalcConstraintAccelProblemData(
      ConstraintAccelProblemData<double>* data) {
    CalcConstraintAccelProblemData(
        data,
        0,    // no contacts duplicated
        0);   // no friction directions duplicated
  }

  // Computes constraint data without duplicate contacts or friction directions.
  void CalcDiscreteTimeProblemData(
      double dt,
      ConstraintVelProblemData<double>* data) {
    CalcDiscreteTimeProblemData(dt, data, 0, 0);
  }

  // Computes rigid impacting contact data.
  // @param contact_points_dup the number of times (>= 0) each contact point
  //        should be duplicated in the contact data.
  // @param friction_directions_dup the number of times (>= 0) that each
  //        friction basis direction should be duplicated. Since the 2D tests
  //        only use one friction direction, duplication ensures that the
  //        algorithms are able to handle duplicated directions without error.
  void CalcConstraintProblemDataForImpact(
      ConstraintVelProblemData<double>* data,
      int contact_points_dup,
      int friction_directions_dup) {
    DRAKE_DEMAND(contact_points_dup >= 0);
    DRAKE_DEMAND(friction_directions_dup >= 0);

    // Reset constraint acceleration data.
    const int num_velocities = 3;
    *data = ConstraintVelProblemData<double>(num_velocities);

    // Get the points of contact from Rod2D.
    std::vector<Vector2d> contacts;
    rod_->GetContactPoints(*context_, &contacts);

    // Duplicate the contact points.
    std::vector<Vector2d> contacts_dup;
    for (int i = 0; i < static_cast<int>(contacts.size()); ++i) {
      for (int j = 0; j < contact_points_dup; ++j)
        contacts_dup.push_back(contacts[i]);
    }
    contacts.insert(contacts.end(), contacts_dup.begin(), contacts_dup.end());

    // Compute the problem data.
    rod_->CalcImpactProblemData(*context_, contacts, data);

    // Get old F.
    const int num_fdir = std::accumulate(data->r.begin(), data->r.end(), 0);
    const int ngc = get_rod_num_coordinates();
    MatrixX<double> Fold = MatrixX<double>::Zero(num_fdir, ngc);
    for (int j = 0; j < ngc; ++j)
      Fold.col(j) = data->F_mult(VectorX<double>::Unit(ngc, j));

    // Determine new F by duplicating each row the specified number of times.
    const int new_friction_directions = friction_directions_dup + 1;
    MatrixX<double> F(new_friction_directions * contacts.size(), ngc);
    for (int i = 0, k = 0; i < Fold.rows(); ++i)
      for (int j = 0; j < new_friction_directions; ++j)
        F.row(k++) = Fold.row(i);

    // Redetermine F_mult and F_transpose_mult operator lambdas.
    data->F_mult = [F](const VectorX<double>& w) -> VectorX<double> {
      return F * w;
    };
    data->F_transpose_mult = [F](const VectorX<double>& w) -> VectorX<double> {
      return F.transpose() * w;
    };

    // Resize kF (recall the vector always is zero for this 2D problem),
    // gammaF, and gammaE.
    data->kF.setZero(contacts.size() * new_friction_directions);
    data->gammaF.setZero(contacts.size() * new_friction_directions);
    data->gammaE.setZero(contacts.size());

    // Update r with the new friction directions per contact.
    for (int i = 0; i < static_cast<int>(data->r.size()); ++i)
      data->r[i] = new_friction_directions;

    // Add in empty rows to G, by default, allowing us to verify that no
    // constraint forces are added (and that solution method is robust to
    // unnecessary constraints).
    data->G_mult = [](const VectorX<double>& w) -> VectorX<double> {
      return VectorX<double>::Zero(1);
    };
    data->G_transpose_mult = [ngc](const VectorX<double>& w)
        -> VectorX<double> {
      return VectorX<double>::Zero(ngc);
    };
    data->kG.setZero(0);

    // Check the consistency of the data.
    CheckProblemConsistency(*data, contacts.size());
  }

  // Computes rigid impacting contact data with no duplication of contact points
  // or friction directions.
  void CalcConstraintProblemDataForImpact(
      ConstraintVelProblemData<double>* data) {
    CalcConstraintProblemDataForImpact(
        data,
        0,    // no contact points duplicated
        0);   // no friction directions duplicated
  }

  // Gets the number of generalized coordinates for the rod.
  int get_rod_num_coordinates() const { return 3; }

  // Gets the output dimension of a Jacobian multiplication operator.
  int GetOperatorDim(std::function<VectorX<double>(
      const VectorX<double>&)> J) const {
    return J(VectorX<double>(get_rod_num_coordinates())).size();
  }

  // Checks the consistency of a transpose operator.
  void CheckTransOperatorDim(
      std::function<VectorX<double>(const VectorX<double>&)> JT,
      int num_constraints) const {
    EXPECT_EQ(JT(VectorX<double>(num_constraints)).size(),
              get_rod_num_coordinates());
  }

  // Checks consistency of rigid contact problem data.
  void CheckProblemConsistency(
      const ConstraintAccelProblemData<double>& data,
      int num_contacts) const {
    const int ngc = get_rod_num_coordinates();
    const int num_fdir = std::accumulate(data.r.begin(), data.r.end(), 0);
    EXPECT_EQ(num_contacts, data.sliding_contacts.size() +
        data.non_sliding_contacts.size());
    EXPECT_EQ(GetOperatorDim(data.N_mult), num_contacts);
    CheckTransOperatorDim(data.N_minus_muQ_transpose_mult, num_contacts);
    EXPECT_EQ(GetOperatorDim(data.F_mult), num_fdir);
    CheckTransOperatorDim(data.F_transpose_mult, num_fdir);
    EXPECT_EQ(GetOperatorDim(data.L_mult), data.kL.size());
    CheckTransOperatorDim(data.L_transpose_mult, data.kL.size());
    EXPECT_EQ(data.tau.size(), ngc);
    EXPECT_EQ(data.kN.size(), num_contacts);
    EXPECT_EQ(data.kF.size(), num_fdir);
    EXPECT_EQ(data.mu_non_sliding.size(), data.non_sliding_contacts.size());
    EXPECT_EQ(data.mu_sliding.size(), data.sliding_contacts.size());
    EXPECT_EQ(data.r.size(), data.non_sliding_contacts.size());
    EXPECT_TRUE(data.solve_inertia);
    EXPECT_TRUE(std::is_sorted(data.sliding_contacts.begin(),
                               data.sliding_contacts.end()));
    EXPECT_TRUE(std::is_sorted(data.non_sliding_contacts.begin(),
                               data.non_sliding_contacts.end()));
  }

  // Checks consistency of rigid impact problem data.
  void CheckProblemConsistency(
      const ConstraintVelProblemData<double>& data,
      int num_contacts) const {
    const int ngc = get_rod_num_coordinates();
    const int num_spanning_directions = std::accumulate(
        data.r.begin(), data.r.end(), 0);
    EXPECT_EQ(GetOperatorDim(data.N_mult), num_contacts);
    CheckTransOperatorDim(data.N_transpose_mult, num_contacts);
    EXPECT_EQ(GetOperatorDim(data.F_mult), num_spanning_directions);
    CheckTransOperatorDim(data.F_transpose_mult, num_spanning_directions);
    EXPECT_EQ(GetOperatorDim(data.L_mult), data.kL.size());
    CheckTransOperatorDim(data.L_transpose_mult, data.kL.size());
    EXPECT_EQ(data.gammaN.size(), num_contacts);
    EXPECT_EQ(data.gammaF.size(), num_spanning_directions);
    EXPECT_EQ(data.gammaE.size(), num_contacts);
    EXPECT_EQ(data.gammaL.size(), data.kL.size());
    EXPECT_EQ(data.Mv.size(), ngc);
    EXPECT_EQ(data.mu.size(), num_contacts);
    EXPECT_EQ(data.r.size(), num_contacts);
    EXPECT_TRUE(data.solve_inertia);

    // TODO(edrumwri): Relax test for kF to be zero in the future (i.e., we
    // currently allow no constraint stabilization along the F direction).
    EXPECT_LT(data.kF.norm(), std::numeric_limits<double>::epsilon());
  }

  // Tests the rod in a single-point sticking configuration, with an external
  // force applied either to the right or to the left. Given sufficiently small
  // force and sufficiently large friction coefficient, the contact should
  // remain in stiction.
  void SinglePointSticking(bool force_applied_to_right) {
    // Set the contact to large friction. Note that only the static friction
    // coefficient will be used since there are no sliding contacts. However,
    // set_mu_static() throws an exception if it is not at least as large as
    // the Coulomb friction coefficient.
    rod_->set_mu_coulomb(15.0);
    rod_->set_mu_static(15.0);

    // Get the acceleration due to gravity.
    const double grav_accel = rod_->get_gravitational_acceleration();

    // Duplicate contact points up to two times and the friction directions up
    // to three times.
    for (int contact_dup = 0; contact_dup < 3; ++contact_dup) {
      for (int friction_dir_dup = 0; friction_dir_dup < 4; ++friction_dir_dup) {
        const int n_contacts = contact_dup + 1;

        // Set the state of the rod to resting vertically with no velocity.
        SetRodToRestingVerticalConfig();

        // Construct the contact frame.
        std::vector<Matrix2<double>> frames;
        for (int i = 0; i < n_contacts; ++i)
          frames.push_back(GetNonSlidingContactFrameToWorldTransform());

        // Compute the problem data.
        CalcConstraintAccelProblemData(
          accel_data_.get(), contact_dup, friction_dir_dup);
        EXPECT_TRUE(accel_data_->sliding_contacts.empty());

        // Add a force, acting at the point of contact, that pulls the rod
        // horizontally.
        const double horz_f = (force_applied_to_right) ? 100 : -100;
        accel_data_->tau[0] += horz_f;
        accel_data_->tau[2] += horz_f * rod_->get_rod_half_length();

        // Case 1: set kN as if the bodies are not accelerating into each
        // other along the contact normal and verify that no contact forces
        // are applied.
        {
          accel_data_->kN.setOnes() *= std::fabs(grav_accel);

          // Compute the contact forces.
          VectorX<double> cf;
          solver_.SolveConstraintProblem(*accel_data_, &cf);

          // Verify that no forces are applied.
          EXPECT_LT(cf.norm(), std::numeric_limits<double>::epsilon());
        }

        // Case 2: zero stabilization term and recompute the contact forces.
        {
          accel_data_->kN.setZero();
          VectorX<double> cf;
          solver_.SolveConstraintProblem(*accel_data_, &cf);

          // The primary test below is conducted when there is no friction
          // direction duplication because CalcContactForcesInContactFrames()
          // would throw an exception (the secondary test looks for this case).
          std::vector<Vector2<double>> contact_forces;
          if (friction_dir_dup == 0) {
            // Get the contact forces expressed in the contact frame.
            ConstraintSolver<double>::CalcContactForcesInContactFrames(cf,
                *accel_data_, frames, &contact_forces);

            // Verify that the number of contact force vectors is correct.
            ASSERT_EQ(contact_forces.size(), frames.size());

            // Verify that the frictional forces equal the horizontal force.
            const int total_cone_edges = std::accumulate(
                accel_data_->r.begin(), accel_data_->r.end(), 0);
            double ffriction = cf.segment(n_contacts, total_cone_edges).sum();
            EXPECT_NEAR(ffriction, -horz_f, eps_ * cf.size());
          } else {
            EXPECT_THROW(
                ConstraintSolver<double>::CalcContactForcesInContactFrames(cf,
                    *accel_data_, frames, &contact_forces), std::logic_error);
          }

          // Verify the generalized acceleration of the rod is equal to zero.
          VectorX<double> ga;
          solver_.ComputeGeneralizedAcceleration(*accel_data_, cf, &ga);
          EXPECT_LT(ga.norm(), eps_ * cf.size());
        }

        // Case 3: set kN as if the bodies are accelerating twice as hard into
        // each other along the contact normal.
        {
          accel_data_->kN.setOnes() *= -std::fabs(grav_accel);

          // Recompute the contact forces.
          VectorX<double> cf;
          solver_.SolveConstraintProblem(*accel_data_, &cf);

          // The primary test below is conducted when there is no friction
          // direction duplication because CalcContactForcesInContactFrames()
          // would throw an exception (the secondary test looks for this case).
          std::vector<Vector2<double>> contact_forces;
          if (friction_dir_dup == 0) {
            // Get the contact forces expressed in the contact frame.
            ConstraintSolver<double>::CalcContactForcesInContactFrames(
                cf, *accel_data_, frames, &contact_forces);

            // Verify that the number of contact force vectors is correct.
            ASSERT_EQ(contact_forces.size(), frames.size());

            // Verify that the frictional forces equal the horizontal force.
            const int total_cone_edges = std::accumulate(
                accel_data_->r.begin(), accel_data_->r.end(), 0);
            double ffriction = cf.segment(n_contacts, total_cone_edges).sum();
            EXPECT_NEAR(ffriction, -horz_f, eps_ * cf.size());
          } else {
            EXPECT_THROW(
                ConstraintSolver<double>::CalcContactForcesInContactFrames(
                    cf, *accel_data_, frames, &contact_forces),
                std::logic_error);
          }

          // Verify that the generalized acceleration of the rod is equal to the
          // gravitational acceleration.
          VectorX<double> ga;
          solver_.ComputeGeneralizedAcceleration(*accel_data_, cf, &ga);
          EXPECT_NEAR(ga.norm(), std::fabs(grav_accel), eps_ * cf.size());
        }
      }
    }
  }

  // Tests the rod in a single-point sticking configuration, with an external
  // force applied either to the right or to the left. Given sufficiently small
  // force and sufficiently large friction coefficient, the contact should
  // remain in stiction.
  void SinglePointStickingDiscretized(bool force_applied_to_right) {
    // Set the contact to large friction. However, set_mu_static() throws an
    // exception if it is not at least as large as the Coulomb friction
    // coefficient.
    rod_->set_mu_coulomb(15.0);
    rod_->set_mu_static(15.0);

    // Get the acceleration due to gravity.
    const double grav_accel = rod_->get_gravitational_acceleration();

    // Set the state of the rod to resting vertically with no velocity.
    SetRodToRestingVerticalConfig();

    // Duplicate contact points up to two times and the friction directions up
    // to three times.
    for (int contact_dup = 0; contact_dup < 3; ++contact_dup) {
      for (int friction_dir_dup = 0; friction_dir_dup < 4; ++friction_dir_dup) {
        const int n_contacts = contact_dup + 1;

        // Construct the contact frame.
        std::vector<Matrix2<double>> frames;
        for (int i = 0; i < n_contacts; ++i)
          frames.push_back(GetNonSlidingContactFrameToWorldTransform());

        // Compute the problem data.
        CalcDiscreteTimeProblemData(
            dt_, vel_data_.get(), contact_dup, friction_dir_dup);

        // Add a force applied at the point of contact that results in a torque
        // at the rod center-of-mass.
        const double horz_f = (force_applied_to_right) ? 100 : -100;
        vel_data_->Mv += Vector3<double>(
            horz_f, 0, horz_f * rod_->get_rod_half_length()) * dt_;

        // Case 1: set kN as if the bodies are not moving into each other along
        // the contact normal and verify that no contact forces are applied.
        {
          vel_data_->kN.setOnes() *= std::fabs(grav_accel) * dt_;

          // Compute the contact forces.
          VectorX<double> cf;
          SolveDiscretizationProblem(*vel_data_, dt_, &cf);

          // Verify that no forces are applied.
          EXPECT_LT(cf.norm(), std::numeric_limits<double>::epsilon());
        }

        // Case 2: zero the stabilization term and recompute the contact forces.
        {
          vel_data_->kN.setZero();
          VectorX<double> cf;
          SolveDiscretizationProblem(*vel_data_, dt_, &cf);

          // The primary test below is conducted when there is no friction
          // direction duplication because CalcContactForcesInContactFrames()
          // would throw an exception (the secondary test looks for this case).
          std::vector<Vector2<double>> contact_forces;
          if (friction_dir_dup == 0) {
            // Get the contact forces expressed in the contact frame.
            ConstraintSolver<double>::CalcContactForcesInContactFrames(
                cf, *vel_data_, frames, &contact_forces);

            // Verify that the number of contact force vectors is correct.
            ASSERT_EQ(contact_forces.size(), frames.size());

            // Verify that the frictional forces equal the horizontal force.
            const int total_cone_edges = std::accumulate(
                vel_data_->r.begin(), vel_data_->r.end(), 0);
            double ffriction = cf.segment(n_contacts, total_cone_edges).sum();
            EXPECT_NEAR(ffriction, -horz_f, eps_ * cf.size());
          } else {
            EXPECT_THROW(
                ConstraintSolver<double>::CalcContactForcesInContactFrames(
                    cf, *vel_data_, frames, &contact_forces), std::logic_error);
          }

          // Get the rod velocity.
          const VectorX<double> v0 = rod_->GetRodVelocity(*context_);

          // Verify the generalized acceleration of the rod is equal to zero.
          VectorX<double> ga;
          solver_.ComputeGeneralizedAcceleration(*vel_data_, v0, cf, dt_, &ga);
          EXPECT_LT(ga.norm(), eps_ * cf.size());
        }

        // Case 3: set kN as if the bodies are accelerating twice as hard into
        // each other along the contact normal.
        {
          vel_data_->kN.setOnes() *= -std::fabs(grav_accel) * dt_;

          // Recompute the contact forces.
          VectorX<double> cf;
          SolveDiscretizationProblem(*vel_data_, dt_, &cf);

          // The primary test below is conducted when there is no friction
          // direction duplication because CalcContactForcesInContactFrames()
          // would throw an exception (the secondary test looks for this case).
          std::vector<Vector2<double>> contact_forces;
          if (friction_dir_dup == 0) {
            // Get the contact forces expressed in the contact frame.
            ConstraintSolver<double>::CalcContactForcesInContactFrames(
                cf, *vel_data_, frames, &contact_forces);

            // Verify that the number of contact force vectors is correct.
            ASSERT_EQ(contact_forces.size(), frames.size());

            // Verify that the frictional forces equal the applied,
            // horizontal force.
            const int total_cone_edges = std::accumulate(
                vel_data_->r.begin(), vel_data_->r.end(), 0);
            double ffriction = cf.segment(n_contacts, total_cone_edges).sum();
            EXPECT_NEAR(ffriction, -horz_f, eps_ * cf.size());
          } else {
            EXPECT_THROW(
                ConstraintSolver<double>::CalcContactForcesInContactFrames(cf,
                    *vel_data_, frames, &contact_forces), std::logic_error);
          }

          // Get the rod velocity.
          const VectorX<double> v0 = rod_->GetRodVelocity(*context_);

          // Verify that the generalized acceleration of the rod is equal to
          // the gravitational acceleration.
          VectorX<double> ga;
          solver_.ComputeGeneralizedAcceleration(*vel_data_, v0, cf, dt_, &ga);
          EXPECT_NEAR(ga.norm(), std::fabs(grav_accel), eps_ * cf.size());
        }
      }
    }
  }

  // Tests the rod in a two-point sticking configuration (i.e., force should
  // be applied with no resulting tangential motion), with force applied either
  // to the right or to the left (force_applied_to_right = false) and using
  // either the LCP solver or the linear system solver (use_lcp_solver = false).
  void TwoPointSticking(
      bool force_applied_to_right, bool use_lcp_solver) {
    // Set the contact to large friction. Note that only the static friction
    // coefficient will be used since there are no sliding contacts.
    rod_->set_mu_coulomb(0.0);
    rod_->set_mu_static(15.0);

    // Get the acceleration due to gravity.
    const double grav_accel = rod_->get_gravitational_acceleration();

    // Duplicate contact points up to two times and the friction directions up
    // to three times.
    for (int contact_dup = 0; contact_dup < 3; ++contact_dup) {
      for (int friction_dir_dup = 0; friction_dir_dup < 4; ++friction_dir_dup) {
        const int n_contacts = 2 * (contact_dup + 1);

        // Set the state of the rod to resting on its side with no velocity.
        SetRodToRestingHorizontalConfig();

        // Construct the contact frames.
        std::vector<Matrix2<double>> frames;
        for (int i = 0; i < n_contacts; ++i)
          frames.push_back(GetNonSlidingContactFrameToWorldTransform());

        // Compute the problem data.
        CalcConstraintAccelProblemData(
            accel_data_.get(), contact_dup, friction_dir_dup);
        EXPECT_TRUE(accel_data_->sliding_contacts.empty());

        // Indicate whether to use the linear system solver.
        accel_data_->use_complementarity_problem_solver = use_lcp_solver;

        // Add a force pulling the rod horizontally.
        const double horz_f = (force_applied_to_right) ? 100 : -100;
        accel_data_->tau[0] += horz_f;

        // Case 1: set kN as if the bodies are not accelerating into each
        // other along the contact normal and verify that no contact forces
        // are applied.
        {
          accel_data_->kN.setOnes() *= std::fabs(grav_accel);

          // Compute the contact forces.
          VectorX<double> cf;
          solver_.SolveConstraintProblem(*accel_data_, &cf);

          // Verify that no forces are applied.
          EXPECT_LT(cf.norm(), std::numeric_limits<double>::epsilon());
        }

        // Case 2: zero stabilization term and recompute the contact forces.
        {
          accel_data_->kN.setZero();
          VectorX<double> cf;
          solver_.SolveConstraintProblem(*accel_data_, &cf);

          // Do this next part *only* if the friction directions are not
          // duplicated (which would cause an exception to be thrown).
          std::vector<Vector2<double>> contact_forces;
          if (friction_dir_dup == 0) {
            // Get the contact forces expressed in the contact frames.
            ConstraintSolver<double>::CalcContactForcesInContactFrames(cf,
                *accel_data_, frames, &contact_forces);

            // Verify that the number of contact force vectors is correct.
            ASSERT_EQ(contact_forces.size(), n_contacts);

            // Verify that the normal forces equal the gravitational force.
            // Normal forces are in the first component of each vector.
            // Frictional forces are in the second component of each vector.
            const double mg = rod_->get_rod_mass() *
                std::fabs(rod_->get_gravitational_acceleration());
            double normal_force_mag = 0;
            double fric_force = 0;
            for (int i = 0; i < static_cast<int>(contact_forces.size()); ++i) {
              normal_force_mag += contact_forces[i][0];
              fric_force += contact_forces[i][1];
            }
            EXPECT_NEAR(normal_force_mag, mg, eps_ * cf.size());

            // Verify that the negation of the frictional forces equal the
            // horizontal forces.
            EXPECT_NEAR(fric_force, -horz_f, eps_ * cf.size());
          } else {
            EXPECT_THROW(
                ConstraintSolver<double>::CalcContactForcesInContactFrames(
                    cf, *accel_data_, frames, &contact_forces),
                std::logic_error);
          }

          // Verify the generalized acceleration of the rod is equal to zero.
          VectorX<double> ga;
          solver_.ComputeGeneralizedAcceleration(*accel_data_, cf, &ga);
          EXPECT_LT(ga.norm(), eps_ * cf.size());
        }

        // Case 3: set kN as if the bodies are accelerating twice as hard into
        // each other along the contact normal.
        {
          accel_data_->kN.setOnes() *= -std::fabs(grav_accel);

          // Recompute the contact forces.
          VectorX<double> cf;
          solver_.SolveConstraintProblem(*accel_data_, &cf);

          // The primary test below is conducted when there is no friction
          // direction duplication because CalcContactForcesInContactFrames()
          // would throw an exception (the secondary test looks for this case).
          std::vector<Vector2<double>> contact_forces;
          if (friction_dir_dup == 0) {
            // Get the contact forces expressed in the contact frame.
            ConstraintSolver<double>::CalcContactForcesInContactFrames(
                cf, *accel_data_, frames, &contact_forces);

            // Verify that the number of contact force vectors is correct.
            ASSERT_EQ(contact_forces.size(), frames.size());

            // Verify that the frictional forces equal the horizontal force.
            const int total_cone_edges = std::accumulate(
                accel_data_->r.begin(), accel_data_->r.end(), 0);
            double ffriction = cf.segment(n_contacts, total_cone_edges).sum();
            EXPECT_NEAR(ffriction, -horz_f, eps_ * cf.size());
          } else {
            EXPECT_THROW(
                ConstraintSolver<double>::CalcContactForcesInContactFrames(cf,
                    *accel_data_, frames, &contact_forces), std::logic_error);
          }

          // Verify that the generalized acceleration of the rod is equal to the
          // gravitational acceleration.
          VectorX<double> ga;
          solver_.ComputeGeneralizedAcceleration(*accel_data_, cf, &ga);
          EXPECT_NEAR(ga.norm(), std::fabs(grav_accel), eps_ * cf.size());
        }
      }
    }
  }

  // Tests the rod in a two-point non-sticking configuration that will
  // transition to sliding from the external force (applied toward the right
  // or the left, as specified.
  void TwoPointNonSlidingToSliding(bool applied_to_right) {
    // Set the contact to large friction. Note that only the static friction
    // coefficient will be used since there are no sliding contacts.
    const double mu_static = 0.1;
    rod_->set_mu_coulomb(0.0);
    rod_->set_mu_static(mu_static);

    // Get the acceleration due to gravity.
    const double grav_accel = rod_->get_gravitational_acceleration();

    // Duplicate contact points up to two times and the friction directions up
    // to three times.
    for (int contact_dup = 0; contact_dup < 3; ++contact_dup) {
      for (int friction_dir_dup = 0; friction_dir_dup < 4; ++friction_dir_dup) {
        const int n_contacts = 2 * (1 + contact_dup);

        // Set the state of the rod to resting on its side with no velocity.
        SetRodToRestingHorizontalConfig();

        // Construct the contact frames.
        std::vector<Matrix2<double>> frames;
        for (int i = 0; i < n_contacts; ++i)
          frames.push_back(GetNonSlidingContactFrameToWorldTransform());

        // Compute the problem data.
        CalcConstraintAccelProblemData(
            accel_data_.get(), contact_dup, friction_dir_dup);

        // Add a force pulling the rod horizontally.
        const double horz_f = (applied_to_right) ? 100.0 : -100;
        accel_data_->tau[0] += horz_f;

        // Case 1: set kN as if the bodies are not accelerating into each
        // other along the contact normal and verify that no contact forces
        // are applied.
        {
          accel_data_->kN.setOnes() *= std::fabs(grav_accel);

          // Compute the contact forces.
          VectorX<double> cf;
          solver_.SolveConstraintProblem(*accel_data_, &cf);

          // Verify that no forces are applied.
          EXPECT_LT(cf.norm(), std::numeric_limits<double>::epsilon());
        }

        // Case 2: zero stabilization term and recompute the contact forces.
        {
          accel_data_->kN.setZero();
          VectorX<double> cf;
          solver_.SolveConstraintProblem(*accel_data_, &cf);

          // Get the contact forces expressed in the contact frames *only*
          // if the friction directions are not duplicated (which would cause an
          // exception to be thrown).
          std::vector<Vector2<double>> contact_forces;
          if (friction_dir_dup == 0) {
            EXPECT_TRUE(accel_data_->sliding_contacts.empty());

            // Get the contact forces expressed in the contact frames.
            ConstraintSolver<double>::CalcContactForcesInContactFrames(
                cf, *accel_data_, frames, &contact_forces);

            // Verify that the number of contact force vectors is correct.
            ASSERT_EQ(contact_forces.size(), n_contacts);

            // Verify that the frictional forces are maximized.
            double fnormal = 0;
            double ffrictional = 0;
            for (int i = 0; i < static_cast<int>(contact_forces.size()); ++i) {
              fnormal += contact_forces[i][0];
              ffrictional += std::fabs(contact_forces[i][1]);
            }
            EXPECT_NEAR(ffrictional, mu_static * fnormal, eps_ * cf.size());
          } else {
            EXPECT_THROW(
                ConstraintSolver<double>::CalcContactForcesInContactFrames(
                    cf, *accel_data_, frames, &contact_forces),
                std::logic_error);
          }

          // Verify that the horizontal acceleration is in the appropriate
          // direction.
          VectorX<double> ga;
          solver_.ComputeGeneralizedAcceleration(*accel_data_, cf, &ga);
          EXPECT_GT((applied_to_right) ? ga[0] : -ga[0], 0);
        }

        // Case 3: now, set kN as if the bodies are accelerating twice as hard
        // into each other along the contact normal.
        {
          accel_data_->kN.setOnes() *= -std::fabs(grav_accel);

          // Recompute the contact forces.
          VectorX<double> cf;
          solver_.SolveConstraintProblem(*accel_data_, &cf);

          // The primary test below is conducted when there is no friction
          // direction duplication because CalcContactForcesInContactFrames()
          // would throw an exception (the secondary test looks for this case).
          std::vector<Vector2<double>> contact_forces;
          if (friction_dir_dup == 0) {
            // Get the contact forces expressed in the contact frame.
            ConstraintSolver<double>::CalcContactForcesInContactFrames(
                cf, *accel_data_, frames, &contact_forces);

            // Verify that the number of contact force vectors is correct.
            ASSERT_EQ(contact_forces.size(), frames.size());

            // Verify that the frictional forces are maximized.
            double fnormal = 0;
            double ffrictional = 0;
            for (int i = 0; i < static_cast<int>(contact_forces.size()); ++i) {
              fnormal += contact_forces[i][0];
              ffrictional += std::fabs(contact_forces[i][1]);
            }
            EXPECT_NEAR(ffrictional, mu_static * fnormal, eps_ * cf.size());
          } else {
            EXPECT_THROW(
                ConstraintSolver<double>::CalcContactForcesInContactFrames(cf,
                    *accel_data_, frames, &contact_forces), std::logic_error);
          }

          // Verify that the vertical acceleration of the rod is equal to the
          // gravitational acceleration and that the horizontal acceleration is
          // in the appropriate direction.
          VectorX<double> ga;
          solver_.ComputeGeneralizedAcceleration(*accel_data_, cf, &ga);
          EXPECT_NEAR(ga[1], std::fabs(grav_accel), eps_ * cf.size());
          EXPECT_GT((applied_to_right) ? ga[0] : -ga[0], 0);
        }
      }
    }
  }

  // Tests the rod in a two-point contact configuration, without sliding
  // occurring at either contact point, that will transition to sliding from the
  // external force (applied toward the right or the left, as specified).
  void TwoPointNonSlidingToSlidingDiscretized(bool applied_to_right) {
    // Set the contact to large friction.
    const double mu_static = 0.1;
    rod_->set_mu_coulomb(mu_static);
    rod_->set_mu_static(mu_static);

    // Get the acceleration due to gravity.
    const double grav_accel = rod_->get_gravitational_acceleration();

    // Duplicate contact points up to two times and the friction directions up
    // to three times.
    for (int contact_dup = 0; contact_dup < 3; ++contact_dup) {
      for (int friction_dir_dup = 0; friction_dir_dup < 4; ++friction_dir_dup) {
        const int n_contacts = 2 * (1 + contact_dup);

        // Set the state of the rod to resting on its side with zero velocity.
        SetRodToRestingHorizontalConfig();

        // Construct the contact frames.
        std::vector<Matrix2<double>> frames;
        for (int i = 0; i < n_contacts; ++i)
          frames.push_back(GetNonSlidingContactFrameToWorldTransform());

        // Compute the problem data.
        CalcDiscreteTimeProblemData(
            dt_, vel_data_.get(), contact_dup, friction_dir_dup);

        // Add a force pulling the rod horizontally.
        const double horz_f = (applied_to_right) ? 100.0 : -100;
        vel_data_->Mv[0] += horz_f * dt_;

        // Case 1: set kN as if the bodies are not accelerating into each
        // other along the contact normal and verify that no contact forces
        // are applied.
        {
          vel_data_->kN.setOnes() *= std::fabs(grav_accel) * dt_;

          // Compute the contact forces.
          VectorX<double> cf;
          SolveDiscretizationProblem(*vel_data_, dt_, &cf);

          // Verify that no forces are applied.
          EXPECT_LT(cf.norm(), std::numeric_limits<double>::epsilon());
        }

        // Case 2: zero stabilization term and recompute the contact forces.
        {
          vel_data_->kN.setZero();
          VectorX<double> cf;
          SolveDiscretizationProblem(*vel_data_, dt_, &cf);

          // Get the contact forces expressed in the contact frames *only*
          // if the friction directions are not duplicated (which would cause an
          // exception to be thrown).
          std::vector<Vector2<double>> contact_forces;
          if (friction_dir_dup == 0) {
            // Get the contact forces expressed in the contact frames.
            ConstraintSolver<double>::CalcContactForcesInContactFrames(
                cf, *vel_data_, frames, &contact_forces);

            // Verify that the number of contact force vectors is correct.
            ASSERT_EQ(contact_forces.size(), n_contacts);

            // Get the expected sign of the frictional forces.
            const double ff_sign = (applied_to_right) ? -1.0 : 1.0;

            // Verify that the frictional forces are maximized and in the
            // correct direction.
            double fnormal = 0;
            double ffrictional = 0;
            for (int i = 0; i < static_cast<int>(contact_forces.size()); ++i) {
              fnormal += contact_forces[i][0];
              ffrictional += contact_forces[i][1];
              EXPECT_NEAR(ffrictional * ff_sign, mu_static * fnormal,
                          eps_ * cf.size());
            }
          } else {
            EXPECT_THROW(
                ConstraintSolver<double>::CalcContactForcesInContactFrames(
                    cf, *vel_data_, frames, &contact_forces), std::logic_error);
          }

          // Verify that the horizontal acceleration is in the appropriate
          // direction.
          const VectorX<double> v0 = rod_->GetRodVelocity(*context_);
          VectorX<double> ga;
          solver_.ComputeGeneralizedAcceleration(*vel_data_, v0, cf, dt_, &ga);
          EXPECT_GT((applied_to_right) ? ga[0] : -ga[0], 0);
        }

        // Case 3: set kN as if the bodies are accelerating twice as hard into
        // each other along the contact normal.
        {
          vel_data_->kN.setOnes() *= -std::fabs(grav_accel) * dt_;

          // Recompute the contact forces.
          VectorX<double> cf;
          SolveDiscretizationProblem(*vel_data_, dt_, &cf);

          // The primary test below is conducted when there is no friction
          // direction duplication because CalcContactForcesInContactFrames()
          // would throw an exception (the secondary test looks for this case).
          std::vector<Vector2<double>> contact_forces;
          if (friction_dir_dup == 0) {
            // Get the contact forces expressed in the contact frame.
            ConstraintSolver<double>::CalcContactForcesInContactFrames(
                cf, *vel_data_, frames, &contact_forces);

            // Verify that the number of contact force vectors is correct.
            ASSERT_EQ(contact_forces.size(), frames.size());

            // Get the sign of the force application.
            const double ff_sign = (applied_to_right) ? -1.0 : 1.0;

            // Verify that the frictional forces are maximized.
            double fnormal = 0;
            double ffrictional = 0;
            for (int i = 0; i < static_cast<int>(contact_forces.size()); ++i) {
              fnormal += contact_forces[i][0];
              ffrictional += contact_forces[i][1];
            }
            EXPECT_NEAR(ff_sign * ffrictional, mu_static * fnormal,
                        eps_ * cf.size());
          } else {
            EXPECT_THROW(
                ConstraintSolver<double>::CalcContactForcesInContactFrames(
                    cf, *vel_data_, frames, &contact_forces), std::logic_error);
          }

          // Verify that the vertical acceleration of the rod is equal to the
          // gravitational acceleration and that the horizontal acceleration is
          // in the appropriate direction.
          const VectorX<double> v0 = rod_->GetRodVelocity(*context_);
          VectorX<double> ga;
          solver_.ComputeGeneralizedAcceleration(*vel_data_, v0, cf, dt_, &ga);
          EXPECT_NEAR(ga[1], std::fabs(grav_accel), eps_ * cf.size());
          EXPECT_GT((applied_to_right) ? ga[0] : -ga[0], 0);
        }
      }
    }
  }

  // Tests the rod in a two-point impact, with initial velocity sliding to the
  // right or left, as specified, with impulses insufficient to put the rod
  // into stiction.
  void TwoPointImpactNoTransitionToStiction(bool sliding_to_right) {
    // Set the coefficient of friction to very small.
    const double mu = 1e-4;
    rod_->set_mu_coulomb(mu);

    // Duplicate contact points up to two times and the friction directions up
    // to three times.
    for (int contact_dup = 0; contact_dup < 3; ++contact_dup) {
      for (int friction_dir_dup = 0; friction_dir_dup < 4; ++friction_dir_dup) {
        const int n_contacts = 2 * (contact_dup + 1);

        // Construct the contact frames.
        std::vector<Matrix2<double>> frames;
        for (int i = 0; i < n_contacts; ++i)
          frames.push_back(GetNonSlidingContactFrameToWorldTransform());

        // Set the configuration of the rod to lying on its side and impacting.
        SetRodToSlidingImpactingHorizontalConfig(sliding_to_right);

        // Get the vertical velocity of the rod.
        const double vert_vel = context_->get_continuous_state().
            CopyToVector()[4];

        // Set the sign of the sliding direction.
        const double sign = (sliding_to_right) ? 1 : -1;

        // Compute the problem data.
        CalcConstraintProblemDataForImpact(
           vel_data_.get(), contact_dup, friction_dir_dup);

        // Get the generalized velocity of the rod.
        const VectorX<double> v0 = vel_data_->solve_inertia(vel_data_->Mv);

        // Case 1: set kN as if the bodies are not moving toward each
        // other along the contact normal and verify that no contact forces
        // are applied.
        {
          vel_data_->kN.setOnes() *= -vert_vel;

          // Compute the contact impulses.
          VectorX<double> cf;
          solver_.SolveImpactProblem(*vel_data_, &cf);

          // Verify that no impulses are applied.
          EXPECT_LT(cf.norm(), std::numeric_limits<double>::epsilon());
        }

        // Case 2: zero stabilization term and recompute the contact impulses.
        {
          vel_data_->kN.setZero();
          VectorX<double> cf;
          solver_.SolveImpactProblem(*vel_data_, &cf);

          // Get the sign of the force application.
          const double ff_sign = (sliding_to_right) ? -1.0 : 1.0;

          // Get the impact forces expressed in the contact frames *only*
          // if the friction directions are not duplicated (which would cause an
          // exception to be thrown).
          std::vector<Vector2<double>> contact_impulses;
          if (friction_dir_dup == 0) {
            // Get the contact impulses expressed in the contact frames.
            ConstraintSolver<double>::CalcContactForcesInContactFrames(
                cf, *vel_data_, frames, &contact_impulses);

            // Verify that the number of contact impulse vectors is correct.
            ASSERT_EQ(contact_impulses.size(), n_contacts);

            // Verify that the frictional impulses are maximized.
            double jnormal = 0;
            double jfrictional = 0;
            for (int i = 0; i < static_cast<int>(contact_impulses.size());
                 ++i) {
              jnormal += contact_impulses[i][0];
              jfrictional += contact_impulses[i][1];
              EXPECT_NEAR(ff_sign * jfrictional, mu * jnormal, eps_);
            }
          } else {
            EXPECT_THROW(
                ConstraintSolver<double>::CalcContactForcesInContactFrames(
                    cf, *vel_data_, frames, &contact_impulses),
                std::logic_error);
          }

          // Verify that the horizontal velocity is in the proper direction.
          VectorX<double> dgv;
          solver_.ComputeGeneralizedVelocityChange(*vel_data_, cf, &dgv);
          EXPECT_GT(sign * v0[0] + dgv[0], 0);
        }

        // Case 3: set kN as if the bodies are moving twice as fast into
        // each other along the contact normal.
        {
          vel_data_->kN.setOnes() *= vert_vel;

          // Recompute the contact impulses.
          VectorX<double> cf;
          solver_.SolveImpactProblem(*vel_data_, &cf);

          // Get the impact forces expressed in the contact frames if the
          // friction directions are not duplicated.
          std::vector<Vector2<double>> contact_impulses;
          if (friction_dir_dup == 0) {
            // Get the contact impulses expressed in the contact frames.
            ConstraintSolver<double>::CalcContactForcesInContactFrames(
                cf, *vel_data_, frames, &contact_impulses);

            // Verify that the number of contact impulse vectors is correct.
            ASSERT_EQ(contact_impulses.size(), n_contacts);

            // Verify that the frictional impulses are maximized.
            double jnormal = 0;
            double jfrictional = 0;
            for (int i = 0; i < static_cast<int>(contact_impulses.size());
                 ++i) {
              jnormal += contact_impulses[i][0];
              jfrictional += std::fabs(contact_impulses[i][1]);
            }
            EXPECT_NEAR(jfrictional, mu * jnormal, eps_);
          } else {
            EXPECT_THROW(
                ConstraintSolver<double>::CalcContactForcesInContactFrames(
                    cf, *vel_data_, frames, &contact_impulses),
                std::logic_error);
          }

          // Verify that the horizontal velocity is in the proper direction and
          // that the vertical velocity is essentially reversed.
          VectorX<double> dgv;
          solver_.ComputeGeneralizedVelocityChange(*vel_data_, cf, &dgv);
          EXPECT_GT(sign * v0[0] + dgv[0], 0);
          EXPECT_NEAR(v0[1] + dgv[1], -v0[1], eps_);
        }
      }
    }
  }

  // Tests the rod in a two-point impacting and sticking configuration,
  // with initial velocity sliding to the right or left, as specified.
  void TwoPointImpactingAndSticking(bool sliding_to_right) {
    // Set the rod to large friction.
    rod_->set_mu_coulomb(15.0);

    // Duplicate contact points up to two times and the friction directions up
    // to three times.
    for (int contact_dup = 0; contact_dup < 3; ++contact_dup) {
      for (int friction_dir_dup = 0; friction_dir_dup < 4; ++friction_dir_dup) {
        const int n_contacts = 2 * (contact_dup + 1);

        // Set the state of the rod to lying on its side with both impacting
        // velocity and horizontally moving velocity.
        SetRodToSlidingImpactingHorizontalConfig(sliding_to_right);

        // Construct the contact frames.
        std::vector<Matrix2<double>> frames;
        for (int i = 0; i < n_contacts; ++i)
          frames.push_back(GetNonSlidingContactFrameToWorldTransform());

        // Get the vertical velocity of the rod.
        const double vert_vel = context_->get_continuous_state().
            CopyToVector()[4];

        // Compute the impact problem data.
        CalcConstraintProblemDataForImpact(
            vel_data_.get(), contact_dup, friction_dir_dup);

        // Get the generalized velocity of the rod.
        const VectorX<double> v0 = vel_data_->solve_inertia(vel_data_->Mv);

        // Case 1: set kN as if the bodies are not moving toward each
        // other along the contact normal and verify that no contact forces
        // are applied.
        {
          vel_data_->kN.setOnes() *= -vert_vel;

          // Compute the contact impulses.
          VectorX<double> cf;
          solver_.SolveImpactProblem(*vel_data_, &cf);

          // Verify that no impulses are applied.
          EXPECT_LT(cf.norm(), std::numeric_limits<double>::epsilon());
        }

        // Case 2: zero stabilization term and recompute the contact impulses.
        {
          vel_data_->kN.setZero();
          VectorX<double> cf;
          solver_.SolveImpactProblem(*vel_data_, &cf);

          // Verify that the x-axis of the contact frame, which corresponds to
          // the contact normal, points along the world y-axis, and the y-axis
          //  of the contact frame, which corresponds to a contact tangent
          // vector, points along the world x-axis.
          for (int i = 0; i < static_cast<int>(frames.size()); ++i) {
            EXPECT_LT(
                std::fabs(frames[i].col(0).dot(Vector2<double>::UnitY()) - 1.0),
                std::numeric_limits<double>::epsilon());
            EXPECT_LT(
                std::fabs(frames[i].col(1).dot(Vector2<double>::UnitX()) - 1.0),
                std::numeric_limits<double>::epsilon());
          }

          // Get the impulsive contact forces expressed in the contact frames
          // *only* if the friction directions are not duplicated (which would
          // cause an exception to be thrown).
          std::vector<Vector2<double>> contact_forces;
          if (friction_dir_dup == 0) {
            ConstraintSolver<double>::CalcContactForcesInContactFrames(
                cf, *vel_data_, frames, &contact_forces);

            // Verify that the number of contact force vectors is correct.
            ASSERT_EQ(contact_forces.size(), n_contacts);
          } else {
            EXPECT_THROW(
                ConstraintSolver<double>::CalcContactForcesInContactFrames(
                    cf, *vel_data_, frames, &contact_forces), std::logic_error);
          }

          // Verify that the generalized velocity of the rod is equal to zero.
          VectorX<double> dgv;
          solver_.ComputeGeneralizedVelocityChange(*vel_data_, cf, &dgv);
          EXPECT_LT((v0 + dgv).norm(), eps_);
        }

        // Case 3: set kN as if the bodies are moving twice as fast into
        // each other along the contact normal.
        {
          vel_data_->kN.setOnes() *= vert_vel;

          // Recompute the contact impulses.
          VectorX<double> cf;
          solver_.SolveImpactProblem(*vel_data_, &cf);

          // Verify that all components of the generalized velocity of the rod
          // except that corresponding to the vertical motion are equal to zero;
          // the vertical motion should oppose the initial vertical motion.
          VectorX<double> dgv;
          solver_.ComputeGeneralizedVelocityChange(*vel_data_, cf, &dgv);
          EXPECT_LT((v0[0] + dgv[0]), eps_);
          EXPECT_NEAR((v0[1] + dgv[1]), -vert_vel, eps_);
          EXPECT_LT((v0[2] + dgv[2]), eps_);
        }
      }
    }
  }

  // Tests the rod in a sliding configuration with sliding velocity as
  // specified. If `upright` is true, then the rod makes contact at a single
  // point. Otherwise, it will be on its side and make contact at two points.
  // The fully upright and on-side configurations permit readily predicting
  // the requisite force(s) and motion.
  void Sliding(bool sliding_to_right, bool upright, bool use_lcp_solver) {
    if (upright) {
      SetRodToRestingVerticalConfig();
    } else {
      // Set the state of the rod to resting on its side w/ horizontal velocity.
      SetRodToRestingHorizontalConfig();
    }
    ContinuousState<double>& xc = context_->get_mutable_continuous_state();
    xc[3] = (sliding_to_right) ? 1 : -1;

    // Get the gravitational acceleration.
    const double grav_accel = rod_->get_gravitational_acceleration();

    // Set the coefficient of friction.
    rod_->set_mu_coulomb(0.0);

    // Compute the problem data.
    CalcConstraintAccelProblemData(accel_data_.get());
    accel_data_->use_complementarity_problem_solver = use_lcp_solver;

    // Case 1: counteract the acceleration from gravity using the kN term.
    {
      accel_data_->kN.setOnes() *= -grav_accel;

      // Compute the contact forces.
      VectorX<double> cf;
      solver_.SolveConstraintProblem(*accel_data_, &cf);

      // Verify that no forces were applied.
      EXPECT_LT(cf.norm(), eps_);
    }

    // Case 2: zero out the kN term and try again.
    {
      accel_data_->kN.setZero();
      VectorX<double> cf;
      solver_.SolveConstraintProblem(*accel_data_, &cf);

      // Get the contact tangent velocities.
      std::vector<Vector2d> contacts;
      std::vector<double> tangent_vels;
      rod_->GetContactPoints(*context_, &contacts);
      rod_->GetContactPointsTangentVelocities(*context_,
                                              contacts,
                                              &tangent_vels);

      // Construct the contact frame(s).
      std::vector<Matrix2<double>> frames;
      frames.push_back(
          GetSlidingContactFrameToWorldTransform(tangent_vels.front()));
      if (!upright) {
        frames.push_back(
            GetSlidingContactFrameToWorldTransform(tangent_vels.back()));
      }

      // Get the contact forces expressed in the contact frame.
      std::vector<Vector2<double>> contact_forces;
      ConstraintSolver<double>::CalcContactForcesInContactFrames(cf,
          *accel_data_, frames, &contact_forces);

      // Verify that the number of contact force vectors is correct.
      ASSERT_EQ(contact_forces.size(), frames.size());

      // Verify that there are no non-sliding frictional forces.
      EXPECT_TRUE(accel_data_->non_sliding_contacts.empty());
      const int nc = accel_data_->sliding_contacts.size();
      EXPECT_EQ(cf.size(), nc);

      // Verify that the normal contact forces exactly oppose gravity (the
      // frictional forces should be zero).
      const double mg = std::fabs(rod_->get_gravitational_acceleration()) *
          rod_->get_rod_mass();
      double fN = 0, fF = 0;
      for (int i = 0; i < static_cast<int>(contact_forces.size()); ++i) {
        fN += contact_forces[i][0];
        fF += std::fabs(contact_forces[i][1]);
      }
      EXPECT_NEAR(fN, mg, eps_);
      EXPECT_NEAR(fF, 0, eps_);
    }

    // Case 3: set the kN term to indicate that the rod is accelerating downward
    // with twice the gravitational acceleration.
    {
      accel_data_->kN.setOnes() *= grav_accel;
      VectorX<double> cf;
      solver_.SolveConstraintProblem(*accel_data_, &cf);

      // Verify that the normal component of the generalized acceleration is now
      // equal to the negated gravitational acceleration.
      VectorX<double> ga;
      solver_.ComputeGeneralizedAcceleration(*accel_data_, cf, &ga);
      EXPECT_NEAR(ga[1], -grav_accel, eps_);
    }
  }

  // Tests the rod in a sliding configuration with sliding velocity as
  // specified. If `upright` is true, then the rod makes contact at a single
  // point. Otherwise, it will be on its side and make contact at two points.
  // The fully upright and on-side configurations permit readily predicting
  // the requisite force(s) and motion.
  void SlidingDiscretized(bool sliding_to_right, bool upright) {
    if (upright) {
      SetRodToRestingVerticalConfig();
    } else {
      // Set the state of the rod to resting on its side w/ horizontal velocity.
      SetRodToRestingHorizontalConfig();
    }
    ContinuousState<double>& xc = context_->get_mutable_continuous_state();
    xc[3] = (sliding_to_right) ? 1 : -1;

    // Get the gravitational acceleration.
    const double grav_accel = rod_->get_gravitational_acceleration();

    // Set the coefficient of friction to zero. Note that a test that uses
    // a non-zero coefficient of friction with sliding is
    // SlidingPlusBilateralDiscretized().
    rod_->set_mu_coulomb(0.0);

    // Compute the problem data.
    const VectorX<double> v0 = rod_->GetRodVelocity(*context_);
    CalcDiscreteTimeProblemData(dt_, vel_data_.get());

    // Case 1: counteract the acceleration from gravity using the kN term.
    {
      vel_data_->kN.setOnes() *= -grav_accel * dt_;

      // Compute the contact forces.
      VectorX<double> cf;
      SolveDiscretizationProblem(*vel_data_, dt_, &cf);

      // Verify that no forces were applied.
      EXPECT_LT(cf.norm(), eps_);
    }

    // Case 2: zero out the kN term and try again.
    {
      vel_data_->kN.setZero();
      VectorX<double> cf;
      SolveDiscretizationProblem(*vel_data_, dt_, &cf);

      // Get the contact tangent velocities.
      std::vector<Vector2d> contacts;
      std::vector<double> tangent_vels;
      rod_->GetContactPoints(*context_, &contacts);
      rod_->GetContactPointsTangentVelocities(*context_,
                                              contacts,
                                              &tangent_vels);

      // Construct the contact frame(s).
      std::vector<Matrix2<double>> frames;
      frames.push_back(
          GetSlidingContactFrameToWorldTransform(tangent_vels.front()));
      if (!upright) {
        frames.push_back(
            GetSlidingContactFrameToWorldTransform(tangent_vels.back()));
      }

      // Get the contact forces expressed in the contact frame.
      std::vector<Vector2<double>> contact_forces;
      ConstraintSolver<double>::CalcContactForcesInContactFrames(
          cf, *vel_data_, frames, &contact_forces);

      // Verify that the number of contact force vectors is correct.
      ASSERT_EQ(contact_forces.size(), frames.size());

      // Verify that the normal contact forces exactly oppose gravity (the
      // frictional forces should be zero).
      const double mg = std::fabs(rod_->get_gravitational_acceleration()) *
          rod_->get_rod_mass();
      double fN = 0, fF = 0;
      for (int i = 0; i < static_cast<int>(contact_forces.size()); ++i) {
        fN += contact_forces[i][0];
        fF += std::fabs(contact_forces[i][1]);
      }
      EXPECT_NEAR(fN, mg, eps_);
      EXPECT_NEAR(fF, 0, eps_);
    }

    // Case 3: set the kN term to indicate that the rod is accelerating downward
    // with twice the gravitational acceleration.
    {
      vel_data_->kN.setOnes() *= grav_accel * dt_;
      VectorX<double> cf;
      SolveDiscretizationProblem(*vel_data_, dt_, &cf);

      // Verify that the normal component of the generalized acceleration is now
      // equal to the negated gravitational acceleration.
      VectorX<double> ga;
      solver_.ComputeGeneralizedAcceleration(*vel_data_, v0, cf, dt_, &ga);
      EXPECT_NEAR(ga[1], -grav_accel, eps_);
    }
  }

  // Tests the rod in an upright sliding configuration with sliding velocity as
  // specified. The rod will be constrained to prevent rotational acceleration
  // using a bilateral constraint as well.
  void SlidingPlusBilateral(bool sliding_to_right, bool use_lcp_solver) {
      SetRodToRestingVerticalConfig();
    ContinuousState<double>& xc = context_->get_mutable_continuous_state();
    xc[3] = (sliding_to_right) ? 1 : -1;

    // Set the coefficient of friction. A nonzero coefficient of friction should
    // cause the rod to rotate.
    rod_->set_mu_coulomb(0.1);

    // Compute the problem data.
    CalcConstraintAccelProblemData(accel_data_.get());
    accel_data_->use_complementarity_problem_solver = use_lcp_solver;

    // Add in bilateral constraints on rotational motion.
    accel_data_->kG.setZero(1);    // No right hand side term.
    accel_data_->G_mult = [](const VectorX<double>& v) -> VectorX<double> {
      VectorX<double> result(1);   // Only one constraint.

      // Constrain the angular velocity (and hence angular acceleration) to be
      // zero.
      result[0] = v[2];
      return result;
    };
    accel_data_->G_transpose_mult =
        [this](const VectorX<double>& f) -> VectorX<double> {
      // A force (torque) applied to the third component needs no
      // transformation.
      DRAKE_DEMAND(f.size() == 1);
      VectorX<double> result(get_rod_num_coordinates());
      result.setZero();
      result[2] = f[0];
      return result;
    };

    // Compute the contact forces.
    VectorX<double> cf;
    solver_.SolveConstraintProblem(*accel_data_, &cf);

    // Get the contact tangent velocities.
    std::vector<Vector2d> contacts;
    std::vector<double> tangent_vels;
    rod_->GetContactPoints(*context_, &contacts);
    rod_->GetContactPointsTangentVelocities(*context_, contacts, &tangent_vels);

    // Construct the contact frame(s).
    std::vector<Matrix2<double>> frames;
    frames.push_back(
        GetSlidingContactFrameToWorldTransform(tangent_vels.front()));

    // Get the contact forces expressed in the contact frame.
    std::vector<Vector2<double>> contact_forces;
    ConstraintSolver<double>::CalcContactForcesInContactFrames(
        cf, *accel_data_, frames, &contact_forces);

    // Verify that the number of contact force vectors is correct.
    ASSERT_EQ(contact_forces.size(), frames.size());

    // Verify that there are no non-sliding frictional forces.
    EXPECT_TRUE(accel_data_->non_sliding_contacts.empty());
    const int num_contacts = accel_data_->sliding_contacts.size();
    const int num_bilateral_eqns = accel_data_->kG.size();
    EXPECT_EQ(cf.size(), num_contacts + num_bilateral_eqns);

    // Verify that the normal contact forces exactly oppose gravity and the
    // friction forces are of the appropriate size. Frictional forces must
    // always be negative- it is the contact frame that can change.
    const double mg = std::fabs(rod_->get_gravitational_acceleration()) *
        rod_->get_rod_mass();
    double fN = 0, fF = 0;
    for (int i = 0; i < static_cast<int>(contact_forces.size()); ++i) {
      fN += contact_forces[i][0];
      fF += contact_forces[i][1];
    }
    EXPECT_NEAR(fN, mg, eps_);
    EXPECT_LT(fF, 0);
    EXPECT_NEAR(fF, -mg * rod_->get_mu_coulomb(), eps_);

    // Get the generalized acceleration and verify that there is no angular
    // acceleration.
    VectorX<double> ga;
    solver_.ComputeGeneralizedAcceleration(*accel_data_, cf, &ga);
    EXPECT_LT(ga[2], eps_ * cf.size());

    // Indicate through modification of the kG term that the system already has
    // angular velocity (which violates our desire to constrain the
    // orientation) and solve again.
    accel_data_->kG[0] = 1.0;    // Indicate a ccw angular motion..
    solver_.SolveConstraintProblem(*accel_data_, &cf);
    solver_.ComputeGeneralizedAcceleration(*accel_data_, cf, &ga);
    EXPECT_NEAR(ga[2], -accel_data_->kG[0], eps_ * cf.size());
  }

  // Tests the rod in an upright sliding and impacting state, with sliding
  // velocity as specified. The rod will be constrained to prevent rotational
  // velocity using a bilateral constraint as well.
  void SlidingPlusBilateralImpact(bool sliding_to_right) {
    SetRodToSlidingImpactingVerticalConfig(sliding_to_right);

    // Set the coefficient of friction. A nonzero coefficient of friction should
    // cause the rod to rotate.
    rod_->set_mu_coulomb(0.1);

    // Compute the problem data.
    CalcConstraintProblemDataForImpact(vel_data_.get());

    // Compute the generalized velocity.
    const VectorX<double> v0 = vel_data_->solve_inertia(vel_data_->Mv);

    // Add in bilateral constraints on rotational motion.
    vel_data_->kG.setZero(1);    // No right hand side term.
    vel_data_->G_mult = [](const VectorX<double>& w) -> VectorX<double> {
      VectorX<double> result(1);   // Only one constraint.

      // Constrain the angular velocity to be zero.
      result[0] = w[2];
      return result;
    };
    vel_data_->G_transpose_mult =
        [this](const VectorX<double>& f) -> VectorX<double> {
          // An impulsive force (torque) applied to the third component needs no
          // transformation.
          DRAKE_DEMAND(f.size() == 1);
          VectorX<double> result(get_rod_num_coordinates());
          result.setZero();
          result[2] = f[0];
          return result;
        };

    // Compute the impact forces.
    VectorX<double> cf;
    solver_.SolveImpactProblem(*vel_data_, &cf);

    // Construct the contact frame(s).
    std::vector<Matrix2<double>> frames;
    frames.push_back(GetNonSlidingContactFrameToWorldTransform());

    // Get the contact impulses expressed in the contact frame.
    std::vector<Vector2<double>> contact_forces;
    ConstraintSolver<double>::CalcContactForcesInContactFrames(
        cf, *vel_data_, frames, &contact_forces);

    // Verify that the number of contact force vectors is correct.
    ASSERT_EQ(contact_forces.size(), frames.size());

    // Verify that there are no non-sliding frictional forces.
    const int num_contacts = vel_data_->mu.size();
    const int num_bilateral_eqns = vel_data_->kG.size();
    EXPECT_EQ(cf.size(), num_contacts * 2 + num_bilateral_eqns);

    // Get the pre-impact vertical momentum.
    ContinuousState<double>& xc = context_->get_mutable_continuous_state();
    const double mv = rod_->get_rod_mass() * xc[4];

    // Verify that the normal contact impulses exactly oppose the pre-impact
    double fN = 0, fF = 0;
    for (int i = 0; i < static_cast<int>(contact_forces.size()); ++i) {
      fN += contact_forces[i][0];
      fF += contact_forces[i][1];
    }
    const double sign = (sliding_to_right) ? 1 : -1;
    EXPECT_NEAR(fN, -mv, eps_);
    EXPECT_NEAR(fF, sign * mv * rod_->get_mu_coulomb(), eps_);

    // Get the change in generalized velocity and verify that there is no
    // angular velocity.
    VectorX<double> gv;
    solver_.ComputeGeneralizedVelocityChange(*vel_data_, cf, &gv);
    EXPECT_LT((v0[2] + gv[2]), eps_ * cf.size());

    // Indicate through modification of the kG term that the system already has
    // angular orientation (which violates our desire to keep the rod at
    // zero rotation) and solve again.
    vel_data_->kG[0] = 1.0;    // Indicate a ccw orientation..
    solver_.SolveImpactProblem(*vel_data_, &cf);
    solver_.ComputeGeneralizedVelocityChange(*vel_data_, cf, &gv);
    EXPECT_NEAR(v0[2] + gv[2], -vel_data_->kG[0],
                eps_ * cf.size());
  }

  // Tests the rod in an upright sliding state, with sliding
  // direction as specified. The rod will be constrained to prevent rotational
  // acceleration using a bilateral constraint as well.
  void SlidingPlusBilateralDiscretized(bool sliding_to_right) {
    using std::max;

    SetRodToRestingVerticalConfig();

    // Set the coefficient of friction. A nonzero coefficient of friction should
    // cause the rod to rotate.
    rod_->set_mu_coulomb(0.1);

    // Compute the problem data with no duplicated contacts or friction
    // directions.
    CalcDiscreteTimeProblemData(dt_, vel_data_.get());

    // Update Mv to slide to the given direction and to account for
    // gravitational force.
    const double rod_mass = rod_->get_rod_mass();
    vel_data_->Mv[0] += ((sliding_to_right) ? 1.0 : -1.0) * rod_mass;

    // Add in bilateral constraints on rotational motion.
    vel_data_->kG.setZero(1);    // No right hand side term.
    vel_data_->G_mult = [](const VectorX<double>& w) -> VectorX<double> {
      VectorX<double> result(1);   // Only one constraint.
      // Constrain the angular velocity to be zero.
      result[0] = w[2];
      return result;
    };
    vel_data_->G_transpose_mult =
        [this](const VectorX<double>& f) -> VectorX<double> {
          // A force (torque) applied to the third component needs no
          // transformation.
          DRAKE_DEMAND(f.size() == 1);
          VectorX<double> result(get_rod_num_coordinates());
          result.setZero();
          result[2] = f[0];
          return result;
        };

    // Solve the discretization problem.
    VectorX<double> cf;
    SolveDiscretizationProblem(*vel_data_, dt_, &cf);

    // cf should be 3-dimensional. First dimension is normal force (along
    // global +y), second dimension is frictional force (along +x),
    // third dimension is constraint force (along +theta). Since the bilateral
    // constraint should prevent angular motion, the constraint force (cf[2])
    // should counteract the rotation that would be induced by the frictional
    // force (cf[1]).
    ASSERT_EQ(cf.size(), 3);
    const double zero_tol = 10 * std::numeric_limits<double>::epsilon();
    EXPECT_NEAR(cf[1], -cf[2], zero_tol);

    // Construct the contact frame(s).
    std::vector<Matrix2<double>> frames;
    frames.push_back(GetNonSlidingContactFrameToWorldTransform());

    // Get the contact forces expressed in the contact frame.
    std::vector<Vector2<double>> contact_forces;
    ConstraintSolver<double>::CalcContactForcesInContactFrames(
        cf, *vel_data_, frames, &contact_forces);

    // Verify that the number of contact force vectors is correct.
    ASSERT_EQ(contact_forces.size(), frames.size());

    // Verify that there are no non-sliding frictional forces.
    const int num_contacts = vel_data_->mu.size();
    const int num_bilateral_eqns = vel_data_->kG.size();
    EXPECT_EQ(cf.size(), num_contacts * 2 + num_bilateral_eqns);

    // Determine the force that should be acting on the system.
    const double fdown = -rod_->get_gravitational_acceleration() * rod_mass;

    // Verify that the normal contact forces exactly oppose the discretized
    // force.
    double fN = 0, fF = 0;
    for (int i = 0; i < static_cast<int>(contact_forces.size()); ++i) {
      fN += contact_forces[i][0];
      fF += contact_forces[i][1];
    }
    const double sign = (sliding_to_right) ? 1 : -1;
    EXPECT_NEAR(fN, fdown, eps_);
    EXPECT_NEAR(fF, sign * -fdown * rod_->get_mu_coulomb(), eps_);

    // Get the generalized acceleration from the constraint forces and verify
    // that the moment is zero.
    const VectorX<double> v0 = rod_->GetRodVelocity(*context_);
    VectorX<double> ga;
    solver_.ComputeGeneralizedAcceleration(*vel_data_, v0, cf, dt_, &ga);
    EXPECT_LT(ga[2], eps_ * cf.size());

    // Indicate through modification of the kG term that the system already has
    // angular orientation (which violates our desire to keep the rod at
    // zero rotation) and solve again.
    vel_data_->kG[0] = 1.0;    // Indicate a ccw orientation..
    SolveDiscretizationProblem(*vel_data_, dt_, &cf);

    // Compute the generalized acceleration.
    solver_.ComputeGeneralizedAcceleration(*vel_data_, v0, cf, dt_, &ga);
    EXPECT_NEAR(ga[2] * dt_, -vel_data_->kG[0], eps_ * cf.size());
  }

  // Tests the rod in a one-point sliding contact configuration with a second
  // constraint that prevents horizontal acceleration. This test tests the
  // interaction between contact and limit constraints.
  void OnePointPlusLimit(bool use_lcp_solver) {
    // Set the state of the rod to vertically-at-rest and sliding to the left.
    // Set the state of the rod to resting on its side with horizontal velocity.
    SetRodToRestingHorizontalConfig();
    ContinuousState<double>& xc = context_->
      get_mutable_continuous_state();
    xc[3] = 1.0;

    // Set the coefficient of friction to somewhat small (to limit the sliding
    // force).
    rod_->set_mu_coulomb(1e-1);

    // Get the gravitational acceleration.
    const double grav_accel = rod_->get_gravitational_acceleration();

    // First, construct the acceleration-level problem data as normal to set
    // inertia solver and external forces.
    CalcConstraintAccelProblemData(accel_data_.get());
    accel_data_->use_complementarity_problem_solver = use_lcp_solver;

    // Get the original N and Nᵀ - μQᵀ
    const int ngc = get_rod_num_coordinates();
    const int num_old_contacts = 2;
    MatrixX<double> N(num_old_contacts, ngc);
    MatrixX<double> N_minus_muQ_transpose(ngc, num_old_contacts);
    for (int i = 0; i < num_old_contacts; ++i) {
      N_minus_muQ_transpose.col(i) = accel_data_->N_minus_muQ_transpose_mult(
        VectorX<double>::Unit(2, i));
    }
    for (int i = 0; i < ngc; ++i)
      N.col(i) = accel_data_->N_mult(VectorX<double>::Unit(ngc, i));

    // Construct the problem as a limit constraint preventing movement in the
    // downward direction.
    accel_data_->sliding_contacts.resize(1);
    accel_data_->mu_sliding.resize(1);
    accel_data_->N_mult = [&N](const VectorX<double>& v) {
      return N.row(0) * v;
    };
    accel_data_->kN.setZero(1);
    accel_data_->kL.setZero(1);
    accel_data_->N_minus_muQ_transpose_mult =
        [&N_minus_muQ_transpose](const VectorX<double>& l) {
      return N_minus_muQ_transpose.col(0) * l;
    };

    // Set the Jacobian entry- in this case, the limit is a lower limit on the
    // second coordinate (vertical position).
    const int num_limits = 1;
    accel_data_->L_mult = [&N](const VectorX<double>& v) -> VectorX<double> {
      return N.row(1) * v;
    };
    accel_data_->L_transpose_mult = [&N](const VectorX<double>& v) ->
      VectorX<double> {
        return N.row(1).transpose() * v;
    };
    accel_data_->kL.setZero(num_limits);

    // Case 1: set kN and kL terms to counteract gravity, which should prevent
    // any constraint forces from being applied.
    {
      accel_data_->kN.setOnes() *= -grav_accel;
      accel_data_->kL.setOnes() *= -grav_accel;

      // Compute the constraint forces and verify that none are applied.
      VectorX<double> cf;
      solver_.SolveConstraintProblem(*accel_data_, &cf);
      EXPECT_LT(cf.norm(), eps_);
    }

    // Case 2: reset kN and kL and recompute constraint forces.
    {
      accel_data_->kN.setZero();
      accel_data_->kL.setZero();
      VectorX<double> cf;
      solver_.SolveConstraintProblem(*accel_data_, &cf);

      // Verify the size of cf is as expected.
      const int num_contacts = 1;
      EXPECT_EQ(cf.size(), num_contacts + num_limits);

      // Verify that the vertical acceleration is zero. If the cross-constraint
      // term LM⁻¹(Nᵀ - μQᵀ) is not computed properly, this acceleration might
      // not be zero. Note that μQᵀ will not have any effect here.
      VectorX<double> vdot;
      solver_.ComputeGeneralizedAcceleration(*accel_data_, cf, &vdot);
      EXPECT_NEAR(vdot[1], 0, eps_);
    }

    // Case 3: set kN and kL terms to effectively double gravity, which should
    // cause the rod to accelerate upward.
    {
      accel_data_->kN.setOnes() *= grav_accel;
      accel_data_->kL.setOnes() *= grav_accel;
      VectorX<double> cf;
      solver_.SolveConstraintProblem(*accel_data_, &cf);
      VectorX<double> vdot;
      solver_.ComputeGeneralizedAcceleration(*accel_data_, cf, &vdot);
      EXPECT_NEAR(vdot[1], -grav_accel, eps_);
    }
  }

  // Tests the rod in a one-point sliding contact configuration with a second
  // constraint that prevents horizontal acceleration. This test tests the
  // interaction between contact and limit constraints.
  void OnePointPlusLimitDiscretized() {
    // Set the state of the rod to resting on its side with horizontal velocity.
    SetRodToRestingHorizontalConfig();
    ContinuousState<double>& xc = context_->
        get_mutable_continuous_state();
    xc[3] = 1.0;

    // Set the coefficient of friction to somewhat small (to limit the sliding
    // force).
    rod_->set_mu_coulomb(1e-1);

    // Get the gravitational acceleration.
    const double grav_accel = rod_->get_gravitational_acceleration();

    // First, construct the acceleration-level problem data as normal to set
    // inertia solver and external forces.
    CalcDiscreteTimeProblemData(dt_, vel_data_.get());

    // Get the original N
    const int ngc = get_rod_num_coordinates();
    const int num_old_contacts = 2;
    MatrixX<double> N(num_old_contacts, ngc);
    MatrixX<double> N_transpose(ngc, num_old_contacts);
    for (int i = 0; i < num_old_contacts; ++i) {
      N_transpose.col(i) = vel_data_->N_transpose_mult(
          VectorX<double>::Unit(2, i));
    }
    for (int i = 0; i < ngc; ++i)
      N.col(i) = vel_data_->N_mult(VectorX<double>::Unit(ngc, i));

    // Get the original F.
    const int num_friction_dirs = 1;
    MatrixX<double> F(num_old_contacts * num_friction_dirs, ngc);
    MatrixX<double> F_transpose(ngc, num_old_contacts * num_friction_dirs);
    for (int i = 0; i < num_old_contacts * num_friction_dirs; ++i) {
      F_transpose.col(i) = vel_data_->F_transpose_mult(
          VectorX<double>::Unit(num_old_contacts * num_friction_dirs, i));
    }
    for (int i = 0; i < ngc; ++i)
      F.col(i) = vel_data_->F_mult(VectorX<double>::Unit(ngc, i));

    // Construct the problem as a limit constraint preventing movement in the
    // downward direction.
    vel_data_->mu.setZero(1);
    vel_data_->r = {1};
    vel_data_->N_mult = [&N](const VectorX<double>& v) {
      return N.row(0) * v;
    };
    vel_data_->F_mult = [&F](const VectorX<double>& v) {
      return F.row(0) * v;
    };
    vel_data_->kN.setZero(1);
    vel_data_->kF.setZero(1);
    vel_data_->gammaN.setZero(1);
    vel_data_->gammaF.setZero(1);
    vel_data_->gammaE.setZero(1);
    vel_data_->kL.setZero(1);
    vel_data_->gammaL.setZero(1);
    vel_data_->N_transpose_mult =
        [&N_transpose](const VectorX<double>& l) {
          return N_transpose.col(0) * l;
        };
    vel_data_->F_transpose_mult =
        [&F_transpose](const VectorX<double>& l) {
          return F_transpose.col(0) * l;
        };

    // Set the Jacobian entry- in this case, the limit is a lower limit on the
    // second coordinate (vertical position).
    const int num_limits = 1;
    vel_data_->L_mult = [&N](const VectorX<double>& v) -> VectorX<double> {
      return N.row(1) * v;
    };
    vel_data_->L_transpose_mult = [&N](const VectorX<double>& v) ->
        VectorX<double> {
      return N.row(1).transpose() * v;
    };
    vel_data_->kL.setZero(num_limits);

    // Case 1: set kN and kL terms to counteract gravity, which should prevent
    // any constraint forces from being applied.
    {
      vel_data_->kN.setOnes() *= -grav_accel * dt_;
      vel_data_->kL.setOnes() *= -grav_accel * dt_;

      // Compute the constraint forces and verify that none are applied.
      VectorX<double> cf;
      SolveDiscretizationProblem(*vel_data_, dt_, &cf);
      EXPECT_LT(cf.norm(), eps_);
    }

    // Case 2: reset kN and kL and recompute constraint forces.
    {
      vel_data_->kN.setZero();
      vel_data_->kL.setZero();
      VectorX<double> cf;
      SolveDiscretizationProblem(*vel_data_, dt_, &cf);

      // Verify the size of cf is as expected.
      const int num_contacts = 1;
      EXPECT_EQ(cf.size(), num_contacts + num_friction_dirs * num_contacts +
          num_limits);

      // Verify that the vertical acceleration is zero. If the cross-constraint
      // term LM⁻¹Nᵀ is not computed properly, this acceleration might not
      // be zero.
      const VectorX<double> v0 = rod_->GetRodVelocity(*context_);
      VectorX<double> vdot;
      solver_.ComputeGeneralizedAcceleration(*vel_data_, v0, cf, dt_, &vdot);
      EXPECT_NEAR(vdot[1], 0, eps_);
    }

    // Case 3: set kN and kL terms to effectively double gravity, which should
    // cause the rod to accelerate upward.
    {
      vel_data_->kN.setOnes() *= grav_accel * dt_;
      vel_data_->kL.setOnes() *= grav_accel * dt_;
      VectorX<double> cf;
      SolveDiscretizationProblem(*vel_data_, dt_, &cf);
      const VectorX<double> v0 = rod_->GetRodVelocity(*context_);
      VectorX<double> vdot;
      solver_.ComputeGeneralizedAcceleration(*vel_data_, v0, cf, dt_, &vdot);
      EXPECT_NEAR(vdot[1], -grav_accel, eps_);
    }
  }

  // Tests the rod in a two-point contact configuration with both sticking and
  // sliding contacts. This test tests that the cross-term interaction between
  // sliding friction forces and non-sliding friction forces constraints is
  // correct.
  void TwoPointContactCrossTerms(bool use_lcp_solver) {
    // Set the state of the rod to resting.
    SetRodToRestingHorizontalConfig();

    // Set the sliding coefficient of friction to somewhat small and the static
    // coefficient of friction to very large.
    rod_->set_mu_coulomb(1e-1);
    rod_->set_mu_static(1.0);

    // First, construct the acceleration-level problem data as usual to set
    // inertia solver and external forces.
    std::vector<Vector2d> contacts;
    std::vector<double> tangent_vels;
    rod_->GetContactPoints(*context_, &contacts);
    rod_->GetContactPointsTangentVelocities(*context_, contacts, &tangent_vels);

    // Modify the tangent velocity on the left contact to effect a sliding
    // contact. This modification can be imagined as the left end of the rod
    // is touching a conveyor belt moving to the right.
    tangent_vels[0] = 1.0;

    // Compute the constraint problem data.
    rod_->CalcConstraintProblemData(
      *context_, contacts, tangent_vels, accel_data_.get());
    accel_data_->use_complementarity_problem_solver = use_lcp_solver;

    // Check the consistency of the data.
    CheckProblemConsistency(*accel_data_, contacts.size());

    // Compute the constraint forces. Note that we increase cfm to prevent the
    // occasional "failure to solve LCP" exception.
    VectorX<double> cf;
    solver_.SolveConstraintProblem(*accel_data_, &cf);

    // Verify the size of cf is as expected.
    EXPECT_EQ(cf.size(), accel_data_->sliding_contacts.size() +
                         accel_data_->non_sliding_contacts.size() * 2);

    // Verify that the horizontal acceleration is zero (since mu_static is so
    // large, meaning that the sticking friction force is able to overwhelm the
    // sliding friction force. If the cross-constraint term FM⁻¹(Nᵀ - μQᵀ) is
    // not computed properly, this acceleration might not be zero.
    VectorX<double> vdot;
    solver_.ComputeGeneralizedAcceleration(*accel_data_, cf, &vdot);
    EXPECT_NEAR(vdot[0], 0, eps_);
  }

  // Tests that the cross-term interaction between contact forces and generic
  // unilateral constraints is computed correctly at the acceleration level.
  void ContactLimitCrossTermAccel(bool use_lcp_solver) {
    // Set the state of the rod to resting.
    SetRodToRestingHorizontalConfig();

    // Set the sliding coefficient of friction to zero (it won't be used) and
    // the static coefficient of friction to a relatively large value.
    rod_->set_mu_coulomb(0.0);
    rod_->set_mu_static(1.0);

    // First, construct the acceleration-level problem data as normal to set
    // inertia solver and external forces.
    std::vector<Vector2d> contacts;
    std::vector<double> tangent_vels;
    rod_->GetContactPoints(*context_, &contacts);
    rod_->GetContactPointsTangentVelocities(*context_, contacts, &tangent_vels);

    // Compute the constraint problem data.
    rod_->CalcConstraintProblemData(
      *context_, contacts, tangent_vels, accel_data_.get());
    accel_data_->use_complementarity_problem_solver = use_lcp_solver;

    // Add some horizontal force.
    accel_data_->tau[0] = 1.0;

    // Construct the problem as a limit constraint preventing movement in the
    // upward direction.
    const int ngc = get_rod_num_coordinates();
    const int num_generic_unilateral_constraints = 1;
    accel_data_->kL.resize(num_generic_unilateral_constraints);

    // Set the Jacobian entry- in this case, the limit is an upper limit on the
    // second coordinate (vertical position). The constraint is: v̇₂ ≤ 0, which
    // we transform to the form: -v̇₂ ≥ 0 (explaining the provenance of the minus
    // sign in L).
    const int num_limit_constraints = 1;
    MatrixX<double> L(accel_data_->kL.size(), ngc);
    L.setZero();
    L(0, 1) = -1;
    accel_data_->L_mult = [&L](const VectorX<double>& v) -> VectorX<double> {
      return L * v;
    };
    accel_data_->L_transpose_mult = [&L](const VectorX<double>& v) ->
      VectorX<double> {
      return L.transpose() * v;
    };
    accel_data_->kL.setZero(num_limit_constraints);

    // Check the consistency of the data.
    CheckProblemConsistency(*accel_data_, contacts.size());

    // Compute the constraint forces.
    VectorX<double> cf;
    solver_.SolveConstraintProblem(*accel_data_, &cf);

    // Verify the size of cf is as expected.
    EXPECT_EQ(cf.size(), accel_data_->non_sliding_contacts.size() * 2 + 1);

    // Verify that the horizontal and vertical acceleration of the rod c.o.m.
    // is zero.
    VectorX<double> vdot;
    solver_.ComputeGeneralizedAcceleration(*accel_data_, cf, &vdot);
    EXPECT_NEAR(vdot[0], 0, eps_);
    EXPECT_NEAR(vdot[1], 0, eps_);
  }

  // Tests the rod in a two-point contacting configuration *realized through
  // a configuration limit constraint*. No frictional forces are applied, so
  // any velocity projections along directions other than the contact normal
  // will be irrelevant.
  void TwoPointAsLimit(bool use_lcp_solver) {
    // Set the state of the rod to resting on its side.
    SetRodToRestingHorizontalConfig();

    // First, construct the acceleration-level problem data as normal to set
    // inertia solver and external forces.
    CalcConstraintAccelProblemData(accel_data_.get());
    accel_data_->use_complementarity_problem_solver = use_lcp_solver;

    // Construct the problem as a limit constraint preventing movement in the
    // downward direction.
    const int ngc = get_rod_num_coordinates();
    accel_data_->sliding_contacts.resize(0);
    accel_data_->non_sliding_contacts.resize(0);
    accel_data_->mu_sliding.resize(0);
    accel_data_->mu_non_sliding.resize(0);
    accel_data_->r.resize(0);
    accel_data_->N_mult = [](const VectorX<double>&) {
      return VectorX<double>(0);
    };
    accel_data_->kN.resize(0);
    accel_data_->F_mult = [](const VectorX<double>&) {
      return VectorX<double>(0);
    };
    accel_data_->F_transpose_mult = [ngc](const VectorX<double>&) {
      return VectorX<double>::Zero(ngc);
    };
    accel_data_->kF.resize(0);
    accel_data_->kL.resize(1);
    accel_data_->N_minus_muQ_transpose_mult = [ngc](const VectorX<double>&) {
      return VectorX<double>::Zero(ngc);
    };

    // Set the Jacobian entry- in this case, the limit is a lower limit on the
    // second coordinate (vertical position).
    const int num_limit_constraints = 1;
    MatrixX<double> L(accel_data_->kL.size(), ngc);
    L.setZero();
    L(0, 1) = 1;
    accel_data_->L_mult = [&L](const VectorX<double>& v) -> VectorX<double> {
      return L * v;
    };
    accel_data_->L_transpose_mult = [&L](const VectorX<double>& v) ->
      VectorX<double> {
      return L.transpose() * v;
    };
    accel_data_->kL.setZero(num_limit_constraints);

    // Compute the constraint forces.
    VectorX<double> cf;
    solver_.SolveConstraintProblem(*accel_data_, &cf);

    // Verify the size of cf is as expected.
    EXPECT_EQ(cf.size(), 1);

    // Verify that the normal force exactly opposes gravity.
    const double mg = std::fabs(rod_->get_gravitational_acceleration()) *
        rod_->get_rod_mass();
    EXPECT_NEAR(cf[0], mg, eps_);

    // Set the Jacobian entry- in this case, the limit is an upper limit on the
    // second coordinate (vertical position).
    L *= -1;

    // Reverse the external force (gravity) on the rod. tau was set by the
    // call to Rod2D::CalcConstraintProblemData().
    accel_data_->tau *= -1;

    // Recompute the constraint forces, and verify that they're still equal
    // to the force from gravity. Note: if the forces were to be applied to the
    // rod, one will need to compute Lᵀcf[0] to obtain the generalized force;
    // this is how we can handle upper and lower limits with only non-negativity
    // constraints.
    solver_.SolveConstraintProblem(*accel_data_, &cf);
    EXPECT_EQ(cf.size(), 1);
    EXPECT_NEAR(cf[0], mg, eps_);

    // Verify that the vertical acceleration is zero.
    VectorX<double> vdot;
    solver_.ComputeGeneralizedAcceleration(*accel_data_, cf, &vdot);
    EXPECT_NEAR(vdot[1], 0, eps_);
  }

  // Tests the rod in a two-point contacting configuration *realized through
  // a configuration limit constraint*. No frictional forces are applied, so
  // any velocity projections along directions other than the contact normal
  // will be irrelevant.
  void TwoPointAsLimitDiscretized() {
    // Set the state of the rod to resting on its side.
    SetRodToRestingHorizontalConfig();

    // First, construct the acceleration-level problem data as normal to set
    // inertia solver and external forces.
    CalcDiscreteTimeProblemData(dt_, vel_data_.get());

    // Construct the problem as a limit constraint preventing movement in the
    // downward direction.
    const int ngc = get_rod_num_coordinates();
    vel_data_->mu.resize(0);
    vel_data_->r.resize(0);
    vel_data_->N_mult = [](const VectorX<double>&) {
      return VectorX<double>(0);
    };
    vel_data_->kN.resize(0);
    vel_data_->gammaN.resize(0);
    vel_data_->F_mult = [](const VectorX<double>&) {
      return VectorX<double>(0);
    };
    vel_data_->F_transpose_mult = [ngc](const VectorX<double>&) {
      return VectorX<double>::Zero(ngc);
    };
    vel_data_->kF.resize(0);
    vel_data_->gammaF.resize(0);
    vel_data_->gammaE.resize(0);
    vel_data_->kL.resize(1);
    vel_data_->gammaL.setZero(1);
    vel_data_->N_transpose_mult = [ngc](const VectorX<double>&) {
      return VectorX<double>::Zero(ngc);
    };

    // Set the Jacobian entry- in this case, the limit is a lower limit on the
    // second coordinate (vertical position).
    const int num_limit_constraints = 1;
    MatrixX<double> L(vel_data_->kL.size(), ngc);
    L.setZero();
    L(0, 1) = 1;
    vel_data_->L_mult = [&L](const VectorX<double>& v) -> VectorX<double> {
      return L * v;
    };
    vel_data_->L_transpose_mult = [&L](const VectorX<double>& v) ->
        VectorX<double> {
      return L.transpose() * v;
    };
    vel_data_->kL.setZero(num_limit_constraints);

    // Compute the constraint forces.
    VectorX<double> cf;
    SolveDiscretizationProblem(*vel_data_, dt_, &cf);

    // Verify the size of cf is as expected.
    EXPECT_EQ(cf.size(), 1);

    // Verify that the normal force exactly opposes gravity.
    const double mg = std::fabs(rod_->get_gravitational_acceleration()) *
        rod_->get_rod_mass();
    EXPECT_NEAR(cf[0], mg, eps_);

    // Set the Jacobian entry- in this case, the limit is an upper limit on the
    // second coordinate (vertical position).
    L *= -1;

    // Reverse the external force (gravity) on the rod. tau was set by the
    // call to Rod2D::CalcConstraintProblemData().
    vel_data_->Mv *= -1;

    // Recompute the constraint forces, and verify that they're still equal
    // to the force from gravity. Note: if the forces were to be applied to the
    // rod, one will need to compute Lᵀcf[0] to obtain the generalized force;
    // this is how we can handle upper and lower limits with only non-negativity
    // constraints.
    SolveDiscretizationProblem(*vel_data_, dt_, &cf);
    EXPECT_EQ(cf.size(), 1);
    EXPECT_NEAR(cf[0], mg, eps_);

    // Verify that the vertical acceleration is zero.
    const VectorX<double> v0 = rod_->GetRodVelocity(*context_);
    VectorX<double> vdot;
    solver_.ComputeGeneralizedAcceleration(*vel_data_, v0, cf, dt_, &vdot);
    EXPECT_NEAR(vdot[1], 0, eps_);
  }

 private:
  // The timestep size for discretization. Note: if the timestep is too
  // large, the accuracy might be too low for the necessary effect to emerge).
  // But we also want the step size to be large enough to test robustness. The
  // selected value should be a good compromise.
  const double dt_{1e-4};
};

// Tests the rod in single-point sticking configurations.
TEST_F(Constraint2DSolverTest, SinglePointStickingBothSigns) {
  // Test sticking with applied force to the right (true) and the left (false).
  SinglePointSticking(kForceAppliedToRight);
  SinglePointSticking(kForceAppliedToLeft);
  SinglePointStickingDiscretized(kForceAppliedToRight);
  SinglePointStickingDiscretized(kForceAppliedToLeft);
}

// Tests the rod in a two-point sticking configurations.
TEST_F(Constraint2DSolverTest, TwoPointStickingSign) {
  // Test sticking with applied force to the right and the left, and with both
  // the LCP solver and the linear system solver.
  TwoPointSticking(kForceAppliedToRight, kLCPSolver);
  TwoPointSticking(kForceAppliedToLeft, kLCPSolver);
  TwoPointSticking(kForceAppliedToRight, kLinearSystemSolver);
  TwoPointSticking(kForceAppliedToLeft, kLinearSystemSolver);
}

// Tests the rod in two-point non-sliding configurations that will transition
// to sliding.
TEST_F(Constraint2DSolverTest, TwoPointNonSlidingToSlidingSign) {
  // Test sticking with applied force to the right (true) and the left (false).
  TwoPointNonSlidingToSliding(kForceAppliedToRight);
  TwoPointNonSlidingToSliding(kForceAppliedToLeft);
  TwoPointNonSlidingToSlidingDiscretized(kForceAppliedToRight);
  TwoPointNonSlidingToSlidingDiscretized(kForceAppliedToLeft);
}

// Tests the rod in a two-point impact which is insufficient to put the rod
// into stiction, with pre-impact velocity in two directions (right = true,
// left = false).
TEST_F(Constraint2DSolverTest, TwoPointImpactNoTransitionToStictionTest) {
  TwoPointImpactNoTransitionToStiction(kSlideRight);
  TwoPointImpactNoTransitionToStiction(kSlideLeft);
}

// Tests the rod in a two-point impacting and sticking configuration with
// pre-impact velocity to the right (true) or left (false).
TEST_F(Constraint2DSolverTest, TwoPointImpactingAndStickingTest) {
  TwoPointImpactingAndSticking(kSlideRight);
  TwoPointImpactingAndSticking(kSlideLeft);
}

// Tests the rod in a two-point sliding configuration, both to the right
// and to the left and using both the LCP and linear system solvers.
TEST_F(Constraint2DSolverTest, TwoPointSlidingTest) {
  Sliding(kSlideRight, false /* not upright */, kLCPSolver);
  Sliding(kSlideLeft, false /* not upright */, kLCPSolver);
  Sliding(kSlideRight, false /* not upright */, kLinearSystemSolver);
  Sliding(kSlideLeft, false /* not upright */, kLinearSystemSolver);
  SlidingDiscretized(kSlideRight, false /* not upright */);
  SlidingDiscretized(kSlideLeft, false /* not upright */);
}

// Tests the rod in a single point sliding configuration, with sliding both
// to the right and to the left and using both the LCP and linear system
// solvers.
TEST_F(Constraint2DSolverTest, SinglePointSlidingTest) {
  Sliding(kSlideRight, true /* upright */, kLCPSolver);
  Sliding(kSlideLeft, true /* upright */, kLCPSolver);
  Sliding(kSlideRight, true /* upright */, kLinearSystemSolver);
  Sliding(kSlideLeft, true /* upright */, kLinearSystemSolver);
  SlidingDiscretized(kSlideRight, true /* upright */);
  SlidingDiscretized(kSlideLeft, true /* upright */);
}

// Tests the rod in a single point sliding configuration, with sliding both
// to the right and to the left, and with a bilateral constraint imposed, and
// using both the LCP and linear system solvers.
TEST_F(Constraint2DSolverTest, SinglePointSlidingPlusBilateralTest) {
  SlidingPlusBilateral(kSlideRight, kLCPSolver);
  SlidingPlusBilateral(kSlideLeft, kLCPSolver);
  SlidingPlusBilateral(kSlideRight, kLinearSystemSolver);
  SlidingPlusBilateral(kSlideLeft, kLinearSystemSolver);
  SlidingPlusBilateralDiscretized(kSlideRight);
  SlidingPlusBilateralDiscretized(kSlideLeft);
}

// Tests the rod in a single point impacting configuration, with sliding both
// to the right and to the left, and with a bilateral constraint imposed.
TEST_F(Constraint2DSolverTest, SinglePointSlidingImpactPlusBilateralTest) {
  SlidingPlusBilateralImpact(kSlideRight);
  SlidingPlusBilateralImpact(kSlideLeft);
}

// Tests the rod in a one-point sliding contact configuration with a second
// constraint that prevents horizontal acceleration. This test tests the
// interaction between contact and limit constraints using both the LCP solver
// and the linear system solver.
TEST_F(Constraint2DSolverTest, OnePointPlusLimitTest) {
  OnePointPlusLimit(kLCPSolver);
  OnePointPlusLimit(kLinearSystemSolver);
  OnePointPlusLimitDiscretized();
}

// Tests the rod in a two-point contact configuration with both sticking and
// sliding contacts and using both the LCP and linear system solvers. This test
// tests that the cross-term interaction between sliding friction forces and
// non-sliding friction forces constraints is correct.
TEST_F(Constraint2DSolverTest, TwoPointContactCrossTermsTest) {
  TwoPointContactCrossTerms(kLCPSolver);
  TwoPointContactCrossTerms(kLinearSystemSolver);
}

// Tests that the cross-term interaction between contact forces and generic
// unilateral constraints is computed correctly at the acceleration level. Tests
// both the LCP solver and the linear system solver.
TEST_F(Constraint2DSolverTest, ContactLimitCrossTermAccelTest) {
  ContactLimitCrossTermAccel(kLCPSolver);
  ContactLimitCrossTermAccel(kLinearSystemSolver);
}

// Tests the rod in a two-point contacting configuration *realized through
// a configuration limit constraint* using both the LCP and linear system
// solvers. No frictional forces are applied, so any velocity projections along
// directions other than the contact normal will be irrelevant.
TEST_F(Constraint2DSolverTest, TwoPointAsLimitTest) {
  TwoPointAsLimit(kLCPSolver);
  TwoPointAsLimit(kLinearSystemSolver);
  TwoPointAsLimitDiscretized();
}

// Tests the rod in a two-point configuration, in a situation where a force
// pulls the rod upward (and no contact forces should be applied).
TEST_F(Constraint2DSolverTest, TwoPointPulledUpward) {
  // Duplicate contact points up to two times and the friction directions up
  // to three times.
  for (int contact_dup = 0; contact_dup < 3; ++contact_dup) {
    for (int friction_dir_dup = 0; friction_dir_dup < 4; ++friction_dir_dup) {
      // Set the state of the rod to resting on its side with no velocity.
      SetRodToRestingHorizontalConfig();

      // Compute the problem data.
      CalcConstraintAccelProblemData(
        accel_data_.get(), contact_dup, friction_dir_dup);

      // Add a force pulling the rod upward.
      accel_data_->tau[1] += 100.0;

      // Compute the contact forces.
      VectorX<double> cf;
      solver_.SolveConstraintProblem(*accel_data_, &cf);

      // Verify that the contact forces are zero.
      EXPECT_LT(cf.norm(), eps_);
    }
  }
}

// Tests the rod in a two-point configuration, in a situation where the rod
// is moving upward, so no impulsive forces should be applied.
TEST_F(Constraint2DSolverTest, NoImpactImpliesNoImpulses) {
  // Duplicate contact points up to two times and the friction directions up
  // to three times.
  for (int contact_dup = 0; contact_dup < 3; ++contact_dup) {
    for (int friction_dir_dup = 0; friction_dir_dup < 4; ++friction_dir_dup) {
      // Set the state of the rod to resting on its side with upward velocity.
      SetRodToUpwardMovingHorizontalConfig();

      // Compute the problem data.
      CalcConstraintProblemDataForImpact(
          vel_data_.get(), contact_dup, friction_dir_dup);

      // Compute the contact forces.
      VectorX<double> cf;
      solver_.SolveImpactProblem(*vel_data_, &cf);

      // Verify that the impact forces are zero.
      EXPECT_LT(cf.norm(), eps_);
    }
  }
}

// Tests that the cross-term interaction between contact forces and generic
// unilateral constraints is computed correctly at the velocity level.
TEST_F(Constraint2DSolverTest, ContactLimitCrossTermVel) {
  // Set the state of the rod to resting.
  SetRodToSlidingImpactingHorizontalConfig(true);

  // Set the coefficient of friction (mu_static won't be used in this problem,
  // but an exception will be thrown if mu_coulomb is greater than mu_static)
  // to a relatively large number.
  rod_->set_mu_coulomb(1.0);
  rod_->set_mu_static(1.0);

  // First, construct the velocity-level problem data as normal to set
  // inertia solver and external forces.
  std::vector<Vector2d> contacts;
  std::vector<double> tangent_vels;
  rod_->GetContactPoints(*context_, &contacts);
  rod_->GetContactPointsTangentVelocities(*context_, contacts, &tangent_vels);

  // Compute the constraint problem data.
  rod_->CalcImpactProblemData(
    *context_, contacts, vel_data_.get());

  // Add in some horizontal velocity to test the transition to stiction too.
  vel_data_->Mv[0] = 1.0;
  const VectorX<double> v0 = vel_data_->solve_inertia(vel_data_->Mv);

  // Construct the problem as a limit constraint preventing movement in the
  // downward direction.
  const int ngc = get_rod_num_coordinates();
  const int num_generic_unilateral_constraints = 1;
  vel_data_->kL.resize(num_generic_unilateral_constraints);
  vel_data_->gammaL.setZero(num_generic_unilateral_constraints);

  // Set the Jacobian entry- in this case, the limit is an upper limit on the
  // second coordinate (vertical position). The constraint is: v₂ ≤ 0, which
  // we transform to the form: -v₂ ≥ 0 (explaining the provenance of the minus
  // sign in L).
  const int num_limit_constraints = 1;
  MatrixX<double> L(vel_data_->kL.size(), ngc);
  L.setZero();
  L(0, 1) = -1;
  vel_data_->L_mult = [&L](const VectorX<double>& vv) -> VectorX<double> {
    return L * vv;
  };
  vel_data_->L_transpose_mult = [&L](const VectorX<double>& vv) ->
    VectorX<double> {
    return L.transpose() * vv;
  };
  vel_data_->kL.setZero(num_limit_constraints);

  // Check the consistency of the data.
  CheckProblemConsistency(*vel_data_, contacts.size());

  // Compute the constraint forces. Note that we increase cfm to prevent the
  // occasional "failure to solve LCP" exception.
  VectorX<double> cf;
  solver_.SolveImpactProblem(*vel_data_, &cf);

  // Verify the size of cf is as expected.
  EXPECT_EQ(cf.size(), vel_data_->mu.size() * 2 + 1);

  // Verify that the horizontal velocity is unchanged and that the vertical
  // velocity is zero.
  VectorX<double> dv;
  solver_.ComputeGeneralizedVelocityChange(*vel_data_, cf, &dv);
  EXPECT_NEAR(v0[0] + dv[0], 0, eps_);
  EXPECT_NEAR(v0[1] + dv[1], 0, eps_);
}

// Tests the rod in a two-point configuration *realized through a configuration
// limit constraint*, velocity-level version. No frictional forces are applied,
// so any velocity projections along directions other than the contact normal
// will be irrelevant.
TEST_F(Constraint2DSolverTest, TwoPointImpactAsLimit) {
  // Set the state of the rod to impacting on its side.
  SetRodToSlidingImpactingHorizontalConfig(true /* moving to the right */);
  ContinuousState<double>& xc = context_->get_mutable_continuous_state();
  const double vert_vel = xc[4];

  // First, construct the velocity-level problem data as normal to set
  // inertia solver and external forces.
  CalcConstraintProblemDataForImpact(vel_data_.get());

  // Compute v.
  VectorX<double> v0 = vel_data_->solve_inertia(vel_data_->Mv);

  // Construct the problem as a limit constraint preventing movement in the
  // downward direction.
  const int ngc = get_rod_num_coordinates();
  vel_data_->mu.resize(0);
  vel_data_->r.resize(0);
  vel_data_->N_mult = [](const VectorX<double>&) {
    return VectorX<double>(0);
  };
  vel_data_->N_transpose_mult = [ngc](const VectorX<double>&) {
    return VectorX<double>::Zero(ngc);
  };
  vel_data_->kN.resize(0);
  vel_data_->gammaN.resize(0);
  vel_data_->F_mult = [](const VectorX<double>&) {
    return VectorX<double>(0);
  };
  vel_data_->F_transpose_mult = [ngc](const VectorX<double>&) {
    return VectorX<double>::Zero(ngc);
  };
  vel_data_->kF.resize(0);
  vel_data_->gammaF.resize(0);
  vel_data_->gammaE.resize(0);

  // Set the Jacobian entry- in this case, the limit is a lower limit on the
  // second coordinate (vertical position).
  const int num_limits = 1;
  MatrixX<double> L(num_limits, ngc);
  L.setZero();
  L(0, 1) = 1;
  vel_data_->L_mult = [&L](const VectorX<double>& w) -> VectorX<double> {
    return L * w;
  };
  vel_data_->L_transpose_mult = [&L](const VectorX<double>& w) ->
    VectorX<double> {
    return L.transpose() * w;
  };
  vel_data_->kL.setZero(num_limits);
  vel_data_->gammaL.setZero(num_limits);

  // Compute the constraint impulses.
  VectorX<double> cf;
  solver_.SolveImpactProblem(*vel_data_, &cf);

  // Verify the size of cf is as expected.
  EXPECT_EQ(cf.size(), 1);

  // Verify that the normal force exactly opposes the momentum.
  const double mv = std::fabs(vert_vel) * rod_->get_rod_mass();
  EXPECT_NEAR(cf[0], mv, eps_);

  // Set the Jacobian entry- in this case, the limit is an upper limit on the
  // second coordinate (vertical position).
  L *= -1;

  // Reverse the velocity on the rod, which was set by the call to
  // Rod2D::CalcImpactProblemData().
  vel_data_->Mv *= -1;
  v0 *= -1;

  // Recompute the constraint impulses, and verify that they're still equal
  // to the momentum. Note: if the impulses were to be applied to the
  // rod, one will need to compute Lᵀcf[0] to obtain the generalized impulse;
  // this is how we can handle upper and lower limits with only non-negativity
  // constraints.
  solver_.SolveImpactProblem(*vel_data_, &cf);
  EXPECT_EQ(cf.size(), 1);
  EXPECT_NEAR(cf[0], mv, eps_);

  // Verify that the vertical velocity is zero.
  VectorX<double> vnew;
  solver_.ComputeGeneralizedVelocityChange(*vel_data_, cf, &vnew);
  EXPECT_NEAR(v0[1] + vnew[1], 0, eps_);

  // Now test whether constraint stabilization works by trying to get the rod to
  // move downward as fast as it's currently moving upward
  // (according to vel_data_->v). Note that Lv is negative, indicating "error"
  // to be corrected (as desired in this test).
  vel_data_->kL = L * v0;

  // Recompute the constraint impulses, and verify that they're now equal to
  // twice the momentum.
  solver_.SolveImpactProblem(*vel_data_, &cf);
  EXPECT_EQ(cf.size(), 1);
  EXPECT_NEAR(cf[0], mv*2, eps_);
}

// Tests that a purely bilaterally constrained problem is handled correctly.
TEST_F(Constraint2DSolverTest, BilateralOnly) {
  // Set the rod to a ballistic state.
  ContinuousState<double>& xc = context_->get_mutable_continuous_state();
  xc[0] = 0.0;     // com horizontal position.
  xc[1] = 10.0;    // com vertical position.
  xc[2] = 0.0;     // rod rotation.
  xc[3] = 0.0;     // no horizontal velocity.
  xc[4] = 0.0;     // upward velocity.
  xc[5] = 0.0;     // no angular velocity.

  // Compute the problem data.
  CalcConstraintProblemDataForImpact(vel_data_.get());

  // Compute the generalized velocity.
  const VectorX<double> v = vel_data_->solve_inertia(vel_data_->Mv);

  // Add in bilateral constraints on rotational motion.
  vel_data_->G_mult = [](const VectorX<double>& w) -> VectorX<double> {
    VectorX<double> result(1);   // Only one constraint.
    result[0] = w[2];            // Constrain the angular velocity to be zero.
    return result;
  };
  vel_data_->G_transpose_mult =
      [this](const VectorX<double>& f) -> VectorX<double> {
    // An impulsive force (torque) applied to the third component needs no
    // transformation.
    DRAKE_DEMAND(f.size() == 1);
    VectorX<double> result(get_rod_num_coordinates());
    result.setZero();
    result[2] = f[0];
    return result;
  };

  // Indicate through construction of the kG term that the system already has
  // angular orientation (which violates our desire to keep the rod at
  // zero rotation).
  vel_data_->kG.resize(1);
  vel_data_->kG[0] = 1.0;    // Indicate a ccw orientation.

  // Compute the impact forces.
  VectorX<double> cf;
  solver_.SolveImpactProblem(*vel_data_, &cf);

  // Get the change in generalized velocity and verify that the angular
  // velocity has changed counter-clockwise.
  VectorX<double> gv;
  solver_.ComputeGeneralizedVelocityChange(*vel_data_, cf, &gv);
  EXPECT_NEAR(v[2] + gv[2], -vel_data_->kG[0],
              eps_ * cf.size());
}

}  // namespace
}  // namespace constraint
}  // namespace multibody
}  // namespace drake
