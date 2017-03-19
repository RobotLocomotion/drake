#include "drake/systems/controllers/zmp_planner.h"

#include <gtest/gtest.h>

#include "drake/common/eigen_matrix_compare.h"
#include "drake/examples/ZMP/zmp_test_util.h"

namespace drake {
namespace examples {
namespace zmp {

class ZMPPlannerTest : public ::testing::Test {
 protected:
  // A default setup.
  void SetUp() override {
    double height = 1;
    Eigen::Vector4d x0(0, 0, 0, 0);
    std::vector<double> time = {0, 1, 2, 3};
    std::vector<Eigen::MatrixXd> zmp_knots(time.size());
    zmp_knots[0] = Eigen::Vector2d(0, 0);
    zmp_knots[1] = Eigen::Vector2d(0.2, -0.1);
    zmp_knots[2] = Eigen::Vector2d(0.4, 0.1);
    zmp_knots[3] = Eigen::Vector2d(0.4, 0.1);

    PiecewisePolynomial<double> zmp_d =
        PiecewisePolynomial<double>::FirstOrderHold(time, zmp_knots);

    PlanZMP(zmp_d, x0, height);
  }

  // Plan ZMP and set up internal matrices given a desired ZMP trajectory.
  void PlanZMP(const PiecewisePolynomial<double>& zmp_d,
               const Eigen::Vector4d& x0, double height) {
    zmp_planner_.Plan(zmp_d, x0, height);

    A_ = zmp_planner_.get_A();
    B_ = zmp_planner_.get_B();
    C_ = zmp_planner_.get_C();
    D_ = zmp_planner_.get_D();
    Qy_ = zmp_planner_.get_Qy();
    R_ = zmp_planner_.get_R();
    Vxx_ = zmp_planner_.get_Vxx();

    R1_ = R_ + D_.transpose() * Qy_ * D_;
    R1i_ = R1_.inverse();
    N_ = C_.transpose() * Qy_ * D_;
    NB_ = (N_.transpose() + B_.transpose() * Vxx_);
  }

  // Returns the nominal [com; comd]
  Eigen::Vector4d get_x(double time) const {
    Eigen::Vector4d x;
    x << zmp_planner_.get_nominal_com(time),
        zmp_planner_.get_nominal_comd(time);
    return x;
  }

  // Returns the nominal [com - y_tf; comd]
  Eigen::Vector4d get_xbar(double time) const {
    Eigen::Vector4d x_bar = get_x(time);
    x_bar.head<2>() -= zmp_planner_.get_final_desired_zmp();
    return x_bar;
  }

  // Returns the desired [y_d - y_tf]
  Eigen::Vector2d get_ybar(double time) const {
    return zmp_planner_.get_desired_zmp(time) -
           zmp_planner_.get_final_desired_zmp();
  }

  // Computes s3dot using Eq. 19 in [1].
  Eigen::Matrix<double, 1, 1> ComputeS3dot(double time) const {
    Eigen::Vector2d y_bar = get_ybar(time);
    Eigen::Vector2d r2 = -2 * D_ * Qy_ * y_bar;
    Eigen::Vector2d rs =
        0.5 * (r2 + B_.transpose() * zmp_planner_.get_Vx(time));

    return -y_bar.transpose() * Qy_ * y_bar + rs.transpose() * R1i_ * rs;
  }

  // Computes s3dot using Eq. 18 in [1].
  Eigen::Vector4d ComputeS2dot(double time) const {
    Eigen::Vector2d y_bar = get_ybar(time);

    Eigen::Vector4d q2 = -2 * C_.transpose() * Qy_ * y_bar;
    Eigen::Vector2d r2 = -2 * D_ * Qy_ * y_bar;
    Eigen::Vector2d rs =
        0.5 * (r2 + B_.transpose() * zmp_planner_.get_Vx(time));
    Eigen::Vector4d s2dot = -q2 + 2 * NB_.transpose() * R1i_ * rs -
                            A_.transpose() * zmp_planner_.get_Vx(time);

    return s2dot;
  }

  // Computes s3dot using Eq. 17 in [1].
  Eigen::Matrix4d ComputeS1dot() const {
    return -C_.transpose() * Qy_ * C_ + NB_.transpose() * R1i_ * NB_ -
           Vxx_ * A_ - A_.transpose() * Vxx_;
  }

  // Given the value function of the form:
  // J(x_bar) = x_bar^T * S1 * x_bar + x_bar^T * S2 + constant,
  // where x_bar = [com - zmp_d_tf; comd],
  // the optimal control u (CoM acceleration) is achieved by:
  // min_u L(y, u) + dV / dx_bar * x_bar_dot,
  // where L = (y - y_d)^T * Qy * (y - y_d) + u^T * R u.
  // The minimum is achived when the derivative w.r.t u equals to 0.
  // This expands to:
  // 2 * (C * x + D * u - y_d)^T * Qy * D + 2 * u^T * R +
  // (2 * S1 * x_bar + S2)^T * B = 0.
  // If we collect the terms,
  // (R + D^T * Qy * D) * u =
  // -(C * x - y_d)^T * Qy * D - (S1 * x_bar + 0.5 * S2)^T * B
  Eigen::Vector2d ComputeOptimalCoMddGivenValueFunction(
      double time, const Eigen::Vector4d& x) const {
    Eigen::Vector4d x_bar = x;
    x_bar.head<2>() -= zmp_planner_.get_final_desired_zmp();
    Eigen::Vector2d zmp_d = zmp_planner_.get_desired_zmp(time);

    Eigen::Vector2d lin =
        -(C_ * x - zmp_d).transpose() * Qy_ * D_ -
        (Vxx_ * x_bar + 0.5 * zmp_planner_.get_Vx(time)).transpose() * B_;
    Eigen::Vector2d u = R1_.inverse() * lin;

    return u;
  }

  drake::systems::ZMPPlanner zmp_planner_;

  Eigen::Matrix<double, 4, 4> A_;
  Eigen::Matrix<double, 4, 2> B_;
  Eigen::Matrix<double, 2, 4> C_;
  Eigen::Matrix<double, 2, 2> D_;
  Eigen::Matrix<double, 2, 2> Qy_;
  Eigen::Matrix<double, 2, 2> R_;
  Eigen::Matrix<double, 4, 4> Vxx_;
  Eigen::Matrix<double, 4, 1> Vx_;

  Eigen::Matrix<double, 2, 2> R1_;
  Eigen::Matrix<double, 2, 2> R1i_;
  Eigen::Matrix<double, 4, 2> N_;
  Eigen::Matrix<double, 2, 4> NB_;
};

// The HJB equations should be satisfied at the optimal u and x:
// -Vdot = L(y, u) + dV / dx_bar * x_bar_dot
TEST_F(ZMPPlannerTest, TestHJB) {
  double dt = 1e-2;
  double t0 = zmp_planner_.get_desired_zmp().getStartTime();
  double t1 = zmp_planner_.get_desired_zmp().getEndTime();

  Eigen::Vector4d x, x_bar, Vx, Vxdot;
  Eigen::Vector2d u, y_diff;
  Eigen::Matrix<double, 1, 1> L, H, Vdot;
  for (double t = t0 + dt; t + dt < t1; t += dt) {
    x = get_x(t);
    x_bar = get_xbar(t);
    u = zmp_planner_.get_nominal_comdd(t);
    Vx = zmp_planner_.get_Vx(t);
    Vxdot = ComputeS2dot(t);
    y_diff = C_ * x + D_ * u - zmp_planner_.get_desired_zmp(t);

    L = y_diff.transpose() * Qy_ * y_diff + u.transpose() * R_ * u;
    H = (2 * Vxx_ * x_bar + Vx).transpose() * (A_ * x + B_ * u) + L;
    // Vdot = x_bar * S1dot * x_bar + x_bar * s2dot + s3dot, and S1dot is zero.
    Vdot = Vxdot.transpose() * x_bar + ComputeS3dot(t);

    EXPECT_TRUE(CompareMatrices(-Vdot, H, 1e-8, MatrixCompareType::absolute));
  }
}

// S1dot (Vxxdot) is zero since S1 should be constant.
TEST_F(ZMPPlannerTest, TestS1dot) {
  EXPECT_TRUE(CompareMatrices(ComputeS1dot(), Eigen::Matrix4d::Zero(), 1e-8,
                              MatrixCompareType::absolute));
}

// s2dot computed by Eq. 18 should be the same as s2.derivative.
TEST_F(ZMPPlannerTest, TestS2dot) {
  double dt = 1e-2;
  double t0 = zmp_planner_.get_desired_zmp().getStartTime();
  double t1 = zmp_planner_.get_desired_zmp().getEndTime();

  ExponentialPlusPiecewisePolynomial<double> Vxdot =
      zmp_planner_.get_Vx().derivative();

  for (double t = t0 + dt; t + dt < t1; t += dt) {
    EXPECT_TRUE(CompareMatrices(Vxdot.value(t), ComputeS2dot(t), 1e-8,
                                MatrixCompareType::absolute));
  }
}

// The nominal CoM acceleration (given by the derivatives from the CoM
// position trajectory) should equal to calling the linear controller
// with the nominal CoM state.
TEST_F(ZMPPlannerTest, TestCoMdd) {
  double dt = 1e-2;
  double t0 = zmp_planner_.get_desired_zmp().getStartTime();
  double t1 = zmp_planner_.get_desired_zmp().getEndTime();
  for (double t = t0 + dt; t + dt < t1; t += dt) {
    EXPECT_TRUE(CompareMatrices(zmp_planner_.get_nominal_comdd(t),
                                zmp_planner_.ComputeOptimalCoMdd(t, get_x(t)),
                                1e-8, MatrixCompareType::absolute));
  }
}

// Simulate the optimal policy starting from a perturbed initial condition.
// The end state should converge to the nominal trajectory.
// Also test control computed by the optimal linear policy against directly
// minimizing the Hamiltonian.
TEST_F(ZMPPlannerTest, TestOptimalControl) {
  Eigen::Vector4d x0(0, 0, 0, 0);
  Eigen::Vector2d u0, u1;

  std::vector<Eigen::Vector2d> footsteps = {
      Eigen::Vector2d(0, 0),    Eigen::Vector2d(0.5, 0.1),
      Eigen::Vector2d(1, -0.1), Eigen::Vector2d(1.5, 0.1),
      Eigen::Vector2d(2, -0.1), Eigen::Vector2d(2.5, 0)};

  std::vector<PiecewisePolynomial<double>> zmp_trajs =
      drake::examples::zmp::GenerateDesiredZMPTrajs(footsteps, 0.5, 1);

  double z = 1;

  for (const auto& zmp_d : zmp_trajs) {
    PlanZMP(zmp_d, x0, z);

    // Simulate.
    double sample_dt = 1e-2;
    double simulation_past_end_time = 2;

    x0 << 0.1, -0.05, 0.1, 0.1;
    ZMPTestTraj result = SimulateZMPPolicy(zmp_planner_, x0, sample_dt,
                                           simulation_past_end_time);

    int N = result.time.size();
    // Expect the trajectory converges to the desired at the end.
    EXPECT_TRUE(CompareMatrices(result.x.col(N - 1).head<4>(),
                                result.nominal_com.col(N - 1).head<4>(), 1e-4,
                                MatrixCompareType::absolute));

    for (int i = 0; i < result.time.size(); i++) {
      x0 = result.x.col(i);
      u0 = ComputeOptimalCoMddGivenValueFunction(result.time[i], x0);
      u1 = zmp_planner_.ComputeOptimalCoMdd(result.time[i], x0);
      EXPECT_TRUE(CompareMatrices(u0, u1, 1e-8, MatrixCompareType::absolute));
    }
  }
}

}  // namespace zmp
}  // namespace examples
}  // namespace drake
