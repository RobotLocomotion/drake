#pragma once

#include <vector>

#include <Eigen/Dense>

#include "drake/common/drake_copyable.h"
#include "drake/solvers/fbstab/fbstab_mpc.h"

namespace drake {
namespace solvers {
namespace fbstab {
namespace test {

/**
 * This class is used to create optimal control problems (OCPs) in a format
 * FBstab accepts. It stores the problem data internally and only passes
 * pointers to FBstab. Make sure these pointers are valid for the length of the
 * solve.
 */
class OcpGenerator {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(OcpGenerator)

  /**
   * Represents data needed to simulate the system
   *
   *     x(i+1) = Ax(i) + Bu(i)
   *     y(i) = Cx(i) + Du(i)
   *
   * for T steps starting from x(0) = x0.
   */
  struct SimulationInputs {
    Eigen::VectorXd x0;
    Eigen::MatrixXd A;
    Eigen::MatrixXd B;
    Eigen::MatrixXd C;
    Eigen::MatrixXd D;
    int T = 0;
  };

  OcpGenerator() = default;

  /**
   * Returns problem data in the form accepted by FBstab.
   * @return problem data
   *
   * Throws an exception if one of the problem creator methods hasn't been
   * called first.
   */
  FBstabMpc::QPData GetFBstabInput() const;

  /**
   * Returns the data needed to simulate the linear time invariant systems
   * used in the examples.
   *
   * @return simulation inputs
   *
   * Throws an exception if one of the problem creator methods hasn't been
   * called first.
   */
  SimulationInputs GetSimulationInputs() const;

  /**
   * Fills internal storage with data
   * for a copolymerization reactor control problem.
   *
   * The example is from:
   *
   * Congalidis, John P., John R. Richards, and W. Harmon Ray.
   * "Modeling and control of a copolymerization reactor."
   * 1986 American Control Conference. IEEE, 1986.
   *
   * See https://arxiv.org/pdf/1901.04046.pdf for more details.
   *
   * @param N prediction horizon length
   */
  void CopolymerizationReactor(int N = 70);

  /**
   * Fills internal storage with data
   * for a servo motor control problem.
   *
   * The example is from:
   * Bemporad, Alberto, and Edoardo Mosca. "Fulfilling hard constraints in
   * uncertain linear systems by reference managing." Automatica 34.4 (1998):
   * 451-461.
   *
   * See https://arxiv.org/pdf/1901.04046.pdf for more details.
   *
   * @parm[in] N prediction horizon length
   *
   * Throws an exception if N <= 0.
   */
  void ServoMotor(int N = 20);

  /**
   * Fills internal storage with data
   * for a spacecraft relative motion control problem with horizon N.
   * The example is from:
   *
   * Weiss, Avishai, et al.
   * "Model predictive control of three dimensional spacecraft relative motion."
   * 2012 American Control Conference (ACC). IEEE, 2012.
   *
   * See https://arxiv.org/pdf/1901.04046.pdf for more details.
   *
   * @param[in] N prediction horizon length
   *
   * Throws an exception if N <= 0.
   */
  void SpacecraftRelativeMotion(int N = 40);

  /**
   * Fills internal storage with data
   * for a constrained double integrator problem.
   *
   * @param[in] N prediction horizon length
   *
   * Throws an exception if N <= 0.
   */
  void DoubleIntegrator(int N = 10);

  // TODO(dliaomcp@umich.edu) Add a random system generator.
  // i.e., void RandomSystem(int N = 10);

  /**
   * Get a summary of the problem size,
   * in the following order:
   * - N:  horizon length
   * - nx: number of states
   * - nu: number of control inputs
   * - nc: number of constraint per stage
   *
   * @return vector of problem sizes
   */
  Eigen::Vector4d ProblemSize();

  /** Number of decision variables. */
  int nz() const { return nz_; }

  /** Number of equality duals. */
  int nl() const { return nl_; }

  /** Number of inequality duals. */
  int nv() const { return nv_; }

 private:
  // Repeats the given matrices into N or N+1 length vectors.
  void CopyOverHorizon(const Eigen::MatrixXd& Q, const Eigen::MatrixXd& R,
                       const Eigen::MatrixXd& S, const Eigen::VectorXd& q,
                       const Eigen::VectorXd& r, const Eigen::MatrixXd& A,
                       const Eigen::MatrixXd& B, const Eigen::VectorXd& c,
                       const Eigen::MatrixXd& E, const Eigen::MatrixXd& L,
                       const Eigen::VectorXd& d, const Eigen::VectorXd& x0,
                       int N);

  std::vector<Eigen::MatrixXd> Q_;
  std::vector<Eigen::MatrixXd> R_;
  std::vector<Eigen::MatrixXd> S_;
  std::vector<Eigen::VectorXd> q_;
  std::vector<Eigen::VectorXd> r_;

  std::vector<Eigen::MatrixXd> A_;
  std::vector<Eigen::MatrixXd> B_;
  std::vector<Eigen::VectorXd> c_;

  std::vector<Eigen::MatrixXd> E_;
  std::vector<Eigen::MatrixXd> L_;
  std::vector<Eigen::VectorXd> d_;

  Eigen::VectorXd x0_;

  Eigen::MatrixXd Asim_;
  Eigen::MatrixXd Bsim_;
  Eigen::MatrixXd Csim_;
  Eigen::MatrixXd Dsim_;

  int T_ = 0;  // Simulation length.

  int nz_ = 0;
  int nl_ = 0;
  int nv_ = 0;
  int N_ = 0;
  int nx_ = 0;
  int nu_ = 0;
  int nc_ = 0;

  bool data_populated_ = false;
};

}  // namespace test
}  // namespace fbstab
}  // namespace solvers
}  // namespace drake
