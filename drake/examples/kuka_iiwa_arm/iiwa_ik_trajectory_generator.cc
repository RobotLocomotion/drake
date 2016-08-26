#include "iiwa_ik_trajectory_generator.h"

#include "drake/common/drake_assert.h"
#include "drake/systems/plants/IKoptions.h"
#include "drake/systems/plants/RigidBodyIK.h"

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {

using Eigen::MatrixXd;
using Eigen::VectorXd;
using Eigen::Vector2d;
using std::vector;
//
//using TrajectoryGenerator::PPType;
//using TrajectoryGenerator::PPMatrix;
//using TrajectoryGenerator::PPPoly;

TrajectoryGenerator::TrajectoryGenerator(
    const std::vector<RigidBodyConstraint *> &constraint_array,
    const std::vector<double> &time_stamps,
    const std::shared_ptr<RigidBodyTree> &iiwa_tree
    )
    : constraint_array_(constraint_array), time_stamps_(time_stamps),
      iiwa_tree_(iiwa_tree) {
  DRAKE_ASSERT(time_stamps_.size() == constraint_array_.size());
  num_constraints_ = time_stamps.size();
}

PiecewisePolynomial<double>
  TrajectoryGenerator::GenerateTrajectoryPolynomial()  {

  IKoptions ikoptions(iiwa_tree_.get());
  int info[num_constraints_];
  VectorXd zero_conf = iiwa_tree_->getZeroConfiguration();

  MatrixXd q0(iiwa_tree_->number_of_positions(), num_constraints_);
  q0 = zero_conf.replicate(1,num_constraints_);
//  for (int i = 0; i < num_constraints_; i++) {
//    q0.col(i) = zero_conf;
//  }

  std::cout<<"Inside GenerateTrajectoryPolynomial1 \n";
  std::cout<<"Matrix q0 =\n"<<q0<<"\n";


  MatrixXd q_sol(iiwa_tree_->number_of_positions(), num_constraints_);
  std::vector<std::string> infeasible_constraint;

  std::cout<<"Inside GenerateTrajectoryPolynomial2 \n";
  std::cout<<"Matrix q_sol =\n"<<q_sol<<"\n";


  std::stringstream op;
  std::copy(time_stamps_.begin(), time_stamps_.end(), std::ostream_iterator<double>(op, " "));
  std::cout<<"Time stamps :\n"<<op.str()<<"\n";

  inverseKinPointwise(&tree, kNumTimesteps, t, q0, q0, constraint_array.size(),
                      constraint_array.data(), ikoptions, &q_sol, info,
                      &infeasible_constraint);

  inverseKinPointwise(iiwa_tree_.get(), num_constraints_, time_stamps_.data(), q0, q0,
                      constraint_array_.size(), constraint_array_.data(),
                      ikoptions, &q_sol, info,
                      &infeasible_constraint);


  bool info_good = true;
  for (int i = 0; i < num_constraints_; i++) {
    printf("INFO[%d] = %d ", i, info[i]);
    if (info[i] != 1) {
      info_good = false;
    }
  }
  printf("\n");

  if (!info_good) {
    std::cerr << "Solution failed, not sending." << std::endl;
//    return 1;
  }


  vector <TrajectoryGenerator::PPMatrix> polys;
//  vector<double> times;

  // For each timestep, create a PolynomialMatrix for each joint
  // position.  Each column of q_sol represents a particular time,
  // and the rows of that column contain values for each joint
  // coordinate.
  for (int i = 0; i < num_constraints_; i++) {
    TrajectoryGenerator::PPMatrix poly_matrix(q_sol.rows(), 1);
    const auto traj_now = q_sol.col(i);

    // Produce interpolating polynomials for each joint coordinate.
    for (int row = 0; row < q_sol.rows(); row++) {
      Eigen::Vector2d coeffs(0, 0);
      coeffs[0] = traj_now(row);
      if (i != num_constraints_ - 1) {
        // Set the coefficient such that it will reach the value of
        // the next timestep at the time when we advance to the next
        // piece.  In the event that we're at the end of the
        // trajectory, this will be left 0.
        coeffs[1] = (q_sol(row, i + 1) - coeffs[0]) / (time_stamps_[i + 1] - time_stamps_[i]);
      }
      poly_matrix(row) = TrajectoryGenerator::PPPoly(coeffs);
    }
    polys.push_back(poly_matrix);
   // times.push_back(time_stamps_[i]);
  }

  TrajectoryGenerator::PPType pp_traj(polys, time_stamps_);
//
//  bool time_initialized = false;
//  int64_t start_time_ms = -1;
//  int64_t cur_time_ms = -1;
//  const int64_t end_time_offset_ms = (t_[num_constraints_ - 1] * 1e3);
//  DRAKE_ASSERT(end_time_offset_ms > 0);

  return(pp_traj);
}

} // namespace kuka_iia_arm
} // namespace examples
} // namespace drake