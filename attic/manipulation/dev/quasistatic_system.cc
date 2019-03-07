#include "drake/manipulation/dev/quasistatic_system.h"

#include <algorithm>
#include <limits>
#include <unordered_map>
#include <vector>

#include "drake/math/quaternion.h"
#include "drake/systems/controllers/controlUtil.h"

namespace drake {
namespace manipulation {

using Eigen::Matrix3Xd;
using Eigen::MatrixXd;
using Eigen::RowVectorXd;
using Eigen::VectorXd;
using Eigen::VectorXi;
using drake::MatrixX;
using drake::VectorX;
using std::cos;
using std::sin;

const double kInfinity = std::numeric_limits<double>::infinity();

template <typename T>
T MaxStlVector(const std::vector<T>& v) {
  auto result = std::max_element(v.begin(), v.end());
  return *result;
}

/* returns the indices of positions/velocities of the bodies in idx_body. The
 * indices are into the rigidbodytree containing these bodies.
 */
std::vector<int> GetPositionOrVelocityIndicesOfBodiesFromRBT(
    const RigidBodyTree<double>& tree, const std::vector<int>& idx_body,
    bool is_position,
    const std::unordered_map<int, std::vector<int>>& fixed_body_dofs) {
  std::vector<int> idx_q;
  for (auto i : idx_body) {
    int start = 0;
    int size = 0;
    if (is_position) {
      start = tree.get_body(i).get_position_start_index();
      size = tree.get_body(i).getJoint().get_num_positions();
    } else {
      start = tree.get_body(i).get_velocity_start_index();
      size = tree.get_body(i).getJoint().get_num_velocities();
    }
    idx_q.reserve(static_cast<uint64_t>(size));
    for (int j = 0; j < size; j++) {
      auto it = fixed_body_dofs.find(i);
      std::vector<int> vtr{};
      if (it != fixed_body_dofs.end()) {
        vtr = it->second;
      }
      const bool j_not_in_vtr =
          vtr.end() == std::find(vtr.begin(), vtr.end(), j);

      if (j_not_in_vtr) idx_q.push_back(start + j);
    }
  }
  return idx_q;
}

template <class Scalar>
QuasistaticSystem<Scalar>::QuasistaticSystem(
    int idx_base, const std::vector<int>& idx_unactuated_bodies,
    const std::vector<int>& fixed_base_velocities,
    const std::vector<int>& fixed_base_positions,
    const QuasistaticSystemOptions& options)
    : period_sec_(options.period_sec),
      idx_unactuated_bodies_(idx_unactuated_bodies),
      idx_base_(idx_base),
      fixed_base_positions_(fixed_base_positions),
      fixed_base_velocities_(fixed_base_velocities),
      is_contact_2d_(options.is_contact_2d),
      mu_(options.mu),
      kBigM_(options.kBigM),
      is_analytic_(options.is_analytic),
      is_using_kinetic_energy_minimizing_QP_(
          options.is_using_kinetic_energy_minimizing_QP) {
  if (!solver_.available()) {
    throw std::runtime_error("Gurobi solver not available.");
  }
}

template <class Scalar>
void QuasistaticSystem<Scalar>::Initialize() {
  nq_tree_ = tree_->get_num_positions();

  // TODO(pang) this is correct only if the DOFs of the base of
  // the unactuated bodies are not constrained.
  is_base_quaternion_ = false;
  if (tree_->get_body(idx_base_).getJoint().get_num_positions() == 7) {
    is_base_quaternion_ = true;
  }

  // checks if idx_base is an element of idx_unactuated_bodies.
  DRAKE_ASSERT(idx_unactuated_bodies_.end() !=
               std::find(idx_unactuated_bodies_.begin(),
                         idx_unactuated_bodies_.end(), idx_base_));

  // create the list of actuated bodies, which is:
  // list_of_all_bodies - list_of_unactuated_bodies - 0 (world)
  std::sort(idx_unactuated_bodies_.begin(), idx_unactuated_bodies_.end());
  std::vector<int> idx_actuated_bodies;
  int idx_u = 0;
  for (int i = 1; i < tree_->get_num_bodies(); i++) {
    if (idx_u < static_cast<int>(idx_unactuated_bodies_.size()) &&
        i == idx_unactuated_bodies_[idx_u]) {
      idx_u++;
    } else {
      idx_actuated_bodies.push_back(i);
    }
  }

  // create maps of fixed positions/velocities of each body, keyed by body
  // indices in rigidbodytree. fixed_actuated_positions is empty.
  std::unordered_map<int, std::vector<int>> fixed_body_positions,
      fixed_body_velocities, fixed_actuated_body_positions;
  fixed_body_positions[idx_base_] = fixed_base_positions_;
  fixed_body_velocities[idx_base_] = fixed_base_velocities_;

  // create idx_qu_
  idx_qu_.clear();
  idx_qu_ = GetPositionOrVelocityIndicesOfBodiesFromRBT(
      *tree_, idx_unactuated_bodies_, true, fixed_body_positions);

  // assuming that the first body in idx_unactuated_bodies is the base link of
  // the unactuated rigid body mechanism.
  DRAKE_ASSERT(idx_base_ == idx_unactuated_bodies_[0]);
  // create idx_vu_
  idx_vu_ = GetPositionOrVelocityIndicesOfBodiesFromRBT(
      *tree_, idx_unactuated_bodies_, false, fixed_body_velocities);

  // create idx_qa_
  idx_qa_.clear();
  idx_qa_ = GetPositionOrVelocityIndicesOfBodiesFromRBT(
      *tree_, idx_actuated_bodies, true, fixed_actuated_body_positions);

  // get contact information
  int nc_actuated = 0;  // number of collision elements of actauted bodies
  for (auto i : idx_actuated_bodies) {
    nc_actuated += tree_->get_body(i).get_num_collision_elements();
  }
  nc_ = 0;
  for (auto i : idx_unactuated_bodies_) {
    nc_ += tree_->get_body(i).get_num_collision_elements() * nc_actuated;
  }

  // By default, all contacts are 3D.
  if (is_contact_2d_.empty()) {
    for (int i = 0; i < nc_; i++) {
      is_contact_2d_.push_back(false);
    }
  } else {
    DRAKE_ASSERT(static_cast<int>(is_contact_2d_.size()) == nc_);
  }
  UpdateNs();

  this->DeclarePeriodicDiscreteUpdate(period_sec_);
  this->DeclareVectorInputPort(systems::BasicVector<Scalar>(na_));
  this->DeclareVectorOutputPort(drake::systems::BasicVector<Scalar>(nq_tree_),
                                &QuasistaticSystem::CopyStateOut);
  this->DeclareVectorOutputPort(drake::systems::BasicVector<Scalar>(n_),
                                &QuasistaticSystem::CopyDecisionVariablesOut);
  // q consists of qu and qa, dim(q) = n1_
  // dim(MIQP_decision_variables) = n_
  // discrete_state (of QuasistaticSystem) = [q; MIQP_decision_variables]
  this->DeclareDiscreteState(n1_ + n_);

  // creates idx_q_ from idx_q_u and idx_q_a_
  idx_q_ = idx_qu_;
  idx_q_.insert(idx_q_.end(), idx_qa_.begin(), idx_qa_.end());
  std::sort(idx_q_.begin(), idx_q_.end());
  std::sort(idx_qu_.begin(), idx_qu_.end());
  std::sort(idx_qa_.begin(), idx_qa_.end());

  // creates idx_qa_ in q_quasistatic_system, used to index actuated DOF's
  // in the cost function.
  int j_a = 0;  // index for idx_qa_in_q
  int j_u = 0;  // index for idx_qu_in_q
  for (int i = 0; i < static_cast<int>(idx_q_.size()); i++) {
    if (j_a < static_cast<int>(idx_qa_.size()) && idx_qa_[j_a] == idx_q_[i]) {
      idx_qa_in_q_.push_back(i);
      j_a++;
    }
    if (j_u < static_cast<int>(idx_qu_.size()) && idx_qu_[j_u] == idx_q_[i]) {
      idx_qu_in_q_.push_back(i);
      j_u++;
    }
  }

  // initialize initial guess and intended penetration vectors
  lambda_n_start_.resize(nc_);
  lambda_f_start_.resize(nd_);
  gamma_start_.resize(nc_);
  z_n_start_.resize(nc_);
  z_f_start_.resize(nd_);
  z_gamma_start_.resize(nc_);
  phi_bar_.resize(nc_);

  lambda_n_start_.setZero();
  lambda_f_start_.setZero();
  gamma_start_.setZero();
  z_n_start_.setZero();
  z_f_start_.setZero();
  z_gamma_start_.setZero();
  phi_bar_.setZero();

  if (is_base_quaternion_) {
    DRAKE_ASSERT(nu_ == n_vu_ - 1);
  } else {
    DRAKE_ASSERT(nu_ == n_vu_);
  }

  // initialize mathematical program
  prog_.reset(new solvers::MathematicalProgram());
  prog_result_.reset(new solvers::MathematicalProgramResult());

  delta_q_ = prog_->NewContinuousVariables(n1_, "delta_q");
  lambda_n_ = prog_->NewContinuousVariables(nc_, "lambda_n");
  lambda_f_ = prog_->NewContinuousVariables(nd_, "lambda_f");
  gamma_ = prog_->NewContinuousVariables(nc_, "gamma");
  z_n_ = prog_->NewBinaryVariables(nc_, "z_n");
  z_f_ = prog_->NewBinaryVariables(nd_, "z_f");
  z_gamma_ = prog_->NewBinaryVariables(nc_, "z_gamma");

  // Add uppder bound to the norms of all decision variables.
  bounds_delta_q_ =
      prog_->AddBoundingBoxConstraint(-kInfinity, kInfinity, delta_q_)
          .evaluator()
          .get();
  bounds_gamma_ =
      prog_->AddBoundingBoxConstraint(0, kInfinity, gamma_).evaluator().get();
  bounds_lambda_n_ = prog_->AddBoundingBoxConstraint(0, kInfinity, lambda_n_)
                         .evaluator()
                         .get();
  bounds_lambda_f_ = prog_->AddBoundingBoxConstraint(0, kInfinity, lambda_f_)
                         .evaluator()
                         .get();
  // Force balance
  force_balance_ = prog_
                       ->AddLinearEqualityConstraint(
                           MatrixXd::Zero(n_vu_, nc_ + nd_),
                           VectorXd::Zero(n_vu_), {lambda_n_, lambda_f_})
                       .evaluator()
                       .get();

  // prog.AddLinearConstraint(delta_phi_n >= -phi);
  non_penetration_ =
      prog_
          ->AddLinearConstraint(MatrixXd::Zero(nc_, n1_), VectorXd::Zero(nc_),
                                VectorXd::Constant(nc_, kInfinity), delta_q_)
          .evaluator()
          .get();

  // prog.AddLinearConstraint(delta_phi_f >= -E * gamma);
  coulomb_friction1_ =
      prog_
          ->AddLinearConstraint(
              MatrixXd::Zero(nd_, n1_ + nc_), VectorXd::Zero(nd_),
              VectorXd::Constant(nd_, kInfinity), {delta_q_, gamma_})
          .evaluator()
          .get();

  // prog.AddLinearConstraint(U * lambda_n >= E.transpose() * lambda_f);
  coulomb_friction2_ =
      prog_
          ->AddLinearConstraint(
              MatrixXd::Zero(nc_, nc_ + nd_), VectorXd::Zero(nc_),
              VectorXd::Constant(nc_, kInfinity), {lambda_n_, lambda_f_})
          .evaluator()
          .get();

  // prog.AddLinearConstraint(delta_phi_n + phi <= kBigM * z_n);
  non_penetration_complementary_ =
      prog_
          ->AddLinearConstraint(MatrixXd::Zero(nc_, n1_ + nc_),
                                -VectorXd::Constant(nc_, kInfinity),
                                VectorXd::Zero(nc_), {delta_q_, z_n_})
          .evaluator()
          .get();

  // prog.AddLinearConstraint(delta_phi_f + E * gamma <= kBigM * z_f);
  coulomb_friction1_complementary_ =
      prog_
          ->AddLinearConstraint(MatrixXd::Zero(nd_, n1_ + nc_ + nd_),
                                -VectorXd::Constant(nd_, kInfinity),
                                VectorXd::Zero(nd_), {delta_q_, gamma_, z_f_})
          .evaluator()
          .get();

  // prog.AddLinearConstraint(U * lambda_n - E.transpose() * lambda_f <=
  //                         kBigM * z_gamma);
  coulomb_friction2_complementary_ =
      prog_
          ->AddLinearConstraint(MatrixXd::Zero(nc_, nc_ + nd_ + nc_),
                                -VectorXd::Constant(nc_, kInfinity),
                                VectorXd::Zero(nc_),
                                {lambda_n_, lambda_f_, z_gamma_})
          .evaluator()
          .get();

  // decision_variables_complementarity
  decision_variables_complementary_ =
      prog_
          ->AddLinearConstraint(
              MatrixXd::Zero(n2_, n2_ * 2), -VectorXd::Constant(n2_, kInfinity),
              VectorXd::Ones(n2_),
              {lambda_n_, lambda_f_, gamma_, z_n_, z_f_, z_gamma_})
          .evaluator()
          .get();

  // objective
  MatrixXd Q(n1_, n1_);
  Q.setZero();
  VectorXd b(n1_);
  b.setZero();
  objective_ = prog_->AddQuadraticCost(Q, b, delta_q_).evaluator().get();

  // set solver options.
  prog_->SetSolverOption(solvers::GurobiSolver::id(), "OutputFlag", 0);
}

template <class Scalar>
drake::VectorX<Scalar>
QuasistaticSystem<Scalar>::GetQuasistaticSystemStatesFromRigidBodyTreePositions(
    const KinematicsCache<double>& cache) const {
  auto q_tree = cache.getQ();
  VectorX<Scalar> q_quasistatic_system(n1_);
  for (int i = 0; i < n1_; i++) {
    q_quasistatic_system(i) = q_tree(idx_q_[i]);
  }
  return q_quasistatic_system;
}

template <class Scalar>
VectorX<Scalar>
QuasistaticSystem<Scalar>::GetRigidBodyTreePositionsFromQuasistaticSystemStates(
    const Eigen::Ref<const VectorX<Scalar>>& q_quasistatic_system) const {
  // Entries of q_tree that are not defined in q_quasistatic_system are set to
  // zero.
  VectorX<Scalar> q_tree(tree_->get_num_positions());
  q_tree.setZero();
  for (int i = 0; i < n1_; i++) {
    q_tree(idx_q_[i]) = q_quasistatic_system(i);
  }
  return q_tree;
}

template <class Scalar>
MatrixXd QuasistaticSystem<Scalar>::CalcE() const {
  MatrixXd A(nc_, nd_);
  A.setZero();
  int first = 0;
  for (int i = 0; i < nc_; i++) {
    A.block(i, first, 1, nf_(i)).setOnes();
    first += nf_(i);
  }
  return A.transpose();
}

template <class Scalar>
const systems::OutputPort<Scalar>& QuasistaticSystem<Scalar>::state_output()
    const {
  return this->get_output_port(0);
}

template <class Scalar>
const systems::OutputPort<Scalar>&
QuasistaticSystem<Scalar>::decision_variables_output() const {
  return this->get_output_port(1);
}

template <class Scalar>
void QuasistaticSystem<Scalar>::DoCalcWnWfJnJfPhiAnalytic(
    const KinematicsCache<double>& cache, MatrixXd* const Wn_ptr,
    MatrixXd* const Wf_ptr, MatrixXd* const Jn_ptr, MatrixXd* const Jf_ptr,
    VectorXd* const phi_ptr) const {
  static_cast<void>(cache);
  static_cast<void>(Wn_ptr);
  static_cast<void>(Wn_ptr);
  static_cast<void>(Wf_ptr);
  static_cast<void>(Jn_ptr);
  static_cast<void>(Jf_ptr);
  static_cast<void>(phi_ptr);
  throw std::logic_error(
      "Analytic expressions for Wn, Wf, Jn, Jf and phi are not "
      "available.");
}

template <class Scalar>
void QuasistaticSystem<Scalar>::CalcWnWfJnJfPhi(
    const KinematicsCache<double>& cache, MatrixXd* const Wn_ptr,
    MatrixXd* const Wf_ptr, MatrixXd* const Jn_ptr, MatrixXd* const Jf_ptr,
    VectorXd* const phi_ptr) const {
  if (!is_analytic_) {
    DoCalcWnWfJnJfPhi(cache, Wn_ptr, Wf_ptr, Jn_ptr, Jf_ptr, phi_ptr);
  } else {
    DoCalcWnWfJnJfPhiAnalytic(cache, Wn_ptr, Wf_ptr, Jn_ptr, Jf_ptr, phi_ptr);
  }
}

template <class Scalar>
void QuasistaticSystem<Scalar>::CalcJf(
    const Eigen::Ref<const MatrixXd>& Jf_half,
    Eigen::MatrixXd* const Jf_ptr) const {
  MatrixXd& Jf = *Jf_ptr;
  Jf.resize(nd_, Jf_half.cols());
  int first_J = 0;
  for (int i = 0; i < nc_; i++) {
    int n_tangents_half_count = 2;
    if (is_contact_2d_[i]) {
      n_tangents_half_count = 1;
    }
    for (int j = 0; j < n_tangents_half_count; j++) {
      Jf.row(first_J + j) = Jf_half.row(2 * i + j);
      Jf.row(first_J + n_tangents_half_count + j) = -Jf_half.row(2 * i + j);
    }
    first_J += 2 * n_tangents_half_count;
  }
}

template <class Scalar>
void QuasistaticSystem<Scalar>::DoCalcWnWfJnJfPhi(
    const KinematicsCache<double>& cache, Eigen::MatrixXd* const Wn_ptr,
    Eigen::MatrixXd* const Wf_ptr, Eigen::MatrixXd* const Jn_ptr,
    Eigen::MatrixXd* const Jf_ptr, Eigen::VectorXd* const phi_ptr) const {
  MatrixXd& Wn = *Wn_ptr;
  MatrixXd& Wf = *Wf_ptr;
  MatrixXd& Jn = *Jn_ptr;
  VectorXd& phi = *phi_ptr;

  const int nq_tree = tree_->get_num_positions();
  const int nv_tree = tree_->get_num_velocities();

  // run collision detection
  Matrix3Xd normals, xA, xB;
  VectorXd phi_all;
  std::vector<int> bodyA_idx;
  std::vector<int> bodyB_idx;
  tree_->collisionDetect(cache, phi_all, normals, xA, xB, bodyA_idx, bodyB_idx);

  // D and flip_sign are used to compute Jn and Jf.
  std::vector<Eigen::Matrix<Scalar, 3, Eigen::Dynamic>> D;

  // compute Wn and Wf
  const int n_objects = idx_unactuated_bodies_.size();
  const int n_collisions = bodyA_idx.size();

  std::vector<int> idx_relevant_contact;
  for (int j = 0; j < n_objects; j++) {
    for (int i = 0; i < n_collisions; i++) {
      Eigen::Vector3d n_world, x_contact_body;
      if (bodyA_idx[i] == idx_unactuated_bodies_[j] ||
          bodyB_idx[i] == idx_unactuated_bodies_[j]) {
        // Normals returned by collisionDetect point from body B to body A.
        idx_relevant_contact.push_back(i);
        n_world = normals.col(i);
        Matrix3kd d;
        surfaceTangents(n_world, d);
        D.push_back(d);
      }
    }
  }

  // Compute Jn, Jf and Wn, Wf
  Wn.resize(n_vu_, nc_);
  Wn.setZero();
  Wf.resize(n_vu_, nd_);
  Wf.setZero();

  MatrixX<Scalar> Jf_half, J;
  Eigen::Map<VectorXi> idxA(&(bodyA_idx[0]), bodyA_idx.size());
  Eigen::Map<VectorXi> idxB(&(bodyB_idx[0]), bodyB_idx.size());
  tree_->computeContactJacobians(cache, idxA, idxB, xA, xB, J);
  const int nc = idx_relevant_contact.size();
  DRAKE_DEMAND(nc == nc_);
  Jn.resize(nc, nq_tree);
  phi.resize(nc_);
  Jf_half.resize(nc * 2, nq_tree);

  MatrixX<Scalar> Jv = J * tree_->GetVelocityToQDotMapping(cache);
  DRAKE_ASSERT(Jv.cols() == nv_tree);
  DRAKE_ASSERT(Jv.rows() == 3 * nc);

  int idx_first = 0;
  int idx_tangent = 0;
  for (int i = 0; i < nc; i++) {
    int idx = idx_relevant_contact[i];
    phi(i) = phi_all(idx);

    Jn.row(i) = normals.col(idx).transpose() * J.block(idx * 3, 0, 3, nq_tree);

    MatrixX<Scalar> Wn_column(1, nv_tree);
    Wn_column = normals.col(idx).transpose() * Jv.block(idx * 3, 0, 3, nv_tree);
    Wn_column.transposeInPlace();

    for (int k = 0; k < n_vu_; k++) {
      Wn(k, i) = Wn_column(idx_vu_[k]);
    }

    Jf_half.block(idx_first, 0, 2, nq_tree) =
        D[i].transpose() * J.block(idx * 3, 0, 3, nq_tree);

    int n_tangents_half_count;
    if (is_contact_2d_[i]) {
      n_tangents_half_count = 1;
    } else {
      n_tangents_half_count = D[i].cols();
    }

    MatrixX<Scalar> Wf_columns(n_tangents_half_count, nv_tree);
    Wf_columns = D[i].leftCols(n_tangents_half_count).transpose() *
                 Jv.block(idx * 3, 0, 3, nv_tree);
    Wf_columns.transposeInPlace();

    for (int k = 0; k < n_vu_; k++) {
      Wf.block(k, idx_tangent, 1, n_tangents_half_count) =
          Wf_columns.block(idx_vu_[k], 0, 1, n_tangents_half_count);

      Wf.block(k, idx_tangent + n_tangents_half_count, 1,
               n_tangents_half_count) =
          -Wf_columns.block(idx_vu_[k], 0, 1, n_tangents_half_count);
    }

    idx_first += 2;
    idx_tangent += 2 * n_tangents_half_count;
  }

  CalcJf(Jf_half, Jf_ptr);
}

template <class Scalar>
VectorXd QuasistaticSystem<Scalar>::CalcExternalGeneralizedForce(
    KinematicsCache<double>* const cache) const {
  RigidBodyTree<double>::BodyToWrenchMap f_ext;
  auto g = tree_->dynamicsBiasTerm(*cache, f_ext, false);

  VectorXd f(n_vu_);
  for (int i = 0; i < n_vu_; i++) {
    f[i] = -g[idx_vu_[i]];
  }
  return f;
}

template <class Scalar>
double QuasistaticSystem<Scalar>::CalcBigM(
    double max_impulse, double max_delta_q, double max_gamma,
    const Eigen::Ref<const Eigen::MatrixXd>& Jn,
    const Eigen::Ref<const Eigen::MatrixXd>& Jf,
    const Eigen::Ref<const Eigen::VectorXd>& phi,
    const Eigen::Ref<const Eigen::VectorXd>& qa_dot_d) const {
  // finding a good BigM using interval arithmetic.
  std::vector<double> max_of_normal_constraints;
  for (int i = 0; i < nc_; i++) {
    max_of_normal_constraints.push_back(phi(i));
    for (int j = 0; j < na_; j++) {
      max_of_normal_constraints[i] +=
          Jn(i, idx_qa_[j]) * qa_dot_d[j] * period_sec_;
    }
    for (int j = 0; j < nu_; j++) {
      max_of_normal_constraints[i] += std::abs(Jn(i, idx_qu_[j]) * max_delta_q);
    }
  }

  std::vector<double> max_of_tangent_constraints;
  for (int i = 0; i < nd_; i++) {
    max_of_tangent_constraints.push_back(max_gamma);
    for (int j = 0; j < na_; j++) {
      max_of_tangent_constraints[i] +=
          Jf(i, idx_qa_[j]) * qa_dot_d[j] * period_sec_;
    }
    for (int j = 0; j < nu_; j++) {
      max_of_tangent_constraints[i] +=
          std::abs(Jf(i, idx_qu_[j]) * max_delta_q);
    }
  }

  double max_friction = mu_ * max_impulse;

  std::vector<double> maxes = {
      max_friction, MaxStlVector(max_of_normal_constraints),
      MaxStlVector(max_of_tangent_constraints), max_impulse, max_gamma};

  return MaxStlVector(maxes);
}

template <class Scalar>
void QuasistaticSystem<Scalar>::MinimizeKineticEnergy(
    VectorXd* const delta_q_value_ptr,
    const Eigen::Ref<const Eigen::MatrixXd>& Jn,
    const Eigen::Ref<const Eigen::MatrixXd>& Jf,
    const Eigen::Ref<const Eigen::VectorXd>& z_n_value,
    const Eigen::Ref<const Eigen::VectorXd>& z_f_value,
    const Eigen::Ref<const Eigen::VectorXd>& z_gamma_value,
    const Eigen::Ref<const Eigen::VectorXd>& phi,
    KinematicsCache<double>* const cache) const {
  VectorXd& delta_q_value = *delta_q_value_ptr;
  solvers::MathematicalProgram prog_QP;
  auto delta_qu_QP = prog_QP.NewContinuousVariables(nu_, "delta_qu_QP");
  VectorXd Jna_times_delta_qa = VectorXd::Zero(nc_);
  VectorXd Jfa_times_delta_qa = VectorXd::Zero(nd_);
  VectorXd delta_phi_f_value = VectorXd::Zero(nd_);
  for (int i = 0; i < na_; i++) {
    int idx = idx_qa_in_q_[i];
    Jna_times_delta_qa += Jn.col(idx_qa_[i]) * delta_q_value(idx);
    Jfa_times_delta_qa += Jf.col(idx_qa_[i]) * delta_q_value(idx);
  }
  for (int i = 0; i < n1_; i++) {
    delta_phi_f_value += Jf.col(idx_q_[i]) * delta_q_value(i);
  }
  // index of first row of Jf_i (corresponding to contact i) in Jf.
  int idx_fi0 = 0;
  for (int i = 0; i < nc_; i++) {
    // row of Jn corresponding to contact i
    MatrixXd Jnu_row;
    Jnu_row.resize(1, nu_);
    for (int j = 0; j < nu_; j++) {
      Jnu_row(0, j) = Jn(i, idx_qu_[j]);
    }
    // if contact force == 0
    if (z_n_value(i) > 0.5) {
      prog_QP.AddLinearConstraint(Jnu_row * delta_qu_QP +
                                      Jna_times_delta_qa.segment(i, 1) >=
                                  -phi.segment(i, 1));
    } else {
      // z_n_value(i) == 0 --> phi_n(i) == 0
      prog_QP.AddLinearConstraint(Jnu_row * delta_qu_QP +
                                      Jna_times_delta_qa.segment(i, 1) ==
                                  -phi.segment(i, 1));
      const int tangent_vector_half_count = nf_(i) / 2;
      MatrixXd Jfu_half(tangent_vector_half_count, nu_);
      MatrixXd Jfu(nf_(i), nu_);
      for (int j = 0; j < nu_; j++) {
        Jfu_half.col(j) =
            Jf.block(idx_fi0, idx_qu_[j], tangent_vector_half_count, 1);
        Jfu.col(j) = Jf.block(idx_fi0, idx_qu_[j], nf_(i), 1);
      }
      VectorXd Jfa_times_delta_qa_i =
          Jfa_times_delta_qa.segment(idx_fi0, nf_(i));
      // if contact i is not sliding.
      if (z_gamma_value(i) > 0.5) {
        prog_QP.AddLinearConstraint(
            Jfu_half * delta_qu_QP ==
            -Jfa_times_delta_qa_i.segment(0, tangent_vector_half_count));

      } else {
        // if contact i is sliding:
        // 1. if lambda_f_ij <= M, the sliding velocity opposite to d_ij
        // must be positive.

        std::vector<int> non_zero_friction_idx;
        int non_zero_friction_count = 0;

        for (int j = 0; j < nf_(i); j++) {
          RowVectorXd Jfu_row(nu_);
          Jfu_row = Jfu.row(j);
          if (z_f_value(idx_fi0 + j) < 0.5) {  // lambda_f_ij <= M
            prog_QP.AddLinearConstraint(Jfu_row * delta_qu_QP <=
                                        -Jfa_times_delta_qa_i.segment(j, 1));

            non_zero_friction_count++;
            non_zero_friction_idx.push_back(j);
          }
        }
        // 2. if there are friction forces in two adjacent directions, the
        // corresponding sliding velocities in those directions must be
        // equal.
        if (tangent_vector_half_count > 1) {
          if (non_zero_friction_count == 1) {
            const int j_opposite =
                (non_zero_friction_idx[0] + tangent_vector_half_count) % nf_(i);
            const int j_opposite_next = (j_opposite + 1) % nf_(i);
            const int j_opposite_previous = (j_opposite - 1 + nf_(i)) % nf_(i);
            RowVectorXd Jfu_opposite_next(nu_);
            RowVectorXd Jfu_opposite_previous(nu_);
            RowVectorXd Jfu_opposite(nu_);
            Jfu_opposite_next = Jfu.row(j_opposite_next);
            Jfu_opposite_previous = Jfu.row(j_opposite_previous);
            Jfu_opposite = Jfu.row(j_opposite);

            const symbolic::Expression delta_phi_f_ij_opposite =
                Jfu_opposite.transpose().dot(delta_qu_QP) +
                Jfa_times_delta_qa_i(j_opposite);
            const symbolic::Expression delta_phi_f_ij_opposite_next =
                Jfu_opposite_next.transpose().dot(delta_qu_QP) +
                Jfa_times_delta_qa_i(j_opposite_next);
            const symbolic::Expression delta_phi_f_ij_opposite_previous =
                Jfu_opposite_previous.transpose().dot(delta_qu_QP) +
                Jfa_times_delta_qa_i(j_opposite_previous);
            prog_QP.AddLinearConstraint(delta_phi_f_ij_opposite_next <=
                                        delta_phi_f_ij_opposite);
            prog_QP.AddLinearConstraint(delta_phi_f_ij_opposite_previous <=
                                        delta_phi_f_ij_opposite);

          } else if (non_zero_friction_count == 2) {
            DRAKE_DEMAND(non_zero_friction_count == 2);
            int j1 = non_zero_friction_idx[0];
            int j2 = non_zero_friction_idx[1];
            RowVectorXd Jfu_row1(nu_);
            RowVectorXd Jfu_row2(nu_);
            Jfu_row1 = Jfu.row(j1);
            Jfu_row2 = Jfu.row(j2);
            const symbolic::Expression lhs =
                Jfa_times_delta_qa(idx_fi0 + j1) +
                Jfu_row1.transpose().dot(delta_qu_QP);
            const symbolic::Expression rhs =
                Jfa_times_delta_qa(idx_fi0 + j2) +
                Jfu_row2.transpose().dot(delta_qu_QP);
            prog_QP.AddLinearConstraint(lhs == rhs);
          }
        }
      }
    }
    idx_fi0 += nf_(i);
  }
  // cost = Δq_u' * H * Δq_u
  MatrixXd H(nu_, nu_);

  H.setZero();
  MatrixXd H_all = tree_->massMatrix(*cache);
  for (int i = 0; i < nu_; i++) {
    for (int j = 0; j < nu_; j++) {
      H(i, j) = H_all(idx_qu_[i], idx_qu_[j]);
    }
  }

  prog_QP.AddQuadraticCost(H, VectorXd::Zero(nu_), delta_qu_QP);

  // solve QP
  prog_QP.SetSolverOption(solvers::GurobiSolver::id(), "OutputFlag", 0);
  const auto result_QP = solver_.Solve(prog_QP, {}, {});

  // replace qu in the MIQP solution with the QP solution
  if (result_QP.is_success()) {
    auto delta_qu_QP_value = result_QP.GetSolution(delta_qu_QP);
    for (int i = 0; i < nu_; i++) {
      delta_q_value(idx_qu_in_q_[i]) = delta_qu_QP_value(i);
    }
  }
}

template <class Scalar>
void QuasistaticSystem<Scalar>::StepForward(
    const Eigen::Ref<const Eigen::MatrixXd>& Wn,
    const Eigen::Ref<const Eigen::MatrixXd>& Wf,
    const Eigen::Ref<const Eigen::MatrixXd>& Jn,
    const Eigen::Ref<const Eigen::MatrixXd>& Jf,
    const Eigen::Ref<const Eigen::MatrixXd>& U,
    const Eigen::Ref<const Eigen::MatrixXd>& E,
    const Eigen::Ref<const Eigen::VectorXd>& phi,
    const Eigen::Ref<const Eigen::VectorXd>& f,
    const Eigen::Ref<const Eigen::VectorXd>& qa_dot_d) const {
  // upper bounds on delta_q
  const Scalar max_delta_q = period_sec_;  // The multiplier is arbitrary.
  const Scalar max_gamma = max_delta_q;

  // uppder bounds on impulses (force * delta_t)
  Scalar max_impulse = 0;
  for (int i = 0; i < f.size(); i++) {
    if (max_impulse < period_sec_ * std::abs(f(i))) {
      max_impulse = period_sec_ * std::abs(f(i));
    }
  }
  max_impulse *= 3;  // This upper bound is arbitrary.

  // calculate big M
  const Scalar kBigM =
      CalcBigM(max_impulse, max_delta_q, max_gamma, Jn, Jf, phi, qa_dot_d);

  // find columns of Jn and Jq corresponding to q.
  MatrixXd Jn_q(nc_, n1_);
  MatrixXd Jf_q(nd_, n1_);
  for (int i = 0; i < n1_; i++) {
    Jn_q.col(i) = Jn.col(idx_q_[i]);
    Jf_q.col(i) = Jf.col(idx_q_[i]);
  }

  // Add uppder bound to the norms of all decision variables.
  bounds_delta_q_->UpdateLowerBound(-VectorXd::Constant(n1_, max_delta_q));
  bounds_delta_q_->UpdateUpperBound(VectorXd::Constant(n1_, max_delta_q));
  bounds_gamma_->UpdateUpperBound(VectorXd::Constant(nc_, max_gamma));
  bounds_lambda_n_->UpdateUpperBound(VectorXd::Constant(nc_, max_impulse));
  bounds_lambda_f_->UpdateUpperBound(VectorXd::Constant(nd_, max_impulse));

  // force balance constraints
  // Wn * lambda_n + Wf * lambda_f == -f *period_sec_
  force_balance_->UpdateCoefficients(
      (MatrixXd(n_vu_, nc_ + nd_) << Wn, Wf).finished(), -f * period_sec_);

  // Lower bound on contact forces (impulses) can be added by adding
  // "springs" at contact points. This is the mechanically correct way to
  // lower bound normal forces, but it seems to be numerically bad.

  // Instead, lower bounds are added by looking at
  // "hypothetical penetration" from the previous time step. Less elegant but
  // numerically better.
  const Scalar kEpsilon = 1e-6;
  VectorXd lb_lambda_n(nc_);
  lb_lambda_n.setZero();
  for (int i = 0; i < nc_; i++) {
    if (phi_bar_(i) < -kEpsilon) {
      lb_lambda_n(i) = period_sec_ * 0.8;
    }
  }
  bounds_lambda_n_->UpdateLowerBound(lb_lambda_n);

  // delta_phi_n = Jn.col(idx_q_) * delta_q
  // delta_phi_f = Jf.col(idx_q_) * delta_q

  // delta_phi_n >= -phi
  non_penetration_->UpdateCoefficients(Jn_q, -phi,
                                       VectorXd::Constant(nc_, kInfinity));

  // delta_phi_f >= -E * gamma
  coulomb_friction1_->UpdateCoefficients(
      (MatrixXd(nd_, n1_ + nc_) << Jf_q, E).finished(), VectorXd::Zero(nd_),
      VectorXd::Constant(nd_, kInfinity));

  // U * lambda_n >= E' * lambda_f
  coulomb_friction2_->UpdateCoefficients(
      (MatrixXd(nc_, nc_ + nd_) << U, -E.transpose()).finished(),
      VectorXd::Zero(nc_), VectorXd::Constant(nc_, kInfinity));

  // delta_phi_n + phi <= kBigM * z_n
  non_penetration_complementary_->UpdateCoefficients(
      (MatrixXd(nc_, n1_ + nc_) << Jn_q, -kBigM * MatrixXd::Identity(nc_, nc_))
          .finished(),
      -VectorXd::Constant(nc_, kInfinity), -phi);

  // delta_phi_f + E * gamma <= kBigM * z_f
  coulomb_friction1_complementary_->UpdateCoefficients(
      (MatrixXd(nd_, n1_ + nc_ + nd_) << Jf_q, E,
       -kBigM * MatrixXd::Identity(nd_, nd_))
          .finished(),
      -VectorXd::Constant(nd_, kInfinity), VectorXd::Zero(nd_));

  // U * lambda_n - E' * lambda_f <= kBigM * z_gamma
  coulomb_friction2_complementary_->UpdateCoefficients(
      (MatrixXd(nc_, nc_ + nd_ + nc_) << U, -E.transpose(),
       -kBigM * MatrixXd::Identity(nc_, nc_))
          .finished(),
      -VectorXd::Constant(nc_, kInfinity), VectorXd::Zero(nc_));

  // lambda_n <= kBigM * (1 - z_n))
  // gamma <= kBigM * (1 - z_gamma))
  // lambda_f <= kBigM * (1 - z_f))
  decision_variables_complementary_->UpdateCoefficients(
      (MatrixXd(n2_, n2_ * 2) << MatrixXd::Identity(n2_, n2_),
       kBigM * MatrixXd::Identity(n2_, n2_))
          .finished(),
      -VectorXd::Constant(n2_, kInfinity), kBigM * VectorXd::Ones(n2_));

  // objective = ||Δq_a/period_sec_ - qa_dot_d||^2
  MatrixXd Q(n1_, n1_);
  Q.setZero();
  VectorXd b(n1_);
  b.setZero();
  for (int i = 0; i < static_cast<int>(idx_qa_in_q_.size()); i++) {
    Q(idx_qa_in_q_[i], idx_qa_in_q_[i]) = 1 / std::pow(period_sec_, 2);
    b(idx_qa_in_q_[i]) = -(1 / period_sec_) * qa_dot_d(i);
  }
  objective_->UpdateCoefficients(Q, b);

  // initial guesses
  prog_->SetInitialGuess(delta_q_, VectorXd::Constant(n1_, 0));
  prog_->SetInitialGuess(lambda_n_, lambda_n_start_);
  prog_->SetInitialGuess(lambda_f_, lambda_f_start_);
  prog_->SetInitialGuess(gamma_, gamma_start_);
  prog_->SetInitialGuess(z_n_, z_n_start_);
  prog_->SetInitialGuess(z_f_, z_f_start_);
  prog_->SetInitialGuess(z_gamma_, z_gamma_start_);

  solver_.Solve(*prog_, {}, {}, prog_result_.get());
  DRAKE_DEMAND(prog_result_->is_success());
}

template <class Scalar>
void QuasistaticSystem<Scalar>::DoCalcDiscreteVariableUpdates(
    const drake::systems::Context<Scalar>& context,
    const std::vector<const drake::systems::DiscreteUpdateEvent<Scalar>*>&,
    drake::systems::DiscreteValues<Scalar>* discrete_state_ptr) const {
  // copy of discrete states at current time step (l)
  VectorX<Scalar> discrete_state_vector =
      context.get_discrete_state(0).CopyToVector();
  VectorXd qa_dot_d = (*this->EvalVectorInput(context, 0)).CopyToVector();
  VectorX<Scalar> q_quasistatic_system = discrete_state_vector.head(n1_);
  KinematicsCache<double> cache = tree_->CreateKinematicsCache();
  VectorXd v(tree_->get_num_velocities());
  v.setZero();
  cache.initialize(GetRigidBodyTreePositionsFromQuasistaticSystemStates(
                       q_quasistatic_system),
                   v);
  tree_->doKinematics(cache);

  MatrixXd Jn, Jf, Wn, Wf;
  VectorXd phi;
  CalcWnWfJnJfPhi(cache, &Wn, &Wf, &Jn, &Jf, &phi);

  MatrixXd U(nc_, nc_);
  U.setZero();
  U.diagonal().setConstant(mu_);

  MatrixXd E = CalcE();
  VectorXd f = CalcExternalGeneralizedForce(&cache);
  f /= f.norm();  // normalize f, which generally makes kBigM smaller.

  DRAKE_DEMAND(Wn.rows() == n_vu_);
  DRAKE_DEMAND(Wn.cols() == nc_);
  DRAKE_DEMAND(Wf.rows() == n_vu_);
  DRAKE_DEMAND(Wf.cols() == nd_);
  DRAKE_DEMAND(Jn.rows() == nc_);
  DRAKE_DEMAND(Jn.cols() == nq_tree_);
  DRAKE_DEMAND(Jf.rows() == nd_);
  DRAKE_DEMAND(Jf.cols() == nq_tree_);

  StepForward(Wn, Wf, Jn, Jf, U, E, phi, f, qa_dot_d);

  // get solution
  DRAKE_DEMAND(prog_result_->is_success());
  auto delta_q_value = prog_result_->GetSolution(delta_q_);
  auto lambda_n_value = prog_result_->GetSolution(lambda_n_);
  auto lambda_f_value = prog_result_->GetSolution(lambda_f_);
  auto gamma_value = prog_result_->GetSolution(gamma_);
  auto z_n_value = prog_result_->GetSolution(z_n_);
  auto z_f_value = prog_result_->GetSolution(z_f_);
  auto z_gamma_value = prog_result_->GetSolution(z_gamma_);

  VectorXd delta_qu_value(nu_);
  for (int i = 0; i < nu_; i++) {
    delta_qu_value(i) = delta_q_value(idx_qu_in_q_[i]);
  }

  // second QP to minimize kinetic energy
  if (is_using_kinetic_energy_minimizing_QP_) {
    MinimizeKineticEnergy(&delta_q_value, Jn, Jf, z_n_value, z_f_value,
                          z_gamma_value, phi, &cache);
  }

  // update quasistatic system configuration at time step l+1
  VectorXd ql1(n1_);
  ql1 << q_quasistatic_system;
  ql1 += delta_q_value;

  // update initial guesses
  lambda_n_start_ = lambda_n_value;
  gamma_start_ = gamma_value;
  z_n_start_ = z_n_value;
  z_gamma_start_ = z_gamma_value;
  lambda_f_start_ = lambda_f_value;
  z_f_start_ = z_f_value;

  // calculate "hypothetical penetration"
  phi_bar_ = phi;
  VectorXd delta_q_bar(n1_);
  delta_q_bar = delta_q_value;
  for (int i = 0; i < na_; i++) {
    delta_q_bar(idx_qa_in_q_[i]) = qa_dot_d(i) * period_sec_;
  }
  for (int i = 0; i < n1_; i++) {
    phi_bar_ += Jn.col(idx_q_[i]) * delta_q_bar(i);
  }

  // Normalize quaternion if there is one.
  if (is_base_quaternion_) {
    Eigen::Vector4d qw;
    for (int i = 0; i < 4; i++) {
      qw(i) = ql1(idx_qu_[i + 3]);
    }
    qw.normalize();
    for (int i = 0; i < 4; i++) {
      ql1(idx_qu_[i + 3]) = qw(i);
    }
  }

  discrete_state_vector << ql1, delta_q_value, lambda_n_value, lambda_f_value,
      gamma_value, z_n_value, z_f_value, z_gamma_value;
  discrete_state_ptr->get_mutable_vector().SetFromVector(discrete_state_vector);
}

// explicit template instantiations
template class QuasistaticSystem<double>;

}  // namespace manipulation
}  // namespace drake
