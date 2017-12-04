#include "drake/perception/estimators/dev/articulated_icp.h"

using std::vector;
using Eigen::VectorXd;
using Eigen::MatrixXd;
using Eigen::Matrix3Xd;

namespace drake {
namespace perception {
namespace estimators {

ArticulatedBodyInfluence IsBodyCorrespondenceInfluential(const Scene& scene,
                                                         BodyIndex) {
  ArticulatedBodyInfluence out;
  out.will_affect_camera = (scene.frame_camera() != scene.frame_world());
  // TODO(eric.cousineau): Add selection for different DOFs.
  out.will_affect_model = true;
  return out;
}

void ComputeBodyCorrespondenceInfluences(
    const Scene& scene, ArticulatedBodyInfluences* influences) {
  int num_bodies = scene.tree().get_num_bodies();
  influences->resize(num_bodies);
  for (int i = 0; i < num_bodies; ++i) {
    (*influences)[i] = IsBodyCorrespondenceInfluential(scene, i);
  }
}

void ArticulatedIcpBodyPoints::Finalize() {
  meas_pts_W_.conservativeResize(Eigen::NoChange, num_actual_);
  body_pts_W_.conservativeResize(Eigen::NoChange, num_actual_);
}

void ArticulatedIcpBodyPoints::Add(const Eigen::Vector3d& meas_W,
                                   const Eigen::Vector3d& body_W) {
  int i = num_actual_++;
  DRAKE_DEMAND(num_actual_ <= num_max_);
  meas_pts_W_.col(i) = meas_W;
  body_pts_W_.col(i) = body_W;
}

void ArticulatedIcpBodyPoints::ComputeError(const SceneState& scene_state,
                                            ArticulatedIcpErrorSet* es) const {
  // Compute measured and body (model) points in their respective frames for
  // Jacobian computation.
  DRAKE_DEMAND(es != nullptr);
  es->resize(num_actual_);
  // Compute error
  es->errors() = meas_pts_W_ - body_pts_W_;

  // Get point jacobian w.r.t. camera frame, as that is the only influence
  // on the measured point cloud.
  // TODO(eric.cousineau): If camera is immovable w.r.t. formulation, do not
  // update. Consider passing in body influence information.
  bool compute_J = true;
  if (compute_J) {
    const auto& scene = scene_state.scene();
    const auto& tree = scene.tree();
    const auto& cache = scene_state.tree_cache();
    // TODO(eric.cousineau): Do not allocate here if possible.
    Eigen::Matrix3Xd meas_pts_C = tree.transformPoints(
        cache, meas_pts_W_, scene.frame_world(), scene.frame_camera());
    // TODO(eric.cousineau): Consider changing `transformPointsJacobian` to
    // accept points in the base frame rather than the body frame.
    Eigen::MatrixXd J_meas_pts_W = tree.transformPointsJacobian(
        cache, meas_pts_C, scene.frame_camera(), scene.frame_world(), false);
    Eigen::Matrix3Xd body_pts_Bi =
        tree.transformPoints(cache, body_pts_W_, scene.frame_world(),
                             frame_Bi_);
    Eigen::MatrixXd J_body_pts_W = tree.transformPointsJacobian(
        cache, body_pts_Bi, frame_Bi_, scene.frame_world(), false);
    // Compute Jacobian of error.
    es->J_errors() = J_meas_pts_W - J_body_pts_W;
  }
}

void ArticulatedIcpErrorNormCost::Reset() {
  cost_ = 0;
  J_cost_.setZero();
  num_points_ = 0;
  finalized_ = false;
}

void ArticulatedIcpErrorNormCost::Add(const SceneState&,
                                      const ArticulatedIcpErrorSet& error_set) {
  // Get error squared.
  for (int i = 0; i < error_set.size(); ++i) {
    auto&& e = error_set.error(i);
    auto&& Je = error_set.J_error(i);
    cost_ += e.dot(e);
    J_cost_ += 2 * e.transpose() * Je;
    ++num_points_;
  }
}

void ArticulatedIcpErrorNormCost::Finalize() {
  if (do_normalize_) {
    cost_ /= num_points_;
    J_cost_ /= num_points_;
  }
  finalized_ = true;
}

void ArticulatedIcpLinearizedNormCost::Reset() {
  Q_.setZero();
  b_.setZero();
  c_ = 0;
  num_points_ = 0;
  finalized_ = false;
}

void ArticulatedIcpLinearizedNormCost::Add(
    const SceneState& scene_state, const ArticulatedIcpErrorSet& error_set) {
  DRAKE_ASSERT(!finalized_);
  const VectorXd& q0 = scene_state.q();
  for (int i = 0; i < error_set.size(); ++i) {
    auto&& e = error_set.error(i);
    auto&& Je = error_set.J_error(i);
    Eigen::Vector3d k = e - Je * q0;
    Q_ += 2 * Je.transpose() * Je;
    b_ += 2 * Je.transpose() * k;
    c_ += k.dot(k);
    cost_ += e.dot(e);
    num_points_ += 1;
  }
}

void ArticulatedIcpLinearizedNormCost::UpdateCost(
    solvers::QuadraticCost* cost) const {
  DRAKE_ASSERT(finalized_);
  cost->UpdateCoefficients(Q_, b_, c_);
}
void ArticulatedIcpLinearizedNormCost::Finalize() {
  finalized_ = true;
  Q_ /= num_points_;
  b_ /= num_points_;
  c_ /= num_points_;
}

void ComputeCorrespondences(const SceneState& scene_state,
                            const ArticulatedBodyInfluences& influences,
                            const Eigen::Matrix3Xd& meas_pts_W,
                            ArticulatedPointCorrespondences* pcorrespondence) {
  DRAKE_DEMAND(pcorrespondence != nullptr);
  const Scene& scene = scene_state.scene();
  int num_points = meas_pts_W.cols();
  VectorXd distances(num_points);
  Matrix3Xd body_normals_W(3, num_points);  // world frame.
  Matrix3Xd body_pts_W(3, num_points);      // body point, world frame.
  Matrix3Xd body_pts_Bi(3, num_points);     // body point, body frame.
  vector<BodyIndex> body_indices(num_points);

  const bool use_margins = false;
  // TOOD(eric.cousineau): Figure out better access.
  // TODO(eric.cousineau): Tie this directly to the visuals, rather than the
  // collision geometry.
  RigidBodyTreed& mutable_tree = const_cast<RigidBodyTreed&>(scene.tree());
  mutable_tree.collisionDetectFromPoints(
      scene_state.tree_cache(), meas_pts_W, distances, body_normals_W,
      body_pts_W, body_pts_Bi, body_indices, use_margins);

  // Cache whether or not there is influence. This should be effectively used
  // to filter out points from the set.
  int num_bodies = scene.tree().get_num_bodies();
  vector<bool> is_body_influential(num_bodies);
  for (int i = 0; i < num_bodies; ++i) {
    is_body_influential[i] = influences[i].is_influential();
  }

  // Bin each correspondence to the given body.
  for (int i = 0; i < num_points; ++i) {
    BodyIndex body_index = body_indices[i];
    if (body_index != -1 && is_body_influential[body_index]) {
      vector<PointCorrespondence>& body_correspondences =
          (*pcorrespondence)[body_index];
      PointCorrespondence pc(meas_pts_W.col(i), body_pts_W.col(i),
                             distances[i]);
      body_correspondences.push_back(pc);
    }
  }
}

void ComputeCost(const SceneState& scene_state,
                 const ArticulatedPointCorrespondences& correspondence,
                 ArticulatedIcpErrorCost* cost) {
  cost->Reset();
  ArticulatedIcpErrorSet error_set(
      scene_state.scene().tree().get_num_positions());
  for (const auto& pair : correspondence) {
    const BodyIndex body_index = pair.first;
    const vector<PointCorrespondence>& body_correspondence = pair.second;
    ArticulatedIcpBodyPoints body_pts(body_index, body_correspondence.size());
    for (const auto& pc_W : body_correspondence) {
      body_pts.Add(pc_W.meas_point, pc_W.model_point);
    }
    body_pts.Finalize();
    body_pts.ComputeError(scene_state, &error_set);
    cost->Add(scene_state, error_set);
  }
  cost->Finalize();
}

}  // namespace estimators
}  // namespace perception
}  // namespace drake
