#pragma once

#include <map>
#include <vector>

#include "drake/perception/estimators/dev/scene.h"
#include "drake/solvers/cost.h"

namespace drake {
namespace perception {
namespace estimators {

/**
 * A set of Cartesian errors and their gradients:
 *   {(eᵢ, Jᵢ)}ᵢ₌₁..ₙ
 */
class ArticulatedIcpErrorSet {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(ArticulatedIcpErrorSet)
  /**
   * @param num_var Number of variables for the gradients.
   */
  explicit ArticulatedIcpErrorSet(int num_var) : num_var_(num_var) {}

  /**
   * Resize (and clear) the stored terms.
   * @param num_points Number of points.
   */
  void resize(int num_points) {
    errors_.resize(Eigen::NoChange, num_points);
    J_errors_.resize(3 * num_points, num_var_);
    errors_.setZero();
    J_errors_.setZero();
  }
  int size() const { return errors_.cols(); }
  Eigen::Ref<const Eigen::Vector3d> error(int i) const {
    return errors_.col(i);
  }
  Eigen::Ref<const Eigen::Matrix3Xd> J_error(int i) const {
    return J_errors_.middleRows<3>(3 * i);
  }
  Eigen::Ref<Eigen::Matrix3Xd> errors() { return errors_; }
  Eigen::Ref<Eigen::MatrixXd> J_errors() { return J_errors_; }

 private:
  Eigen::Matrix3Xd errors_;
  Eigen::MatrixXd J_errors_;
  int num_var_{};
};

/**
 * Accumulate errors and render the cost into an appropriate format.
 */
class ArticulatedIcpErrorCost {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(ArticulatedIcpErrorCost)
  explicit ArticulatedIcpErrorCost(const Scene* scene) : scene_(scene) {}
  virtual ~ArticulatedIcpErrorCost() {}

  /**
   * Prepare to start adding points.
   */
  virtual void Reset() = 0;
  /**
   * Add a given set of errors.
   */
  virtual void Add(const SceneState& scene_state,
                   const ArticulatedIcpErrorSet& error_set) = 0;
  /**
   * Finish adding all error sets.
   */
  virtual void Finalize() = 0;

  const Scene& scene() const { return *scene_; }

 private:
  const Scene* scene_{};
};

/**
 * Aggregate L2 norm cost of point-to-point error set. The total cost, `E`, and
 * its Jacobian `J`, w.r.t. the
 * position variables `q` of the scene, is:
 *   E = ∑ eᵢ'eᵢ
 *   J = ∑ 2 eᵢ'Jᵢ
 */
// TODO(eric.cousineau): Consider merging with QP cost aggregator.
class ArticulatedIcpErrorNormCost : public ArticulatedIcpErrorCost {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(ArticulatedIcpErrorNormCost)
  /**
   * @param do_normalize Normalize the cost by the number of the points in
   * the scene.
   */
  explicit ArticulatedIcpErrorNormCost(const Scene* scene,
                                       bool do_normalize = false)
      : ArticulatedIcpErrorCost(scene), do_normalize_(do_normalize) {
    J_cost_.resize(1, scene->tree().get_num_positions());
  }
  void Reset() override;
  void Add(const SceneState&, const ArticulatedIcpErrorSet& error_set) override;
  void Finalize() override;
  double cost() const {
    DRAKE_ASSERT(finalized_);
    return cost_;
  }
  const Eigen::MatrixXd& J_cost() const {
    DRAKE_ASSERT(finalized_);
    return J_cost_;
  }

 private:
  bool do_normalize_{};
  double cost_{};
  Eigen::MatrixXd J_cost_;
  double num_points_;
  bool finalized_{};
};

/**
 * Accumulate the L2 norm of the linearized point-to-point errors to be
 * rendered into a QuadraticCost operating on variables `q`:
 *   | eᵢ + Jᵢ (q - q0) |^2
 * where `eᵢ` is the ith 3x1 error in Cartesian space, `Jᵢ` is the 3 x nq
 * Jacobian of this error.
 * The resulting quadratic coefficients (for 0.5 x'Qx + b'x + c) are:
 *   Q = ∑ 2*Jᵢ'Jᵢ
 *   b = ∑ 2*Jᵢ'kᵢ
 *   c = ∑ kᵢ'kᵢ
 * where
 *   kᵢ = eᵢ - Jᵢ q0
 * The total cost is:
 *   E = ∑ eᵢ'eᵢ
 */
class ArticulatedIcpLinearizedNormCost : public ArticulatedIcpErrorCost {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(ArticulatedIcpLinearizedNormCost)
  explicit ArticulatedIcpLinearizedNormCost(const Scene* scene)
      : ArticulatedIcpErrorCost(scene) {
    const int nvar = scene->tree().get_num_positions();
    Q_.resize(nvar, nvar);
    b_.resize(nvar);
    Reset();
  }

  void Reset() override;
  void Add(const SceneState& scene_state,
           const ArticulatedIcpErrorSet& error_set) override;
  void Finalize() override;

  /**
   * Total cost of L2 norm of the errors.
   */
  double cost() const {
    DRAKE_ASSERT(finalized_);
    return cost_;
  }

  /**
   * Render cost to to be incorporated into a MathematicalProgram.
   */
  void UpdateCost(solvers::QuadraticCost* cost) const;

 private:
  Eigen::MatrixXd Q_;
  Eigen::VectorXd b_;
  double c_{};
  double cost_{};
  int num_points_{};
  bool finalized_{};
};

/**
 * Contains a group of points to be rendered in a linearized ICP cost.
 */
class ArticulatedIcpBodyPoints {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(ArticulatedIcpBodyPoints)
  /**
   * @param frame_Bi Body's frame index.
   * @param num_max Maximum number of points to be stored.
   */
  ArticulatedIcpBodyPoints(FrameIndex frame_Bi, int num_max)
      : frame_Bi_(frame_Bi),
        num_max_(num_max) {
    meas_pts_W_.resize(3, num_max);
    body_pts_W_.resize(3, num_max);
  }
  /**
   * Add a given measured and body point, both in the world frame.
   * @param meas_W Measured point, fixed in camera frame, expressed in world
   * frame.
   * @param body_W Body point, expressed in world frame.
   */
  void Add(const Eigen::Vector3d& meas_W, const Eigen::Vector3d& body_W);
  /**
   * Call once all points are added.
   */
  void Finalize();
  /**
   * Add this sets of points to a given articulated error set.
   */
  void ComputeError(const SceneState& scene_state,
                    ArticulatedIcpErrorSet* es) const;

 private:
  /* Frame of the given body. */
  FrameIndex frame_Bi_{-1};
  /* Points measured, fixed in camera frame, `C`, expressed in the world frame,
   * `W`. */
  Eigen::Matrix3Xd meas_pts_W_;
  /* Points on the given body, `Bi`, fixed in the body frame, expressed in the
   * world frame, `W`. */
  Eigen::Matrix3Xd body_pts_W_;
  int num_max_{};
  int num_actual_{0};
};

/**
 * Simple point-to-point correspondence.
 */
struct PointCorrespondence {
  PointCorrespondence() {}
  PointCorrespondence(const Eigen::Vector3d& meas_point_in,
                      const Eigen::Vector3d& model_point_in,
                      double distance_in)
    : meas_point(meas_point_in),
      model_point(model_point_in),
      distance(distance_in) {}
  /** @brief Measured point. */
  Eigen::Vector3d meas_point;
  /** @brief Model point, same frame as measured point. */
  Eigen::Vector3d model_point;
  /** @brief Distance between the points. */
  double distance{-1};
};

typedef std::map<BodyIndex, std::vector<PointCorrespondence>>
    ArticulatedPointCorrespondences;

/**
 * Description on how the optimization affects a body.
 *
 * Please see the documentation for each of the fields for how the
 * influences are determined.
 */
struct ArticulatedBodyInfluence {
  /**
   * @brief Indicate that a correspondence with this body will influence the
   * camera.
   *
   * This is true if the camera frame is not fixed to the world frame, in
   * which case the appropriate degrees of freedom will be influenced by the
   * estimator.
   */
  bool will_affect_camera{};

  /**
   * @brief Indicate that a correspondence with this body will influence the
   * model.
   *
   * If the body is a fixed instance (either no modeled degrees of freedom
   * between the world, or the object / the necessary degrees of freedom are
   * excluded from the optimization), then the body itself will not be
   * influenced by the estimator.
   */
  bool will_affect_model{};

  /**
   * @brief Check if a correspondence with this body will influence the
   * estimation.
   */
  bool is_influential() const {
    return will_affect_camera || will_affect_model;
  }
};

typedef std::vector<ArticulatedBodyInfluence> ArticulatedBodyInfluences;

/**
 * Determines if a correspondence with a given body influences the optimization
 * formulation (camera or body position).
 * @see ArticulatedBodyInfluence for more information on how this is determined.
 */
ArticulatedBodyInfluence IsBodyCorrespondenceInfluential(const Scene& scene,
                                                         BodyIndex);

/**
 * Compute all body correspondences for the entire scene.
 * @see ArticulatedBodyInfluence for more information on how this is determined.
 */
void ComputeBodyCorrespondenceInfluences(const Scene& scene,
                                         ArticulatedBodyInfluences* influences);

/**
 * Compute point-to-point correspondences from a measured point cloud and a
 * given scene.
 */
void ComputeCorrespondences(const SceneState& scene_state,
                            const ArticulatedBodyInfluences& influences,
                            const Eigen::Matrix3Xd& meas_pts_W,
                            ArticulatedPointCorrespondences* pcorrespondence);

/**
 * Compute cost for a given set of correspondences.
 */
void ComputeCost(const SceneState& scene_state,
                 const ArticulatedPointCorrespondences& correspondence,
                 ArticulatedIcpErrorCost* cost);

}  // namespace estimators
}  // namespace perception
}  // namespace drake
