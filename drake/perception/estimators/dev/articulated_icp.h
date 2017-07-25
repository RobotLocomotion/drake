#pragma once

#include <map>
#include <vector>

#include "drake/perception/estimators/dev/scene.h"
#include "drake/solvers/cost.h"

namespace drake {
namespace perception {
namespace estimators {

/**
 * A set of Cartesian errors and their gradients.
 */
class ArticulatedIcpErrorSet {
 public:
  /**
   * @param num_var Number of variables for the gradients.
   */
  explicit ArticulatedIcpErrorSet(int num_var) : num_var_(num_var) {}

  /**
   * Resize (and clear) the vector.
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
  const int num_var_{};
};

/**
 * Accumulate errors and render the cost into an appropriate format.
 */
class ArticulatedIcpErrorCost {
 public:
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
 * Aggregate L2 norm cost of point-to-point errors.
 */
// TODO(eric.cousineau): Consider merging with QP cost aggregator.
class ArticulatedIcpErrorNormCost : public ArticulatedIcpErrorCost {
 public:
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
  const bool do_normalize_{};
  double cost_{};
  Eigen::MatrixXd J_cost_;
  double num_points_;
  bool finalized_{};
};

/**
 * Accumulate the L2 norm of the linearized point-to-point errors (e) to be
 * rendered into a QuadraticCost operating on variables `q`:
 * | e + Je*q |^2
 */
class ArticulatedIcpLinearizedNormCost : public ArticulatedIcpErrorCost {
 public:
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

  double cost() const {
    DRAKE_ASSERT(finalized_);
    return c_;
  }
  void UpdateCost(solvers::QuadraticCost* cost) const;

 private:
  Eigen::MatrixXd Q_;
  Eigen::VectorXd b_;
  double num_points_;
  double c_;
  bool finalized_{};
};

/**
 * Contains a group of points to be rendered in a linearized ICP cost.
 */
class ArticulatedIcpBodyPoints {
 public:
  /**
   * @param frame_Bi Body's frame index.
   * @param num_max Maximum number of points to be stored.
   */
  ArticulatedIcpBodyPoints(FrameIndex frame_Bi, int num_max)
      : frame_Bi(frame_Bi) {
    meas_pts_W.resize(3, num_max);
    body_pts_W.resize(3, num_max);
  }
  /**
   * Add a given measured and body point, both in the world frame.
   * @param meas_W Measured point, world frame.
   * @param body_W Body point, world frame.
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
  const FrameIndex frame_Bi{-1};
  Eigen::Matrix3Xd meas_pts_W;
  Eigen::Matrix3Xd body_pts_W;
  int num_actual{0};
};

/**
 * Simple point-to-point correspondence.
 */
struct PointCorrespondence {
  /** @brief Measured point. */
  Eigen::Vector3d meas_point;
  /** @brief Model point, same frame as measured point. */
  Eigen::Vector3d model_point;
  /** @brief Distance between the points. */
  double distance{-1};
};

typedef std::map<BodyIndex, std::vector<PointCorrespondence>>
    ArticulatedPointCorrespondences;

struct ArticulatedBodyInfluence {
  /**
   * @brief Will a correspondence with this body affect the camera
   * positions?
   */
  bool will_affect_camera{};
  /**
   * @brief Will a correspondence with this body affect the body's kinematics?
   */
  bool will_affect_model{};
  /**
   * @brief Check if a correspondence with this body will influence the
   * Jacobian at all.
   */
  bool is_influential() const {
    return will_affect_camera || will_affect_model;
  }
};
typedef std::vector<ArticulatedBodyInfluence> ArticulatedBodyInfluences;

/**
 * Determines if a correspondence with a given body influences the optimization
 * formulation (camera or body position).
 */
ArticulatedBodyInfluence IsBodyCorrespondenceInfluential(const Scene& scene,
                                                         BodyIndex);

/**
 * Compute all body correspondences for the entire scene.
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
