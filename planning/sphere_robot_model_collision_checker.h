#pragma once

#include <functional>
#include <limits>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include <Eigen/Geometry>

#include "drake/geometry/scene_graph.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/tree/body.h"
#include "drake/planning/collision_checker.h"
#include "drake/planning/collision_checker_params.h"
#include "planning/robot_diagram.h"

namespace anzu {
namespace planning {

/// Class modelling collision spheres used for collision checking.
/// This code uses Vector4d because Vector4d allows for SIMD vector operations
/// when performing Isometry3d * Vector4d, which Vector3d does not.
class SphereSpecification {
 public:
  /// Make a sphere.
  /// @param p_BSo Origin of sphere S in frame of body B.
  /// @param radius Radius of sphere.
  SphereSpecification(const Eigen::Vector3d& p_BSo, double radius) {
    DRAKE_THROW_UNLESS(radius >= 0.0);
    p_BSo_ = Eigen::Vector4d(p_BSo.x(), p_BSo.y(), p_BSo.z(), 1.0);
    radius_ = radius;
  }

  /// Make a sphere.
  /// @param p_BSo Origin of sphere S in frame of body B, as Eigen::Vector4d to
  /// match the fastest path in VoxelGrid<T> lookups. Note that last element of
  /// vector *must* be 1.0.
  /// @param radius Radius of sphere.
  SphereSpecification(const Eigen::Vector4d& p_BSo, double radius)
      : p_BSo_(p_BSo), radius_(radius) {
    DRAKE_THROW_UNLESS(p_BSo_(3) == 1.0);
    DRAKE_THROW_UNLESS(radius_ >= 0.0);
  }

  /// Make a sphere at position p_BSo, origin of sphere S in frame of body B.
  /// @param x X value of p_BSo.
  /// @param y Y value of p_BSo.
  /// @param z Z value of p_BSo.
  /// @param radius Radius of sphere.
  /// p_BSo is provided by (x, y, z)
  SphereSpecification(double x, double y, double z, double radius) {
    DRAKE_THROW_UNLESS(radius >= 0.0);
    p_BSo_ = Eigen::Vector4d(x, y, z, 1.0);
    radius_ = radius;
  }

  const Eigen::Vector4d& Origin() const { return p_BSo_; }

  double Radius() const { return radius_; }

  void SetOrigin(const Eigen::Vector4d& p_BSo) {
    DRAKE_THROW_UNLESS(p_BSo_(3) == 1.0);
    p_BSo_ = p_BSo;
  }

 private:
  Eigen::Vector4d p_BSo_ = Eigen::Vector4d(0.0, 0.0, 0.0, 1.0);
  double radius_ = 0.0;
};

/// Wrapper that combines a distance value and gradient.
/// This code uses Vector4d because Vector4d allows for SIMD vector operations,
/// which Vector3d does not.
class DistanceAndGradient {
 public:
  DistanceAndGradient(
      double distance, const Eigen::Vector4d& gradient,
      drake::multibody::BodyIndex colliding_body_index)
      : distance_(distance), gradient_(gradient),
        colliding_body_index_(colliding_body_index) {
    if (gradient_(3) != 0.0) {
      throw std::invalid_argument("gradient(3) != 0.0");
    }
  }

  DistanceAndGradient(
      double distance, drake::multibody::BodyIndex colliding_body_index)
      : distance_(distance), colliding_body_index_(colliding_body_index) {}

  const double& Distance() const { return distance_; }

  const Eigen::Vector4d& Gradient() const { return gradient_; }

  const drake::multibody::BodyIndex& CollidingBodyIndex() const {
    return colliding_body_index_;
  }

  std::string Print() const {
    std::string rep = "Distance: " + std::to_string(Distance()) +
                      " Gradient: (" + std::to_string(Gradient()(0)) + "," +
                      std::to_string(Gradient()(1)) + "," +
                      std::to_string(Gradient()(2)) + ") + Colliding body: " +
                      std::to_string(CollidingBodyIndex());
    return rep;
  }

 private:
  double distance_ = std::numeric_limits<double>::infinity();
  Eigen::Vector4d gradient_ = Eigen::Vector4d(0.0, 0.0, 0.0, 0.0);
  drake::multibody::BodyIndex colliding_body_index_{};
};

/// Wrapper for multiple distance and gradient values. Keeps track of the
/// minimum distance of all DistanceAndGradients added.
class PointSignedDistanceAndGradientResult {
 public:
  PointSignedDistanceAndGradientResult() {}

  void AddDistanceAndGradient(
      double distance, const Eigen::Vector4d& gradient,
      drake::multibody::BodyIndex colliding_body_index) {
    distances_and_gradients_.emplace_back(
        distance, gradient, colliding_body_index);
    if (distance < minimum_distance_) {
      minimum_distance_ = distance;
      minimum_colliding_body_index_ = colliding_body_index;
    }
  }

  void AddDistance(
      double distance, drake::multibody::BodyIndex colliding_body_index) {
    distances_and_gradients_.emplace_back(distance, colliding_body_index);
    if (distance < minimum_distance_) {
      minimum_distance_ = distance;
      minimum_colliding_body_index_ = colliding_body_index;
    }
  }

  double MinimumDistance() const { return minimum_distance_; }

  // If MinimumDistance is finite, returns the colliding BodyIndex which
  // reported that minimum distance.
  drake::multibody::BodyIndex MinimumCollidingBodyIndex() const {
    return minimum_colliding_body_index_;
  }

  const DistanceAndGradient& GetDistanceAndGradient(size_t index) const {
    return distances_and_gradients_.at(index);
  }

  size_t NumberOfGradients() const { return distances_and_gradients_.size(); }

  std::string Print() const {
    std::string rep = "Min distance: " + std::to_string(MinimumDistance()) +
                      " Distances & Gradients:";
    for (size_t idx = 0; idx < NumberOfGradients(); idx++) {
      const DistanceAndGradient& distance_and_gradient =
          GetDistanceAndGradient(idx);
      rep += ("\n" + std::to_string(idx + 1) + " - " +
              distance_and_gradient.Print());
    }
    return rep;
  }

 private:
  std::vector<DistanceAndGradient> distances_and_gradients_;
  double minimum_distance_ = std::numeric_limits<double>::infinity();
  drake::multibody::BodyIndex minimum_colliding_body_index_;
};

/// Base class for collision checkers using a sphere-geometry robot model.
/// Derived classes must implement distance and gradient checks against
/// environment geometry.
/// Note: this class is designed such that derived classes can support
/// copy/move/assignment.
class SphereRobotModelCollisionChecker
    : public drake::planning::CollisionChecker {
 public:
  /** @name     Does not allow copy, move, or assignment. */
  /** @{ */
  // N.B. The copy constructor is protected for use in implementing Clone().
  void operator=(const SphereRobotModelCollisionChecker&) = delete;
  /** @} */

  /// Update the sphere collision model of a specific body of the robot.
  /// @param body_index Index of the body (body 0 is world).
  /// @param spheres Sphere geometry to add to the collision model.
  /// @param append If true, appends the spheres to the existing model,
  /// otherwise the new model replaces the existing model.
  /// Note: changes to the collision model here *do not* go through the added
  /// geometry mechanism.
  void UpdateBodyCollisionModel(drake::multibody::BodyIndex body_index,
                                const std::vector<SphereSpecification>& spheres,
                                bool append);

  /// Get a reference to the internal collision model of the robot.
  const std::vector<
      std::unordered_map<drake::geometry::GeometryId, SphereSpecification>>&
  RobotCollisionModel() const {
    return robot_sphere_model_;
  }

  /// Get a mutable reference to the internal collision model of the robot.
  std::vector<
      std::unordered_map<drake::geometry::GeometryId, SphereSpecification>>&
  MutableRobotCollisionModel() {
    return robot_sphere_model_;
  }

  /// Generates the URDF <collision> elements corresponding to the robot's
  /// collision model.
  std::string GetURDFCollisionGeometriesForRobotCollisionModel() const;

  /// Check if the robot is in self-collision.
  /// @param X_WB_set Vector of X_WB, pose of robot body B in world W.
  /// @return true if self-collision free, false otherwise.
  bool CheckSelfCollisionFree(
      const std::vector<Eigen::Isometry3d>& X_WB_set) const;

  /// Compute pose of all bodies, using the current thread's context.
  /// @param q Configuration to compute poses for.
  /// @return Vector of X_WB, pose of body B in world W, for all bodies.
  std::vector<Eigen::Isometry3d> ComputeBodyPoses(
      const Eigen::VectorXd& q) const;

  /// Get pose of all bodies.
  /// @param context MbP context, already updated, to use.
  /// @return Vector of X_WB, pose of body B in world W, for all bodies.
  std::vector<Eigen::Isometry3d> GetBodyPoses(
      const drake::systems::Context<double>& plant_context) const;

  /// Compute sphere collision model in world frame.
  /// @param X_WB_set Body poses to use.
  /// @return Vector of link sphere models.
  std::vector<
      std::unordered_map<drake::geometry::GeometryId, SphereSpecification>>
  ComputeSphereLocationsInWorldFrame(
      const std::vector<Eigen::Isometry3d>& X_WB_set) const;

  /// Compute sphere collision model in world frame, using the current thread's
  /// context.
  /// @param q Configuration to compute poses for.
  /// @return Vector of link sphere models.
  std::vector<
      std::unordered_map<drake::geometry::GeometryId, SphereSpecification>>
  ComputeSphereLocationsInWorldFrame(const Eigen::VectorXd& q) const {
    return ComputeSphereLocationsInWorldFrame(ComputeBodyPoses(q));
  }

  const std::unordered_set<drake::geometry::GeometryId>& RobotGeometries()
      const {
    return robot_geometries_;
  }

  /// Query the (distance, gradient) of the provided point from obstacles.
  /// @param context Context of the MbP model.
  /// @param query_object Query object for `context`.
  /// @param p_WQ Query position in world frame W.
  /// @param query_radius Gradients do not need to be computed for queries
  /// with distance > query radius. This allows for faster performance
  /// by reducing the number of gradients to compute as well as reducing the
  /// number of gradients this method returns.
  /// @param X_WB_set Poses X_WB for all bodies in the model. This is
  /// used to move gradients from signed distance fields back to world frame.
  /// @param X_WB_inverse_set Poses X_BW for all bodies in the model. This is
  /// useful for some implementations to move the provided p_WQ into a different
  /// body frame.
  /// @return signed distances and gradients where signed distance is positive
  /// if @param p_WQ is outside of objects, and negative if it is inside. The
  /// gradient is ∂d/∂p. Note that queries farther than @param
  /// query_radius from collision need not return exact distance.
  virtual PointSignedDistanceAndGradientResult
  ComputePointToEnvironmentSignedDistanceAndGradient(
      const drake::systems::Context<double>& context,
      const drake::geometry::QueryObject<double>& query_object,
      const Eigen::Vector4d& p_WQ, double query_radius,
      const std::vector<Eigen::Isometry3d>& X_WB_set,
      const std::vector<Eigen::Isometry3d>& X_WB_inverse_set) const = 0;

  /// Query the distance of the provided point from obstacles.
  /// @param context Context of the MbP model.
  /// @param query_object Query object for `context`.
  /// @param p_WQ Query position in world frame W.
  /// @param query_radius Gradients do not need to be computed for queries
  /// with distance > query_radius. This parameter is needed because the
  /// default implementation calls ComputePointSignedDistanceAndGradient, and
  /// only needing to check within a bound can improve performance.
  /// @param X_WB_set Poses X_WB for all bodies in the model. This is
  /// used to move gradients from signed distance fields back to world frame.
  /// @param X_WB_inverse_set Poses X_BW for all bodies in the model. This is
  /// useful for some implementations to move the provided p_WQ into a different
  /// body frame.
  /// @return signed distances where signed distance is positive
  /// if @param p_WQ is outside of objects, and negative if it is inside.
  /// The default implementation here is always sufficient, but in some cases,
  /// it may be faster for an implementation to only compute the distance,
  /// rather than the distance gradient. Since the binary collision checking
  /// methods only need distance, we want them to be as fast as possible.
  /// Note that queries farther than @param query_radius from collision need not
  /// return exact distance.
  virtual PointSignedDistanceAndGradientResult
  ComputePointToEnvironmentSignedDistance(
      const drake::systems::Context<double>& context,
      const drake::geometry::QueryObject<double>& query_object,
      const Eigen::Vector4d& p_WQ, double query_radius,
      const std::vector<Eigen::Isometry3d>& X_WB_set,
      const std::vector<Eigen::Isometry3d>& X_WB_inverse_set) const {
    return ComputePointToEnvironmentSignedDistanceAndGradient(
        context, query_object, p_WQ, query_radius, X_WB_set, X_WB_inverse_set);
  }

  PointSignedDistanceAndGradientResult
  ComputeSelfCollisionSignedDistanceAndGradient(
      const std::vector<
          std::unordered_map<drake::geometry::GeometryId, SphereSpecification>>&
          sphere_locations_in_world_frame,
      drake::multibody::BodyIndex query_link_index,
      drake::geometry::GeometryId query_sphere_id,
      double influence_distance) const;

 protected:
  /// Construct a base collision checker for sphere-geometry robot models.
  /// @param model a Diagram+MbP+SG model of the robot and environment.
  /// @param robot_model_instances is a vector of model instance indices that
  /// identify which model instances belong to the robot.
  /// @param distance_fn Configuration (probably weighted) distance function.
  /// @param edge_step_size Step size for collision checking, in radians.
  /// Collision checking of edges q1->q2 is performed by interpolating from q1
  /// to q2 at edge_step_size steps and checking the interpolated
  /// configuration for collision.
  /// @param env_collision_padding Additional padding to apply to all
  /// robot-environment collision queries. If distance between robot and
  /// environment is less than padding, the checker reports a collision.
  /// @param self_collision_padding Additional padding to apply to all
  /// robot-robot self collision queries. If distance between robot and
  /// itself is less than padding, the checker reports a collision.
  explicit SphereRobotModelCollisionChecker(
      drake::planning::CollisionCheckerParams params);

  /// To support Clone(), allow copying (but not move nor assign).
  explicit SphereRobotModelCollisionChecker(
      const SphereRobotModelCollisionChecker&);

  virtual std::optional<drake::geometry::GeometryId>
  AddEnvironmentCollisionShapeToBody(
      const std::string& group_name,
      const drake::multibody::Body<double>& bodyA,
      const drake::geometry::Shape& shape,
      const drake::math::RigidTransform<double>& X_AG) = 0;

  virtual void RemoveAllAddedEnvironment(
      const std::vector<drake::planning::CollisionChecker::AddedShape>&
          shapes) = 0;

 private:
  /// Implement from CollisionChecker; no additional actions are required to
  /// update positions.
  void DoUpdateContextPositions(
      drake::planning::CollisionCheckerContext*) const override {}

  /// Implement from CollisionChecker.
  bool DoCheckContextConfigCollisionFree(
      const drake::planning::CollisionCheckerContext& model_context)
      const override;

  /// Implement from CollisionChecker.
  std::optional<drake::geometry::GeometryId> DoAddCollisionShapeToBody(
      const std::string& group_name,
      const drake::multibody::Body<double>& bodyA,
      const drake::geometry::Shape& shape,
      const drake::math::RigidTransform<double>& X_AG) override;

  /// Implement from CollisionChecker.
  void RemoveAddedGeometries(
      const std::vector<drake::planning::CollisionChecker::AddedShape>& shapes)
      override;

  /// Implement from CollisionChecker; no additional actions are required to
  /// update collision filters.
  void UpdateCollisionFilters() override {}

  // TODO(sean.curtis): This implementation is not compliant with the
  // documented requirements. This aggregates clearance measurements so that
  // the partials are not truly the partials for the reported minimum distance.
  // This needs to be corrected so it is compliant.
  /// Implement from CollisionChecker.
  drake::planning::RobotClearance DoCalcContextRobotClearance(
      const drake::planning::CollisionCheckerContext& model_context,
      double influence_distance) const override;

  /// Implement from CollisionChecker.
  std::vector<drake::planning::RobotCollisionType>
  DoClassifyContextBodyCollisions(
      const drake::planning::CollisionCheckerContext& model_context)
      const override;

  /// Implement from CollisionChecker.
  int DoMaxContextNumDistances(const drake::planning::CollisionCheckerContext&
                                   model_context) const override;

  /// Check if the robot is in collision with environment.
  /// @return true if environment-collision free, false otherwise.
  bool CheckEnvironmentCollisionFree(
      const drake::systems::Context<double>& context,
      const drake::geometry::QueryObject<double>& query_object,
      const std::vector<Eigen::Isometry3d>& X_WB_set,
      const std::vector<Eigen::Isometry3d>& X_WB_inverse_set,
      const std::vector<
          std::unordered_map<drake::geometry::GeometryId, SphereSpecification>>&
          spheres_in_world_frame) const;

  /// Check if the robot is in self-collision.
  /// @return true if self-collision free, false otherwise.
  bool CheckSelfCollisionFree(
      const std::vector<
          std::unordered_map<drake::geometry::GeometryId, SphereSpecification>>&
          spheres_in_world_frame) const;

  /// Internal helper to build the internal models of the robot.
  void InitializeRobotModel();

  /// Store the geometry ids of the geometries that make up the robot.
  std::unordered_set<drake::geometry::GeometryId> robot_geometries_;

  /// Internal storage of the robot's sphere model geometry.
  std::vector<
      std::unordered_map<drake::geometry::GeometryId, SphereSpecification>>
      robot_sphere_model_;
};
}  // namespace planning
}  // namespace anzu
