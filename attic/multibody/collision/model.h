#pragma once

#include <memory>
#include <unordered_map>
#include <utility>
#include <vector>

#include <Eigen/Dense>

#include "drake/common/drake_copyable.h"
#include "drake/multibody/collision/element.h"
#include "drake/multibody/collision/point_pair.h"

namespace drake {
namespace multibody {
namespace collision {
typedef std::pair<ElementId, ElementId> ElementIdPair;

/** Model is an abstract base class of a collision model. Child classes of Model
 implement the actual collision detection logic. */
class Model {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(Model)

  Model() {}

  virtual ~Model() {}

  /** Adds a collision element to this model.

   This operation is frequently referred to as "registering" the collision
   element.  This is the process by which a fully-realized Drake collision
   element is fed to a specific drake::multibody::collision::Model
   implementation.  Prior to this *registration*, the collision model knows
   nothing about the collision element.  After registration, it owns the
   collision element.

   @param element The element to add.

   @return A pointer to the added element.

   @throws std::runtime_error if there was a problem (e.g., duplicate element
   id, error configuring collision model, etc.) */
  Element* AddElement(std::unique_ptr<Element> element);

  /** Removes a collision element from this model.

   @param id The id of the element that will be removed.

   @return True if the element is successfully removed, false if the model does
   not contain an element with the given id. */
  bool RemoveElement(ElementId id);

  /** Gets a read-only pointer to a collision element in this model.

   @param id An ElementId corresponding to the desired collision element

   @return A read-only pointer to the collision element corresponding to
   the given `id` or `nullptr` if no such collision element is present in the
   model. */
  virtual const Element* FindElement(ElementId id) const;

  /** Gets a pointer to a mutable collision element in this model.

   @param[in] id An ElementId corresponding to the desired collision
   element.

   @returns A pointer to a mutable collision element corresponding to
   the given `id` or `nullptr` if no such collision element is present in the
   model. */
  virtual Element* FindMutableElement(ElementId id);

  void GetTerrainContactPoints(ElementId id0, Eigen::Matrix3Xd* terrain_points);

  /** Informs the model that the collision filter criteria for the given `id`
   has changed (or *may* have changed -- it can be called conservatively). The
   model has the opportunity to perform whatever is necessary to respond.

   * @param id  The id of the element whose filter criteria may have changed.
   */
  virtual void NotifyFilterCriteriaChanged(ElementId) {}

  /** Updates the collision model. This method is typically called after changes
   are made to its collision elements. */
  virtual void UpdateModel() = 0;

  /** Updates the stored transformation from a collision element's canonical
  space to world space (`X_WE`), where `X_WE = X_WL * X_LE`. `X_LE` is the
  transform that maps the element to its parent body's space, referred to as
  "local". `X_WL` maps the body/local space to the world.

   @param id The ID of the element being updated.

   @param X_WL The new value for the local-to-world transform. It reflects the
   current world pose of the parent body in a given context.

   @return Whether the update was successful. */
  virtual bool UpdateElementWorldTransform(
      ElementId id, const Eigen::Isometry3d& X_WL);

  /** Computes the points of closest approach between all eligible pairs of
   collision elements drawn from a specified set of elements

   @param ids_to_check The vector of ElementId for which the all-to-all
   collision detection should be performed

   @param use_margins A flag indicating whether or not to use the version of
   this model with collision margins

   @param[out] closest_points A reference to a vector of PointPair objects that
   contains the closest point information after this method is called

   @return Whether this method successfully ran. */
  virtual bool ClosestPointsAllToAll(
      const std::vector<ElementId>& ids_to_check, bool use_margins,
      std::vector<PointPair<double>>* closest_points) = 0;

  /** Computes the point of closest approach between collision elements that
   are in contact.

   @param[in] use_margins If `true` the model uses the representation with
   margins. If `false`, the representation without margins is used instead.

   @param[out] closest_points A reference to a vector of PointPair objects
   that contains the closest point information after this method is called.

   @returns Whether this method successfully ran. */
  virtual bool ComputeMaximumDepthCollisionPoints(
      bool use_margins, std::vector<PointPair<double>>* closest_points) = 0;

  /** Computes the points of closest approach between specified pairs of
   collision elements.

   @param id_pairs A vector of ElementIdPair specifying which pairs of
   elements to consider

   @param use_margins A flag indicating whether or not to use the version
   of this model with collision margins

   @param[out] closest_points A reference to a vector of PointPair objects
   that contains the closest point information after this method is
   called

   @return Whether this method successfully ran. */
  virtual bool ClosestPointsPairwise(
      const std::vector<ElementIdPair>& id_pairs, bool use_margins,
      std::vector<PointPair<double>>* closest_points) = 0;

  /** Clears possibly cached results so that a fresh computation can be
  performed.

  @param[in] use_margins If `true`, the cache of the model with margins is
  cleared. If `false`, the cache of the model without margins is cleared.

  Depending on the implementation, the collision model may cache results on each
  dispatch. For instance, Bullet uses cached results to warm-start its LCP
  solvers.

  Clearing cached results allows the collision model to perform a fresh
  computation without any coupling with previous history.

  @see drake/multibody/collision/test/model_test.cc. */
  virtual void ClearCachedResults(bool use_margins) = 0;

  /** Computes the closest distance from each point to any surface in the
   collision model utilizing Bullet's collision detection code.

   @param points An ordered list of `N` points represented column-wise by a
   `3 x N` Matrix.

   @param use_margins A flag indicating whether to use the version of this
   model with collision margins.

   @param[out] closest_points A vector of `N` PointPair instances such that the
   i'th instance reports the query result for the i'th input point. */
  virtual void CollisionDetectFromPoints(
      const Eigen::Matrix3Xd& points, bool use_margins,
      std::vector<PointPair<double>>* closest_points) = 0;

  // TODO(SeanCurtis-TRI): Add a C++ version of "collidingPointsTest.m". Once
  // such a test exists, update the @see reference to it below.
  //
  /** Given a vector of points in the world coordinate frame, returns the
  indices of those points that are within the provided `collision_threshold`
  distance of any collision geometry in the model.

  In other words, the index `i` is included in the returned vector of indices
  iff a sphere of radius `collision_threshold`, located at `input_points[i]`
  collides with any collision element in the model.

  @param input_points The list of points to check for collisions against the
  model.

  @param collision_threshold The radius of a control sphere around each point
  used to check for collisions with the model.

  @return A vector with indexes in input_points of all those points that do
  collide with the model within the specified threshold.

  @see drake/matlab/systems/plants/test/collidingPointsTest.m for a MATLAB
  %test. */
  virtual std::vector<size_t> CollidingPoints(
      const std::vector<Eigen::Vector3d>& input_points,
      double collision_threshold) = 0;

  /** Given a vector of points in the world coordinate frame, reports if _any_
  of those points lie within a specified distance of any collision geometry in
  the model.

  In other words, this method tests if any of the spheres of radius
  `collision_threshold` located at `input_points[i]` collides with any part of
  the model. This method returns as soon as any of these spheres collides
  with the model. Points are not checked against one another but only against
  the existing model.

  @param input_points The list of points to check for collisions against the
  model.

  @param collision_threshold The radius of a control sphere around each point
  used to check for collisions with the model.

  @return Whether any of the points positively checks for collision. */
  virtual bool CollidingPointsCheckOnly(
      const std::vector<Eigen::Vector3d>& input_points,
      double collision_threshold) = 0;

  /** Performs a raycast intersection test (like a LIDAR / laser range finder).

   @param origin A 3 x N matrix where each column specifies the position of a
   ray's origin in the world coordinate frame.  If `origin` has dimensions of
   3 x 1, the same origin is used for all rays.

   @param ray_endpoint A 3 x N matrix where each column specifies a second point
   on the corresponding ray.

   @param use_margins A flag indicating whether or not to use the version of
   this model with collision margins.

   @param[out] distance The distance to the first collision, or -1 if no
   collision occurs.

   @return Whether this method successfully ran. */
  virtual bool CollisionRaycast(const Eigen::Matrix3Xd& origin,
                                const Eigen::Matrix3Xd& ray_endpoint,
                                bool use_margins, Eigen::VectorXd* distances,
                                Eigen::Matrix3Xd* normals) = 0;

  /** Modifies a collision element's local transform to be relative to a joint's
   frame rather than a link's frame. This is necessary because Drake requires
   that link frames be defined by their parent joint frames.

   @param eid The ID of the collision element to update.

   @param transform_body_to_joint The transform from the collision element's
   link's frame to the joint's coordinate frame.

   @return Whether the collision element was successfully updated. */
  virtual bool TransformCollisionFrame(
      const drake::multibody::collision::ElementId& eid,
      const Eigen::Isometry3d& transform_body_to_joint);

  /** A toString method for this class. */
  friend std::ostream& operator<<(std::ostream&, const Model&);

 protected:
  /** Allows sub-classes to do additional processing on elements added to the
   collision model.  This is called each time Model::AddElement is called.

   @param element The element that has been added.

   @throws std::runtime_error If there was a problem processing the element. */
  virtual void DoAddElement(const Element& element);

  /** Allows sub-classes to do additional processing when elements are
   removed from the collision model. This is called by Model::RemoveElement()
   prior to removing id from elements. The derived class should not do this
   removal.

   @param id The id of the element that will be removed. */
  virtual void DoRemoveElement(ElementId id);

  // Protected member variables are forbidden by the style guide.
  // Please do not add new references to this member.  Instead, use
  // the accessors.
  std::unordered_map<ElementId, std::unique_ptr<Element>> elements;
};

}  // namespace collision
}  // namespace multibody
}  // namespace drake
