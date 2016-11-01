#pragma once

#include <memory>
#include <unordered_map>
#include <utility>
#include <vector>

#include <Eigen/Dense>

#include "drake/common/drake_export.h"
#include "drake/systems/plants/collision/Element.h"
#include "drake/systems/plants/collision/point_pair.h"

namespace DrakeCollision {
typedef std::pair<ElementId, ElementId> ElementIdPair;

class DRAKE_EXPORT Model {
 public:
  Model() {}

  virtual ~Model() {}

  /** \brief Add a collision element to this model.
  * \param element the collision element to be added to this model
  * \return an ElementId that uniquely identifies the added element within
  * this model
  */
  virtual ElementId addElement(const Element& element);

  bool removeElement(const ElementId& id);

  /** \brief Get a read-only pointer to a collision element in this model.
   * \param id an ElementId corresponding to the desired collision element
   * \return a read-only pointer to the collision element corresponding to
   * the given id or nullptr if no such collision element is present in the
   * model.
   */
  virtual const Element* FindElement(ElementId id) const;

  /** Gets a pointer to a mutable collision element in this model.
   * @param[in] id an ElementId corresponding to the desired collision
   * element.
   * @returns a pointer to a mutable collision element corresponding to
   * the given id or nullptr if no such collision element is present in the
   * model.
   **/
  virtual Element* FindMutableElement(ElementId id);

  virtual void getTerrainContactPoints(
      ElementId id0,
      // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
      Eigen::Matrix3Xd& terrain_points);

  /** \brief Perform any operations needed to bring the model up-to-date
   * after making changes to its collision elements
   */
  virtual void updateModel() = 0;

  /** \brief Change the element-to-world transform of a specified collision
   * element.
   * \param id an ElementId corresponding to the element to be updated
   * \param T_local_to_world the new value for the element-to-world
   * transform
   */
  virtual bool updateElementWorldTransform(
      const ElementId id, const Eigen::Isometry3d& T_local_to_world);

  /** \brief Compute the points of closest approach between all eligible
   * pairs of collision elements drawn from a specified set of elements
   * \param ids_to_check the vector of ElementId for which the all-to-all
   * collision detection should be performed
   * \param use_margins flag indicating whether or not to use the version
   * of this model with collision margins
   * \param[out] closest_points reference to a vector of PointPair objects
   * that contains the closest point information after this method is
   * called
   * \return true if this method ran successfully
   */
  virtual bool closestPointsAllToAll(const std::vector<ElementId>& ids_to_check,
                                     const bool use_margins,
                                     std::vector<PointPair>&
                                     closest_points) = 0;

  /** Computes the point of closest approach between collision elements that
   are in contact.

   @param[in] use_margins If `true` the model uses the representation with
   margins. If `false`, the representation without margins is used instead.

   @param[out] closest_points reference to a vector of PointPair objects
   that contains the closest point information after this method is called.

   @returns `true` if this method ran successfully and `false` otherwise.
   **/
  virtual bool ComputeMaximumDepthCollisionPoints(
      const bool use_margins,
      // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
      std::vector<PointPair>& closest_points) = 0;

  /** \brief Compute the points of closest approach between specified pairs
   * of collision elements
   * \param id_pairs vector of ElementIdPair specifying which pairs of
   * elements to consider
   * \param use_margins flag indicating whether or not to use the version
   * of this model with collision margins
   * \param[out] closest_points reference to a vector of PointPair objects
   * that contains the closest point information after this method is
   * called
   * \return true if this method ran successfully
   */
  virtual bool closestPointsPairwise(const std::vector<ElementIdPair>& id_pairs,
                                     const bool use_margins,
                                     std::vector<PointPair>&
                                     closest_points) = 0;

  /** Clears possibly cached results so that a fresh computation can be
  performed.

  @param use_margins[in] If `true`, the cache of the model with margins is
  cleared. If `false`, the cache of the model without margins is cleared.

  Depending on the implementation, the collision model may cache results on each
  dispatch. For instance, Bullet uses cached results to warm-start its LCP
  solvers.

  Clearing cached results allows the collision model to perform a fresh
  computation without any coupling with previous history.

  @see drake/systems/plants/collision/test/model_test.cc. **/
  virtual void ClearCachedResults(bool use_margins) = 0;

  /** \brief Compute closest distance from each point to any surface in the
   * collision model utilizing Bullet's collision detection code.
   * \param points Matrix of points computing distance from.
   * \param use_margins flag indicating whether or not to use the version
   * of this model with collision margins
   * \param[out] closest_points a vector of PointPair objects containing
   * the signed distances
   */
  virtual void collisionDetectFromPoints(
      const Eigen::Matrix3Xd& points, bool use_margins,
      // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
      std::vector<PointPair>& closest_points) = 0;

  /** \brief Compute the set of potential collision points for all
   * eligible pairs of collision geometries in this model. This includes
   * the points of closest approach, but may also include additional points
   * that are "close" to being in contact. This can be useful when
   * simulating scenarios in which two collision elements have more than
   * one point of contact.
   * \param use_margins flag indicating whether or not to use the version
   * of this model with collision margins
   * \return a vector of PointPair objects containing the potential
   * collision points
   */
  virtual std::vector<PointPair> potentialCollisionPoints(bool use_margins) = 0;

  /** Given a vector of points in world coordinates, returns the indices
  of those points within a specified distance of any collision geometry in
  the model.

  In other words, this method tests if a sphere of radius
  collision_threshold located at input_points[i] collides with any part of
  the model. The result is returned as a vector of indexes in input_points
  that do collide with the model.
  Points are not checked against one another but only against the existing
  model.

  @param input_points The list of points to check for collisions against the
  model.
  @param collision_threshold The radius of a control sphere around each point
  used to check for collisions with the model.

  @return A vector with indexes in input_points of all those points that do
  collide with the model within the specified threshold.

  @see systems/plants/test/collidingPointsTest.m for a Matlab test. **/
  virtual std::vector<size_t> collidingPoints(
      const std::vector<Eigen::Vector3d>& input_points,
      double collision_threshold) = 0;

  /** Tests if any of the points supplied in input_points collides with
  any part of the model within a given threshold.

  In other words, this method tests if any of the spheres of radius
  collision_threshold located at input_points[i] collides with any part of
  the model. This method returns as soon as any of these spheres collides
  with the model.
  Points are not checked against one another but only against the existing
  model.

  @param input_points The list of points to check for collisions against the
  model.
  @param collision_threshold The radius of a control sphere around each point
  used to check for collisions with the model.

  @return `true` if any of the points positively checks for collision.
  `false` otherwise. **/
  virtual bool collidingPointsCheckOnly(
      const std::vector<Eigen::Vector3d>& input_points,
      double collision_threshold) = 0;

  /** Performs raycasting collision detecting (like a LIDAR / laser rangefinder)
   *
   * \param origin 3 x N matrix in which each column specifies the position
   * of a ray's origin in world coordinates.  if origin is 3x1, then the same
   *origin is used for all raycasts
   * \param ray_endpoint 3 x N matrix in which each column specifies a
   * second point on the corresponding ray
   * \param use_margins flag indicating whether or not to use the version
   * of this model with collision margins
   * \param[out] distance to the first collision, or -1 on no collision
   * \return true if this method ran successfully
   */
  virtual bool collisionRaycast(
      const Eigen::Matrix3Xd& origin,
      const Eigen::Matrix3Xd& ray_endpoint,
      bool use_margins,
      // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
      Eigen::VectorXd& distances,
      // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
      Eigen::Matrix3Xd& normals) = 0;

  /**
   * Modifies a collision element's local transform to be relative to a joint's
   * frame rather than a link's frame. This is necessary because Drake requires
   * that link frames by defined by their parent joint frames.
   *
   * @param eid The ID of the collision element to update.
   * @param transform_body_to_joint The transform from the collision element's
   * link's frame to the joint's coordinate frame.
   * @param true if the collision element was successfully updated.
   */
  virtual bool transformCollisionFrame(
      const DrakeCollision::ElementId& eid,
      const Eigen::Isometry3d& transform_body_to_joint);

  /**
   * A toString method for this class.
   */
  friend DRAKE_EXPORT std::ostream& operator<<(std::ostream&,
                                                        const Model&);

 protected:
  // Protected member variables are forbidden by the style guide.
  // Please do not add new references to this member.  Instead, use
  // the accessors.
  std::unordered_map<ElementId, std::unique_ptr<Element>> elements;

 private:
  Model(const Model&) {}
  Model& operator=(const Model&) { return *this; }
};

}  // namespace DrakeCollision
