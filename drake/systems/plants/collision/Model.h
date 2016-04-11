#ifndef DRAKE_SYSTEMS_PLANTS_COLLISION_MODEL_H_
#define DRAKE_SYSTEMS_PLANTS_COLLISION_MODEL_H_

#include <memory>
#include <unordered_map>

#include <Eigen/Dense>
#include <Eigen/StdVector>

#include "Element.h"
#include "drake/drakeCollision_export.h"
#include "drake/systems/plants/collision/point_pair.h"

namespace DrakeCollision {
typedef std::pair<ElementId, ElementId> ElementIdPair;

class DRAKECOLLISION_EXPORT Model {
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
  virtual const Element* readElement(ElementId id) const;

  virtual void getTerrainContactPoints(ElementId id0,
                                       Eigen::Matrix3Xd& terrain_points);

  /** \brief Perform any operations needed to bring the model up-to-date
   * after making changes to its collision elements
   */
  virtual void updateModel() {}

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
                                     std::vector<PointPair>& closest_points) {
    return false;
  }

  /** \brief Compute the points of closest approach between all eligible
   * pairs of collision elements in this model
   * \param use_margins flag indicating whether or not to use the version
   * of this model with collision margins
   * \param[out] closest_points reference to a vector of PointPair objects
   * that contains the closest point information after this method is
   * called
   * \return true if this method ran successfully
   */
  virtual bool collisionPointsAllToAll(const bool use_margins,
                                       std::vector<PointPair>& points) {
    return false;
  }

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
                                     std::vector<PointPair>& closest_points) {
    return false;
  }

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
      std::vector<PointPair>& closest_points) {}

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
  virtual std::vector<PointPair> potentialCollisionPoints(
      const bool use_margins) {
    return std::vector<PointPair>();
  }

  /** \brief Given a vector of points in world coordinates, returns the
   * indices of those points within a specified distance of any collision
   * geometry in the model.
   * \param points a vector of points in world coordinates
   * \param collision_threshold points are considered "in collision" if
   * they lie within this distance of any collision geometry
   * \return a vector containing the indices of those points that are "in
   * collision" with the model.
   */
  virtual std::vector<size_t> collidingPoints(
      const std::vector<Eigen::Vector3d>& points, double collision_threshold) {
    return std::vector<size_t>();
  }

  /** \brief Returns true if any of the given points are within a specified
   * distance of the collision geometries in this model.
   * \param points a vector of points in world coordinates
   * \param collision_threshold points are considered "in collision" if
   * they lie within this distance of any collision geometry
   * \return a boolean value indicating if any points are "in collision"
   */
  virtual bool collidingPointsCheckOnly(
      const std::vector<Eigen::Vector3d>& points, double collision_threshold) {
    return false;
  }

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
  virtual bool collisionRaycast(const Eigen::Matrix3Xd& origin,
                                const Eigen::Matrix3Xd& ray_endpoint,
                                bool use_margins, Eigen::VectorXd& distances,
                                Eigen::Matrix3Xd& normals) {
    return false;
  }

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
  friend DRAKECOLLISION_EXPORT std::ostream& operator<<(std::ostream&,
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

#endif  // DRAKE_SYSTEMS_PLANTS_COLLISION_MODEL_H_
