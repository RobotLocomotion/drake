#pragma once

#include <string>
#include <vector>

#include "btBulletCollisionCommon.h"
#include "BulletCollision/NarrowPhaseCollision/btGjkEpaPenetrationDepthSolver.h"
#include "BulletCollision/NarrowPhaseCollision/btGjkPairDetector.h"
#include "BulletCollision/NarrowPhaseCollision/btPointCollector.h"

#include "drake/multibody/collision/element.h"
#include "drake/multibody/collision/model.h"

namespace DrakeCollision {

class BulletModel;  // forward declaration

typedef std::unordered_map<ElementId, std::unique_ptr<btCollisionObject>>
    ElementToBtObjMap;

struct OverlapFilterCallback : public btOverlapFilterCallback {
  // return true when pairs need collision
  virtual bool needBroadphaseCollision(btBroadphaseProxy* proxy0,
                                       btBroadphaseProxy* proxy1) const;

  BulletModel* parent_model;
};

struct BulletCollisionWorldWrapper {
  BulletCollisionWorldWrapper();
  ElementToBtObjMap bt_collision_objects;

  btDefaultCollisionConfiguration bt_collision_configuration;
  btDbvtBroadphase bt_collision_broadphase;
  OverlapFilterCallback filter_callback;

  std::unique_ptr<btCollisionDispatcher> bt_collision_dispatcher;
  std::unique_ptr<btCollisionWorld> bt_collision_world;
};

class UnknownShapeException : public std::exception {
 public:
  explicit UnknownShapeException(DrakeShapes::Shape shape);
  virtual const char* what() const throw();
  virtual ~UnknownShapeException() throw() {}

 private:
  std::string shape_name_;
};

class BulletModel : public Model {
 public:
  BulletModel() {}

  virtual ~BulletModel() {}

  void updateModel() override;

  ElementId addElement(const Element& element) override;

  bool updateElementWorldTransform(
      const ElementId, const Eigen::Isometry3d& T_local_to_world) override;

  /**
   * Finds the points where each pair of the elements in ids_to_check are
   * closest.  Inserts those points in closest_points.
   *
   * \return true if any points are found.
   */
  bool closestPointsAllToAll(const std::vector<ElementId>& ids_to_check,
                             const bool use_margins,
                             std::vector<PointPair>& closest_points) override;

  bool ComputeMaximumDepthCollisionPoints(
      const bool use_margins, std::vector<PointPair>& points) override;

  /**
   * Finds the points where each pair of elements in id_pairs are
   * closest.  Inserts those points in closest_points.
   *
   * \return true if any points are found.
   */
  bool closestPointsPairwise(const std::vector<ElementIdPair>& id_pairs,
                             const bool use_margins,
                             std::vector<PointPair>& closest_points) override;

  void collisionDetectFromPoints(
      const Eigen::Matrix3Xd& points, bool use_margins,
      std::vector<PointPair>& closest_points) override;

  void ClearCachedResults(bool use_margins) override;

  bool collisionRaycast(const Eigen::Matrix3Xd& origins,
                        const Eigen::Matrix3Xd& ray_endpoints, bool use_margins,
                        Eigen::VectorXd& distances,
                        Eigen::Matrix3Xd& normals) override;

  /** Computes the set of potential collision points for all
  eligible pairs of collision geometries in this model. This includes
  the points of closest approach, but may also include additional points
  that are "close" to being in contact. This can be useful when
  simulating scenarios in which two collision elements have more than
  one point of contact.

  This implementation uses Bullet's random perturbation approach to
  generate the additional contact points. Bullet performs discrete
  collision detection multiple times with small perturbations to the
  pose of each collision geometry. The potential collision points are
  the union of the closest points found in each of these perturbed runs.

  @param use_margins flag indicating whether or not to use the version
  of this model with collision margins.

  @returns a vector of PointPair objects containing the potential
  collision points.

  @throws An std::runtime_error if calls to this method are mixed with other
  dispatching methods such as BulletModel::collisionPointsAllToAll since mixing
  dispatch methods causes undefined behavior. **/
  std::vector<PointPair> potentialCollisionPoints(bool use_margins) override;

  bool collidingPointsCheckOnly(
      const std::vector<Eigen::Vector3d>& input_points,
      double collision_threshold) override;

  std::vector<size_t> collidingPoints(
      const std::vector<Eigen::Vector3d>& input_points,
      double collision_threshold) override;

 private:
  enum DispatchMethod {
    kNotYetDecided,
    kClosestPointsAllToAll,
    kCollisionPointsAllToAll,
    kPotentialCollisionPoints
  };

  static constexpr double kSmallMargin = 1e-9;
  static constexpr double kLargeMargin = 0.05;

  // BulletModel objects are not copyable
  BulletModel(const BulletModel&) = delete;
  BulletModel& operator=(const BulletModel&) = delete;

  /**
   * \brief Finds the points where elements A and B are closest.
   *
   * Emits the PointPair (a, b) into the result_collector and returns true, if
   * the closest points can be identified.  Otherwise emits nothing and
   * returns false.
   */
  virtual PointPair findClosestPointsBetweenElements(
      const ElementId idA, const ElementId idB, const bool use_margins);

  BulletCollisionWorldWrapper& getBulletWorld(bool use_margins);
  static std::unique_ptr<btCollisionShape> newBulletBoxShape(
      const DrakeShapes::Box& geometry, bool use_margins);
  static std::unique_ptr<btCollisionShape> newBulletSphereShape(
      const DrakeShapes::Sphere& geometry, bool use_margins);
  static std::unique_ptr<btCollisionShape> newBulletCylinderShape(
      const DrakeShapes::Cylinder& geometry, bool use_margins);
  static std::unique_ptr<btCollisionShape> newBulletCapsuleShape(
      const DrakeShapes::Capsule& geometry, bool use_margins);
  static std::unique_ptr<btCollisionShape> newBulletMeshShape(
      const DrakeShapes::Mesh& geometry, bool use_margins);
  static std::unique_ptr<btCollisionShape> newBulletStaticMeshShape(
      const DrakeShapes::Mesh& geometry, bool use_margins);
  static std::unique_ptr<btCollisionShape> newBulletMeshPointsShape(
      const DrakeShapes::MeshPoints& geometry, bool use_margins);

  std::vector<std::unique_ptr<btCollisionShape>> bt_collision_shapes_;
  BulletCollisionWorldWrapper bullet_world_;
  BulletCollisionWorldWrapper bullet_world_no_margin_;
  DispatchMethod dispatch_method_in_use_{kNotYetDecided};
};

}  // namespace DrakeCollision
