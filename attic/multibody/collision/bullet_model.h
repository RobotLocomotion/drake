#pragma once

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "BulletCollision/NarrowPhaseCollision/btGjkEpaPenetrationDepthSolver.h"
#include "BulletCollision/NarrowPhaseCollision/btGjkPairDetector.h"
#include "BulletCollision/NarrowPhaseCollision/btPointCollector.h"
#include "btBulletCollisionCommon.h"

#include "drake/attic_warning.h"
#include "drake/common/drake_copyable.h"
#include "drake/multibody/collision/element.h"
#include "drake/multibody/collision/model.h"

namespace drake {
namespace multibody {
namespace collision {

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

class UnknownShapeException : public std::runtime_error {
 public:
  explicit UnknownShapeException(DrakeShapes::Shape shape);
};

class BulletModel : public Model {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(BulletModel)

  BulletModel() {}

  virtual ~BulletModel() {}

  void NotifyFilterCriteriaChanged(ElementId id) override;

  void UpdateModel() override;

  void DoAddElement(const Element& element) override;

  void DoRemoveElement(ElementId id) override;

  bool UpdateElementWorldTransform(
      ElementId, const Eigen::Isometry3d& T_local_to_world) override;

  /**
   * Finds the points where each pair of the elements in ids_to_check are
   * closest.  Inserts those points in closest_points.
   *
   * \return true if any points are found.
   */
  bool ClosestPointsAllToAll(
      const std::vector<ElementId>& ids_to_check, bool use_margins,
      std::vector<PointPair<double>>* closest_points) override;

  bool ComputeMaximumDepthCollisionPoints(
      bool use_margins, std::vector<PointPair<double>>* points) override;

  /**
   * Finds the points where each pair of elements in id_pairs are
   * closest.  Inserts those points in closest_points.
   *
   * \return true if any points are found.
   */
  bool ClosestPointsPairwise(
      const std::vector<ElementIdPair>& id_pairs, bool use_margins,
      std::vector<PointPair<double>>* closest_points) override;

  /**
   * Computes the closest point in the collision world to each of a set of
   * points. For each query point, a PointPair instance, `p`, is returned with
   * the following semantics:
   *
   *    - p.elementA = p.elementB = pointer to the closest element.
   *    - p.idA = p.idB = ElementId of closest element.
   *    - p.ptA = the point on the closest element's surface expressed and
   *        measured in the element's local frame.
   *    - p.ptB = the point on the closest element's surface expressed and
   *        measured in the world frame.
   *    - p.normal = the normal direction from the nearest object to the query
   *        point, expressed in the world frame.
   *    - p.distance = The *signed* distance between the query point and the
   *        nearest point.  Negative values indicate penetration.
   *
   * If there are no objects in the scene, then the pointers will be nullptr,
   * the ids, 0, the distance infinite, and the nearest points, infinitely far
   * away.
   *
   * This query will *not* determine the distance to non-convex geometry.
   * A scene with only non-convex geometry is effectively empty to this
   * method.
   *
   * @param points                  A set of points measured and expressed in
   *                                the world frame.  One per column.
   * @param use_margins             Determines whether margins are used (true)
   *                                or not.
   * @param[out] closest_points     The vector for which all the closest point
   *                                data will be returned.
   */
  void CollisionDetectFromPoints(
      const Eigen::Matrix3Xd& points, bool use_margins,
      std::vector<PointPair<double>>* closest_points) override;

  void ClearCachedResults(bool use_margins) override;

  bool CollisionRaycast(const Eigen::Matrix3Xd& origins,
                        const Eigen::Matrix3Xd& ray_endpoints, bool use_margins,
                        Eigen::VectorXd* distances,
                        Eigen::Matrix3Xd* normals) override;

  bool CollidingPointsCheckOnly(
      const std::vector<Eigen::Vector3d>& input_points,
      double collision_threshold) override;

  std::vector<size_t> CollidingPoints(
      const std::vector<Eigen::Vector3d>& input_points,
      double collision_threshold) override;

  static std::unique_ptr<btCollisionShape> newBulletBoxShape(
      const DrakeShapes::Box& geometry, bool use_margins);

 private:
  enum DispatchMethod {
    kNotYetDecided,
    kClosestPointsAllToAll,
    kCollisionPointsAllToAll
  };

  static constexpr double kSmallMargin = 1e-9;
  static constexpr double kLargeMargin = 0.05;

  /**
   * \brief Finds the points where elements A and B are closest.
   *
   * Emits the PointPair (a, b) into the result_collector and returns true, if
   * the closest points can be identified.  Otherwise emits nothing and
   * returns false.
   */
  virtual PointPair<double> findClosestPointsBetweenElements(
      ElementId idA, ElementId idB, bool use_margins);

  BulletCollisionWorldWrapper& getBulletWorld(bool use_margins);
  static std::unique_ptr<btCollisionShape> newBulletSphereShape(
      const DrakeShapes::Sphere& geometry, bool use_margins);
  static std::unique_ptr<btCollisionShape> newBulletCylinderShape(
      const DrakeShapes::Cylinder& geometry, bool use_margins);
  static std::unique_ptr<btCollisionShape> newBulletCapsuleShape(
      const DrakeShapes::Capsule& geometry, bool use_margins);
  static std::unique_ptr<btCollisionShape> newBulletMeshShape(
      const DrakeShapes::Mesh& geometry, bool use_margins);
  std::unique_ptr<btCollisionShape> newBulletStaticMeshShape(
      const DrakeShapes::Mesh& geometry, bool use_margins);
  static std::unique_ptr<btCollisionShape> newBulletMeshPointsShape(
      const DrakeShapes::MeshPoints& geometry, bool use_margins);

  std::vector<std::unique_ptr<btCollisionShape>> bt_collision_shapes_;
  // Bullet doesn't clean up its own triangle meshes.  This collects up the
  // meshes created by the model and gives the model responsibility for deleting
  // them.
  std::vector<std::unique_ptr<btTriangleMesh>> bt_triangle_meshes_{};
  BulletCollisionWorldWrapper bullet_world_;
  BulletCollisionWorldWrapper bullet_world_no_margin_;
  DispatchMethod dispatch_method_in_use_{kNotYetDecided};
};

}  // namespace collision
}  // namespace multibody
}  // namespace drake
