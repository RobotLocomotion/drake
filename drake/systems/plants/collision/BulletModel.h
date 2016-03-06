#ifndef __DrakeCollisionBulletModel_H__
#define __DrakeCollisionBulletModel_H__

#include <btBulletCollisionCommon.h>
#include <BulletCollision/NarrowPhaseCollision/btGjkEpaPenetrationDepthSolver.h>
#include <BulletCollision/NarrowPhaseCollision/btGjkPairDetector.h>
#include <BulletCollision/NarrowPhaseCollision/btPointCollector.h>

#include "Element.h"
#include "Model.h"
#include "BulletResultCollector.h"

namespace DrakeCollision {
class BulletModel;  // forward declaration

#define PERTURBATION_ITERATIONS 8
#define MINIMUM_POINTS_PERTURBATION_THRESHOLD 8

typedef std::unordered_map<ElementId, std::unique_ptr<btCollisionObject> >
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

class BulletModel : public Model {
 public:
  BulletModel(){};

  virtual ~BulletModel(){};

  // Required member functions for Model interface
  virtual void updateModel();

  virtual void resize(int num_bodies){};

  virtual ElementId addElement(const Element& element);

  virtual bool updateElementWorldTransform(
      const ElementId, const Eigen::Isometry3d& T_local_to_world);

  virtual bool closestPointsAllToAll(const std::vector<ElementId>& ids_to_check,
                                     const bool use_margins,
                                     std::vector<PointPair>& closest_points);

  virtual bool collisionPointsAllToAll(const bool use_margins,
                                       std::vector<PointPair>& points);

  virtual bool closestPointsPairwise(const std::vector<ElementIdPair>& id_pairs,
                                     const bool use_margins,
                                     std::vector<PointPair>& closest_points);

  virtual bool findClosestPointsBtwElements(
      const ElementId elemA, const ElementId elemB, const bool use_margins,
      std::unique_ptr<ResultCollector>& c);

  virtual void collisionDetectFromPoints(
      const Eigen::Matrix3Xd& points, bool use_margins,
      std::vector<PointPair>& closest_points);

  virtual bool collisionRaycast(const Eigen::Matrix3Xd& origins,
                                const Eigen::Matrix3Xd& ray_endpoints,
                                bool use_margins, Eigen::VectorXd& distances,
                                Eigen::Matrix3Xd& normals);

  /** \brief Compute the set of potential collision points for all
   * eligible pairs of collision geometries in this model. This includes
   * the points of closest approach, but may also include additional points
   * that are "close" to being in contact. This can be useful when
   * simulating scenarios in which two collision elements have more than
   * one point of contact.
   *
   * This implementation uses Bullet's random perturbation approach to
   * generate the additional contact points. Bullet performs discrete
   * collision detection multiple times with small perturbations to the
   * pose of each collision geometry. The potential collision points are
   * the union of the closest points found in each of these perturbed runs.
   * \param use_margins flag indicating whether or not to use the version
   * of this model with collision margins
   * \return a vector of PointPair objects containing the potential
   * collision points
   */
  virtual std::vector<PointPair> potentialCollisionPoints(bool use_margins);

  virtual bool collidingPointsCheckOnly(
      const std::vector<Eigen::Vector3d>& points, double collision_threshold);

  virtual std::vector<size_t> collidingPoints(
      const std::vector<Eigen::Vector3d>& points, double collision_threshold);

  // END Required member functions

 protected:
  BulletCollisionWorldWrapper& getBulletWorld(bool use_margins);

  std::vector<std::unique_ptr<btCollisionShape> > bt_collision_shapes;
  BulletCollisionWorldWrapper bullet_world;
  BulletCollisionWorldWrapper bullet_world_no_margin;

 protected:
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
  static std::unique_ptr<btCollisionShape> newBulletMeshPointsShape(
      const DrakeShapes::MeshPoints& geometry, bool use_margins);

  static constexpr double small_margin = 1e-9;
  static constexpr double large_margin = 0.05;

  class unknownShapeException : public std::exception {
   public:
    unknownShapeException(DrakeShapes::Shape shape);
    virtual const char* what() const throw();
    virtual ~unknownShapeException() throw(){};

   protected:
    std::string shape_str;
  };

 private:
  BulletModel(const BulletModel&) {}
  BulletModel& operator=(const BulletModel&) { return *this; }
};
}
#endif
