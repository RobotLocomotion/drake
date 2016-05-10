#pragma once

#include "Element.h"
#include "Model.h"
#include "ResultCollector.h"

#include "fcl/BVH/BVH_model.h"

namespace DrakeCollision {

class FCLModel : public Model {
 public:
  FCLModel() {}

  virtual ~FCLModel() {}

  // Required member functions for Model interface
  virtual void updateModel();

  virtual void resize(int num_bodies) {}

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
   */
  virtual std::vector<PointPair> potentialCollisionPoints(bool use_margins);

  virtual bool collidingPointsCheckOnly(
      const std::vector<Eigen::Vector3d>& points, double collision_threshold);

  virtual std::vector<size_t> collidingPoints(
      const std::vector<Eigen::Vector3d>& points, double collision_threshold);

  // END Required member functions

 protected:
  // FCL elements that were created from the drake elements.
  std::unordered_map<ElementId, std::unique_ptr<fcl::BVHModel<fcl::OBBRSS>>>
      fclElements;

  // Helper methods to create meshes from baskic shapes.
  static fcl::BVHModel<fcl::OBBRSS>* newFCLBoxShape(
      const DrakeShapes::Box& geometry, bool use_margins);
  static fcl::BVHModel<fcl::OBBRSS>* newFCLSphereShape(
      const DrakeShapes::Sphere& geometry, bool use_margins);
  static fcl::BVHModel<fcl::OBBRSS>* newFCLCylinderShape(
      const DrakeShapes::Cylinder& geometry, bool use_margins);
  static fcl::BVHModel<fcl::OBBRSS>* newFCLCapsuleShape(
      const DrakeShapes::Capsule& geometry, bool use_margins);
  static fcl::BVHModel<fcl::OBBRSS>* newFCLMeshShape(
      const DrakeShapes::Mesh& geometry, bool use_margins);

  class unknownShapeException : public std::exception {
   public:
    explicit unknownShapeException(DrakeShapes::Shape shape);
    virtual const char* what() const throw();
    virtual ~unknownShapeException() throw() {}

   protected:
    std::string shape_str;
  };

 private:
  FCLModel(const FCLModel&) {}
  FCLModel& operator=(const FCLModel&) { return *this; }
};
}
