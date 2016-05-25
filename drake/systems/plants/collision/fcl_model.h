#pragma once

#include "Element.h"
#include "Model.h"
#include "ResultCollector.h"

#include "fcl/BVH/BVH_model.h"

namespace DrakeCollision {

class FCLModel : public Model {
 public:
  FCLModel() {}

  ~FCLModel() override {}

  // Required member functions for Model interface
  void updateModel() override;

  ElementId addElement(const Element& element) override;

  bool updateElementWorldTransform(
      const ElementId, const Eigen::Isometry3d& T_local_to_world) override;

  bool closestPointsAllToAll(const std::vector<ElementId>& ids_to_check,
                             const bool use_margins,
                             std::vector<PointPair>& closest_points) override;

  bool collisionPointsAllToAll(const bool use_margins,
                               std::vector<PointPair>& points) override;

  bool closestPointsPairwise(const std::vector<ElementIdPair>& id_pairs,
                             const bool use_margins,
                             std::vector<PointPair>& closest_points) override;

  void collisionDetectFromPoints(
      const Eigen::Matrix3Xd& points, bool use_margins,
      std::vector<PointPair>& closest_points) override;

  bool collisionRaycast(const Eigen::Matrix3Xd& origins,
                        const Eigen::Matrix3Xd& ray_endpoints,
                        bool use_margins, Eigen::VectorXd& distances,
                        Eigen::Matrix3Xd& normals) override;

  std::vector<PointPair> potentialCollisionPoints(bool use_margins) override;

  bool collidingPointsCheckOnly(
      const std::vector<Eigen::Vector3d>& points,
      double collision_threshold) override;

  std::vector<size_t> collidingPoints(
      const std::vector<Eigen::Vector3d>& points,
      double collision_threshold) override;

  // END Required member functions

 protected:
  // FCL elements that were created from the drake elements.
  std::unordered_map<ElementId, std::unique_ptr<fcl::BVHModel<fcl::OBBRSS>>>
      fclElements;

  virtual bool findClosestPointsBetweenElements(
      const ElementId elemA, const ElementId elemB, const bool use_margins,
      std::unique_ptr<ResultCollector>& c);

  // Helper methods to create meshes from basic shapes.
  static std::unique_ptr<fcl::BVHModel<fcl::OBBRSS>> newFCLBoxShape(
      const DrakeShapes::Box& geometry, bool use_margins);
  static std::unique_ptr<fcl::BVHModel<fcl::OBBRSS>> newFCLSphereShape(
      const DrakeShapes::Sphere& geometry, bool use_margins);
  static std::unique_ptr<fcl::BVHModel<fcl::OBBRSS>> newFCLCylinderShape(
      const DrakeShapes::Cylinder& geometry, bool use_margins);
  static std::unique_ptr<fcl::BVHModel<fcl::OBBRSS>> newFCLCapsuleShape(
      const DrakeShapes::Capsule& geometry, bool use_margins);
  static std::unique_ptr<fcl::BVHModel<fcl::OBBRSS>> newFCLMeshShape(
      const DrakeShapes::Mesh& geometry, bool use_margins);

 private:
  FCLModel(const FCLModel&) {}
  FCLModel& operator=(const FCLModel&) { return *this; }
};
}
