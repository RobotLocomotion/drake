#ifndef __DrakeCollisionModel_H__
#define __DrakeCollisionModel_H__

#include <stdint.h>
#include <memory>
#include <Eigen/Dense>
#include <vector>

#include "simpleClosestPointFunctions.h"
using namespace Eigen;

namespace DrakeCollision
{
  enum Shape {
    UNKNOWN,
    BOX,
    SPHERE,
    CYLINDER,
    MESH,
    CAPSULE
  };

  enum ModelType {
    NONE,
    AUTO,
    BULLET
  };

  class Model {
    public:
      virtual void resize(int num_bodies)=0;

      virtual void addElement(const int body_idx, const int parent_idx, 
                              const Matrix4d& T_element_to_link, Shape shape, 
                              const std::vector<double>& params,
                              bool is_static)=0;

      virtual bool updateElementsForBody(const int body_idx, 
                                  const Matrix4d& T_link_to_world)=0;
      
      virtual bool setCollisionFilter(const int body_idx, const uint16_t group, 
                                     const uint16_t mask)=0;


      virtual bool getPointCollision(const int body_idx, 
                                      const int body_collision_idx, 
                                      Vector3d &ptA, Vector3d &ptB, 
                                      Vector3d &normal)=0;

      virtual bool getPairwiseCollision(const int bodyA_idx, const int bodyB_idx, 
                                MatrixXd& ptsA, MatrixXd& ptsB, 
                                MatrixXd& normals)=0;

      virtual bool getPairwisePointCollision(const int bodyA_idx, const int bodyB_idx, 
                                      const int body_collisionA_idx, 
                                      Vector3d &ptA, Vector3d &ptB, 
                                      Vector3d &normal)=0;

      virtual bool getClosestPoints(const int bodyA_idx, const int bodyB_idx,
                            Vector3d& ptA, Vector3d& ptB, Vector3d& normal,
                            double& distance)=0;

      virtual bool closestPointsAllBodies(std::vector<int>& bodyA_idx, 
                                               std::vector<int>& bodyB_idx, 
                                               MatrixXd& ptsA, MatrixXd& ptsB,
                                               MatrixXd& normal, 
                                               VectorXd& distance,
                                               std::vector<int>& bodies_idx)=0;

      /**
       * Performs raycasting collision detecting (like a LIDAR / laser rangefinder)
       * 
       * @param origin Vector3d specifying the position of the ray's origin
       * @param ray_endpoint Vector3d specifying a second point on the ray in world coordinates
       * 
       * @retval distance to the first collision, or -1 on no collision
       */
      virtual double collisionRaycast(const Vector3d &origin, const Vector3d &ray_endpoint)=0;
  };

  std::shared_ptr<Model> newModel();

  std::shared_ptr<Model> newModel(ModelType model_type);
}
#endif
