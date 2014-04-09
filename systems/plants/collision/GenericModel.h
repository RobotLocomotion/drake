#ifndef __DrakeCollisionGenericModel_H__
#define __DrakeCollisionGenericModel_H__

#include "ModelTemplate.h"

namespace DrakeCollision
{
  class GenericModel : public ModelTemplate<Element> {
    public:
      //Required member functions for ModelTemplate
      virtual bool getPointCollision(const int body_idx, 
          const int body_collision_idx, 
          Vector3d &ptA, Vector3d &ptB, 
          Vector3d &normal);

    protected:
      virtual bool findClosestPointsBtwElements(const int bodyA_idx,
                                                const int bodyB_idx, 
                                                const Element& elemA, 
                                                const Element& elemB, 
                                                const ResultCollShPtr& c);

      virtual bool findCollisionPointsBtwElements(const int bodyA_idx,
                                                  const int bodyB_idx, 
                                                  const Element& elemA, 
                                                  const Element& elemB, 
                                                  const ResultCollShPtr& c);
                                                  
      virtual double collisionRaycast(const Vector3d &origin, const Vector3d &ray_endpoint);
      // END Required member functions
  };
}
#endif
