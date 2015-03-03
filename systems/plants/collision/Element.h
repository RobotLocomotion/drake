#ifndef __DrakeCollisionElement_H__
#define __DrakeCollisionElement_H__

#include <memory>
#include <utility>
#include <stdint.h>

#include <Eigen/Dense>
#include <Eigen/StdVector>

#include "DLLEXPORT_drakeCollision.h"
#include "Geometry.h"

namespace DrakeCollision
{
  typedef uintptr_t ElementId;

  class DLLEXPORT_drakeCollision Element {
    public:
      Element(std::unique_ptr<Geometry> geometry, 
          const Eigen::Matrix4d T_element_to_local = Eigen::Matrix4d::Identity())
        : geometry(move(geometry)), T_element_to_local(T_element_to_local), id((ElementId) this) 
      {};

      const Eigen::Matrix4d& getWorldTransform() const; 

      const Eigen::Matrix4d& getLocalTransform() const; 

      virtual void updateWorldTransform(const Eigen::Matrix4d& T_local_to_world);

      const Shape getShape() const;

      ElementId getId() const;

      const Geometry* getGeometry() const;

      virtual bool isStatic() const { return false; };

      virtual bool collidesWith( const Element* other) const { return true; };

    protected:

      virtual void setWorldTransform(const Eigen::Matrix4d& T_elem_to_world);
      Eigen::Matrix4d T_element_to_world;
      const Eigen::Matrix4d T_element_to_local;
      std::unique_ptr<Geometry> geometry;
      const ElementId id;

    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };
}
#endif
