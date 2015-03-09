#ifndef __DrakeCollisionElement_H__
#define __DrakeCollisionElement_H__

#include <memory>
#include <utility>
#include <stdint.h>

#include <Eigen/Dense>
#include <Eigen/StdVector>

#include "../shapes/DrakeShapes.h"

#include "drakeCollisionMacros.h"

namespace DrakeCollision
{
  typedef uintptr_t ElementId;

  class DLLEXPORT_drakeCollision Element : public DrakeShapes::Element {
    public:
      Element(std::unique_ptr<DrakeShapes::Geometry> geometry, 
          const Eigen::Matrix4d& T_element_to_local = Eigen::Matrix4d::Identity())
        : DrakeShapes::Element(move(geometry), T_element_to_local), id((ElementId) this) 
      {};

      ElementId getId() const;

      virtual bool isStatic() const { return false; };

      virtual bool collidesWith( const Element* other) const { return true; };

    protected:

      const ElementId id;

    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };
}
#endif
