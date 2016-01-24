#ifndef __DrakeCollisionElement_H__
#define __DrakeCollisionElement_H__

#include <memory>
#include <utility>
#include <stdint.h>

#include <Eigen/Dense>
#include <Eigen/StdVector>

#include "../shapes/DrakeShapes.h"
#include "drake/drakeCollision_export.h"

namespace DrakeCollision
{
  typedef uintptr_t ElementId;

  class DRAKECOLLISION_EXPORT Element : public DrakeShapes::Element {
    public:
      Element(const Eigen::Isometry3d& T_element_to_local = Eigen::Isometry3d::Identity());

      Element(const DrakeShapes::Geometry& geometry, 
              const Eigen::Isometry3d& T_element_to_local = Eigen::Isometry3d::Identity());

      virtual ~Element(){};

      virtual Element* clone() const;

      ElementId getId() const;

      virtual bool isStatic() const { return false; };

      virtual bool collidesWith( const Element* other) const { return true; };

    protected:
      Element(const Element& other); 

    private:
      ElementId id;

    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };
}
#endif
