#ifndef __DrakeCollisionElement_H__
#define __DrakeCollisionElement_H__

#include "DrakeCollision.h"

namespace DrakeCollision
{
  class Element {
    friend class GenericModel;

    public:
      Element(const Matrix4d& T_elem_to_link, Shape shape, 
              const std::vector<double>& params);

      void updateWorldTransform(const Matrix4d& T_link_to_world);

      const Matrix4d& getWorldTransform() const; 

      const Matrix4d& getLinkTransform() const; 

      const Shape& getShape() const;

    protected:
      virtual void setWorldTransform(const Matrix4d& T_elem_to_world);

      Matrix4d T_elem_to_link;
      Matrix4d T_elem_to_world;
      Shape shape;
  };
}
#endif
