#ifndef __DrakeCollisionElement_H__
#define __DrakeCollisionElement_H__

#include "DrakeCollision.h"

namespace DrakeCollision
{
  class Element {
    public:
      virtual void updateWorldTransform(const Matrix4d T_link_to_world);

      friend class Model;
    protected:
      Element(Matrix4d T_elem_to_link, Shape shape, std::vector<double> params);
      virtual void setWorldTransform(const Matrix4d T_elem_to_world);

      const Matrix4d T_elem_to_link;
      Matrix4d T_elem_to_world;
  };
}
#endif
