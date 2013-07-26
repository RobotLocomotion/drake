#ifndef __DrakeCollisionBulletElement_H__
#define __DrakeCollisionBulletElement_H__

#include <btBulletCollisionCommon.h>

#include "DrakeCollision.h"
#include "Element.h"

namespace DrakeCollision
{
  class BulletElement : public Element
  {
    public:
      BulletElement( Matrix4d T_elem_to_link, Shape shape, std::vector<double> params);
      ~BulletElement();

      virtual void setWorldTransform(const Matrix4d body_transform);

      friend class BulletModel;

    private:
      btCollisionObject* bt_obj;
      btCollisionShape* bt_shape;
  };
}
#endif
