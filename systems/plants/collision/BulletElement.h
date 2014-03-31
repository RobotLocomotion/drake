#ifndef __DrakeCollisionBulletElement_H__
#define __DrakeCollisionBulletElement_H__

#include <btBulletCollisionCommon.h>

#include "DrakeCollision.h"
#include "Element.h"

namespace DrakeCollision
{
  class BulletElement : public Element
  {
    friend class BulletModel;

    public:
      BulletElement(const Matrix4d& T_elem_to_link, Shape shape, 
                    const std::vector<double>& params);

    private:
      virtual void setWorldTransform(const Matrix4d& body_transform);

      std::shared_ptr<btCollisionObject> bt_obj;
  };
}
#endif
