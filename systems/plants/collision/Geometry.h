#ifndef __DrakeCollisionGeometry_H__
#define __DrakeCollisionGeometry_H__

#include <Eigen/Dense>
#include <Eigen/StdVector>

#include "drakeCollisionMacros.h"

namespace DrakeCollision
{
  enum DLLEXPORT_drakeCollision Shape {
    UNKNOWN,
    BOX,
    SPHERE,
    CYLINDER,
    MESH,
    CAPSULE
  };

  class DLLEXPORT_drakeCollision Geometry {
    public:
      const Shape getShape() const;
    protected:
      Geometry(Shape shape);
      const Shape shape;
  };

  class DLLEXPORT_drakeCollision Sphere: public Geometry {
    public:
      Sphere(double radius);
      double radius;
  };

  class DLLEXPORT_drakeCollision Box : public Geometry {
    public:
      Box(const Eigen::Vector3d& size);
      Eigen::Vector3d size;
  };

  class DLLEXPORT_drakeCollision Cylinder : public Geometry {
    public:
      Cylinder(double radius, double length);
      double radius;
      double length;
  };

  class DLLEXPORT_drakeCollision Capsule : public Geometry {
    public:
      Capsule(double radius, double length);
      double radius;
      double length;
  };

  class DLLEXPORT_drakeCollision Mesh : public Geometry {
    public:
      Mesh(const Eigen::Matrix3Xd& points);
      Eigen::Matrix3Xd points;
  };

}
#endif
