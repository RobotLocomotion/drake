#ifndef __DrakeShapesGeometry_H__
#define __DrakeShapesGeometry_H__

#include <string>

#include <Eigen/Dense>
#include <Eigen/StdVector>

#include "drakeShapesMacros.h"

namespace DrakeShapes
{
  enum DLLEXPORT_drakeShapes Shape {
    UNKNOWN,
    BOX,
    SPHERE,
    CYLINDER,
    MESH,
    MESH_POINTS,
    CAPSULE
  };

  class DLLEXPORT_drakeShapes Geometry {
    public:
      const Shape getShape() const;
    protected:
      Geometry(Shape shape);
      const Shape shape;
  };

  class DLLEXPORT_drakeShapes Sphere: public Geometry {
    public:
      Sphere(double radius);
      double radius;
  };

  class DLLEXPORT_drakeShapes Box : public Geometry {
    public:
      Box(const Eigen::Vector3d& size);
      Eigen::Vector3d size;
  };

  class DLLEXPORT_drakeShapes Cylinder : public Geometry {
    public:
      Cylinder(double radius, double length);
      double radius;
      double length;
  };

  class DLLEXPORT_drakeShapes Capsule : public Geometry {
    public:
      Capsule(double radius, double length);
      double radius;
      double length;
  };

  class DLLEXPORT_drakeShapes Mesh : public Geometry {
    public:
      Mesh(const std::string filename);
      std::string filename;
  };

  class DLLEXPORT_drakeShapes MeshPoints : public Geometry {
    public:
      MeshPoints(const Eigen::Matrix3Xd& points);
      Eigen::Matrix3Xd points;
  };

}
#endif
