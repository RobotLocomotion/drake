#ifndef __DrakeShapesGeometry_H__
#define __DrakeShapesGeometry_H__

#include <string>

#include <Eigen/Dense>
#include <Eigen/StdVector>

#include "drakeShapesMacros.h"

namespace DrakeShapes
{
  enum DLLEXPORT_drakeShapes Shape {
    UNKNOWN     = 0,
    BOX         = 1,
    SPHERE      = 2,
    CYLINDER    = 3,
    MESH        = 4,
    MESH_POINTS = 5,
    CAPSULE     = 6
  };

  class DLLEXPORT_drakeShapes Geometry {
    public:
      Geometry();
      Geometry(const Geometry& other);

      virtual ~Geometry() {}

      virtual Geometry* clone() const;

      const Shape getShape() const;

    protected:
      Geometry(Shape shape);
      Shape shape;
  };

  class DLLEXPORT_drakeShapes Sphere: public Geometry {
    public:
      Sphere(double radius);
      virtual ~Sphere() {}
      virtual Sphere* clone() const;
      double radius;
  };

  class DLLEXPORT_drakeShapes Box : public Geometry {
    public:
      Box(const Eigen::Vector3d& size);
      virtual ~Box() {}
      virtual Box* clone() const;
      Eigen::Vector3d size;
  };

  class DLLEXPORT_drakeShapes Cylinder : public Geometry {
    public:
      Cylinder(double radius, double length);
      virtual ~Cylinder() {}
      virtual Cylinder* clone() const;
      double radius;
      double length;
  };

  class DLLEXPORT_drakeShapes Capsule : public Geometry {
    public:
      Capsule(double radius, double length);
      virtual ~Capsule() {}
      virtual Capsule* clone() const;
      double radius;
      double length;
  };

  class DLLEXPORT_drakeShapes Mesh : public Geometry {
    public:
      Mesh(const std::string& filename);
      Mesh(const std::string& filename, const std::string& resolved_filename);
      virtual ~Mesh() {}
      virtual Mesh* clone() const;

      std::string filename;
      std::string resolved_filename;
      bool extractMeshVertices(Eigen::Matrix3Xd& vertex_coordinates) const;
    protected:
      std::string root_dir;
  };

  class DLLEXPORT_drakeShapes MeshPoints : public Geometry {
    public:
      MeshPoints(const Eigen::Matrix3Xd& points);
      virtual ~MeshPoints() {}
      virtual MeshPoints* clone() const;
      Eigen::Matrix3Xd points;
  };

}
#endif
