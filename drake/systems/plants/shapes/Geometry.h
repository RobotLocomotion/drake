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

      virtual void getPoints(Eigen::Matrix3Xd &points) const;
      virtual void getBoundingBoxPoints(Eigen::Matrix3Xd &points) const;
      virtual void getTerrainContactPoints(Eigen::Matrix3Xd &points) const { points = Eigen::Matrix3Xd(); };

    protected:
      Geometry(Shape shape);
      void getBoundingBoxPoints(double x_half_width, double y_half_width, double z_half_width, Eigen::Matrix3Xd &points) const;
      
      Shape shape;
      static const int NUM_BBOX_POINTS = 8;      
  };

  class DLLEXPORT_drakeShapes Sphere: public Geometry {
    public:
      Sphere(double radius);
      virtual ~Sphere() {}
      virtual Sphere* clone() const;
      virtual void getPoints(Eigen::Matrix3Xd &points) const;
      virtual void getBoundingBoxPoints(Eigen::Matrix3Xd &points) const;
      virtual void getTerrainContactPoints(Eigen::Matrix3Xd &points) const;
      
      double radius;
      static const int NUM_POINTS = 1;
  };

  class DLLEXPORT_drakeShapes Box : public Geometry {
    public:
      Box(const Eigen::Vector3d& size);
      virtual ~Box() {}
      virtual Box* clone() const;
      virtual void getPoints(Eigen::Matrix3Xd &points) const;
      virtual void getBoundingBoxPoints(Eigen::Matrix3Xd &points) const;
      virtual void getTerrainContactPoints(Eigen::Matrix3Xd &points) const;
      
      Eigen::Vector3d size;
      
  };

  class DLLEXPORT_drakeShapes Cylinder : public Geometry {
    public:
      Cylinder(double radius, double length);
      virtual ~Cylinder() {}
      virtual Cylinder* clone() const;
      virtual void getPoints(Eigen::Matrix3Xd &points) const;
      virtual void getBoundingBoxPoints(Eigen::Matrix3Xd &points) const;

      double radius;
      double length;
  };

  class DLLEXPORT_drakeShapes Capsule : public Geometry {
    public:
      Capsule(double radius, double length);
      virtual ~Capsule() {}
      virtual Capsule* clone() const;
      virtual void getPoints(Eigen::Matrix3Xd &points) const;
      virtual void getBoundingBoxPoints(Eigen::Matrix3Xd &points) const;
      double radius;
      double length;
      
      static const int NUM_POINTS = 2;
  };

  class DLLEXPORT_drakeShapes Mesh : public Geometry {
    public:
      Mesh(const std::string& filename);
      Mesh(const std::string& filename, const std::string& resolved_filename);
      virtual ~Mesh() {}
      virtual Mesh* clone() const;
      virtual void getPoints(Eigen::Matrix3Xd &points) const;
      virtual void getBoundingBoxPoints(Eigen::Matrix3Xd &points) const;

      double scale;
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

      virtual void getPoints(Eigen::Matrix3Xd &points) const;
      virtual void getBoundingBoxPoints(Eigen::Matrix3Xd &points) const;

      Eigen::Matrix3Xd points;
  };

}
#endif
