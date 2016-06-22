#pragma once

#include <string>

#include <Eigen/Dense>
#include <Eigen/StdVector>

#include "drake/drakeShapes_export.h"

namespace DrakeShapes {
enum DRAKESHAPES_EXPORT Shape {
  UNKNOWN = 0,
  BOX = 1,
  SPHERE = 2,
  CYLINDER = 3,
  MESH = 4,
  MESH_POINTS = 5,
  CAPSULE = 6
};

std::string ShapeToString(Shape ss);

const double MIN_RADIUS = 1e-7;

class DRAKESHAPES_EXPORT Geometry {
 public:
  Geometry();
  Geometry(const Geometry &other);

  virtual ~Geometry() {}

  virtual Geometry *clone() const;

  Shape getShape() const;

  virtual void getPoints(Eigen::Matrix3Xd &points) const;
  virtual void getBoundingBoxPoints(Eigen::Matrix3Xd &points) const;
  virtual void getTerrainContactPoints(Eigen::Matrix3Xd &points) const {
    points = Eigen::Matrix3Xd();
  }

  /**
   * A toString method for this class.
   */
  friend DRAKESHAPES_EXPORT std::ostream &operator<<(std::ostream &,
                                                     const Geometry &);

 protected:
  explicit Geometry(Shape shape);
  void getBoundingBoxPoints(double x_half_width, double y_half_width,
                            double z_half_width,
                            Eigen::Matrix3Xd &points) const;

  Shape shape;
  static const int NUM_BBOX_POINTS;
};

class DRAKESHAPES_EXPORT Sphere : public Geometry {
 public:
  explicit Sphere(double radius);
  virtual ~Sphere() {}
  virtual Sphere *clone() const;
  virtual void getPoints(Eigen::Matrix3Xd &points) const;
  virtual void getBoundingBoxPoints(Eigen::Matrix3Xd &points) const;
  virtual void getTerrainContactPoints(Eigen::Matrix3Xd &points) const;

  /**
   * A toString method for this class.
   */
  friend DRAKESHAPES_EXPORT std::ostream &operator<<(std::ostream &,
                                                     const Sphere &);

  double radius;
  static const int NUM_POINTS;
};

class DRAKESHAPES_EXPORT Box : public Geometry {
 public:
  explicit Box(const Eigen::Vector3d &size);
  virtual ~Box() {}
  virtual Box *clone() const;
  virtual void getPoints(Eigen::Matrix3Xd &points) const;
  virtual void getBoundingBoxPoints(Eigen::Matrix3Xd &points) const;
  virtual void getTerrainContactPoints(Eigen::Matrix3Xd &points) const;

  /**
   * A toString method for this class.
   */
  friend DRAKESHAPES_EXPORT std::ostream &operator<<(std::ostream &,
                                                     const Box &);

  Eigen::Vector3d size;
};

class DRAKESHAPES_EXPORT Cylinder : public Geometry {
 public:
  Cylinder(double radius, double length);
  virtual ~Cylinder() {}
  virtual Cylinder *clone() const;
  virtual void getPoints(Eigen::Matrix3Xd &points) const;
  virtual void getBoundingBoxPoints(Eigen::Matrix3Xd &points) const;

  /**
   * A toString method for this class.
   */
  friend DRAKESHAPES_EXPORT std::ostream &operator<<(std::ostream &,
                                                     const Cylinder &);

  double radius;
  double length;
};

class DRAKESHAPES_EXPORT Capsule : public Geometry {
 public:
  Capsule(double radius, double length);
  virtual ~Capsule() {}
  virtual Capsule *clone() const;
  virtual void getPoints(Eigen::Matrix3Xd &points) const;
  virtual void getBoundingBoxPoints(Eigen::Matrix3Xd &points) const;

  /**
   * A toString method for this class.
   */
  friend DRAKESHAPES_EXPORT std::ostream &operator<<(std::ostream &,
                                                     const Capsule &);

  double radius;
  double length;

  static const int NUM_POINTS;
};

class DRAKESHAPES_EXPORT Mesh : public Geometry {
 public:
  explicit Mesh(const std::string &filename);
  Mesh(const std::string &filename, const std::string &resolved_filename);
  virtual ~Mesh() {}
  virtual Mesh *clone() const;
  virtual void getPoints(Eigen::Matrix3Xd &points) const;
  virtual void getBoundingBoxPoints(Eigen::Matrix3Xd &points) const;

  /**
   * A toString method for this class.
   */
  friend DRAKESHAPES_EXPORT std::ostream &operator<<(std::ostream &,
                                                     const Mesh &);

  Eigen::Vector3d scale;
  std::string filename;
  std::string resolved_filename;
  bool extractMeshVertices(Eigen::Matrix3Xd &vertex_coordinates) const;

  /** Read mesh triangles into the provided array connectivities.

  The array `connectivities` is resized to hold the number of triangles in the
  mesh. If successful, `connectivities` will be a matrix with as many columns as
  triangles in the mesh and three rows. Each column corresponds to a triangle
  and contains the indexes to the points loaded with Mesh::extractMeshVertices.
  **/
  bool ReadMeshConnectivities(Eigen::Matrix3Xi& connectivities) const;

 protected:
  std::string root_dir;
};

class DRAKESHAPES_EXPORT MeshPoints : public Geometry {
 public:
  explicit MeshPoints(const Eigen::Matrix3Xd &points);
  virtual ~MeshPoints() {}
  virtual MeshPoints *clone() const;

  virtual void getPoints(Eigen::Matrix3Xd &points) const;
  virtual void getBoundingBoxPoints(Eigen::Matrix3Xd &points) const;

  /**
   * A toString method for this class.
   */
  friend DRAKESHAPES_EXPORT std::ostream &operator<<(std::ostream &,
                                                     const MeshPoints &);

  Eigen::Matrix3Xd points;
};
}
