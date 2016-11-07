#pragma once

#include <string>
#include <vector>

#include <Eigen/Dense>

#include "drake/common/drake_export.h"

namespace DrakeShapes {
enum DRAKE_EXPORT Shape {
  UNKNOWN = 0,
  BOX = 1,
  SPHERE = 2,
  CYLINDER = 3,
  MESH = 4,
  MESH_POINTS = 5,
  CAPSULE = 6
};

typedef std::vector<Eigen::Vector3d> PointsVector;
typedef std::vector<Eigen::Vector3i> TrianglesVector;

std::string ShapeToString(Shape ss);

const double MIN_RADIUS = 1e-7;

class DRAKE_EXPORT Geometry {
 public:
  Geometry();
  Geometry(const Geometry& other);

  virtual ~Geometry() {}

  virtual Geometry* clone() const;

  Shape getShape() const;

  // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
  virtual void getPoints(Eigen::Matrix3Xd& points) const;
  // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
  virtual void getBoundingBoxPoints(Eigen::Matrix3Xd& points) const;
  // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
  virtual void getTerrainContactPoints(Eigen::Matrix3Xd& points) const {
    points = Eigen::Matrix3Xd();
  }

  /**
   * A toString method for this class.
   */
  friend DRAKE_EXPORT std::ostream& operator<<(std::ostream&,
                                                     const Geometry&);

 protected:
  explicit Geometry(Shape shape);
  void getBoundingBoxPoints(
      double x_half_width, double y_half_width, double z_half_width,
      // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
      Eigen::Matrix3Xd& points) const;

  Shape shape;
  static const int NUM_BBOX_POINTS;
};

class DRAKE_EXPORT Sphere : public Geometry {
 public:
  explicit Sphere(double radius);
  virtual ~Sphere() {}
  virtual Sphere* clone() const;
  // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
  virtual void getPoints(Eigen::Matrix3Xd& points) const;
  // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
  virtual void getBoundingBoxPoints(Eigen::Matrix3Xd& points) const;
  // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
  virtual void getTerrainContactPoints(Eigen::Matrix3Xd& points) const;

  /**
   * A toString method for this class.
   */
  friend DRAKE_EXPORT std::ostream& operator<<(std::ostream&,
                                                     const Sphere&);

  double radius;
  static const int NUM_POINTS;
};

class DRAKE_EXPORT Box : public Geometry {
 public:
  explicit Box(const Eigen::Vector3d& size);
  virtual ~Box() {}
  virtual Box* clone() const;
  // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
  virtual void getPoints(Eigen::Matrix3Xd& points) const;
  // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
  virtual void getBoundingBoxPoints(Eigen::Matrix3Xd& points) const;
  // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
  virtual void getTerrainContactPoints(Eigen::Matrix3Xd& points) const;

  /**
   * A toString method for this class.
   */
  friend DRAKE_EXPORT std::ostream& operator<<(std::ostream&, const Box&);

  Eigen::Vector3d size;
};

class DRAKE_EXPORT Cylinder : public Geometry {
 public:
  Cylinder(double radius, double length);
  virtual ~Cylinder() {}
  virtual Cylinder* clone() const;
  // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
  virtual void getPoints(Eigen::Matrix3Xd& points) const;
  // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
  virtual void getBoundingBoxPoints(Eigen::Matrix3Xd& points) const;

  /**
   * A toString method for this class.
   */
  friend DRAKE_EXPORT std::ostream& operator<<(std::ostream&,
                                                     const Cylinder&);

  double radius;
  double length;
};

class DRAKE_EXPORT Capsule : public Geometry {
 public:
  Capsule(double radius, double length);
  virtual ~Capsule() {}
  virtual Capsule* clone() const;
  // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
  virtual void getPoints(Eigen::Matrix3Xd& points) const;
  // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
  virtual void getBoundingBoxPoints(Eigen::Matrix3Xd& points) const;

  /**
   * A toString method for this class.
   */
  friend DRAKE_EXPORT std::ostream& operator<<(std::ostream&,
                                                     const Capsule&);

  double radius;
  double length;

  static const int NUM_POINTS;
};

class DRAKE_EXPORT Mesh : public Geometry {
 public:
  /** Constructs a representation of a mesh to be loaded from
  @p resolved_filename. @p uri provides a unique identifier used to interact
  with BotVisualizer. **/
  Mesh(const std::string& uri, const std::string& resolved_filename);
  virtual ~Mesh() {}
  virtual Mesh* clone() const;
  // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
  virtual void getPoints(Eigen::Matrix3Xd& points) const;
  // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
  virtual void getBoundingBoxPoints(Eigen::Matrix3Xd& points) const;

  /**
   * A toString method for this class.
   */
  friend DRAKE_EXPORT std::ostream& operator<<(std::ostream&,
                                                     const Mesh&);

  Eigen::Vector3d scale_;
  std::string uri_;
  std::string resolved_filename_;
  // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
  bool extractMeshVertices(Eigen::Matrix3Xd& vertex_coordinates) const;

  /** Loads triangle mesh from an obj file into the provided vectors of vertices
  and triangles.

  @param[out] vertices Vector of 3D vertices in the mesh.
  @param[out] triangles Vector of indices for each triangle in the mesh.
  The i-th entry of @p triangles holds a 3D vector of integer indices into
  @p vertices corresponding to the vertices forming the i-th triangle.

  On output, `vertices.size()` corresponds to the number of vertices in the mesh
  while `triangles.size()` corresponds to the number of triangles in the mesh.
  **/
  void LoadObjFile(PointsVector* vertices, TrianglesVector* triangles) const;

 private:
  // This method finds a juxtaposed obj file from the `resolved_filename_`
  // member. If unable to resolve an obj file it throws an exception.
  // If `resolved_filename_` already is an obj file then it returns
  // `resolved_filename_`. Otherwise it attempts to change the file extension
  // and checks if it exists.
  std::string FindFileWithObjExtension() const;
};

class DRAKE_EXPORT MeshPoints : public Geometry {
 public:
  explicit MeshPoints(const Eigen::Matrix3Xd& points);
  virtual ~MeshPoints() {}
  virtual MeshPoints* clone() const;

  // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
  virtual void getPoints(Eigen::Matrix3Xd& points) const;
  // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
  virtual void getBoundingBoxPoints(Eigen::Matrix3Xd& points) const;

  /**
   * A toString method for this class.
   */
  friend DRAKE_EXPORT std::ostream& operator<<(std::ostream&,
                                                     const MeshPoints&);

  Eigen::Matrix3Xd points;
};

}  // namespace DrakeShapes
