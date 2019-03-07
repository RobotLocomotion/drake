#pragma once

#include <string>
#include <vector>

#include <Eigen/Dense>

#include "drake/common/unused.h"

namespace DrakeShapes {
enum Shape {
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

// TODO(SeanCurtis-TRI): Magic number here.  It also leads to issues where the
// distance between two points (ostensibly represented with zero-radius spheres)
// will differ by this minimum radius (because they're not truly zero radius).
// https://github.com/RobotLocomotion/drake/issues/4555
const double MIN_RADIUS = 1e-7;

class Geometry {
 public:
  Geometry();

  virtual ~Geometry() {}

  virtual Geometry* clone() const;

  Shape getShape() const;

  // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
  virtual void getPoints(Eigen::Matrix3Xd& points) const;
  /**
   * @returns `true` if this geometry can return faces.
   */
  virtual bool hasFaces() const {
    // By default, arbitrary geometry doesn't know how to provide faces.
    return false;
  }
  /**
   * Returns the faces making up this geometry as a vector of triangles.
   * Each triangle contains three indices into the vertex list returned
   * by the Geometry getPoints() method.
   * @param[out] faces Returns a vector of triangles describing
   * this geometry.
   */
  // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
  virtual void getFaces(TrianglesVector* faces) const {
    drake::unused(faces);
    throw std::runtime_error("Error: getFaces() not implemented"
      " for this geometry type.\n");
  }
  // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
  virtual void getBoundingBoxPoints(Eigen::Matrix3Xd& points) const;
  // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
  virtual void getTerrainContactPoints(Eigen::Matrix3Xd& points) const {
    points = Eigen::Matrix3Xd();
  }

  /**
   * A toString method for this class.
   */
  friend std::ostream& operator<<(std::ostream&, const Geometry&);

 protected:
  explicit Geometry(Shape shape);

  // Provide only a copy constructor, only for use by our subclasses' clone().
  Geometry(const Geometry&) = default;
  void operator=(const Geometry&) = delete;

  void getBoundingBoxPoints(
      double x_half_width, double y_half_width, double z_half_width,
      // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
      Eigen::Matrix3Xd& points) const;

  Shape shape;
  static const int NUM_BBOX_POINTS;
};

class Sphere final : public Geometry {
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
  friend std::ostream& operator<<(std::ostream&, const Sphere&);

  double radius;
  static const int NUM_POINTS;

 private:
  // Allow only the copy constructor, only for our clone().
  Sphere(const Sphere&) = default;
  void operator=(const Sphere&) = delete;
};

class Box final : public Geometry {
 public:
  explicit Box(const Eigen::Vector3d& size);
  virtual ~Box() {}
  Box* clone() const override;
  // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
  void getPoints(Eigen::Matrix3Xd& points) const override;
  bool hasFaces() const override {
    return true;
  }
  // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
  void getFaces(TrianglesVector* faces) const override;
  // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
  void getBoundingBoxPoints(Eigen::Matrix3Xd& points) const override;
  // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
  void getTerrainContactPoints(Eigen::Matrix3Xd& points) const override;

  /**
   * A toString method for this class.
   */
  friend std::ostream& operator<<(std::ostream&, const Box&);

  Eigen::Vector3d size;

 private:
  // Allow only the copy constructor, only for our clone().
  Box(const Box&) = default;
  void operator=(const Box&) = delete;
};

class Cylinder : public Geometry {
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
  friend std::ostream& operator<<(std::ostream&, const Cylinder&);

  double radius;
  double length;

 private:
  // Allow only the copy constructor, only for our clone().
  Cylinder(const Cylinder&) = default;
  void operator=(const Cylinder&) = delete;
};

class Capsule : public Geometry {
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
  friend std::ostream& operator<<(std::ostream&, const Capsule&);

  double radius;
  double length;

  static const int NUM_POINTS;

 private:
  // Allow only the copy constructor, only for our clone().
  Capsule(const Capsule&) = default;
  void operator=(const Capsule&) = delete;
};

class Mesh : public Geometry {
 public:
  /** Specification of how the Mesh should process faces during parsing. */
  enum class TriangulatePolicy {
    kFailOnNonTri,    ///< Non-triangular faces cause an exception to be thrown.
    kTry,             ///< The parser will attempt to triangulate non-triangular
                      ///< faces, throwing an exception if the attempt fails.
  };

  /** Constructs a representation of a mesh to be loaded from
  @p resolved_filename. @p uri provides a unique identifier used to interact
  with BotVisualizer. */
  Mesh(const std::string& uri, const std::string& resolved_filename);
  virtual ~Mesh() {}
  Mesh* clone() const override;
  // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
  void getPoints(Eigen::Matrix3Xd& points) const override;
  bool hasFaces() const override {
    return true;
  }
  // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
  void getFaces(TrianglesVector* faces) const override;
  // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
  void getBoundingBoxPoints(Eigen::Matrix3Xd& points) const override;

  /**
   * A toString method for this class.
   */
  friend std::ostream& operator<<(std::ostream&, const Mesh&);

  Eigen::Vector3d scale_;
  std::string uri_;
  std::string resolved_filename_;
  // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
  bool extractMeshVertices(Eigen::Matrix3Xd& vertex_coordinates) const;

  /** Loads triangle mesh from an obj file into the provided vectors of vertices
  and triangles.

  This method can optionally attempt to triangulate the mesh as it is read.
  This triangulation is conservative. Non-triangular faces are decomposed into
  a set of *equivalent* triangles. It places certain requirements on the
  mesh for the triangulation to be valid.  If these requirements are not met,
  an exception is thrown.  These requirements are:

  1. Non-triangular faces cannot contain a sequence of co-linear vertices.
  2. Non-triangular faces must be close to planar; the decomposed triangles
     normals can deviate by no more than 30 degrees from their edge-adjacent
     neighbors.
  3. Decomposed triangles must have an area larger than 10⁻¹⁰ m².

  @note The triangulation method is simple.  Even if these requirements are met,
  triangulation might fail.

  @param[out] vertices Vector of 3D vertices in the mesh.
  @param[out] triangles Vector of indices for each triangle in the mesh.
  The i-th entry of @p triangles holds a 3D vector of integer indices into
  @p vertices corresponding to the vertices forming the i-th triangle.
  @param[in] triangulate  Specifies the triangulation policy.

  On output, `vertices.size()` corresponds to the number of vertices in the mesh
  while `triangles.size()` corresponds to the number of triangles in the mesh.
  */
  void LoadObjFile(
      PointsVector* vertices, TrianglesVector* triangles,
      TriangulatePolicy triangulate = TriangulatePolicy::kFailOnNonTri) const;

 private:
  // Allow only the copy constructor, only for our clone().
  Mesh(const Mesh&) = default;
  void operator=(const Mesh&) = delete;

  // Lower limit on generated triangle area (as documented).
  static constexpr double kMinArea = 1e-10;

  // cosine(30°) -- used to determine if normal deviation of decomposed
  // triangles lies within the documented threshold.
  static const double kCosThreshold;

  // Given a list of vertex values and three indices into that set, computes a
  // unit-length normal vector to the triangle defined by the vertices.
  // Returns false if the indices are invalid, the points are co-linear, or
  // the triangle is "too small".  "Small" is an arbitrary value. By design
  // the triangulation process behaves conservatively. A user-defined mesh can
  // include arbitrarily small triangles, but the triangulation process will
  // not.
  static bool GetNormal(const PointsVector& vertices, int i0, int i1, int i2,
                        Eigen::Vector3d* normal, double minArea);

  // This method finds a juxtaposed obj file from the `resolved_filename_`
  // member. If unable to resolve an obj file it throws an exception.
  // If `resolved_filename_` already is an obj file then it returns
  // `resolved_filename_`. Otherwise it attempts to change the file extension
  // and checks if it exists.
  std::string FindFileWithObjExtension() const;
};

class MeshPoints : public Geometry {
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
  friend std::ostream& operator<<(std::ostream&, const MeshPoints&);

  Eigen::Matrix3Xd points;

 private:
  // Allow only the copy constructor, only for our clone().
  MeshPoints(const MeshPoints&) = default;
  void operator=(const MeshPoints&) = delete;
};

}  // namespace DrakeShapes
