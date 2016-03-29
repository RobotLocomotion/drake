#ifndef __DrakeShapesGeometry_H__
#define __DrakeShapesGeometry_H__

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

std::string shapeToString(Shape ss);

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
  friend DRAKESHAPES_EXPORT std::ostream& operator<<(std::ostream&, const Geometry&);

  /*!
   * Compare this geometry with another one for equivalence.
   *
   * @param gg The geometry to compare with.
   * @param explanation An explanation of why they do not match.
   * @return true if the supplied geometry is the same as this one.
   */
  virtual bool Compare(const Geometry & gg, std::string * explanation = nullptr) const;

  /*!
   * Overload operator== to check whether two Geometry objects are equal.
   */
  friend DRAKESHAPES_EXPORT bool operator==(const Geometry & g1, const Geometry & g2);

  /*!
   * Overload operator!= to check whether two Geometry objects are unequal.
   */
  friend DRAKESHAPES_EXPORT bool operator!=(const Geometry & g1, const Geometry & g2);

 protected:
  Geometry(Shape shape);
  void getBoundingBoxPoints(double x_half_width, double y_half_width,
                            double z_half_width,
                            Eigen::Matrix3Xd &points) const;

  Shape shape;
  static const int NUM_BBOX_POINTS;
};

class DRAKESHAPES_EXPORT Sphere : public Geometry {
 public:
  Sphere(double radius);
  virtual ~Sphere() {}
  virtual Sphere *clone() const;
  virtual void getPoints(Eigen::Matrix3Xd &points) const;
  virtual void getBoundingBoxPoints(Eigen::Matrix3Xd &points) const;
  virtual void getTerrainContactPoints(Eigen::Matrix3Xd &points) const;

  /**
   * A toString method for this class.
   */
  friend DRAKESHAPES_EXPORT std::ostream& operator<<(std::ostream&, const Sphere&);

  /*!
   * Compare this sphere with another one for equivalence.
   *
   * @param ss The sphere to compare with.
   * @param explanation An explanation of why they do not match.
   * @return true if the supplied sphere is the same as this one.
   */
  virtual bool Compare(const Sphere & ss, std::string * explanation = nullptr) const;

  /*!
   * Overload operator== to check whether two Sphere objects are equal.
   */
  friend DRAKESHAPES_EXPORT bool operator==(const Sphere & s1, const Sphere & s2);

  /*!
   * Overload operator!= to check whether two Sphere objects are unequal.
   */
  friend DRAKESHAPES_EXPORT bool operator!=(const Sphere & s1, const Sphere & s2);

  double radius;
  static const int NUM_POINTS;
};

class DRAKESHAPES_EXPORT Box : public Geometry {
 public:
  Box(const Eigen::Vector3d &size);
  virtual ~Box() {}
  virtual Box *clone() const;
  virtual void getPoints(Eigen::Matrix3Xd &points) const;
  virtual void getBoundingBoxPoints(Eigen::Matrix3Xd &points) const;
  virtual void getTerrainContactPoints(Eigen::Matrix3Xd &points) const;

  /**
   * A toString method for this class.
   */
  friend DRAKESHAPES_EXPORT std::ostream& operator<<(std::ostream&, const Box&);

  /*!
   * Compare this box with another one for equivalence.
   *
   * @param bb The box to compare with.
   * @param explanation An explanation of why they do not match.
   * @return true if the supplied box is the same as this one.
   */
  virtual bool Compare(const Box & bb, std::string * explanation = nullptr) const;

  /*!
   * Overload operator== to check whether two Box objects are equal.
   */
  friend DRAKESHAPES_EXPORT bool operator==(const Box & b1, const Box & b2);

  /*!
   * Overload operator!= to check whether two Box objects are unequal.
   */
  friend DRAKESHAPES_EXPORT bool operator!=(const Box & b1, const Box & b2);

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
  friend DRAKESHAPES_EXPORT std::ostream& operator<<(std::ostream&, const Cylinder&);

  /*!
   * Compare this cylinder with another one for equivalence.
   *
   * @param cc The cylinder to compare with.
   * @param explanation An explanation of why they do not match.
   * @return true if the supplied cylinder is the same as this one.
   */
  virtual bool Compare(const Cylinder & cc, std::string * explanation = nullptr) const;

  /*!
   * Overload operator== to check whether two Cylinder objects are equal.
   */
  friend DRAKESHAPES_EXPORT bool operator==(const Cylinder & c1, const Cylinder & c2);

  /*!
   * Overload operator!= to check whether two Cylinder objects are unequal.
   */
  friend DRAKESHAPES_EXPORT bool operator!=(const Cylinder & c1, const Cylinder & c2);

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
  friend DRAKESHAPES_EXPORT std::ostream& operator<<(std::ostream&, const Capsule&);

  /*!
   * Compare this capsule with another one for equivalence.
   *
   * @param cc The capsule to compare with.
   * @param explanation An explanation of why they do not match.
   * @return true if the supplied capsule is the same as this one.
   */
  virtual bool Compare(const Capsule & cc, std::string * explanation = nullptr) const;

  /*!
   * Overload operator== to check whether two Capsule objects are equal.
   */
  friend DRAKESHAPES_EXPORT bool operator==(const Capsule & c1, const Capsule & c2);

  /*!
   * Overload operator!= to check whether two Capsule objects are unequal.
   */
  friend DRAKESHAPES_EXPORT bool operator!=(const Capsule & c1, const Capsule & c2);

  double radius;
  double length;

  static const int NUM_POINTS;
};

class DRAKESHAPES_EXPORT Mesh : public Geometry {
 public:
  Mesh(const std::string &filename);
  Mesh(const std::string &filename, const std::string &resolved_filename);
  virtual ~Mesh() {}
  virtual Mesh *clone() const;
  virtual void getPoints(Eigen::Matrix3Xd &points) const;
  virtual void getBoundingBoxPoints(Eigen::Matrix3Xd &points) const;

  /**
   * A toString method for this class.
   */
  friend DRAKESHAPES_EXPORT std::ostream& operator<<(std::ostream&, const Mesh&);

  /*!
   * Compare this mesh with another one for equivalence.
   *
   * @param mm The mesh to compare with.
   * @param explanation An explanation of why they do not match.
   * @return true if the supplied mesh is the same as this one.
   */
  virtual bool Compare(const Mesh & mm, std::string * explanation = nullptr) const;

  /*!
   * Overload operator== to check whether two Mesh objects are equal.
   */
  friend DRAKESHAPES_EXPORT bool operator==(const Mesh & m1, const Mesh & m2);

  /*!
   * Overload operator!= to check whether two Mesh objects are unequal.
   */
  friend DRAKESHAPES_EXPORT bool operator!=(const Mesh & m1, const Mesh & m2);

  double scale;
  std::string filename;
  std::string resolved_filename;
  bool extractMeshVertices(Eigen::Matrix3Xd &vertex_coordinates) const;

 protected:
  std::string root_dir;
};

class DRAKESHAPES_EXPORT MeshPoints : public Geometry {
 public:
  MeshPoints(const Eigen::Matrix3Xd &points);
  virtual ~MeshPoints() {}
  virtual MeshPoints *clone() const;

  virtual void getPoints(Eigen::Matrix3Xd &points) const;
  virtual void getBoundingBoxPoints(Eigen::Matrix3Xd &points) const;

  /**
   * A toString method for this class.
   */
  friend DRAKESHAPES_EXPORT std::ostream& operator<<(std::ostream&, const MeshPoints&);

  /*!
   * Compare this mesh points with another one for equivalence.
   *
   * @param mp The mesh points to compare with.
   * @param explanation An explanation of why they do not match.
   * @return true if the supplied mesh points is the same as this one.
   */
  virtual bool Compare(const MeshPoints & mp, std::string * explanation = nullptr) const;

  /*!
   * Overload operator== to check whether two MeashPoints objects are equal.
   */
  friend DRAKESHAPES_EXPORT bool operator==(const MeshPoints & mp1, const MeshPoints & mp2);

  /*!
   * Overload operator!= to check whether two MeashPoints objects are unequal.
   */
  friend DRAKESHAPES_EXPORT bool operator!=(const MeshPoints & mp1, const MeshPoints & mp2);

  Eigen::Matrix3Xd points;
};
}
#endif
