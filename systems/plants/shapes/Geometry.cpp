#include "Geometry.h"

using namespace std;
using namespace Eigen;

namespace DrakeShapes
{
  Geometry::Geometry() : shape(UNKNOWN) {}

  Geometry::Geometry(const Geometry& other)
  {
    shape = other.getShape();
  }

  Geometry::Geometry(Shape shape) : shape(shape) {};

  const Shape Geometry::getShape() const
  {
    return shape;
  }

  Geometry* Geometry::clone() const
  {
    return new Geometry(*this);
  }

  Sphere::Sphere(double radius)
    : Geometry(SPHERE), radius(radius) {}

  Sphere* Sphere::clone() const
  {
    return new Sphere(*this);
  }

  Box::Box(const Eigen::Vector3d& size)
    : Geometry(BOX), size(size) {}

  Box* Box::clone() const
  {
    return new Box(*this);
  }

  Cylinder::Cylinder(double radius, double length)
    : Geometry( CYLINDER), radius(radius), length(length) {}

  Cylinder* Cylinder::clone() const
  {
    return new Cylinder(*this);
  }

  Capsule::Capsule(double radius, double length)
    : Geometry(CAPSULE), radius(radius), length(length) {}

  Capsule* Capsule::clone() const
  {
    return new Capsule(*this);
  }

  Mesh::Mesh(const string filename)
    : Geometry(MESH), filename(filename) {}

  Mesh* Mesh::clone() const
  {
    return new Mesh(*this);
  }

  MeshPoints::MeshPoints(const Eigen::Matrix3Xd& points) 
    : Geometry(MESH_POINTS), points(points) {}

  MeshPoints* MeshPoints::clone() const
  {
    return new MeshPoints(*this);
  }

}
