#include "Geometry.h"

using namespace std;
using namespace Eigen;

namespace DrakeCollision
{
  Geometry::Geometry(Shape shape) : shape(shape) {};

  const Shape Geometry::getShape() const
  {
    return shape;
  }

  Sphere::Sphere(double radius)
    : Geometry(SPHERE), radius(radius) {}

  Box::Box(const Eigen::Vector3d& size)
    : Geometry(BOX), size(size) {}

  Cylinder::Cylinder(double radius, double length)
    : Geometry( CYLINDER), radius(radius), length(length) {}

  Capsule::Capsule(double radius, double length)
    : Geometry(CAPSULE), radius(radius), length(length) {}

  Mesh::Mesh(const Eigen::Matrix3Xd& points) 
    : Geometry(MESH), points(points) {}

}
