#include "drake/geometry/shape_specification.h"

#include "drake/common/text_logging.h"

namespace drake {
namespace geometry {

Shape::~Shape() {}

void Shape::Reify(ShapeReifier* reifier, void* user_data) const {
  reifier_(*this, reifier, user_data); }

std::unique_ptr<Shape> Shape::Clone() const { return cloner_(*this); }

Sphere::Sphere(double radius)
    : Shape(ShapeTag<Sphere>()), radius_(radius) {}

Cylinder::Cylinder(double radius, double length)
    : Shape(ShapeTag<Cylinder>()),
      radius_(radius),
      length_(length) {}

HalfSpace::HalfSpace() : Shape(ShapeTag<HalfSpace>()) {}

Isometry3<double> HalfSpace::MakePose(const Vector3<double>& normal_F,
                                      const Vector3<double>& r_FP) {
  double norm = normal_F.norm();
  // Note: this value of epsilon is somewhat arbitrary. It's merely a minor
  // fence over which ridiculous vectors will trip.
  if (norm < 1e-10)
    throw std::logic_error("Can't make pose from a zero vector.");

  // First create basis.
  // Projects the normal into the first quadrant in order to identify the
  // *smallest* component of the normal.
  const Vector3<double> u(normal_F.cwiseAbs());
  int minAxis;
  u.minCoeff(&minAxis);
  // The axis corresponding to the smallest component of the normal will be
  // *most* perpendicular.
  Vector3<double> perpAxis;
  perpAxis << (minAxis == 0 ? 1 : 0), (minAxis == 1 ? 1 : 0),
      (minAxis == 2 ? 1 : 0);
  // Now define x-, y-, and z-axes. The z-axis lies in the normal direction.
  Vector3<double> z_axis_W = normal_F / norm;
  Vector3<double> x_axis_W = normal_F.cross(perpAxis).normalized();
  Vector3<double> y_axis_W = z_axis_W.cross(x_axis_W);
  // Transformation from world frame to local frame.
  Matrix3<double> R_WL;
  R_WL.col(0) = x_axis_W;
  R_WL.col(1) = y_axis_W;
  R_WL.col(2) = z_axis_W;

  // Construct pose from basis and point.
  Isometry3<double> X_FC = Isometry3<double>::Identity();
  X_FC.linear() = R_WL;
  // Find the *minimum* translation to make sure the point lies on the plane.
  X_FC.translation() = z_axis_W.dot(r_FP) * z_axis_W;
  return X_FC;
}

Mesh::Mesh(const std::string& absolute_filename, double scale)
    : Shape(ShapeTag<Mesh>()), filename_(absolute_filename), scale_(scale) {
  // TODO(SeanCurtis-TRI): Remove this when meshes are properly supported.
  drake::log()->warn("Meshes are only supported for drake_visualizer ({})",
                     filename_);
}

}  // namespace geometry
}  // namespace drake
