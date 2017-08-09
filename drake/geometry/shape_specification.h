#pragma once

#include <memory>

#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"

/** @file
 Provides the classes through which geometric shapes are introduced into
 GeometryWorld. This includes the specific classes which specify shapes as well
 as an interface for _processing_ those specifications.
 */

namespace drake {
namespace geometry {

class ShapeReifier;

/** The base interface for all shape specifications. Shapes have two basic
 requirements:
   - hey must be cloneable, and
   - they must invoke the correct method for themselves on a ShapeReifier. */
class Shape {
 public:
  virtual ~Shape() {}

  /** Causes this description to be reified in the given `reifier`. Each
   concrete subclass must invoke the single, matching method on the reifier. */
  virtual void Reify(ShapeReifier* reifier) const = 0;

  /** Creates a unique copy of this shape. Invokes the protected DoClone(). */
  std::unique_ptr<Shape> Clone() const {
    return std::unique_ptr<Shape>(DoClone());
  }

 protected:
  /** Performs the work of cloning a concrete shape. */
  virtual Shape* DoClone() const = 0;
};

/** Utility class for making sure that concrete Shapes satisfy the requirements
 for shapes (cloneable and reifiable).

 Concrete shape classes should be derived from this class (in a CRTP way). They
 must also define a copy constructor. By doing so, concrete classes do not have
 to explicitly implement the cloning mechanism _or_ the reification mechanism.

 @internal Technically, this class could be rolled into Shape. But we want to
 be able to talk about Shapes without a hint of templating. So, this serves as
 an intermediate class to gain the CRTP benefit without cluttering up the
 public API.

 @tparam S The recursively-defined concrete Shape class.
 */
template <typename S>
class ReifiableShape : public Shape {
 public:
  ReifiableShape() : Shape() {
    // This enforces the existence of a copy constructor and the recursive
    // nature of the sub-classes.
    static_assert(std::is_base_of<Shape, S>::value &&
                      std::is_copy_constructible<S>::value,
                  "ReifiableShape should only be instantiated recursively on "
                  "concrete derivations of the Shape class.");
  }

  void Reify(ShapeReifier* reifier) const override;

 protected:
  Shape* DoClone() const override {
    return new S(*static_cast<const S*>(this));
  }
};

/** Definition of sphere. It is centered in its canonical frame with the
 given radius. */
class Sphere : public ReifiableShape<Sphere> {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(Sphere)

  explicit Sphere(double radius) : radius_(radius) {}

  double get_radius() const { return radius_; }

 private:
  double radius_{};
};

/** Definition of a half space. In its canonical frame, the plan defining the
 boundary of the half space is that frame's z = 0 plane. By implication, the
 plane's normal points in the +z direction and the origin lies on the plane.
 Other shapes are considered to be penetrating the half space if there exists
 a point on the test shape that lies on the side of the plane opposite the
 normal. */
class HalfSpace : public ReifiableShape<HalfSpace> {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(HalfSpace)

  HalfSpace() : ReifiableShape() {}

  /** Given a plane `normal` and a point `X_FP` on the plane, both expressed in
   frame F, creates the transform `X_FC` from the half-space's canonical space
   to frame F.
   @param normal   A vector perpendicular to the half-space's plane boundary.
                   Must be a non-zero vector but need not be unit length.
   @param X_FP     A point lying on the half-space's plane boundary.
   @retval `X_FC`  The pose of the canonical half-space in frame F.
   @throws std::logic_error if the normal is _close_ to a zero-vector (e.g.,
                            ‖normal‖₂ < ε).

   @tparam T       The underlying scalar type. Must be a valid Eigen scalar. */
  template <typename T>
  static Isometry3<T> MakePose(const Vector3<T>& normal,
                               const Vector3<T>& X_FP) {
    T norm = normal.norm();
    // Note: this value of epsilon is somewhat arbitrary. It's merely a minor
    // fence over which ridiculous vectors will trip.
    if (norm < 1e-10)
      throw std::logic_error("Can't make pose from a zero vector.");

    // First create basis.
    // Projects the normal into the first quadrant in order to identify the
    // *smallest* component of the normal.
    const Vector3<T> u(normal.cwiseAbs());
    int minAxis;
    u.minCoeff(&minAxis);
    // The axis corresponding to the smallest component of the normal will be
    // *most* perpendicular.
    Vector3<T> perpAxis;
    perpAxis << (minAxis == 0 ? 1 : 0), (minAxis == 1 ? 1 : 0),
        (minAxis == 2 ? 1 : 0);
    // Now define x-, y-, and z-axes. The z-axis lies in the normal direction.
    Vector3<T> z_axis_W = normal / norm;
    Vector3<T> x_axis_W = normal.cross(perpAxis).normalized();
    Vector3<T> y_axis_W = normal.cross(x_axis_W);
    // Transformation from world frame to local frame.
    Matrix3<T> R_WL;
    R_WL.col(0) = x_axis_W;
    R_WL.col(1) = y_axis_W;
    R_WL.col(2) = z_axis_W;

    // Construct pose from basis and point.
    Isometry3<T> X_FC = Isometry3<T>::Identity();
    X_FC.linear() = R_WL;
    X_FC.translation() = X_FP;
    return X_FC;
  }
};

/** The interface for converting shape descriptions to real shapes. Any entity
 that consumes shape descriptions _must_ implement this interface.

 This class explicitly enumerates all concrete shapes in its methods. The
 addition of a new concrete shape class requires the addition of a new
 corresponding method. There should *never* be a method that accepts the Shape
 base class as an argument; it should _only_ operate on concrete derived
 classes. */
class ShapeReifier {
 public:
  virtual ~ShapeReifier() {}
  virtual void implementGeometry(const Sphere& sphere) = 0;
  virtual void implementGeometry(const HalfSpace& half_space) = 0;
};

template <typename S>
void ReifiableShape<S>::Reify(ShapeReifier* reifier) const {
  reifier->implementGeometry(*static_cast<const S*>(this));
}

}  // namespace geometry
}  // namespace drake
