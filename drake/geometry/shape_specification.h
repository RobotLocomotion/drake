#pragma once

#include <functional>
#include <memory>
#include <typeindex>

#include "drake/common/drake_assert.h"
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

/** A tag object that denotes a Shape subclass `S` in function signatures.

 For example, `ShapeTag<MyShape>{}` will create a dummy object that
 can be used to call functions that look like:

 @code
 template <class S>
 const char* get_foo(ShapeTag<S>) { return S::get_foo(); }

 int main() {
    std::cout << get_foo(ShapeTag<MyShape>{});
 }
 @endcode

 In this case, we could directly call get_foo<MyShape>() by specifying the
 template argument, but that is not always possible.  In particular, tag
 objects are acutely useful when calling templated constructors, because
 there is no other mechanism for the caller to specify the template type. */
template <typename T>
struct ShapeTag {};

/** The base interface for all shape specifications. Shapes have two basic
 requirements:
   - they must be cloneable, and
   - they must invoke the correct method for themselves on a ShapeReifier.

 The base class handles both requirements on behalf of derived shape classes.
 However, derived concrete classes must have a public copy constructor to
 work. Furthermore, derived classes should be marked final. */
class Shape {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(Shape)
  virtual ~Shape() {}

  /** Causes this description to be reified in the given `reifier`. Each
   concrete subclass must invoke the single, matching method on the reifier. */
  virtual void Reify(ShapeReifier* reifier) const {
    reifier_(reifier);
  }

  /** Creates a unique copy of this shape. Invokes the protected DoClone(). */
  std::unique_ptr<Shape> Clone() const {
    return cloner_();
  }

 protected:
  template <typename S>
  Shape(ShapeTag<S>);

 private:
  std::function<std::unique_ptr<Shape>()> cloner_;
  std::function<void(ShapeReifier*)> reifier_;
};

/** Definition of sphere. It is centered in its canonical frame with the
 given radius. */
class Sphere final : public Shape {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(Sphere)

  explicit Sphere(double radius) : Shape(ShapeTag<Sphere>()), radius_(radius) {}

  double get_radius() const { return radius_; }

 private:
  double radius_{};
};

/** Definition of a half space. In its canonical frame, the plane defining the
 boundary of the half space is that frame's z = 0 plane. By implication, the
 plane's normal points in the +z direction and the origin lies on the plane.
 Other shapes are considered to be penetrating the half space if there exists
 a point on the test shape that lies on the side of the plane opposite the
 normal. */
class HalfSpace final : public Shape {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(HalfSpace)

  HalfSpace() : Shape(ShapeTag<HalfSpace>()) {}

  /** Given a plane `normal_F` and a point on the plane `X_FP`, both expressed
   in frame F, creates the transform `X_FC` from the half-space's canonical
   space to frame F.
   @param normal_F  A vector perpendicular to the half-space's plane boundary
                    expressed in frame F. It must be a non-zero vector but need
                    not be unit length.
   @param r_FP      A point lying on the half-space's plane boundary measured
                    and expressed in frame F.
   @retval `X_FC`   The pose of the canonical half-space in frame F.
   @throws std::logic_error if the normal is _close_ to a zero-vector (e.g.,
                            ‖normal_F‖₂ < ε).

   @tparam T       The underlying scalar type. Must be a valid Eigen scalar. */
  template <typename T>
  static Isometry3<T> MakePose(const Vector3<T>& normal_F,
                               const Vector3<T>& r_FP) {
    T norm = normal_F.norm();
    // Note: this value of epsilon is somewhat arbitrary. It's merely a minor
    // fence over which ridiculous vectors will trip.
    if (norm < 1e-10)
      throw std::logic_error("Can't make pose from a zero vector.");

    // First create basis.
    // Projects the normal into the first quadrant in order to identify the
    // *smallest* component of the normal.
    const Vector3<T> u(normal_F.cwiseAbs());
    int minAxis;
    u.minCoeff(&minAxis);
    // The axis corresponding to the smallest component of the normal will be
    // *most* perpendicular.
    Vector3<T> perpAxis;
    perpAxis << (minAxis == 0 ? 1 : 0), (minAxis == 1 ? 1 : 0),
        (minAxis == 2 ? 1 : 0);
    // Now define x-, y-, and z-axes. The z-axis lies in the normal direction.
    Vector3<T> z_axis_W = normal_F / norm;
    Vector3<T> x_axis_W = normal_F.cross(perpAxis).normalized();
    Vector3<T> y_axis_W = z_axis_W.cross(x_axis_W);
    // Transformation from world frame to local frame.
    Matrix3<T> R_WL;
    R_WL.col(0) = x_axis_W;
    R_WL.col(1) = y_axis_W;
    R_WL.col(2) = z_axis_W;

    // Construct pose from basis and point.
    Isometry3<T> X_FC = Isometry3<T>::Identity();
    X_FC.linear() = R_WL;
    // Find the *minimum* translation to make sure the point lies on the plane.
    X_FC.translation() = z_axis_W.dot(r_FP) * z_axis_W;
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
Shape::Shape(ShapeTag<S>) {
  static_assert(std::is_base_of<Shape, S>::value,
                "Concrete shapes *must* be derived from the Shape class");
  cloner_ = [this]() {
    DRAKE_DEMAND(typeid(*this) == typeid(S));
    return std::unique_ptr<Shape>(new S(*static_cast<const S*>(this)));
  };
  reifier_ = [this](ShapeReifier* reifier) {
    reifier->implementGeometry(*static_cast<const S*>(this));
  };
}

}  // namespace geometry
}  // namespace drake
