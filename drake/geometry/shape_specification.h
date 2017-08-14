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

/** The base interface for all shape specifications. It has no public
  constructor and cannot be instantiated directly. All Shape classes have two
  basic requirements:
   - they must be cloneable, and
   - they must invoke the correct method for themselves on a ShapeReifier.

 The base class handles both requirements on behalf of derived shape classes.
 However, it places several requirements on derived classes:

   1. they must have a public copy constructor,
   2. they must be marked as final, and
   3. their constructors must invoke the parent constructor with a nullptr
      cast as their type (see Shape() for details), and
   4. The ShapeReifier class must be extended to include a an invocation of
      ShapeReifier::implementGeometry() on the derived Shape class. */
class Shape {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(Shape)
  virtual ~Shape();

  /** Causes this description to be reified in the given `reifier`. Each
   concrete subclass must invoke the single, matching method on the reifier. */
  void Reify(ShapeReifier* reifier) const;

  /** Creates a unique copy of this shape. Invokes the protected DoClone(). */
  std::unique_ptr<Shape> Clone() const;

 protected:
  /** Constructor available for derived class construction. A derived class
   should provide invoke this in its initialization list, passing a `nullptr`
   cast to a pointer of the derived type, e.g.:

   ```
   class MyShape final : public Shape {
    public:
     MyShape() : Shape(static_cast<MyShape*>(nullptr)) {}
     ...
   };
   ```

   @tparam S    The derived shape class. It must derive from Shape. */
  template <typename S>
  explicit Shape(S*);

 private:
  std::function<std::unique_ptr<Shape>()> cloner_;
  std::function<void(ShapeReifier*)> reifier_;
};

/** Definition of sphere. It is centered in its canonical frame with the
 given radius. */
class Sphere final : public Shape {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(Sphere)

  explicit Sphere(double radius);

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

  HalfSpace();

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
                            ‖normal_F‖₂ < ε). */
  static Isometry3<double> MakePose(const Vector3<double>& normal_F,
                                    const Vector3<double>& r_FP);
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
Shape::Shape(S*) {
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
