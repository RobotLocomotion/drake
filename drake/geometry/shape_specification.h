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

/** Simple struct for instantiating the type-specific Shape functionality.
 A class derived from the Shape class will invoke the parent's constructor as
 Shape(ShapeTag<DerivedShape>()). */
template <typename ShapeType>
struct ShapeTag{};

/** The base interface for all shape specifications. It has no public
  constructor and cannot be instantiated directly. The Shape class has two
  key properties:
   - it is cloneable, and
   - it can be "reified" (see ShapeReifier). */
class Shape {
 public:
  virtual ~Shape();

  /** Causes this description to be reified in the given `reifier`. Each
   concrete subclass must invoke the single, matching method on the reifier. */
  void Reify(ShapeReifier* reifier) const;

  /** Creates a unique copy of this shape. Invokes the protected DoClone(). */
  std::unique_ptr<Shape> Clone() const;

 protected:
  // This is *not* in the public section. However, this allows the children to
  // also use this macro, but precludes the possibility of external users
  // slicing Shapes.
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(Shape)

  /** Constructor available for derived class construction. A derived class
   should invoke this in its initialization list, passing a ShapeTag
   instantiated on its derived type, e.g.:

   ```
   class MyShape final : public Shape {
    public:
     MyShape() : Shape(ShapeTag<MyShape>()) {}
     ...
   };
   ```

   The base class provides infrastructure for cloning and reification. To work
   and to maintain sanity, we place the following requirements on derived
   classes:

     1. they must have a public copy constructor,
     2. they must be marked as final, and
     3. their constructors must invoke the parent constructor with a ShapeTag
        instance (as noted above), and
     4. The ShapeReifier class must be extended to include an invocation of
        ShapeReifier::ImplementGeometry() on the derived Shape class.

   @tparam S    The derived shape class. It must derive from Shape. */
  template <typename S>
  explicit Shape(ShapeTag<S> tag);

 private:
  std::function<std::unique_ptr<Shape>(const Shape&)> cloner_;
  std::function<void(const Shape&, ShapeReifier*)> reifier_;
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

/** Definition of a cylinder. It is centered in its canonical frame with the
 length of the cylinder parallel with the frame's z-axis. */
class Cylinder final : public Shape {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(Cylinder)

  Cylinder(double radius, double length);

  double get_radius() const { return radius_; }
  double get_length() const { return length_; }

 private:
  double radius_{};
  double length_{};
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
  virtual void ImplementGeometry(const Sphere& sphere) = 0;
  virtual void ImplementGeometry(const Cylinder& cylinder) = 0;
  virtual void ImplementGeometry(const HalfSpace& half_space) = 0;
};

template <typename S>
Shape::Shape(ShapeTag<S>) {
  static_assert(std::is_base_of<Shape, S>::value,
                "Concrete shapes *must* be derived from the Shape class");
  cloner_ = [](const Shape& shape_arg) {
    DRAKE_DEMAND(typeid(shape_arg) == typeid(S));
    const S& derived_shape = static_cast<const S&>(shape_arg);
    return std::unique_ptr<Shape>(new S(derived_shape));
  };
  reifier_ = [](const Shape& shape_arg, ShapeReifier* reifier) {
    DRAKE_DEMAND(typeid(shape_arg) == typeid(S));
    const S& derived_shape = static_cast<const S&>(shape_arg);
    reifier->ImplementGeometry(derived_shape);
  };
}

}  // namespace geometry
}  // namespace drake
