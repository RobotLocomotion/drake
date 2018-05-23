#pragma once

#include <functional>
#include <memory>
#include <string>
#include <typeindex>

#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"

/** @file
 Provides the classes through which geometric shapes are introduced into
 SceneGraph. This includes the specific classes which specify shapes as well
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
   concrete subclass must invoke the single, matching method on the reifier.
   Provides optional user-data (cast as a void*) for the reifier to consume. */
  void Reify(ShapeReifier* reifier, void* user_data = nullptr) const;

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
  std::function<void(const Shape&, ShapeReifier*, void*)> reifier_;
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

/** Definition of a box. The box is centered on the origin of its canonical
 frame with its dimensions aligned with the frame's axes. The size of the box
 is given by three sizes. */
class Box final : public Shape {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(Box)

  /** Constructs a box with the given `width`, `depth`, and `height`, which
   specify the box's dimension along the canonical x-, y-, and z-axes,
   respectively.  */
  Box(double width, double depth, double height);

  /** Returns the box's dimension along the x axis. */
  double width() const { return size_(0); }

  /** Returns the box's dimension along the y axis. */
  double depth() const { return size_(1); }

  /** Returns the box's dimension along the z axis. */
  double height() const { return size_(2); }

  /** Returns the box's dimensions. */
  const Vector3<double>& size() const { return size_; }

 private:
  Vector3<double> size_;
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

  /** Creates the pose of a canonical half space in frame F.
   The half space's normal is aligned to the positive z-axis of its canonical
   frame C. Given the measure of that axis in frame F (Cz_F) and a position
   vector to a point on the plane expressed in the same frame, `p_FC`, creates
   the pose of the half space in frame F: `X_FC`.
   @param Cz_F      The positive z-axis of the canonical frame expressed in
                    frame F. It must be a non-zero vector but need not be unit
                    length.
   @param p_FC      A point lying on the half-space's boundary measured
                    and expressed in frame F.
   @retval `X_FC`   The pose of the canonical half-space in frame F.
   @throws std::logic_error if the normal is _close_ to a zero-vector (e.g.,
                            ‖normal_F‖₂ < ε). */
  static Isometry3<double> MakePose(const Vector3<double>& Cz_F,
                                    const Vector3<double>& p_FC);
};

// TODO(SeanCurtis-TRI): Update documentation when the level of support for
// meshes extends to collision/rendering.
/** Limited support for meshes. Meshes declared as such will _not_ serve in
 proximity queries or rendering queries. However, they _will_ be propagated
 to drake_visualizer. The mesh is dispatched to drake visualizer via the
 filename. The mesh is _not_ parsed/loaded by Drake. */
class Mesh final : public Shape {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(Mesh)

  /** Constructs a mesh specification from the mesh file located at the given
   _absolute_ file path. Optionally uniformly scaled by the given scale factor.
   */
  explicit Mesh(const std::string& absolute_filename, double scale = 1.0);

  const std::string& filename() const { return filename_; }
  double scale() const { return scale_; }

 private:
  // NOTE: Cannot be const to support default copy/move semantics.
  std::string filename_;
  double scale_;
};

/** The interface for converting shape descriptions to real shapes. Any entity
 that consumes shape descriptions _must_ implement this interface.

 This class explicitly enumerates all concrete shapes in its methods. The
 addition of a new concrete shape class requires the addition of a new
 corresponding method. There should *never* be a method that accepts the Shape
 base class as an argument; it should _only_ operate on concrete derived
 classes.

 The expected workflow is for a class that needs to turn shape specifications
 into concrete geometry instances to implement the %ShapeReifier interface
 _and_ invoke the Shape::Reify() method. For example, a simple reifier that
 requires no user data would look like:

 ```
 class SimpleReifier : public ShapeReifier {
   void ProcessShape(const Shape& shape) {
     // Requires no user data.
     shape.Reify(this);
   }
   ...
   void ImplementGeometry(const Sphere& sphere, void*) override {
     // Do work to create a sphere.
   }
 };
 ```

 Or a complex reifier that requires user data would look like:

 ```
 class ComplexReifier : public ShapeReifier {
   void ProcessShape(const Shape& shape) {
     ImportantData data{...};
     shape.Reify(this, &data);
   }
   ...
   void ImplementGeometry(const Sphere& sphere, void* data) override {
     DRAKE_ASSERT(data);
     ImportantData& data = *reinterpret_cast<ImportantData*>(data);
     // Do work to create a sphere using the provided user data.
   }
 };
 ```

 Implementing a particular shape may require more data than is strictly
 encapsulated in the Shape. The Implement* interface supports passing user
 data through a type-erased `void*`. Because a single class invoked
 Shape::Reify() it is in a position to provide exactly the data the shape
 implementations require.  */
class ShapeReifier {
 public:
  virtual ~ShapeReifier() {}
  virtual void ImplementGeometry(const Sphere& sphere, void* user_data) = 0;
  virtual void ImplementGeometry(const Cylinder& cylinder, void* user_data) = 0;
  virtual void ImplementGeometry(const HalfSpace& half_space,
                                 void* user_data) = 0;
  virtual void ImplementGeometry(const Box& box, void* user_data) = 0;
  virtual void ImplementGeometry(const Mesh& mesh, void* user_data) = 0;
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
  reifier_ = [](const Shape& shape_arg, ShapeReifier* reifier,
                void* user_data) {
    DRAKE_DEMAND(typeid(shape_arg) == typeid(S));
    const S& derived_shape = static_cast<const S&>(shape_arg);
    reifier->ImplementGeometry(derived_shape, user_data);
  };
}

}  // namespace geometry
}  // namespace drake
