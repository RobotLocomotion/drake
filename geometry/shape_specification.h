#pragma once

#include <functional>
#include <memory>
#include <string>
#include <typeindex>

#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/drake_deprecated.h"
#include "drake/common/eigen_types.h"
#include "drake/math/rigid_transform.h"

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
   - it can be "reified" (see ShapeReifier).

  When you add a new subclass of Shape, you must:

  1. add a virtual function ImplementGeometry() for the new shape in
     ShapeReifier that invokes the ThrowUnsupportedGeometry method, and add to
     the test for it in shape_specification_test.cc.
  2. implement ImplementGeometry in derived ShapeReifiers to continue support
     if desired, otherwise ensure unimplemented functions are not hidden in new
     derivations of ShapeReifier with `using`, for example, `using
     ShapeReifier::ImplementGeometry`. Existing subclasses should already have
     this.

  Otherwise, you might get a runtime error. We do not have an automatic way to
  enforce them at compile time.
 */
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

  /** Constructs a sphere with the given `radius`.
   @throws std::logic_error if `radius` is negative. Note that a zero radius is
   is considered valid. */
  explicit Sphere(double radius);

  double radius() const { return radius_; }

 private:
  double radius_{};
};

/** Definition of a cylinder. It is centered in its canonical frame with the
 length of the cylinder parallel with the frame's z-axis. */
class Cylinder final : public Shape {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(Cylinder)

  /** Constructs a cylinder with the given `radius` and `length`.
   @throws std::logic_error if `radius` or `length` are not strictly positive.
   */
  Cylinder(double radius, double length);

  double radius() const { return radius_; }
  double length() const { return length_; }

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
   respectively.
   @throws std::logic_error if `width`, `depth` or `height` are not strictly
   positive. */
  Box(double width, double depth, double height);

  /** Constructs a cube with the given `edge_size` for its width, depth, and
   height. */
  static Box MakeCube(double edge_size);

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

/** Definition of a capsule. It is centered in its canonical frame with the
 length of the capsule parallel with the frame's z-axis. */
class Capsule final : public Shape {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(Capsule)

  /** Constructs a capsule with the given `radius` and `length`.
   @throws std::logic_error if `radius` or `length` are not strictly positive.
   */
  Capsule(double radius, double length);

  double radius() const { return radius_; }
  double length() const { return length_; }

 private:
  double radius_{};
  double length_{};
};

/** Definition of an ellipsoid. It is centered on the origin of its canonical
 frame with its dimensions aligned with the frame's axes. The standard
 equation for the ellipsoid is:

          x²/a² + y²/b² + z²/c² = 1,
 where a,b,c are the lengths of the principal semi-axes of the ellipsoid.
 The bounding box of the ellipsoid is [-a,a]x[-b,b]x[-c,c].
*/
class Ellipsoid final : public Shape {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(Ellipsoid)

  /** Constructs an ellipsoid with the given lengths of its principal
   semi-axes.
   @throws std::logic_error if `a`, `b`, or `c` are not strictly positive.
   */
  Ellipsoid(double a, double b, double c);

  double a() const { return radii_(0); }
  double b() const { return radii_(1); }
  double c() const { return radii_(2); }

 private:
  Vector3<double> radii_;
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
   frame H. Given a vector that points in the same direction, measured in the
   F frame (Hz_dir_F) and a position vector to a point on the half space's
   *boundary* expressed in the same frame, `p_FB`, creates
   the pose of the half space in frame F: `X_FH`.
   @param Hz_dir_F  A vector in the direction of the positive z-axis of the
                    canonical frame expressed in frame F. It must be a non-zero
                    vector but need not be unit length.
   @param p_FB      A point B lying on the half space's boundary measured
                    and expressed in frame F.
   @retval X_FH     The pose of the canonical half-space in frame F.
   @throws std::logic_error if the normal is _close_ to a zero-vector (e.g.,
                            ‖normal_F‖₂ < ε). */
  static math::RigidTransform<double> MakePose(const Vector3<double>& Hz_dir_F,
                                               const Vector3<double>& p_FB);
};

// TODO(DamrongGuoy): Update documentation when the level of support for
//  meshes extends to more collision and rendering.
/** Limited support for meshes. Meshes are supported in Rendering and
 Illustration roles. For Proximity role, Meshes are supported in
 ComputeContactSurfaces() query only. No other proximity queries are supported.
 */
class Mesh final : public Shape {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(Mesh)

  /** Constructs a mesh specification from the mesh file located at the given
   _absolute_ file path. Optionally uniformly scaled by the given scale factor.
   @throws std::logic_error if |scale| < 1e-8. Note that a negative scale is
   considered valid. We want to preclude scales near zero but recognise that
   scale is a convenience tool for "tweaking" models. 8 orders of magnitude
   should be plenty without considering revisiting the model itself. */
  explicit Mesh(const std::string& absolute_filename, double scale = 1.0);

  const std::string& filename() const { return filename_; }
  double scale() const { return scale_; }

 private:
  // NOTE: Cannot be const to support default copy/move semantics.
  std::string filename_;
  double scale_;
};

/** Support for convex shapes. */
class Convex final : public Shape {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(Convex)

  /** Constructs a convex shape specification from the file located at the
   given _absolute_ file path. Optionally uniformly scaled by the given scale
   factor.
   @param absolute_filename     The file name with absolute path. We only
                                support an .obj file with only one polyhedron.
                                We assume that the polyhedron is convex.
   @param scale                 An optional scale to coordinates.

   @throws std::runtime_error   if the .obj file doesn't define a single object.
                                This can happen if it is empty, if there are
                                multiple object-name statements (e.g.,
                                "o object_name"), or if there are faces defined
                                outside a single object-name statement.
   @throws std::logic_error     if |scale| < 1e-8. Note that a negative scale is
                                considered valid. We want to preclude scales
                                near zero but recognise that scale is a
                                convenience tool for "tweaking" models. 8 orders
                                of magnitude should be plenty without
                                considering revisiting the model itself. */
  explicit Convex(const std::string& absolute_filename, double scale = 1.0);

  const std::string& filename() const { return filename_; }
  double scale() const { return scale_; }

 private:
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
  virtual ~ShapeReifier() = default;

  virtual void ImplementGeometry(const Sphere& sphere, void* user_data);
  virtual void ImplementGeometry(const Cylinder& cylinder, void* user_data);
  virtual void ImplementGeometry(const HalfSpace& half_space, void* user_data);
  virtual void ImplementGeometry(const Box& box, void* user_data);
  virtual void ImplementGeometry(const Capsule& capsule, void* user_data);
  virtual void ImplementGeometry(const Ellipsoid& ellipsoid, void* user_data);
  virtual void ImplementGeometry(const Mesh& mesh, void* user_data);
  virtual void ImplementGeometry(const Convex& convex, void* user_data);

 protected:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(ShapeReifier)
  ShapeReifier() = default;

 private:
  void ThrowUnsupportedGeometry(const std::string& shape_name);
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

// TODO(SeanCurtis-TRI): Merge this into shape_to_string.h so that there's a
//  single utility for getting a string from a shape.
/** Class that reports the name of the type of shape being reified (e.g.,
 Sphere, Box, etc.)  */
class ShapeName final : public ShapeReifier {
 public:
  ShapeName() = default;

  /** Constructs a %ShapeName from the given `shape` such that `string()`
   already contains the string representation of `shape`.  */
  explicit ShapeName(const Shape& shape) {
    shape.Reify(this);
  }

  /** @name  Implementation of ShapeReifier interface  */
  //@{

  using ShapeReifier::ImplementGeometry;

  void ImplementGeometry(const Sphere&, void*) final {
    string_ = "Sphere";
  }
  void ImplementGeometry(const Cylinder&, void*) final {
    string_ = "Cylinder";
  }
  void ImplementGeometry(const HalfSpace&, void*) final {
    string_ = "HalfSpace";
  }
  void ImplementGeometry(const Box&, void*) final {
    string_ = "Box";
  }
  void ImplementGeometry(const Capsule&, void*) final {
    string_ = "Capsule";
  }
  void ImplementGeometry(const Ellipsoid&, void*) final {
    string_ = "Ellipsoid";
  }
  void ImplementGeometry(const Mesh&, void*) final {
    string_ = "Mesh";
  }
  void ImplementGeometry(const Convex&, void*) final {
    string_ = "Convex";
  }

  //@}

  /** Returns the name of the last shape reified. Empty if no shape has been
   reified yet.  */
  std::string name() const { return string_; }

 private:
  std::string string_;
};

/** @relates ShapeName */
std::ostream& operator<<(std::ostream& out, const ShapeName& name);

}  // namespace geometry
}  // namespace drake
