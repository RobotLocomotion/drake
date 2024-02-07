#pragma once

#include <memory>
#include <string>

#include "drake/common/drake_copyable.h"
#include "drake/common/drake_deprecated.h"
#include "drake/common/eigen_types.h"
#include "drake/common/fmt_ostream.h"
#include "drake/math/rigid_transform.h"

/** @file
 Provides the classes through which geometric shapes are introduced into
 SceneGraph. This includes the specific classes which specify shapes as well
 as an interface for _processing_ those specifications.
 */

namespace drake {
namespace geometry {

#ifndef DRAKE_DOXYGEN_CXX
class Box;
class Capsule;
class Convex;
class Cylinder;
class Ellipsoid;
class HalfSpace;
class Mesh;
class MeshcatCone;
class ShapeReifier;
class Sphere;
#endif

// Implementation note for Drake developers:
//
// When you add a new subclass of Shape to Drake, you must:
//
// 1. Add a virtual function ImplementGeometry() for the new shape in
//    ShapeReifier that invokes the ThrowUnsupportedGeometry method, and add to
//    the test for it in shape_specification_test.cc.
//
// 2. Implement ImplementGeometry in derived ShapeReifiers to continue support
//    if desired, otherwise ensure unimplemented functions are not hidden in new
//    derivations of ShapeReifier with `using`, for example, `using
//    ShapeReifier::ImplementGeometry`. Existing subclasses should already have
//    this. Otherwise, you might get a runtime error; we do not have an
//    automatic way to enforce them at compile time.

/** The abstract base class for all shape specifications. Concrete subclasses
  exist for specific shapes (e.g., Box, Mesh, etc.).

  The Shape class has two key properties:

   - it is cloneable, and
   - it can be "reified" (see ShapeReifier).

 Note that the Shape class hierarchy is closed to third-party extensions. All
 Shape classes must be defined within Drake directly (and in this h/cc file
 pair in particular). */
class Shape {
 public:
  virtual ~Shape();

  /** Causes this description to be reified in the given `reifier`. Each
   concrete subclass must invoke the single, matching method on the reifier.
   Provides optional user-data (cast as a void*) for the reifier to consume. */
  void Reify(ShapeReifier* reifier, void* user_data = nullptr) const;

  /** Creates a unique copy of this shape. */
  std::unique_ptr<Shape> Clone() const;

  /** Returns the (unqualified) type name of this Shape, e.g., "Box". */
  std::string_view type_name() const { return do_type_name(); }

  /** Returns a string representation of this shape. */
  std::string to_string() const { return do_to_string(); }

 protected:
  /** (Internal use only) Constructor for use by derived classes.
  All subclasses of Shape must be marked `final`. */
  Shape();

  // This is DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN, marked "internal use only".
  /** (Internal use only) For derived classes. */
  Shape(const Shape&) = default;
  /** (Internal use only) For derived classes. */
  Shape& operator=(const Shape&) = default;
  /** (Internal use only) For derived classes. */
  Shape(Shape&&) = default;
  /** (Internal use only) For derived classes. */
  Shape& operator=(Shape&&) = default;

  /** (Internal use only) NVI for Reify(). */
  virtual void DoReify(ShapeReifier*, void*) const = 0;

  /** (Internal use only) NVI for Clone(). */
  virtual std::unique_ptr<Shape> DoClone() const = 0;

  /** (Internal use only) NVI for type_name(). */
  virtual std::string_view do_type_name() const = 0;

  /** (Internal use only) NVI for to_string(). */
  virtual std::string do_to_string() const = 0;
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
   @throws std::exception if `width`, `depth` or `height` are not strictly
   positive. */
  Box(double width, double depth, double height);

  /** Constructs a box with a vector of measures: width, depth, and height --
   the box's dimensions along the canonical x-, y-, and z-axes, respectively.
   @throws std::exception if the measures are not strictly positive. */
  explicit Box(const Vector3<double>& measures);

  ~Box() final;

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
  void DoReify(ShapeReifier*, void*) const final;
  std::unique_ptr<Shape> DoClone() const final;
  std::string_view do_type_name() const final;
  std::string do_to_string() const final;

  Vector3<double> size_;
};

/** Definition of a capsule. The capsule can be thought of as a cylinder with
 spherical caps attached. The capsule's length refers to the length of the
 cylindrical region, and the radius applies to both the cylinder and spherical
 caps. A capsule with zero length is a sphere of the given radius. And a capsule
 with zero radius is a line segment with the given length. The capsule is
 defined in its canonical frame C, centered on the frame origin and with the
 length of the capsule parallel with the frame's z-axis. */
class Capsule final : public Shape {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(Capsule)

  /** Constructs a capsule with the given `radius` and `length`.
   @throws std::exception if `radius` or `length` are not strictly positive.
   */
  Capsule(double radius, double length);

  /** Constructs a capsule with a vector of measures: radius and length.
   @throws std::exception if the measures are not strictly positive. */
  explicit Capsule(const Vector2<double>& measures);

  ~Capsule() final;

  double radius() const { return radius_; }
  double length() const { return length_; }

 private:
  void DoReify(ShapeReifier*, void*) const final;
  std::unique_ptr<Shape> DoClone() const final;
  std::string_view do_type_name() const final;
  std::string do_to_string() const final;

  double radius_{};
  double length_{};
};

/** Definition of a *convex* surface mesh.

 The mesh is defined in a canonical frame C, implicit in the file parsed. Upon
 loading it in SceneGraph it can be scaled around the origin of C by a given
 `scale` amount. */
class Convex final : public Shape {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(Convex)

  /** Constructs a convex shape specification from the file located at the
   given file path. Optionally uniformly scaled by the given scale factor.

   * We only support an .obj file with only one polyhedron.
   * We assume that the polyhedron is convex.

   @param filename     The file name; if it is not absolute, it will be
                       interpreted relative to the current working directory.
   @param scale        An optional scale to coordinates.

   @throws std::exception       if the .obj file doesn't define a single object.
                                This can happen if it is empty, if there are
                                multiple object-name statements (e.g.,
                                "o object_name"), or if there are faces defined
                                outside a single object-name statement.
   @throws std::exception       if |scale| < 1e-8. Note that a negative scale is
                                considered valid. We want to preclude scales
                                near zero but recognise that scale is a
                                convenience tool for "tweaking" models. 8 orders
                                of magnitude should be plenty without
                                considering revisiting the model itself. */
  explicit Convex(const std::string& filename, double scale = 1.0);

  ~Convex() final;

  const std::string& filename() const { return filename_; }
  /** Returns the extension of the mesh filename -- all lower case and including
   the dot. In other words /foo/bar/mesh.obj and /foo/bar/mesh.OBJ would both
   report the ".obj" extension. The "extension" portion of the filename is
   defined as in std::filesystem::path::extension(). */
  const std::string& extension() const { return extension_; }
  double scale() const { return scale_; }

 private:
  void DoReify(ShapeReifier*, void*) const final;
  std::unique_ptr<Shape> DoClone() const final;
  std::string_view do_type_name() const final;
  std::string do_to_string() const final;

  std::string filename_;
  std::string extension_;
  double scale_{};
};

/** Definition of a cylinder. It is centered in its canonical frame with the
 length of the cylinder parallel with the frame's z-axis. */
class Cylinder final : public Shape {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(Cylinder)

  /** Constructs a cylinder with the given `radius` and `length`.
   @throws std::exception if `radius` or `length` are not strictly positive.
   */
  Cylinder(double radius, double length);

  /** Constructs a cylinder with a vector of measures: radius and length.
   @throws std::exception if the measures are not strictly positive. */
  explicit Cylinder(const Vector2<double>& measures);

  ~Cylinder() final;

  double radius() const { return radius_; }
  double length() const { return length_; }

 private:
  void DoReify(ShapeReifier*, void*) const final;
  std::unique_ptr<Shape> DoClone() const final;
  std::string_view do_type_name() const final;
  std::string do_to_string() const final;

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
   semi-axes, with a, b, and c measured along the x-, y-, and z- axes of the
   canonical frame, respectively.
   @throws std::exception if `a`, `b`, or `c` are not strictly positive.
   */
  Ellipsoid(double a, double b, double c);

  /** Constructs an ellipsoid with a vector of measures: the lengths of its
   principal semi-axes, with a, b, and c measured along the x-, y-, and z- axes
   of the canonical frame, respectively.
   @throws std::exception if the measures are not strictly positive. */
  explicit Ellipsoid(const Vector3<double>& measures);

  ~Ellipsoid() final;

  double a() const { return radii_(0); }
  double b() const { return radii_(1); }
  double c() const { return radii_(2); }

 private:
  void DoReify(ShapeReifier*, void*) const final;
  std::unique_ptr<Shape> DoClone() const final;
  std::string_view do_type_name() const final;
  std::string do_to_string() const final;

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

  ~HalfSpace() final;

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
   @throws std::exception if the normal is _close_ to a zero-vector (e.g.,
                          ‖normal_F‖₂ < ε). */
  static math::RigidTransform<double> MakePose(const Vector3<double>& Hz_dir_F,
                                               const Vector3<double>& p_FB);

 private:
  void DoReify(ShapeReifier*, void*) const final;
  std::unique_ptr<Shape> DoClone() const final;
  std::string_view do_type_name() const final;
  std::string do_to_string() const final;
};

// TODO(DamrongGuoy): Update documentation when mesh is fully supported (i.e.,
// doesn't require equivocation here).
/** Definition of a general (possibly non-convex) triangular surface mesh.
 Meshes can be used for illustration and perception roles, but have limited
 proximity support. See the documentation of QueryObject's proximity queries to
 see how meshes are used in each type of proximity query.

 The mesh is defined in a canonical frame C, implicit in the file parsed. Upon
 loading it in SceneGraph it can be scaled around the origin of C by a given
 `scale` amount. */
class Mesh final : public Shape {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(Mesh)

  /** Constructs a mesh specification from the mesh file located at the given
   file path; if the path is not absolute, it will be interpreted relative to
   the current working directory.
   Optionally uniformly scaled by the given scale factor.
   @throws std::exception if |scale| < 1e-8. Note that a negative scale is
   considered valid. We want to preclude scales near zero but recognise that
   scale is a convenience tool for "tweaking" models. 8 orders of magnitude
   should be plenty without considering revisiting the model itself. */
  explicit Mesh(const std::string& filename, double scale = 1.0);

  ~Mesh() final;

  const std::string& filename() const { return filename_; }
  /** Returns the extension of the mesh filename -- all lower case and including
   the dot. In other words /foo/bar/mesh.obj and /foo/bar/mesh.OBJ would both
   report the ".obj" extension. The "extension" portion of the filename is
   defined as in std::filesystem::path::extension(). */
  const std::string& extension() const { return extension_; }
  double scale() const { return scale_; }

 private:
  void DoReify(ShapeReifier*, void*) const final;
  std::unique_ptr<Shape> DoClone() const final;
  std::string_view do_type_name() const final;
  std::string do_to_string() const final;

  // NOTE: Cannot be const to support default copy/move semantics.
  std::string filename_;
  std::string extension_;
  double scale_{};
};

// TODO(russt): Rename this to `Cone` if/when it is supported by more of the
// geometry engine.
/** Definition of a cone. Its point is at the origin, its height extends in the
 direction of the frame's +z axis. Or, more formally: a finite section of a
 Lorentz cone (aka "second-order cone"), defined by

      sqrt(x²/a² + y²/b²) ≤ z;  z ∈ [0, height],

 where `a` and `b` are the lengths of the principal semi-axes of the horizontal
 section at `z=height()`.

 This shape is currently only supported by Meshcat. It will not appear in any
 renderings, proximity queries, or other visualizers.
*/
class MeshcatCone final : public Shape {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(MeshcatCone)

  /** Constructs the parameterized cone.
   @throws std::exception if `height`, `a`, or `b` are not strictly positive.
   */
  explicit MeshcatCone(double height, double a = 1.0, double b = 1.0);

  /** Constructs a cone with a vector of measures: height and principal
   semi-axes.
   @throws std::exception if the measures are not strictly positive. */
  explicit MeshcatCone(const Vector3<double>& measures);

  ~MeshcatCone() final;

  double height() const { return height_; }
  double a() const { return a_; }
  double b() const { return b_; }

 private:
  void DoReify(ShapeReifier*, void*) const final;
  std::unique_ptr<Shape> DoClone() const final;
  std::string_view do_type_name() const final;
  std::string do_to_string() const final;

  double height_{};
  double a_{};
  double b_{};
};

/** Definition of sphere. It is centered in its canonical frame with the
 given radius. */
class Sphere final : public Shape {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(Sphere)

  /** Constructs a sphere with the given `radius`.
   @throws std::exception if `radius` is negative. Note that a zero radius is
   considered valid. */
  explicit Sphere(double radius);

  ~Sphere() final;

  double radius() const { return radius_; }

 private:
  void DoReify(ShapeReifier*, void*) const final;
  std::unique_ptr<Shape> DoClone() const final;
  std::string_view do_type_name() const final;
  std::string do_to_string() const final;

  double radius_{};
};

/** The interface for converting shape descriptions to real shapes. Any entity
 that consumes shape descriptions _must_ implement this interface.

 This class explicitly enumerates all concrete shapes in its methods. The
 addition of a new concrete shape class requires the addition of a new
 corresponding method. There should *never* be an ImplementGeometry method that
 accepts the Shape base class as an argument; it should _only_ operate on
 concrete derived classes.

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
     DRAKE_ASSERT(data != nullptr);
     ImportantData& data = *static_cast<ImportantData*>(data);
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
  virtual ~ShapeReifier();

  virtual void ImplementGeometry(const Box& box, void* user_data);
  virtual void ImplementGeometry(const Capsule& capsule, void* user_data);
  virtual void ImplementGeometry(const Convex& convex, void* user_data);
  virtual void ImplementGeometry(const Cylinder& cylinder, void* user_data);
  virtual void ImplementGeometry(const Ellipsoid& ellipsoid, void* user_data);
  virtual void ImplementGeometry(const HalfSpace& half_space, void* user_data);
  virtual void ImplementGeometry(const Mesh& mesh, void* user_data);
  virtual void ImplementGeometry(const MeshcatCone& cone, void* user_data);
  virtual void ImplementGeometry(const Sphere& sphere, void* user_data);

 protected:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(ShapeReifier)
  ShapeReifier() = default;

  /** The default implementation of ImplementGeometry(): it throws an exception
   using ThrowUnsupportedGeometry(). The purpose of this function is to
   facilitate reifiers that would call the same API on all shapes (e.g., call an
   API with a Shape& parameter). This reduces the implementation boilerplate. */
  virtual void DefaultImplementGeometry(const Shape& shape);

  /** Derived ShapeReifiers can replace the default message for unsupported
   geometries by overriding this method. The name of the unsupported shape type
   is given as the single parameter.  */
  virtual void ThrowUnsupportedGeometry(const std::string& shape_name);
};

class DRAKE_DEPRECATED("2024-06-01",
                       "Use the Shape::type_name() member function instead")
    ShapeName final : public ShapeReifier {
 public:
  ShapeName() = default;

  /** Constructs a %ShapeName from the given `shape` such that `string()`
   already contains the string representation of `shape`.  */
  explicit ShapeName(const Shape& shape);

  ~ShapeName() final;

  /** Returns the name of the last shape reified. Empty if no shape has been
   reified yet.  */
  std::string name() const { return string_; }

 private:
  void DefaultImplementGeometry(const Shape& shape) final;

  std::string string_;
};

#ifndef DRAKE_DOXYGEN_CXX
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
std::ostream& operator<<(std::ostream& out, const ShapeName& name);
#pragma GCC diagnostic pop
#endif

/** Calculates the volume (in meters^3) for the Shape. For convex and mesh
 geometries, the algorithm only supports ".obj" files and only produces
 meaningful results for "closed" shapes.

 @throws std::exception if the derived type hasn't overloaded this
  implementation (yet), if a filetype is unsupported, or if a referenced file
  cannot be opened.
*/
double CalcVolume(const Shape& shape);

}  // namespace geometry
}  // namespace drake

#ifndef DRAKE_DOXYGEN_CXX
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
namespace fmt {
template <>
struct formatter<drake::geometry::ShapeName> : drake::ostream_formatter {};
}  // namespace fmt
#pragma GCC diagnostic pop
#endif

DRAKE_FORMATTER_AS(, drake::geometry, Box, x, x.to_string())
DRAKE_FORMATTER_AS(, drake::geometry, Capsule, x, x.to_string())
DRAKE_FORMATTER_AS(, drake::geometry, Convex, x, x.to_string())
DRAKE_FORMATTER_AS(, drake::geometry, Cylinder, x, x.to_string())
DRAKE_FORMATTER_AS(, drake::geometry, Ellipsoid, x, x.to_string())
DRAKE_FORMATTER_AS(, drake::geometry, HalfSpace, x, x.to_string())
DRAKE_FORMATTER_AS(, drake::geometry, Mesh, x, x.to_string())
DRAKE_FORMATTER_AS(, drake::geometry, MeshcatCone, x, x.to_string())
DRAKE_FORMATTER_AS(, drake::geometry, Shape, x, x.to_string())
DRAKE_FORMATTER_AS(, drake::geometry, Sphere, x, x.to_string())
