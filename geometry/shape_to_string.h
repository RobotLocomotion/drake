#pragma once

#include <string>

#include "drake/geometry/shape_specification.h"

namespace drake {
namespace geometry {

/** Class that turns a Shape into a std::string representation. This reifier has
 a string() member that gets updated for each shape reified. The expected
 workflow would be:

 ```c++
 ShapeToString reifier;
 SceneGraphInspector inspector = ...;  // Get the inspector from somewhere.
 for (GeometryId id : inspector.GetAllGeometryIds()) {
   inspector.Reify(id, reifier);
   std::cout << reifier.string() << "\n";
 }
 ```

 This will write out a string representation of every geometry registered to
 SceneGraph.  */
class ShapeToString final : public ShapeReifier {
 public:
  /** @name  Implementation of ShapeReifier interface  */
  //@{
  using ShapeReifier::ImplementGeometry;
  void ImplementGeometry(const Sphere& sphere, void* user_data) final;
  void ImplementGeometry(const Cylinder& cylinder, void* user_data) final;
  void ImplementGeometry(const HalfSpace& half_space, void* user_data) final;
  void ImplementGeometry(const Box& box, void* user_data) final;
  void ImplementGeometry(const Capsule& capsule, void* user_data) final;
  void ImplementGeometry(const Ellipsoid& ellipsoid, void* user_data) final;
  void ImplementGeometry(const Mesh& mesh, void* user_data) final;
  void ImplementGeometry(const Convex& convex, void* user_data) final;

  //@}
  const std::string& string() const { return string_; }

 private:
  std::string string_;
};

}  // namespace geometry
}  // namespace drake
