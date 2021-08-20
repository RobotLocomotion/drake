#pragma once

#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <msgpack.hpp>

#include "drake/geometry/rgba.h"
#include "drake/math/rigid_transform.h"

namespace drake {
namespace geometry {
namespace internal {

// Note: These types lack many of the standard const qualifiers in order to be
// compatible with msgpack, which wants to be able to unpack into the same
// structure.

// TODO(russt): Can we teach msgpack for std::optional to only add the key if
// has_value() == true?  This would only be for efficiency, but it would allow
// us to support more options without any cost.

// TODO(russt): These are taken verbatim from meshcat-python.  We should
// expose them to the user, but not until we can properly document them.
// Many are documented here: https://threejs.org/docs/#api/en/materials/Material
struct MaterialData {
  std::string uuid{};
  std::string type{};
  int color{(229 << 16) + (229 << 8) + 229};
  // TODO(russt): Make many of these std::optional.
  double reflectivity{0.5};
  int side{2};
  bool transparent{false};
  double opacity{1.0};
  double linewidth{1.0};
  bool wireframe{false};
  double wireframeLineWidth{1.0};
  bool vertexColors{false};
  MSGPACK_DEFINE_MAP(uuid, type, color, reflectivity, side, transparent,
                     opacity, linewidth, wireframe, wireframeLineWidth,
                     vertexColors);
};

// Note: This contains the fields required for all geometry types.  Getting
// msgpack to work with runtime derived types proved to be very complicated.
struct GeometryData {
  std::string uuid;
  std::string type;
  std::optional<double> width;
  std::optional<double> height;
  std::optional<double> depth;
  std::optional<double> radius;
  std::optional<double> widthSegments;
  std::optional<double> heightSegments;
  std::optional<double> radiusTop;
  std::optional<double> radiusBottom;
  std::optional<double> radialSegments;
  std::optional<std::string> format;
  std::vector<char> data;
  MSGPACK_DEFINE_MAP(uuid, type, width, height, depth, radius, widthSegments,
                     heightSegments, radiusTop, radiusBottom, radialSegments,
                     format, data);
};

struct ObjectMetaData {
  std::string type{"Object"};
  double version{4.5};
  MSGPACK_DEFINE_MAP(type, version);
};

struct Object3dData {
  std::string uuid;
  std::string type;
  std::string geometry;
  std::string material;
  double matrix[16] = {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};
  MSGPACK_DEFINE_MAP(uuid, type, geometry, material, matrix);
};

struct LumpedObjectData {
  ObjectMetaData metadata{};
  GeometryData geometries[1];
  MaterialData materials[1];
  Object3dData object;
  MSGPACK_DEFINE_MAP(metadata, geometries, materials, object);
};

struct SetObjectData {
  std::string type{"set_object"};
  std::string path;
  LumpedObjectData object;
  MSGPACK_DEFINE_MAP(type, path, object);
};

struct SetTransformData {
  std::string type{"set_transform"};
  std::string path;
  double matrix[16];
  MSGPACK_DEFINE_MAP(type, path, matrix);
};

struct DeleteData {
  std::string type{"delete"};
  std::string path;
  MSGPACK_DEFINE_MAP(type, path);
};

template <typename T>
struct SetPropertyData {
  std::string type{"set_property"};
  std::string path;
  std::string property;
  T value;
  MSGPACK_DEFINE_MAP(type, path, property, value);
};

}  // namespace internal
}  // namespace geometry
}  // namespace drake
