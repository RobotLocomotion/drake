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
  std::string data;

  // MSGPACK_DEFINE_MAP sends e.g. 'heightSegments':nil when the optional values
  // are not set.  This defeats the defaults in three.js.  We have to implement
  // a custom packer in order to avoid it.
  // TODO(russt): Could make this fancier with e.g. the template parameter packs
  // in msgpack-c/include/msgpack/v1/adaptor/detail/cpp11_define_map.hpp
  template <typename Packer>
  void msgpack_pack(Packer& o) const
  {
    int size=2; // uuid and type are always sent.
    if (width) size++;
    if (height) size++;
    if (depth) size++;
    if (radius) size++;
    if (widthSegments) size++;
    if (heightSegments) size++;
    if (radiusTop) size++;
    if (radiusBottom) size++;
    if (radialSegments) size++;
    if (format) size++;
    if (!data.empty()) size++;
    o.pack_map(size);
    o.pack("uuid");
    o.pack(uuid);
    o.pack("type");
    o.pack(type);
    if (width) {
      o.pack("width");
      o.pack(*width);
    }
    if (height) {
      o.pack("height");
      o.pack(*height);
    }
    if (depth) {
      o.pack("depth");
      o.pack(*depth);
    }
    if (radius) {
      o.pack("radius");
      o.pack(*radius);
    }
    if (widthSegments) {
      o.pack("widthSegments");
      o.pack(*widthSegments);
    }
    if (heightSegments) {
      o.pack("heightSegments");
      o.pack(*heightSegments);
    }
    if (radiusTop) {
      o.pack("radiusTop");
      o.pack(*radiusTop);
    }
    if (radiusBottom) {
      o.pack("radiusBottom");
      o.pack(*radiusBottom);
    }
    if (radialSegments) {
      o.pack("radialSegments");
      o.pack(*radialSegments);
    }
    if (format) {
      o.pack("format");
      o.pack(*format);
    }
    if (!data.empty()) {
      o.pack("data");
      o.pack(data);
    }
  }
  void msgpack_unpack(msgpack::object const&)
  {
    // Intentionally left blank.
    // This method must be defined, but the implementation is not needed in the
    // current workflows.
  }

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
