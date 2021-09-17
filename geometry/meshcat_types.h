#pragma once

#include <limits>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <variant>
#include <vector>

#include <common_robotics_utilities/base64_helpers.hpp>
#include <msgpack.hpp>

#include "drake/geometry/meshcat.h"
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

  // See the GeometryData implementation for discussion.
  template <typename Packer>
  // NOLINTNEXTLINE(runtime/references) cpplint disapproves of msgpack choices.
  void msgpack_pack(Packer& o) const {
    int size = 11;  // All non-optional members are always sent.
    o.pack_map(size);
    o.pack("uuid");
    o.pack(uuid);
    o.pack("type");
    o.pack(type);
    o.pack("color");
    o.pack(color);
    o.pack("reflectivity");
    o.pack(reflectivity);
    o.pack("side");
    o.pack(side);
    o.pack("transparent");
    o.pack(transparent);
    o.pack("opacity");
    o.pack(opacity);
    o.pack("linewidth");
    o.pack(linewidth);
    o.pack("wireframe");
    o.pack(wireframe);
    o.pack("wireframeLineWidth");
    o.pack(wireframeLineWidth);
    o.pack("vertexColors");
    o.pack(vertexColors);
  }
  // This method must be defined, but the implementation is not needed in the
  // current workflows.
  void msgpack_unpack(msgpack::object const&) {
    throw std::runtime_error("unpack is not implemented for ImageTextureData.");
  }
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
  std::string format;
  std::string data;

  // MSGPACK_DEFINE_MAP sends e.g. 'heightSegments':nil when the optional values
  // are not set.  This defeats the defaults in three.js.  We have to implement
  // a custom packer in order to avoid it.
  // TODO(russt): Could make this fancier with e.g. the template parameter packs
  // in msgpack-c/include/msgpack/v1/adaptor/detail/cpp11_define_map.hpp
  template <typename Packer>
  // NOLINTNEXTLINE(runtime/references) cpplint disapproves of msgpack choices.
  void msgpack_pack(Packer& o) const {
    int size = 2;  // uuid and type are always sent.
    if (width) ++size;
    if (height) ++size;
    if (depth) ++size;
    if (radius) ++size;
    if (widthSegments) ++size;
    if (heightSegments) ++size;
    if (radiusTop) ++size;
    if (radiusBottom) ++size;
    if (radialSegments) ++size;
    if (!format.empty()) ++size;
    if (!data.empty()) ++size;
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
    if (!format.empty()) {
      o.pack("format");
      o.pack(format);
    }
    if (!data.empty()) {
      o.pack("data");
      o.pack(data);
    }
  }
  // This method must be defined, but the implementation is not needed in the
  // current workflows.
  void msgpack_unpack(msgpack::object const&) {
    throw std::runtime_error("unpack is not implemented for GeometryData.");
  }
};

struct ObjectMetaData {
  std::string type{"Object"};
  double version{4.5};
  MSGPACK_DEFINE_MAP(type, version);
};

struct MeshFileGeometryData {
  std::string uuid;
  std::string type{"Mesh"};
  std::string geometry;
  std::string material;
  double matrix[16] = {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};
  MSGPACK_DEFINE_MAP(uuid, type, geometry, material, matrix);
};

struct MeshFileObjectData {
  std::string uuid;
  std::string type{"_meshfile_object"};
  std::string format;
  std::string data;
  std::string mtl_library;
  std::map<std::string, std::string> resources;
  double matrix[16] = {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};
  MSGPACK_DEFINE_MAP(uuid, type, format, data, mtl_library, resources, matrix);
};

struct LumpedObjectData {
  ObjectMetaData metadata{};
  std::vector<GeometryData> geometries;
  std::vector<MaterialData> materials;
  std::variant<MeshFileGeometryData, MeshFileObjectData> object;

  template <typename Packer>
  // NOLINTNEXTLINE(runtime/references) cpplint disapproves of msgpack choices.
  void msgpack_pack(Packer& o) const {
    o.pack_map(4);
    o.pack("metadata");
    o.pack(metadata);
    o.pack("geometries");
    o.pack(geometries);
    o.pack("materials");
    o.pack(materials);
    o.pack("object");
    if (std::holds_alternative<MeshFileGeometryData>(object)) {
      o.pack(std::get<MeshFileGeometryData>(object));
    } else {
      o.pack(std::get<MeshFileObjectData>(object));
    }
  }
  // This method must be defined, but the implementation is not needed in the
  // current workflows.
  void msgpack_unpack(msgpack::object const&) {
    throw std::runtime_error("unpack is not implemented for LumpedObjectData.");
  }
};

struct SetObjectData {
  std::string type{"set_object"};
  std::string path;
  LumpedObjectData object;
  MSGPACK_DEFINE_MAP(type, path, object);
};

template <typename CameraData>
struct LumpedCameraData {
  CameraData object;
  MSGPACK_DEFINE_MAP(object);
};

template <typename CameraData>
struct SetCameraData {
  std::string type{"set_object"};
  std::string path;
  LumpedCameraData<CameraData> object;
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

struct SetButtonControl {
  std::string type{"set_control"};
  int num_clicks{0};
  std::string name;
  std::string callback;
  MSGPACK_DEFINE_MAP(type, name, callback);
};

struct SetSliderControl {
  std::string type{"set_control"};
  std::string name;
  std::string callback;
  double value;
  double min;
  double max;
  double step;
  MSGPACK_DEFINE_MAP(type, name, callback, value, min, max, step);
};

struct SetSliderValue {
  std::string type{"set_control_value"};
  std::string name;
  double value;
  // Note: If invoke_callback is true, then we will receive a message back from
  // the slider whose value it being set.  This can lead to mysterious loopy
  // behavior when multiple browsers are connected.
  // TODO(russt): Make it impossible to set this to true.
  bool invoke_callback{false};
  MSGPACK_DEFINE_MAP(type, name, value, invoke_callback);
};

struct DeleteControl {
  std::string type{"delete_control"};
  std::string name;
  MSGPACK_DEFINE_MAP(type, name);
};

struct UserInterfaceEvent {
  std::string type;
  std::string name;
  std::optional<double> value;
  MSGPACK_DEFINE_MAP(type, name, value);
};

}  // namespace internal
}  // namespace geometry
}  // namespace drake


// We use the msgpack "non-intrusive" approach for packing types exposed in the
// public interface. https://github.com/msgpack/msgpack-c/wiki/v2_0_cpp_adaptor
namespace msgpack {
MSGPACK_API_VERSION_NAMESPACE(MSGPACK_DEFAULT_API_NS) {
namespace adaptor {

template<>
struct pack<drake::geometry::Meshcat::OrthographicCamera> {
  template <typename Stream>
  packer<Stream>& operator()(
  // NOLINTNEXTLINE(runtime/references) cpplint disapproves of msgpack choices.
      msgpack::packer<Stream>& o,
      const drake::geometry::Meshcat::OrthographicCamera& v) const {
    o.pack_map(8);
    o.pack("type");
    o.pack("OrthographicCamera");
    o.pack("left");
    o.pack(v.left);
    o.pack("right");
    o.pack(v.right);
    o.pack("top");
    o.pack(v.top);
    o.pack("bottom");
    o.pack(v.bottom);
    o.pack("near");
    o.pack(v.near);
    o.pack("far");
    o.pack(v.far);
    o.pack("zoom");
    o.pack(v.zoom);
    return o;
  }
};

template<>
struct pack<drake::geometry::Meshcat::PerspectiveCamera> {
  template <typename Stream>
  packer<Stream>& operator()(
  // NOLINTNEXTLINE(runtime/references) cpplint disapproves of msgpack choices.
      msgpack::packer<Stream>& o,
      const drake::geometry::Meshcat::PerspectiveCamera& v) const {
    o.pack_map(5);
    o.pack("type");
    o.pack("PerspectiveCamera");
    o.pack("fov");
    o.pack(v.fov);
    o.pack("aspect");
    o.pack(v.aspect);
    o.pack("near");
    o.pack(v.near);
    o.pack("far");
    o.pack(v.far);
    o.pack("zoom");
    o.pack(v.zoom);
    return o;
  }
};

}  // namespace adaptor
}  // namespace MSGPACK_API_VERSION_NAMESPACE(MSGPACK_DEFAULT_API_NS)
}  // namespace msgpack
