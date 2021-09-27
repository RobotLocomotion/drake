#pragma once

#include <limits>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <variant>
#include <vector>

#include <msgpack.hpp>

#include "drake/common/nice_type_name.h"
#include "drake/geometry/meshcat.h"
#include "drake/geometry/rgba.h"
#include "drake/math/rigid_transform.h"

namespace drake {
namespace geometry {
namespace internal {

// The fields in these structures are chosen to match the serialized names in
// the meshcat messages. This simplifies msgpack operations and provides parity
// with the javascript.  The structure names do not have this requirement.
// When they represent a specific message with a single `type`, we take the
// type name (with a `Data` suffix) as the struct name, e.g.
// `MeshFileObjectData` is taken because the message `type` is
// `_meshfile_object`.

// Note: These types lack many of the standard const qualifiers in order to be
// compatible with msgpack, which wants to be able to unpack into the same
// structure.

// TODO(russt): We should expose these options to the user, but not until we
// can properly document them. Many are documented here:
// https://threejs.org/docs/#api/en/materials/Material
struct MaterialData {
  std::string uuid{};
  std::string type{};
  int color{(229 << 16) + (229 << 8) + 229};
  int vertexColors{0};
  std::optional<double> reflectivity;
  std::optional<int> side;
  std::optional<double> size;
  std::optional<bool> transparent;
  std::optional<double> opacity;
  std::optional<double> linewidth;
  std::optional<bool> wireframe;
  std::optional<double> wireframeLineWidth;

  template <typename Packer>
  // NOLINTNEXTLINE(runtime/references) cpplint disapproves of msgpack choices.
  void msgpack_pack(Packer& o) const {
    int n = 4;
    if (reflectivity) ++n;
    if (side) ++n;
    if (size) ++n;
    if (transparent) ++n;
    if (opacity) ++n;
    if (linewidth) ++n;
    if (wireframe) ++n;
    if (wireframeLineWidth) ++n;
    o.pack_map(n);
    o.pack("uuid");
    o.pack(uuid);
    o.pack("type");
    o.pack(type);
    o.pack("color");
    o.pack(color);
    o.pack("vertexColors");
    o.pack(vertexColors);
    if (reflectivity) {
      o.pack("reflectivity");
      o.pack(*reflectivity);
    }
    if (side) {
      o.pack("side");
      o.pack(*side);
    }
    if (size) {
      o.pack("size");
      o.pack(*size);
    }
    if (transparent) {
      o.pack("transparent");
      o.pack(*transparent);
    }
    if (opacity) {
      o.pack("opacity");
      o.pack(*opacity);
    }
    if (linewidth) {
      o.pack("linewidth");
      o.pack(*linewidth);
    }
    if (wireframe) {
      o.pack("wireframe");
      o.pack(*wireframe);
    }
    if (wireframeLineWidth) {
      o.pack("wireframeLineWidth");
      o.pack(*wireframeLineWidth);
    }
  }

  // This method must be defined, but the implementation is not needed in the
  // current workflows.
  void msgpack_unpack(msgpack::object const&) {
    throw std::runtime_error(
        "unpack is not implemented for MaterialData.");
  }
};

struct GeometryData {
  virtual ~GeometryData() = default;
  std::string uuid;
};

struct SphereGeometryData : public GeometryData {
  std::string type{"SphereGeometry"};
  double radius;
  double widthSegments{20};
  double heightSegments{20};
  MSGPACK_DEFINE_MAP(uuid, type, radius, widthSegments, heightSegments);
};

struct CylinderGeometryData : public GeometryData {
  std::string type{"CylinderGeometry"};
  double radiusBottom;
  double radiusTop;
  double height;
  double radialSegments{50};
  MSGPACK_DEFINE_MAP(uuid, type, radiusBottom, radiusTop, height,
                     radialSegments);
};

struct BoxGeometryData : public GeometryData {
  std::string type{"BoxGeometry"};
  double width;
  double height;
  double depth;
  MSGPACK_DEFINE_MAP(uuid, type, width, height, depth);
};

struct MeshFileGeometryData : public GeometryData {
  std::string type{"_meshfile_geometry"};
  std::string format;
  std::string data;
  MSGPACK_DEFINE_MAP(uuid, type, format, data);
};

struct BufferGeometryData : public GeometryData {
  std::string type{"BufferGeometry"};
  // We deviate from the meshcat data structure, since it is an unnecessarily
  // deep hierarchy of dictionaries, and simply implement the packer manually.
  Eigen::Matrix3Xf position;
  Eigen::Matrix3Xf color;

  template <typename Packer>
  // NOLINTNEXTLINE(runtime/references) cpplint disapproves of msgpack choices.
  void msgpack_pack(Packer& o) const {
    o.pack_map(3);
    o.pack("uuid");
    o.pack(uuid);
    o.pack("type");
    o.pack(type);
    o.pack("data");
    o.pack_map(1);
    o.pack("attributes");
    if (color.cols() > 0) {
      o.pack_map(2);
      o.pack("color");
      o.pack(color);
    } else {
      o.pack_map(1);
    }
    o.pack("position");
    o.pack(position);
  }

  // This method must be defined, but the implementation is not needed in the
  // current workflows.
  void msgpack_unpack(msgpack::object const&) {
    throw std::runtime_error(
        "unpack is not implemented for BufferGeometryData.");
  }
};

struct ObjectData {
  std::string type{"Object"};
  double version{4.5};
  MSGPACK_DEFINE_MAP(type, version);
};

struct MeshData {
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
  ObjectData metadata{};
  // We deviate from the msgpack names (geometries, materials) here since we
  // currently only support zero or one geometry/material.
  std::unique_ptr<GeometryData> geometry{};
  std::unique_ptr<MaterialData> material{};
  std::variant<std::monostate, MeshData, MeshFileObjectData> object;

  template <typename Packer>
  // NOLINTNEXTLINE(runtime/references) cpplint disapproves of msgpack choices.
  void msgpack_pack(Packer& o) const {
    int size = 2;
    if (geometry) ++size;
    if (material) ++size;
    o.pack_map(size);
    o.pack("metadata");
    o.pack(metadata);
    if (geometry) {
      o.pack("geometries");
      o.pack_array(1);
      if (auto* sphere = dynamic_cast<SphereGeometryData*>(geometry.get())) {
        o.pack(*sphere);
      } else if (auto* cylinder =
                     dynamic_cast<CylinderGeometryData*>(geometry.get())) {
        o.pack(*cylinder);
      } else if (auto* box =
                     dynamic_cast<BoxGeometryData*>(geometry.get())) {
        o.pack(*box);
      } else if (auto* meshfile =
                     dynamic_cast<MeshFileGeometryData*>(geometry.get())) {
        o.pack(*meshfile);
      } else if (auto* buffer =
                     dynamic_cast<BufferGeometryData*>(geometry.get())) {
        o.pack(*buffer);
      } else {
        throw std::runtime_error(
            "Unknown geometry data type in msgpack_pack for LumpedObjectData.");
      }
    }
    if (material) {
      o.pack("materials");
      o.pack_array(1);
      o.pack(*material);
    }
    o.pack("object");
    if (std::holds_alternative<MeshData>(object)) {
      o.pack(std::get<MeshData>(object));
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

template <typename Scalar, int RowsAtCompileTime, int ColsAtCompileTime,
          int Options, int MaxRowsAtCompileTime, int MaxColsAtCompileTime>
struct pack<Eigen::Matrix<Scalar, RowsAtCompileTime, ColsAtCompileTime, Options,
                          MaxRowsAtCompileTime, MaxColsAtCompileTime> > {
  template <typename Stream>
  packer<Stream>& operator()(
      // NOLINTNEXTLINE(runtime/references) cpplint disapproves of msgpack.
      msgpack::packer<Stream>& o,
      const Eigen::Matrix<Scalar, RowsAtCompileTime, ColsAtCompileTime, Options,
                          MaxRowsAtCompileTime, MaxColsAtCompileTime>& mat)
      const {
    o.pack_map(4);
    o.pack("itemSize");
    o.pack(mat.rows());
    o.pack("type");
    int8_t ext;
    // Based on pack_numpy_array method in meshcat-python geometry.py. See also
    // https://github.com/msgpack/msgpack/blob/master/spec.md#extension-types
    if (std::is_same_v<Scalar, double> || std::is_same_v<Scalar, float>) {
      o.pack("Float32Array");
      ext = 0x17;
    } else if (std::is_same_v<Scalar, uint8_t>) {
      o.pack("Uint8Array");
      ext = 0x12;
    } else {
      throw std::runtime_error("Unsupported Scalar " +
                               drake::NiceTypeName::Get(typeid(Scalar)));
    }
    o.pack("array");
    if (std::is_same_v<Scalar, double>) {
      size_t s = mat.size() * sizeof(float);
      o.pack_ext(s, ext);
      auto mat_float = mat.template cast<float>().eval();
      o.pack_ext_body(reinterpret_cast<const char*>(mat_float.data()), s);
    } else {
      size_t s = mat.size() * sizeof(Scalar);
      o.pack_ext(s, ext);
      o.pack_ext_body(reinterpret_cast<const char*>(mat.data()), s);
    }
    o.pack("normalized");
    o.pack(false);
    return o;
  }
};

template <>
struct pack<drake::geometry::Meshcat::OrthographicCamera> {
  template <typename Stream>
  packer<Stream>& operator()(
      // NOLINTNEXTLINE(runtime/references) cpplint disapproves of msgpack.
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
