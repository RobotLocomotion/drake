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
#include "drake/geometry/meshcat_animation.h"
#include "drake/geometry/rgba.h"
#include "drake/math/rigid_transform.h"

namespace drake {
namespace geometry {
namespace internal {

// This macro not only makes the code shorter, but it also helps avoid spelling
// mistakes by ensuring that the string name matches the variable name.
#define PACK_MAP_VAR(packer, var) \
  packer.pack(#var);           \
  packer.pack(var);

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

// From https://github.com/mrdoob/three.js/blob/dev/src/constants.js
enum ThreeSide { kFrontSide = 0, kBackSide = 1, kDoubleSide = 2 };

// TODO(russt): We should expose these options to the user.
// The documentation of the fields is adapted from
// https://threejs.org/docs/#api/en/materials/Material and its derived classes.
struct MaterialData {
  // UUID of this material instance.
  std::string uuid;

  // The three.js material type (e.g. PointsMaterial, LineBasicMaterial,
  // MeshBasicMaterial, MeshPhongMaterial...)
  std::string type;

  // Default color is a light gray.
  int color{(229 << 16) + (229 << 8) + 229};

  // For LineBasicMaterial, controls line thickness. The three.js default is 1.
  // Due to limitations of the OpenGL Core Profile with the WebGL renderer on
  // most platforms linewidth will always be 1 regardless of the set value.
  std::optional<double> linewidth;

  // Double in the range of 0.0 - 1.0 indicating how transparent the material
  // is. A value of 0.0 indicates fully transparent, 1.0 is fully opaque. If the
  // material's transparent property is not set to true, the material will
  // remain fully opaque and this value will only affect its color. The three.js
  // default is 1.0.
  std::optional<double> opacity;

  // For MeshBasicMaterial, specifies how much the environment map affects the
  // surface. The three.js default value is 1 and the valid range is between 0
  // (no reflections) and 1 (full reflections).
  std::optional<double> reflectivity;

  // Defines which side of faces will be rendered - front, back or both. Default
  // is kFrontSide. Other options are kBackSide and kDoubleSide.
  std::optional<ThreeSide> side;

  // For PointsMaterial, sets the size of the points. The three.js default
  // is 1.0. Will be capped if it exceeds the hardware dependent parameter
  // gl.ALIASED_POINT_SIZE_RANGE.
  std::optional<double> size;

  // Defines whether this material is transparent. This has an effect on
  // rendering as transparent objects need special treatment and are rendered
  // after non-transparent objects.  When set to true, the extent to which the
  // material is transparent is controlled by setting its opacity
  // property.  The three.js default is false.
  std::optional<bool> transparent;

  // Defines whether vertex coloring is used.  @default false.
  bool vertexColors{false};

  // For MeshBasicMaterial, render geometry as wireframe. The three.js default
  // is false (i.e. render as flat polygons).
  std::optional<bool> wireframe;

  // For MeshBasicMaterial, controls wireframe thickness. The three.js default
  // is 1.
  std::optional<double> wireframeLineWidth;

  template <typename Packer>
  // NOLINTNEXTLINE(runtime/references) cpplint disapproves of msgpack choices.
  void msgpack_pack(Packer& o) const {
    int n = 4;
    if (linewidth) ++n;
    if (opacity) ++n;
    if (reflectivity) ++n;
    if (side) ++n;
    if (size) ++n;
    if (transparent) ++n;
    if (wireframe) ++n;
    if (wireframeLineWidth) ++n;
    o.pack_map(n);
    PACK_MAP_VAR(o, uuid);
    PACK_MAP_VAR(o, type);
    PACK_MAP_VAR(o, color);
    PACK_MAP_VAR(o, vertexColors);
    if (linewidth) {
      o.pack("linewidth");
      o.pack(*linewidth);
    }
    if (opacity) {
      o.pack("opacity");
      o.pack(*opacity);
    }
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

  // NOLINTNEXTLINE(runtime/references) cpplint disapproves of msgpack choices.
  virtual void msgpack_pack(msgpack::packer<std::stringstream>& o) const = 0;

  // This method must be defined, but the implementation is not needed in the
  // current workflows.
  void msgpack_unpack(msgpack::object const&) {
    throw std::runtime_error(
        "unpack is not implemented for BufferGeometryData.");
  }
};

struct SphereGeometryData : public GeometryData {
  double radius{};
  double widthSegments{20};
  double heightSegments{20};

  // NOLINTNEXTLINE(runtime/references) cpplint disapproves of msgpack choices.
  void msgpack_pack(msgpack::packer<std::stringstream>& o) const override {
    o.pack_map(5);
    o.pack("type");
    o.pack("SphereGeometry");
    PACK_MAP_VAR(o, uuid);
    PACK_MAP_VAR(o, radius);
    PACK_MAP_VAR(o, widthSegments);
    PACK_MAP_VAR(o, heightSegments);
  }
};

struct CylinderGeometryData : public GeometryData {
  double radiusBottom{};
  double radiusTop{};
  double height{};
  double radialSegments{50};

  // NOLINTNEXTLINE(runtime/references) cpplint disapproves of msgpack choices.
  void msgpack_pack(msgpack::packer<std::stringstream>& o) const override {
    o.pack_map(6);
    o.pack("type");
    o.pack("CylinderGeometry");
    PACK_MAP_VAR(o, uuid);
    PACK_MAP_VAR(o, radiusBottom);
    PACK_MAP_VAR(o, radiusTop);
    PACK_MAP_VAR(o, height);
    PACK_MAP_VAR(o, radialSegments);
  }
};

struct BoxGeometryData : public GeometryData {
  double width{};
  double height{};
  double depth{};

  // NOLINTNEXTLINE(runtime/references) cpplint disapproves of msgpack choices.
  void msgpack_pack(msgpack::packer<std::stringstream>& o) const override {
    o.pack_map(5);
    o.pack("type");
    o.pack("BoxGeometry");
    PACK_MAP_VAR(o, uuid);
    PACK_MAP_VAR(o, width);
    PACK_MAP_VAR(o, height);
    PACK_MAP_VAR(o, depth);
  }
};

struct MeshFileGeometryData : public GeometryData {
  std::string format;
  std::string data;

  // NOLINTNEXTLINE(runtime/references) cpplint disapproves of msgpack choices.
  void msgpack_pack(msgpack::packer<std::stringstream>& o) const override {
    o.pack_map(4);
    o.pack("type");
    o.pack("_meshfile_geometry");
    PACK_MAP_VAR(o, uuid);
    PACK_MAP_VAR(o, format);
    PACK_MAP_VAR(o, data);
  }
};

struct BufferGeometryData : public GeometryData {
  // We deviate from the meshcat data structure, since it is an unnecessarily
  // deep hierarchy of dictionaries, and simply implement the packer manually.
  Eigen::Matrix3Xf position;
  Eigen::Matrix3Xf color;
  Eigen::Matrix<uint32_t, 3, Eigen::Dynamic> faces;

  // NOLINTNEXTLINE(runtime/references) cpplint disapproves of msgpack choices.
  void msgpack_pack(msgpack::packer<std::stringstream>& o) const override {
    o.pack_map(3);
    o.pack("type");
    o.pack("BufferGeometry");
    PACK_MAP_VAR(o, uuid);
    o.pack("data");
    if (faces.cols() > 0) {
      o.pack_map(2);
      o.pack("index");
      o.pack(faces);
    } else {
      o.pack_map(1);
    }
    o.pack("attributes");
    if (color.cols() > 0) {
      o.pack_map(2);
      PACK_MAP_VAR(o, color);
    } else {
      o.pack_map(1);
    }
    PACK_MAP_VAR(o, position);
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
  std::unique_ptr<GeometryData> geometry;
  std::unique_ptr<MaterialData> material;
  std::variant<std::monostate, MeshData, MeshFileObjectData> object;

  template <typename Packer>
  // NOLINTNEXTLINE(runtime/references) cpplint disapproves of msgpack choices.
  void msgpack_pack(Packer& o) const {
    int size = 2;
    if (geometry) ++size;
    if (material) ++size;
    o.pack_map(size);
    PACK_MAP_VAR(o, metadata);
    if (geometry) {
      o.pack("geometries");
      o.pack_array(1);
      o.pack(*geometry);
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
  double matrix[16]{};
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
  double value{};
  double min{};
  double max{};
  double step{};
  MSGPACK_DEFINE_MAP(type, name, callback, value, min, max, step);
};

struct SetSliderValue {
  std::string type{"set_control_value"};
  std::string name;
  double value{};
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

MSGPACK_ADD_ENUM(drake::geometry::internal::ThreeSide);
MSGPACK_ADD_ENUM(drake::geometry::MeshcatAnimation::LoopMode);

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
    if (std::is_floating_point_v<Scalar>) {
      o.pack("Float32Array");
      ext = 0x17;
    } else if (std::is_same_v<std::remove_cv<Scalar>, uint8_t>) {
      o.pack("Uint8Array");
      ext = 0x12;
    } else if (std::is_same_v<Scalar, uint32_t>) {
      // TODO(russt): Using std::remove_cv<Scalar> did not work here (it failed
      // to match).  Need to understand and resolve this.
      o.pack("Uint32Array");
      ext = 0x16;
    } else {
      throw std::runtime_error("Unsupported Scalar " +
                               drake::NiceTypeName::Get(typeid(Scalar)));
    }
    o.pack("array");
    if (std::is_same_v<std::remove_cv<Scalar>, double>) {
      // Three.js only uses float, and meshcat only parses Float32Array (not
      // doubles).
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
