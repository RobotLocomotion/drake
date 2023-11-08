#pragma once

/* This file declares the functions that bind drake/systems/sensors. These
functions form a complete partition of the that directory's bindings.

The implementations of these functions are parceled out into various *.cc
files as indicated in each function's documentation. */

#include "drake/bindings/pydrake/common/type_pack.h"
#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/systems/sensors/pixel_types.h"

namespace drake {
namespace pydrake {
namespace internal {

/* Defines bindings per sensors_py_camera_config.cc. */
void DefineSensorsCameraConfig(py::module m);

/* Defines bindings per sensors_py_image.cc. */
void DefineSensorsImage(py::module m);

/* Defines bindings per sensors_py_image_io.cc. */
void DefineSensorsImageIo(py::module m);

/* Defines bindings per sensors_py_lcm.cc. */
void DefineSensorsLcm(py::module m);

/* Defines bindings per sensors_py_rgbd.cc. */
void DefineSensorsRgbd(py::module m);

/* PixelTypeList provides all of the enumerated PixelType values. */
template <typename T, T kPixelType>
using PixelTypeConstant = std::integral_constant<T, kPixelType>;
template <typename T, T... kPixelTypes>
using PixelTypeConstantPack =
    type_pack<type_pack<PixelTypeConstant<T, kPixelTypes>>...>;
using PixelTypeList = PixelTypeConstantPack<systems::sensors::PixelType,
    systems::sensors::PixelType::kRgba8U, systems::sensors::PixelType::kRgb8U,
    systems::sensors::PixelType::kBgra8U, systems::sensors::PixelType::kBgr8U,
    systems::sensors::PixelType::kDepth16U,
    systems::sensors::PixelType::kDepth32F,
    systems::sensors::PixelType::kLabel16I,
    systems::sensors::PixelType::kGrey8U>;

}  // namespace internal
}  // namespace pydrake
}  // namespace drake
