#pragma once

#include <array>

#include <Eigen/Dense>
#include <vtkNew.h>
#include <vtkPlaneSource.h>
#include <vtkSmartPointer.h>
#include <vtkTransform.h>

#include "drake/common/drake_copyable.h"
#include "drake/math/rigid_transform.h"

namespace drake {
namespace geometry {
namespace render {
namespace vtk_util {
/// An array type for vtkSmartPointer.
///
/// @tparam T The VTK class type stored in vtkSmartPointer.
/// @tparam N The size of array.
template <typename T, size_t N>
using vtkPointerArray = std::array<vtkSmartPointer<T>, N>;

/// Creates a square plane of the given size.  The plane's z axis is its
/// normal and is coincident with the z axis of the world coordinate system.
/// The plane's center is at (0, 0, 0) of both the world coordinate system and
/// plane's coordinate system.  The ranges of the plane's x and y are
/// `(- size / 2, size / 2)`.
///
/// @param size The size of the plane.
vtkSmartPointer<vtkPlaneSource> CreateSquarePlane(double size);

/// Converts the provided `transform` to a vtkTransform.
vtkSmartPointer<vtkTransform> ConvertToVtkTransform(
    const math::RigidTransformd& transform);

/// Makes vtkPointerArray from one or multiple pointer(s) for VTK objects
/// wrapped by vtkNew.
///
/// @tparam transform The transform to convert into a `vtkTransform`.
template <typename T, typename... Ts, size_t N = 1 + sizeof...(Ts)>
const vtkPointerArray<T, N> MakeVtkPointerArray(
    const vtkNew<T>& element, const vtkNew<Ts>&... elements) {
  return vtkPointerArray<T, N>{{
      vtkSmartPointer<T>(element.GetPointer()),
      vtkSmartPointer<Ts>(elements.GetPointer())...}};
}

}  // namespace vtk_util
}  // namespace render
}  // namespace geometry
}  // namespace drake
