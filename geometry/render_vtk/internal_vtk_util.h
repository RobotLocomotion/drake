#pragma once

#include <array>

#include <Eigen/Dense>

// To ease build system upkeep, we annotate VTK includes with their deps.
#include <vtkNew.h>           // vtkCommonCore
#include <vtkPlaneSource.h>   // vtkFiltersSources
#include <vtkSmartPointer.h>  // vtkCommonCore
#include <vtkTransform.h>     // vtkCommonTransforms

#include "drake/common/drake_copyable.h"
#include "drake/math/rigid_transform.h"

namespace drake {
namespace geometry {
namespace render_vtk {
namespace internal {
// An array type for vtkSmartPointer.
//
// @tparam T The VTK class type stored in vtkSmartPointer.
// @tparam N The size of array.
template <typename T, size_t N>
using vtkPointerArray = std::array<vtkSmartPointer<T>, N>;

// Creates a square plane of the given size.  The plane's z axis is its normal
// and is coincident with the z axis of the world coordinate system.
// The plane's center is at (0, 0, 0) of both the world coordinate system and
// plane's coordinate system.  The ranges of the plane's x and y are
// `(- size / 2, size / 2)`.
//
// @param size The size of the plane.
vtkSmartPointer<vtkPlaneSource> CreateSquarePlane(double size);

// Converts the provided `transform` to a vtkTransform. The rigid transform is
// the concatenation of a translation operation T and rotation operation R
// (X = T * R).
//
// We turn it into a general transform by also concatenating a scale matrix:
// T * S * R, where where S is I * `scale` (i.e., a diagonal
// matrix with `scale` on the diagonal). This is _not_ the standard transform
// matrix for graphics (TRS). This particular ordering is intended to
// accommodate the transfrom from glTF to Drake where *first* we rotate the
// mesh from y-up to z-up, then scale in a z-up frame, and, finally, translate
// (as necessary). The scale follows rotation.
vtkSmartPointer<vtkTransform> ConvertToVtkTransform(
    const math::RigidTransformd& transform,
    const Vector3<double>& scale = Vector3<double>::Ones());

// Makes vtkPointerArray from one or multiple pointer(s) for VTK objects
// wrapped by vtkNew.
//
// @tparam transform The transform to convert into a `vtkTransform`.
template <typename T, typename... Ts, size_t N = 1 + sizeof...(Ts)>
const vtkPointerArray<T, N> MakeVtkPointerArray(const vtkNew<T>& element,
                                                const vtkNew<Ts>&... elements) {
  return vtkPointerArray<T, N>{{vtkSmartPointer<T>(element.GetPointer()),
                                vtkSmartPointer<Ts>(elements.GetPointer())...}};
}

}  // namespace internal
}  // namespace render_vtk
}  // namespace geometry
}  // namespace drake
