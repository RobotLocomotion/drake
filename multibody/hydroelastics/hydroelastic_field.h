#pragma once

#include <memory>
#include <utility>

#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/geometry/proximity/volume_mesh.h"
#include "drake/geometry/proximity/volume_mesh_field.h"

namespace drake {
namespace multibody {
namespace hydroelastics {
namespace internal {

/// This class stores the tetrahedral mesh, scalar, and vector fields for the
/// hydroelastic model.
template <typename T>
class HydroelasticField {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(HydroelasticField)

  /// Constructor of a HydroelasticField.
  /// @param[in] mesh_M
  ///   The tetrahedral mesh representation of the geometry, with position
  ///   vectors measured and expressed in the frame M of the model.
  /// @param[in] e_m
  ///   The volumetric scalar field of the hydroelastic model.
  /// @param[in] grad_e_m_M
  ///   The gradient of the scalar field e_m, expressed in the frame M of the
  ///   mesh.
  HydroelasticField(
      std::unique_ptr<geometry::VolumeMesh<T>> mesh_M,
      std::unique_ptr<geometry::VolumeMeshFieldLinear<T, T>> e_m,
      std::unique_ptr<geometry::VolumeMeshFieldLinear<Vector3<T>, T>>
          grad_e_m_M)
      : mesh_M_(std::move(mesh_M)),
        e_m_(std::move(e_m)),
        grad_e_m_M_(std::move(grad_e_m_M)) {}

  const geometry::VolumeMesh<T>& volume_mesh() const { return *mesh_M_; }

  const geometry::VolumeMeshFieldLinear<T, T>& scalar_field() const {
    return *e_m_;
  }

  // TODO(amcastro-tri): when needed, add accessor:
  // const VolumeMeshFieldLinear<Vector3<T>, T>& gradient_field() const;

 private:
  /** The volume mesh of M. */
  std::unique_ptr<geometry::VolumeMesh<T>> mesh_M_;
  /** Represents the scalar field eₘ on the surface mesh. */
  std::unique_ptr<geometry::VolumeMeshFieldLinear<T, T>> e_m_;
  /** Represents the vector field ∇eₘ on the surface mesh, expressed in M's
    frame */
  std::unique_ptr<geometry::VolumeMeshFieldLinear<Vector3<T>, T>> grad_e_m_M_;
};

}  // namespace internal
}  // namespace hydroelastics
}  // namespace multibody
}  // namespace drake
