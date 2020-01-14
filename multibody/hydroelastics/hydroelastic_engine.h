#pragma once

#include "drake/geometry/query_object.h"
#include "drake/geometry/query_results/contact_surface.h"
#include "drake/geometry/scene_graph_inspector.h"

namespace drake {
namespace multibody {
namespace hydroelastics {
namespace internal {

// TODO(SeanCurtis-TRI): This class no longer has any members and the methods
//  could be static. That suggests that this should no longer be a class.
//  Reorganize these methods appropriately.

/// The class that provides some of the encoding of the hydroelastic model.
/// It doesn't handle the geometric work or the integration work, but it
/// contains the logic for combining quantities (e.g., effective elastic
/// modulus). The hydroelastic model is described in:
/// [Elandt, 2019]  R. Elandt, E. Drumwright, M. Sherman, and A. Ruina.
/// A pressure field model for fast, robust approximation of net contact force
/// and moment between nominally rigid objects. Proc. IEEE/RSJ Intl. Conf. on
/// Intelligent Robots and Systems (IROS), 2019.
template <typename T>
class HydroelasticEngine  {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(HydroelasticEngine)

  HydroelasticEngine() = default;

  ~HydroelasticEngine() = default;

  /// Computes the combined elastic modulus for geometries with ids `id_A` and
  /// `id_B`.
  /// Refer to @ref mbp_hydroelastic_materials_properties "Hydroelastic model
  /// material properties" for further details.
  double CalcCombinedElasticModulus(
      geometry::GeometryId id_A, geometry::GeometryId id_B,
      const geometry::SceneGraphInspector<T>& inspector) const;

  /// Computes the combined Hunt & Crossley dissipation for geometries with ids
  /// `id_A` and `id_B`. Refer to @ref mbp_hydroelastic_materials_properties
  /// "Hydroelastic model material properties" for further details.
  double CalcCombinedDissipation(
      geometry::GeometryId id_A, geometry::GeometryId id_B,
      const geometry::SceneGraphInspector<T>& inspector) const;
};

}  // namespace internal
}  // namespace hydroelastics
}  // namespace multibody
}  // namespace drake
