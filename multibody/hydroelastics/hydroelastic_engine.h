#pragma once

#include <limits>
#include <memory>
#include <vector>

#include "drake/common/drake_optional.h"
#include "drake/common/eigen_types.h"
#include "drake/geometry/proximity/volume_mesh.h"
#include "drake/geometry/query_object.h"
#include "drake/geometry/query_results/contact_surface.h"
#include "drake/multibody/hydroelastics/hydroelastic_field.h"
#include "drake/multibody/hydroelastics/level_set_field.h"

namespace drake {
namespace multibody {
namespace hydroelastics {
namespace internal {

/// This class provides the underlying computational representation for each
/// geometry in the model.
template <typename T>
class HydroelasticGeometry {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(HydroelasticGeometry)

  /// Creates a soft model from a given %HydroelasticField object.
  explicit HydroelasticGeometry(
      std::unique_ptr<HydroelasticField<T>> mesh_field);

  /// Constructor for a rigid model with its geometry represented by the
  /// zero-level of a level set function.
  explicit HydroelasticGeometry(std::unique_ptr<LevelSetField<T>> level_set);

  /// Returns `true` iff the underlying model represents a soft object.
  bool is_soft() const {
    // There must be only one representation.
    DRAKE_ASSERT((mesh_field_ != nullptr) != (level_set_ != nullptr));
    return mesh_field_ != nullptr;
  }

  /// Returns a reference to the underlying HydroelasticField object.
  /// It aborts if the model is not soft.
  const HydroelasticField<T>& hydroelastic_field() const {
    DRAKE_DEMAND(is_soft());
    return *mesh_field_;
  }

  /// Returns the underlying LevelSetField object.
  /// It aborts if the model is not rigid.
  const LevelSetField<T>& level_set() const {
    DRAKE_DEMAND(!is_soft());
    return *level_set_;
  }

  /// Returns the modulus of elasticity for `this` model.
  /// Iff infinity, the model is considered to be rigid.
  double elastic_modulus() const { return elastic_modulus_; }

  /// Sets the modulus of elasticity for `this` model.
  /// If infinity, the model is considered to be rigid.
  /// @throws std::exception if the geometry is rigid.
  void set_elastic_modulus(double elastic_modulus) {
    DRAKE_THROW_UNLESS(is_soft());
    elastic_modulus_ = elastic_modulus;
  }

 private:
  // Model is rigid by default.
  double elastic_modulus_{std::numeric_limits<double>::infinity()};
  std::unique_ptr<HydroelasticField<T>> mesh_field_;
  std::unique_ptr<LevelSetField<T>> level_set_;
};

/// The underlying engine to perform the geometric computations needed by the
/// hydroelastic model described in:
/// [Elandt, 2019]  R. Elandt, E. Drumwright, M. Sherman, and A. Ruina.
/// A pressure field model for fast, robust approximation of net contact force
/// and moment between nominally rigid objects. Proc. IEEE/RSJ Intl. Conf. on
/// Intelligent Robots and Systems (IROS), 2019.
///
/// This engine:
///  - Creates the internal representation of the geometric models as
///    HydroelasticGeometry instances. This creation takes place on the first
///    context-based query.
///  - Owns the HydroelasticGeometry instances.
///  - Provides API to perform hydroelastic contact specific queries.
///
/// @warning
/// Currently %HydroelasticEngine only provides support for hydroelastic contact
/// specific queries between soft spheres and rigid half spaces. Other
/// geometries are ignored by the engine to support backards compatibility with
/// simulation cases that only require the supported geometries and ignoring the
/// rest of the collision geometries is enough.
///
/// Instantiated templates for the following kinds of T's are provided:
///
/// - double
/// - AutoDiffXd
///
/// They are already available to link against in the containing library.
/// No other values for T are currently supported.
template <typename T>
class HydroelasticEngine {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(HydroelasticEngine)

  /// Constructor for an engine to be used by a given `plant`.
  /// %HydroelasticEngine keeps a reference to `plant` that must remain valid
  /// for the lifetime of the engine.
  ///
  /// %HydroelasticEngine will create an appropriate computation model for each
  /// geometry registered with this `plant` on the first call to
  /// `ComputeContactSurfaces()`.
  ///
  /// @throws std::exception if `plant` is nullptr.
  HydroelasticEngine();

  ~HydroelasticEngine();

  /// Returns the number of underlying %HydroelasticGeometry models.
  /// As outlined in the class's documentation, %HydroelasticEngine ignores
  /// geometries that are not supported. Therefore, models for unsupported
  /// geometries are not instantiated and num_models() might differ from the
  /// number of collision geometries in the original model passed on the first
  /// call to ComputeContactSurfaces() with the geometry::QueryObject object.
  int num_models() const;

  /// Returns a constant reference to the underlying HydroelasticGeometry for
  /// the given geometry identified by its `id`.
  const HydroelasticGeometry<T>& get_model(geometry::GeometryId id) const;

  /// For a given state of `query_object`, this method computes the contact
  /// surfaces for all geometries in contact.
  /// On the first call to this method, the engine creates the underlying
  /// computational representation for each geometry in the model, which is then
  /// used in future queries. Geometries that are currently not supported by
  /// %HydroelasticEngine (see this class's documentation) are ignored.
  /// TODO(amcastro-tri): Replace this short-term hack with a cache-compliant
  /// mechanism.
  std::vector<geometry::ContactSurface<T>> ComputeContactSurfaces(
      const geometry::QueryObject<T>& query_object) const;

 private:
  class Impl;
  Impl* impl_{};
};

}  // namespace internal
}  // namespace hydroelastics
}  // namespace multibody
}  // namespace drake
