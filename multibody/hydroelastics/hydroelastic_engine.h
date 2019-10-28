#pragma once

#include <limits>
#include <memory>
#include <unordered_map>
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

  /// Creates a soft model from a given %HydroelasticField object, elastic
  /// modulus and dissipation.
  /// `elastic_modulus` must be strictly positive and have units of Pa.
  /// `hunt_crossley_dissipation` must be non-negative and have units of s/m.
  /// @throws std::exception if `elastic_modulus` is infinite or <= 0.
  /// @throws std::exception if `hunt_crossley_dissipation` is infinite and or
  /// negative.
  HydroelasticGeometry(
      std::unique_ptr<HydroelasticField<T>> mesh_field, double elastic_modulus,
      double hunt_crossley_dissipation);

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

  /// Returns the modulus of elasticity for `this` model, with units of Pa.
  /// Iff infinity, the model is considered to be rigid.
  double elastic_modulus() const { return elastic_modulus_; }

  /// Returns the Hunt & Crossley dissipation constant for `this` model, with
  /// units of s/m.
  double hunt_crossley_dissipation() const {
    return hunt_crossley_dissipation_;
  }

 private:
  std::unique_ptr<HydroelasticField<T>> mesh_field_;
  std::unique_ptr<LevelSetField<T>> level_set_;
  // Model is rigid by default.
  double elastic_modulus_{std::numeric_limits<double>::infinity()};
  // Model has zero dissipation by default.
  double hunt_crossley_dissipation_{0};
};

/// The underlying engine to perform the computation of contact surfaces and
/// fields needed by the hydroelastic model described in:
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
/// specific queries between soft spheres and rigid half spaces. The engine will
/// throw an exception if two unsupported geometries are detected to possibly be
/// in contact during the broadphase.
///
/// Instantiated templates for the following kinds of T's are provided:
///
/// - double
/// - AutoDiffXd
///
/// They are already available to link against in the containing library.
/// No other values for T are currently supported.
///
/// %HydroelasticEngine can be instantiated on symbolic::Expression to allow the
/// compilation of calling code with this scalar type. However usage of any of
/// the engine's APIs will throw a runtime exception when used with T =
/// symbolic::Expression.
template <typename T>
class HydroelasticEngine final : public geometry::ShapeReifier {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(HydroelasticEngine)

  HydroelasticEngine() = default;

  ~HydroelasticEngine() = default;

  /// This method builds the underlying computational representation as needed
  /// for the hydroelastic model for each geometry specified in `inspector`.
  /// As outlined in the class's documentation, %HydroelasticEngine ignores
  /// geometries that are not supported. Therefore, models for unsupported
  /// geometries are not instantiated and num_models() might differ from the
  /// number of collision geometries in `inspector`.
  /// Physical properties will be obtained from the
  /// `geometry::ProximityProperties` for each geometry. Hydroelastic model
  /// relevant properties are expected to be within a group named
  /// "hydroelastics". The value of the "elastic modulus" is assumed to be
  /// infinity, rigid object, if not provided.
  void MakeModels(const geometry::SceneGraphInspector<T>& inspector);

  /// Returns the number of underlying HydroelasticGeometry models created on
  /// the call to MakeModels().
  int num_models() const;

  /// Returns the underlying HydroelasticGeometry for the given geometry
  /// identified by its `id` or `nullptr` if the engine did not create a
  /// hydroelastic model for this geometry.
  const HydroelasticGeometry<T>* get_model(geometry::GeometryId id) const;

  /// Computes the combined elastic modulus for geometries with ids `id_A` and
  /// `id_B`.
  /// Refer to @ref mbp_hydroelastic_materials_properties "Hydroelastic model
  /// material properties" for further details.
  double CalcCombinedElasticModulus(geometry::GeometryId id_A,
                                    geometry::GeometryId id_B) const;

  /// Computes the combined Hunt & Crossley dissipation for geometries with ids
  /// `id_A` and `id_B`. Refer to @ref mbp_hydroelastic_materials_properties
  /// "Hydroelastic model material properties" for further details.
  double CalcCombinedDissipation(geometry::GeometryId id_A,
                                 geometry::GeometryId id_B) const;

  /// For a given state of `query_object`, this method computes the contact
  /// surfaces for all geometries in contact.
  /// @warning Unsupported geometries are ignored unless the broadphase pass
  /// detects that they are possibly in contact. In this case an exception is
  /// thrown.
  std::vector<geometry::ContactSurface<T>> ComputeContactSurfaces(
      const geometry::QueryObject<T>& query_object) const;

 private:
  // This struct stores additional data passed to ImplementGeometry() during
  // the reification process.
  struct GeometryImplementationData {
    geometry::GeometryId id;
    double elastic_modulus;
    double dissipation;
  };

  // This struct holds the engines's data, created by the call to MakeModels().
  struct ModelData {
    bool models_are_initialized_{false};
    std::unordered_map<geometry::GeometryId,
                       std::unique_ptr<HydroelasticGeometry<T>>>
        geometry_id_to_model_;
  };

  // Helper method to compute the contact surface betwen a soft model S and a
  // rigid model R given the poses of both R and S in the world frame.
  // Returns nullopt if soft_model_S and rigid_model_R do not intersect.
  optional<geometry::ContactSurface<T>> CalcContactSurface(
      geometry::GeometryId id_S, const HydroelasticGeometry<T>& soft_model_S,
      const math::RigidTransform<T>& X_WS,
      geometry::GeometryId id_R, const HydroelasticGeometry<T>& rigid_model_R,
      const math::RigidTransform<T>& X_WR) const;

  // Implementation of ShapeReifier interface
  using geometry::ShapeReifier::ImplementGeometry;
  void ImplementGeometry(const geometry::Sphere& sphere,
                         void* user_data) override;
  void ImplementGeometry(const geometry::HalfSpace&, void* user_data) override;
  void ImplementGeometry(const geometry::Cylinder&, void*) override;
  void ImplementGeometry(const geometry::Box&, void*) override;
  void ImplementGeometry(const geometry::Capsule&, void*) override;
  void ImplementGeometry(const geometry::Mesh&, void*) override;
  void ImplementGeometry(const geometry::Convex&, void*) override;

  ModelData model_data_;
};

// Specialization to support compilation and linking with T =
// symbolic::Expression. Even though the Clang compiler does not need this, the
// GCC compiler does need it at linking time.
template <>
class HydroelasticEngine<symbolic::Expression> {
 public:
  using T = symbolic::Expression;

  void MakeModels(const geometry::SceneGraphInspector<T>&) {
    Throw("MakeModels");
  }

  std::vector<geometry::ContactSurface<T>> ComputeContactSurfaces(
      const geometry::QueryObject<T>&) const {
    Throw("ComputeContactSurfaces");
    return std::vector<geometry::ContactSurface<T>>();
  }

  // TODO(amcastro-tri): Update this list of methods if the API in
  // HydroelasticEngine grows.

 private:
  static void Throw(const char* operation_name) {
    throw std::logic_error(fmt::format(
        "Cannot call `{}` on a HydroelasticEngine<symbolic::Expression>",
        operation_name));
  }
};

}  // namespace internal
}  // namespace hydroelastics
}  // namespace multibody
}  // namespace drake
