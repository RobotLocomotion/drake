#include "drake/multibody/hydroelastics/hydroelastic_engine.h"

#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <utility>

#include "drake/common/default_scalars.h"
#include "drake/common/drake_assert.h"
#include "drake/common/text_logging.h"
#include "drake/geometry/shape_specification.h"
#include "drake/math/rigid_transform.h"
#include "drake/multibody/hydroelastics/contact_surface_from_level_set.h"
#include "drake/multibody/hydroelastics/hydroelastic_field_sphere.h"

using drake::geometry::Box;
using drake::geometry::Capsule;
using drake::geometry::ContactSurface;
using drake::geometry::Convex;
using drake::geometry::Cylinder;
using drake::geometry::GeometryId;
using drake::geometry::HalfSpace;
using drake::geometry::Mesh;
using drake::geometry::QueryObject;
using drake::geometry::Shape;
using drake::geometry::Sphere;
using drake::geometry::SurfaceMesh;
using drake::math::RigidTransform;

namespace drake {
namespace multibody {
namespace hydroelastics {
namespace internal {

template <typename T>
HydroelasticGeometry<T>::HydroelasticGeometry(
    std::unique_ptr<HydroelasticField<T>> mesh_field, double elastic_modulus,
    double hunt_crossley_dissipation)
    : mesh_field_(std::move(mesh_field)),
      elastic_modulus_(elastic_modulus),
      hunt_crossley_dissipation_(hunt_crossley_dissipation) {
  DRAKE_THROW_UNLESS(std::isfinite(elastic_modulus));
  DRAKE_THROW_UNLESS(std::isfinite(hunt_crossley_dissipation));
  DRAKE_THROW_UNLESS(elastic_modulus > 0);
  DRAKE_THROW_UNLESS(hunt_crossley_dissipation >= 0);
}

template <typename T>
HydroelasticGeometry<T>::HydroelasticGeometry(
    std::unique_ptr<LevelSetField<T>> level_set)
    : level_set_(std::move(level_set)) {}

template <typename T>
void HydroelasticEngine<T>::MakeModels(
    const geometry::SceneGraphInspector<T>& inspector) {
  // Only reify geometries with proximity roles.
  for (const geometry::GeometryId geometry_id : inspector.GetAllGeometryIds()) {
    if (const geometry::ProximityProperties* properties =
            inspector.GetProximityProperties(geometry_id)) {
      const Shape& shape = inspector.GetShape(geometry_id);
      const double elastic_modulus =
          properties->template GetPropertyOrDefault<double>(
              "material", "elastic_modulus",
              std::numeric_limits<double>::infinity());
      const double dissipation =
          properties->template GetPropertyOrDefault<double>(
              "material", "hunt_crossley_dissipation", 0.0);
      GeometryImplementationData specs{geometry_id, elastic_modulus,
                                       dissipation};
      shape.Reify(this, &specs);
    }
  }

  // Mark the model data as initialized so that we don't perform this step on
  // the next call to ComputeContactSurfaces().
  model_data_.models_are_initialized_ = true;
}

template <typename T>
int HydroelasticEngine<T>::num_models() const {
  return static_cast<int>(model_data_.geometry_id_to_model_.size());
}

template <typename T>
const HydroelasticGeometry<T>* HydroelasticEngine<T>::get_model(
    geometry::GeometryId id) const {
  auto it = model_data_.geometry_id_to_model_.find(id);
  if (it != model_data_.geometry_id_to_model_.end()) return it->second.get();
  return nullptr;
}

template <typename T>
double HydroelasticEngine<T>::CalcCombinedElasticModulus(
    geometry::GeometryId id_A, geometry::GeometryId id_B) const {
  const HydroelasticGeometry<T>* geometry_A = get_model(id_A);
  const HydroelasticGeometry<T>* geometry_B = get_model(id_B);
  // We demand id_A and id_B to correspond to hydroelastic model.
  DRAKE_DEMAND(geometry_A && geometry_B);
  const double E_A = geometry_A->elastic_modulus();
  const double E_B = geometry_B->elastic_modulus();
  if (E_A == std::numeric_limits<double>::infinity()) return E_B;
  if (E_B == std::numeric_limits<double>::infinity()) return E_A;
  return E_A * E_B / (E_A + E_B);
}

// TODO(amcastro-tri): as of 09/18/2019 we still are discussing how to combine
// these material properties. Update this method once the discussion is
// resolved.
template <typename T>
double HydroelasticEngine<T>::CalcCombinedDissipation(
    geometry::GeometryId id_A, geometry::GeometryId id_B) const {
  const HydroelasticGeometry<T>* geometry_A = get_model(id_A);
  const HydroelasticGeometry<T>* geometry_B = get_model(id_B);
  // We demand id_A and id_B to correspond to hydroelastic model.
  DRAKE_DEMAND(geometry_A && geometry_B);
  const double E_A = geometry_A->elastic_modulus();
  const double E_B = geometry_B->elastic_modulus();
  const double d_A = geometry_A->hunt_crossley_dissipation();
  const double d_B = geometry_B->hunt_crossley_dissipation();
  const double Estar = CalcCombinedElasticModulus(id_A, id_B);

  // Both bodies are rigid. We simply return the arithmetic average.
  if (Estar == std::numeric_limits<double>::infinity())
    return 0.5 * (d_A + d_B);

  // At least one body is soft.
  double d_star = 0;
  if (E_A != std::numeric_limits<double>::infinity())
    d_star += Estar / E_A * d_A;
  if (E_B != std::numeric_limits<double>::infinity())
    d_star += Estar / E_B * d_B;
  return d_star;
}

template <typename T>
std::vector<ContactSurface<T>> HydroelasticEngine<T>::ComputeContactSurfaces(
    const geometry::QueryObject<T>& query_object) const {
  const std::vector<SortedPair<GeometryId>>& geometry_pairs =
      query_object.FindCollisionCandidates();

  std::vector<ContactSurface<T>> all_contact_surfaces;
  for (const auto& pair : geometry_pairs) {
    GeometryId id_M = pair.first();
    GeometryId id_N = pair.second();
    const HydroelasticGeometry<T>* model_M = get_model(id_M);
    const HydroelasticGeometry<T>* model_N = get_model(id_N);

    // Skip contact surface computation if these ids do not have a hydrostatic
    // model.
    if (!model_M || !model_N) {
      const std::string name_M = query_object.inspector().GetName(id_M);
      const std::string name_N = query_object.inspector().GetName(id_N);
      throw std::runtime_error(
          "HydroelasticEngine. Unsupported geometries possibly in contact. "
          "For the geometry pair ('" + name_M + "', '" + name_N + "')."
          "You can remove the unsupported geometries, replace them with "
          "supported geometry, or filter collisions on them.");
    }

    // Thus far we only support rigid vs. soft.
    if (model_M->is_soft() == model_N->is_soft()) {
      throw std::runtime_error(
          "HydroelasticEngine. The current implementation of the "
          "hydroelastic model only supports soft vs. rigid contact.");
    }
    const RigidTransform<T>& X_WM = query_object.X_WG(id_M);
    const RigidTransform<T>& X_WN = query_object.X_WG(id_N);

    // Determine the poses in the world frame of the soft and rigid models.
    const RigidTransform<T> X_WR = model_M->is_soft() ? X_WN : X_WM;
    const RigidTransform<T> X_WS = model_M->is_soft() ? X_WM : X_WN;
    const GeometryId id_S = model_M->is_soft() ? id_M : id_N;
    const GeometryId id_R = model_M->is_soft() ? id_N : id_M;
    const HydroelasticGeometry<T>& model_S =
        model_M->is_soft() ? *model_M : *model_N;
    const HydroelasticGeometry<T>& model_R =
        model_M->is_soft() ? *model_N : *model_M;

    std::optional<ContactSurface<T>> surface =
        CalcContactSurface(id_S, model_S, X_WS, id_R, model_R, X_WR);
    if (surface) all_contact_surfaces.emplace_back(std::move(*surface));
  }

  return all_contact_surfaces;
}

template <typename T>
std::optional<ContactSurface<T>> HydroelasticEngine<T>::CalcContactSurface(
    GeometryId id_S, const HydroelasticGeometry<T>& soft_model_S,
    const RigidTransform<T>& X_WS,
    GeometryId id_R, const HydroelasticGeometry<T>& rigid_model_R,
    const RigidTransform<T>& X_WR) const {
  DRAKE_DEMAND(soft_model_S.is_soft());
  DRAKE_DEMAND(!rigid_model_R.is_soft());
  const HydroelasticField<T>& soft_field_S = soft_model_S.hydroelastic_field();
  std::vector<T> e_s_surface;

  const auto X_RS = X_WR.inverse() * X_WS;
  // Surface is measured and expressed in frame R. We'll transform to frame W
  // below if non-empty.
  std::unique_ptr<SurfaceMesh<T>> surface_W = CalcZeroLevelSetInMeshDomain(
      soft_field_S.volume_mesh(), rigid_model_R.level_set(), X_RS,
      soft_field_S.scalar_field().values(), &e_s_surface);
  if (surface_W->num_vertices() == 0) return std::nullopt;
  // Transform with vertices measured and expressed in frame W.
  surface_W->TransformVertices(X_WR);

  // TODO(edrumwri): This says that it is a pressure field, but notation
  //                 reflects that it is a strain field. Fix.
  // Compute pressure field.
  for (T& e_s_value : e_s_surface) e_s_value *= soft_model_S.elastic_modulus();
  auto e_s = std::make_unique<geometry::SurfaceMeshFieldLinear<T, T>>(
      "e_MN", std::move(e_s_surface), surface_W.get());

  // The calculation between level set and soft mesh produces a surface with
  // face normals pointing out of the level set and into the soft surface.
  // The ordering of id_R and id_S reflects this.
  return ContactSurface<T>(id_R, id_S, std::move(surface_W), std::move(e_s));
}

template <typename T>
void HydroelasticEngine<T>::ImplementGeometry(const Sphere& sphere,
                                              void* user_data) {
  const GeometryImplementationData& specs =
      *reinterpret_cast<GeometryImplementationData*>(user_data);
  const double elastic_modulus = specs.elastic_modulus;
  const double dissipation = specs.dissipation;
  if (elastic_modulus == std::numeric_limits<double>::infinity()) {
    drake::log()->warn(
        "HydroelasticEngine. The current hydroelastic model implementation "
        "does not support rigid spheres. Geometry ignored.");
  }
  // We arbitrarily choose the refinement level so that we have 512
  // tetrahedron in the tessellation of the sphere. This provides a reasonable
  // tessellation of the sphere with a coarse mesh.
  // TODO(amcastro-tri): Make this a user setable parameter.
  const int refinement_level = 2;
  auto sphere_field =
      MakeSphereHydroelasticField<T>(refinement_level, sphere.radius());
  auto model = std::make_unique<HydroelasticGeometry<T>>(
      std::move(sphere_field), elastic_modulus, dissipation);
  model_data_.geometry_id_to_model_[specs.id] = std::move(model);
}

template <typename T>
void HydroelasticEngine<T>::ImplementGeometry(const HalfSpace&,
                                              void* user_data) {
  const GeometryImplementationData& specs =
      *reinterpret_cast<GeometryImplementationData*>(user_data);
  const double elastic_modulus = specs.elastic_modulus;
  if (elastic_modulus != std::numeric_limits<double>::infinity()) {
    drake::log()->warn(
        "HydroelasticEngine. The current hydroelastic model implementation "
        "does not support soft half-spaces. Geometry ignored.");
  }
  auto level_set = std::make_unique<LevelSetField<T>>(
      [](const Vector3<T>& p) { return p[2]; },
      [](const Vector3<T>&) { return Vector3<double>::UnitZ(); });
  model_data_.geometry_id_to_model_[specs.id] =
      std::make_unique<HydroelasticGeometry<T>>(std::move(level_set));
}

// The following overrides are no-ops given that currently HydroelasticEngine
// does not support these geometries.
template <typename T>
void HydroelasticEngine<T>::ImplementGeometry(const Cylinder&, void*) {
  drake::log()->warn(
      "HydroelasticEngine. The current hydroelastic model implementation "
      "does not support cylinder geometries. Geometry ignored.");
}

template <typename T>
void HydroelasticEngine<T>::ImplementGeometry(const Box&, void*) {
  drake::log()->warn(
      "HydroelasticEngine. The current hydroelastic model implementation "
      "does not support box geometries. Geometry ignored.");
}

template <typename T>
void HydroelasticEngine<T>::ImplementGeometry(const Capsule&, void*) {
  drake::log()->warn(
      "HydroelasticEngine. The current hydroelastic model implementation "
      "does not support capsule geometries. Geometry ignored.");
}

template <typename T>
void HydroelasticEngine<T>::ImplementGeometry(const Mesh&, void*) {
  drake::log()->warn(
      "HydroelasticEngine. The current hydroelastic model implementation "
      "does not support mesh geometries. Geometry ignored.");
}

template <typename T>
void HydroelasticEngine<T>::ImplementGeometry(const Convex&, void*) {
  drake::log()->warn(
      "HydroelasticEngine. The current hydroelastic model implementation "
      "does not support convex geometries. Geometry ignored.");
}

}  // namespace internal
}  // namespace hydroelastics
}  // namespace multibody
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::multibody::hydroelastics::internal::HydroelasticEngine)
