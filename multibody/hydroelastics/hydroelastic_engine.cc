#include "drake/multibody/hydroelastics/hydroelastic_engine.h"

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
    std::unique_ptr<HydroelasticField<T>> mesh_field)
    : mesh_field_(std::move(mesh_field)) {}

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
              "hydroelastics", "elastic modulus",
              std::numeric_limits<double>::infinity());
      GeometryImplementationData specs{geometry_id, elastic_modulus};
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

    optional<ContactSurface<T>> surface =
        CalcContactSurface(id_S, model_S, X_WS, id_R, model_R, X_WR);
    if (surface) all_contact_surfaces.emplace_back(std::move(*surface));
  }

  return all_contact_surfaces;
}

template <typename T>
optional<ContactSurface<T>> HydroelasticEngine<T>::CalcContactSurface(
    GeometryId id_S, const HydroelasticGeometry<T>& soft_model_S,
    const RigidTransform<T>& X_WS,
    GeometryId id_R, const HydroelasticGeometry<T>& rigid_model_R,
    const RigidTransform<T>& X_WR) const {
  DRAKE_DEMAND(soft_model_S.is_soft());
  DRAKE_DEMAND(!rigid_model_R.is_soft());
  const HydroelasticField<T>& soft_field_S = soft_model_S.hydroelastic_field();
  std::vector<T> e_s_surface;
  std::vector<Vector3<T>> grad_level_set_R_surface;

  const auto X_RS = X_WR.inverse() * X_WS;
  std::unique_ptr<SurfaceMesh<T>> surface_W = CalcZeroLevelSetInMeshDomain(
      soft_field_S.volume_mesh(), rigid_model_R.level_set(), X_RS,
      soft_field_S.scalar_field().values(), &e_s_surface,
      &grad_level_set_R_surface);
  if (surface_W->num_vertices() == 0) return nullopt;

  // TODO(edrumwri): This says that it is a pressure field, but notation
  //                 reflects that it is a strain field. Fix.
  // Compute pressure field.
  for (T& e_s : e_s_surface) e_s *= soft_model_S.elastic_modulus();

  // TODO(edrumwri): h_RS should be the gradient of a pressure field, but it
  //                 is currently the gradient of a strain field. Fix.
  // ∇hₘₙ is a vector that points from N (in this case S) into M (in this case
  // R). Note that the gradient of the level set function points into S (N), so
  // we flip its direction. We also re-express this field in the world frame in
  // accordance with the ContactSurface specification.
  std::vector<Vector3<T>> h_RS_W_vectors = std::move(grad_level_set_R_surface);
  for (Vector3<T>& grad : h_RS_W_vectors)
    grad = X_WR.rotation() * -grad;

  auto e_s = std::make_unique<geometry::SurfaceMeshFieldLinear<T, T>>(
      "e_MN", std::move(e_s_surface), surface_W.get());
  auto h_RS_W =
      std::make_unique<geometry::SurfaceMeshFieldLinear<Vector3<T>, T>>(
          "grad_h_MN_W", std::move(h_RS_W_vectors), surface_W.get());

  return ContactSurface<T>(id_R, id_S, std::move(surface_W), std::move(e_s),
                           std::move(h_RS_W));
}

template <typename T>
void HydroelasticEngine<T>::ImplementGeometry(const Sphere& sphere,
                                              void* user_data) {
  const GeometryImplementationData& specs =
      *reinterpret_cast<GeometryImplementationData*>(user_data);
  const double elastic_modulus = specs.elastic_modulus;
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
      MakeSphereHydroelasticField<T>(refinement_level, sphere.get_radius());
  auto model =
      std::make_unique<HydroelasticGeometry<T>>(std::move(sphere_field));
  model->set_elastic_modulus(elastic_modulus);

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
