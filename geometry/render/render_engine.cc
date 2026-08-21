#include "drake/geometry/render/render_engine.h"

#include <typeinfo>
#include <utility>

#include <fmt/format.h>

#include "drake/common/nice_type_name.h"
#include "drake/common/scope_exit.h"
#include "drake/common/text_logging.h"

namespace drake {
namespace geometry {
namespace render {

using math::RigidTransformd;
using systems::sensors::CameraInfo;
using systems::sensors::ImageDepth32F;
using systems::sensors::ImageLabel16I;
using systems::sensors::ImageRgba8U;

RenderEngine::~RenderEngine() = default;

template <class Result>
Result RenderEngine::Clone() const
  requires std::is_same_v<Result, std::unique_ptr<RenderEngine>> ||
           std::is_same_v<Result, std::shared_ptr<RenderEngine>>
{  // NOLINT(whitespace/braces)
  Result clone;
  if constexpr (std::is_same_v<Result, std::unique_ptr<RenderEngine>>) {
    clone = DoClone();
  } else {
    clone = DoCloneShared();
  }
  // Make sure that derived classes have actually overridden DoClone().
  // Particularly important for derivations of derivations.
  // Note: clang considers typeid(*clone) to be an expression with side effects.
  // So, we capture a reference to the polymorphic type and provide that to
  // typeid to make both clang and gcc happy.
  const RenderEngine& clone_ref = *clone;
  if (typeid(*this) != typeid(clone_ref)) {
    throw std::logic_error(fmt::format(
        "Error in cloning RenderEngine class of type {}; the clone returns "
        "type {}. {}::DoClone() was probably not implemented",
        NiceTypeName::Get(*this), NiceTypeName::Get(clone_ref),
        NiceTypeName::Get(*this)));
  }
  return clone;
}

// Explicit template instantiations.
template std::unique_ptr<RenderEngine> RenderEngine::Clone<>() const;
template std::shared_ptr<RenderEngine> RenderEngine::Clone<>() const;

std::shared_ptr<RenderEngine> RenderEngine::DoCloneShared() const {
  // When not overriden, we simply delegate to the unique_ptr flavor.
  return this->DoClone();
}

bool RenderEngine::RegisterVisual(GeometryId id,
                                  const drake::geometry::Shape& shape,
                                  const PerceptionProperties& properties,
                                  const RigidTransformd& X_WG,
                                  bool needs_updates) {
  // TODO(SeanCurtis-TRI): Test that the id hasn't already been used.
  const bool accepted = DoRegisterVisual(id, shape, properties, X_WG);
  if (accepted) {
    if (needs_updates) {
      update_ids_.insert(id);
    } else {
      anchored_ids_.insert(id);
    }
  }
  return accepted;
}

bool RenderEngine::RegisterDeformableVisual(
    GeometryId id, const std::vector<internal::RenderMesh>& render_meshes,
    const PerceptionProperties& properties) {
  DRAKE_THROW_UNLESS(!has_geometry(id));
  DRAKE_THROW_UNLESS(!render_meshes.empty());
  bool accepted = DoRegisterDeformableVisual(id, render_meshes, properties);
  if (accepted) {
    std::vector<int> mesh_dofs;
    for (const auto& mesh : render_meshes) {
      mesh_dofs.emplace_back(mesh.positions.size());
    }
    deformable_mesh_dofs_[id] = std::move(mesh_dofs);
  }
  return accepted;
}

bool RenderEngine::RemoveGeometry(GeometryId id) {
  const bool removed = DoRemoveGeometry(id);
  // The derived sub-class should report geometry removal if and only if the
  // base class is tracking the id.
  if (removed) {
    DRAKE_DEMAND(update_ids_.erase(id) > 0 || anchored_ids_.erase(id) > 0 ||
                 deformable_mesh_dofs_.erase(id) > 0);
  }
  DRAKE_DEMAND(!has_geometry(id));
  return removed;
}

bool RenderEngine::has_geometry(GeometryId id) const {
  return update_ids_.contains(id) || anchored_ids_.contains(id) ||
         deformable_mesh_dofs_.contains(id);
}

void RenderEngine::UpdateDeformableConfigurations(
    GeometryId id, const std::vector<VectorX<double>>& q_WGs,
    const std::vector<VectorX<double>>& nhats_W) {
  if (!deformable_mesh_dofs_.contains(id)) {
    return;
  }
  const std::vector<int>& mesh_dofs = deformable_mesh_dofs_.at(id);
  if (mesh_dofs.size() != q_WGs.size() || mesh_dofs.size() != nhats_W.size()) {
    throw std::runtime_error(fmt::format(
        "Vertex data for the wrong number of meshes. {} meshes are registered "
        "with deformable geometry with id {}, but vertex positions for {} "
        "meshes and vertex normals for {} meshes are provided for the "
        "configuration update.",
        mesh_dofs.size(), id, q_WGs.size(), nhats_W.size()));
  }
  for (int i = 0; i < ssize(mesh_dofs); ++i) {
    if (mesh_dofs[i] != q_WGs[i].size() || mesh_dofs[i] != nhats_W[i].size()) {
      throw std::runtime_error(fmt::format(
          "Wrong dofs in vertex positions and/or normals. There are {} dof "
          "for mesh {} registered with deformable geometry with id {}; "
          "however, positions with {} dofs and normals with {} dofs are "
          "supplied in the configuration update.",
          mesh_dofs[i], i, id, q_WGs[i].size(), nhats_W[i].size()));
    }
  }
  DoUpdateDeformableConfigurations(id, q_WGs, nhats_W);
}

RenderLabel RenderEngine::GetRenderLabelOrThrow(
    const PerceptionProperties& properties) const {
  RenderLabel label =
      properties.GetPropertyOrDefault("label", "id", default_render_label_);
  if (label == RenderLabel::kUnspecified || label == RenderLabel::kEmpty) {
    throw std::logic_error(
        "Cannot register a geometry with the 'unspecified' or 'empty' render "
        "labels. The bad label may have come from a default-constructed "
        "RenderLabel or the RenderEngine may have provided it as a default for "
        "missing render labels in the properties.");
  }
  return label;
}

bool RenderEngine::DoRegisterDeformableVisual(
    GeometryId, const std::vector<internal::RenderMesh>&,
    const PerceptionProperties&) {
  return false;
}

void RenderEngine::DoUpdateDeformableConfigurations(
    GeometryId, const std::vector<VectorX<double>>&,
    const std::vector<VectorX<double>>&) {}

void RenderEngine::DoRenderColorImage(const ColorRenderCamera&,
                                      ImageRgba8U*) const {
  throw std::runtime_error(
      fmt::format("{}: has not implemented DoRenderColorImage().",
                  NiceTypeName::Get(*this)));
}

void RenderEngine::DoRenderDepthImage(const DepthRenderCamera&,
                                      ImageDepth32F*) const {
  throw std::runtime_error(
      fmt::format("{}: has not implemented DoRenderDepthImage().",
                  NiceTypeName::Get(*this)));
}

void RenderEngine::DoRenderLabelImage(const ColorRenderCamera&,
                                      ImageLabel16I*) const {
  throw std::runtime_error(
      fmt::format("{}: has not implemented DoRenderLabelImage().",
                  NiceTypeName::Get(*this)));
}

void RenderEngine::SetDefaultLightPosition(const Vector3<double>&) {}

std::string RenderEngine::DoGetParameterYaml() const {
  return "UnknownRenderEngine: {}";
}

}  // namespace render
}  // namespace geometry
}  // namespace drake
