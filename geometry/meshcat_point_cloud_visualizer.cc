#include "drake/geometry/meshcat_point_cloud_visualizer.h"

#include <memory>
#include <string>
#include <utility>

#include <fmt/format.h>

#include "drake/common/extract_double.h"
#include "drake/geometry/utilities.h"
#include "drake/perception/point_cloud.h"

namespace drake {
namespace geometry {

template <typename T>
MeshcatPointCloudVisualizer<T>::MeshcatPointCloudVisualizer(
    std::shared_ptr<Meshcat> meshcat, std::string path, double publish_period)
    : systems::LeafSystem<T>(
          systems::SystemTypeTag<MeshcatPointCloudVisualizer>{}),
      meshcat_(std::move(meshcat)),
      path_(std::move(path)),
      publish_period_(publish_period) {
  DRAKE_DEMAND(meshcat_ != nullptr);
  DRAKE_DEMAND(publish_period >= 0.0);

  this->DeclarePeriodicPublishEvent(
      publish_period, 0.0,
      &MeshcatPointCloudVisualizer<T>::UpdateMeshcat);
  this->DeclareForcedPublishEvent(
      &MeshcatPointCloudVisualizer<T>::UpdateMeshcat);

  cloud_input_port_ =
      this->DeclareAbstractInputPort("cloud", Value<perception::PointCloud>())
          .get_index();

  pose_input_port_ = this->DeclareAbstractInputPort(
                             "X_ParentCloud", Value<math::RigidTransform<T>>{})
                         .get_index();
}

template <typename T>
template <typename U>
MeshcatPointCloudVisualizer<T>::MeshcatPointCloudVisualizer(
    const MeshcatPointCloudVisualizer<U>& other)
    : MeshcatPointCloudVisualizer(other.meshcat_, other.path_,
                                  other.publish_period_) {
  set_point_size(other.point_size_);
  set_default_rgba(other.default_rgba_);
}

template <typename T>
void MeshcatPointCloudVisualizer<T>::Delete() const {
  meshcat_->Delete(path_);
}

template <typename T>
systems::EventStatus MeshcatPointCloudVisualizer<T>::UpdateMeshcat(
    const systems::Context<T>& context) const {
  const auto& cloud =
      cloud_input_port().template Eval<perception::PointCloud>(context);
  meshcat_->SetObject(path_, cloud, point_size_, default_rgba_);

  const math::RigidTransformd X_ParentCloud =
      pose_input_port().HasValue(context)
          ? internal::convert_to_double(
                pose_input_port().template Eval<math::RigidTransform<T>>(
                    context))
          : math::RigidTransformd::Identity();
  meshcat_->SetTransform(path_, X_ParentCloud);

  return systems::EventStatus::Succeeded();
}

}  // namespace geometry
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::geometry::MeshcatPointCloudVisualizer)
