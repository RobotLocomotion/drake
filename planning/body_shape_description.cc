#include "drake/planning/body_shape_description.h"

#include "drake/common/drake_assert.h"
#include "drake/geometry/query_object.h"
#include "drake/geometry/scene_graph.h"

namespace drake {
namespace planning {

BodyShapeDescription::BodyShapeDescription(const geometry::Shape& shape,
                                           const math::RigidTransformd& X_BS,
                                           std::string model_instance_name,
                                           std::string body_name)
    : shape_(shape.Clone()),
      X_BS_(X_BS),
      model_instance_name_(std::move(model_instance_name)),
      body_name_(std::move(body_name)) {}

BodyShapeDescription MakeBodyShapeDescription(
    const multibody::MultibodyPlant<double>& plant,
    const systems::Context<double>& plant_context,
    const geometry::GeometryId& geometry_id) {
  DRAKE_DEMAND(plant.geometry_source_is_registered());
  plant.ValidateContext(plant_context);
  DRAKE_DEMAND(geometry_id.is_valid());

  // Grab query object to get the inspector.
  const auto& query_object =
      plant.get_geometry_query_input_port()
          .template Eval<geometry::QueryObject<double>>(plant_context);
  const auto& inspector = query_object.inspector();

  // Make sure the geometry_id is associated with the passed plant.
  DRAKE_DEMAND(inspector.BelongsToSource(geometry_id, *plant.get_source_id()));

  const auto frame_id = inspector.GetFrameId(geometry_id);
  // inspector gives us the shape's pose w.r.t. the parent geometry frame F. We
  // rely on MbP registering the geometry frame F to the body B as X_BF = I.
  const math::RigidTransformd& X_BS = inspector.GetPoseInFrame(geometry_id);
  const auto body = plant.GetBodyFromFrameId(frame_id);
  DRAKE_DEMAND(body != nullptr);
  const std::string& model_instance_name =
      plant.GetModelInstanceName(body->model_instance());
  return BodyShapeDescription(inspector.GetShape(geometry_id), X_BS,
                              model_instance_name, body->name());
}

}  // namespace planning
}  // namespace drake
