#include "planning/body_shape_description.h"

#include <Eigen/Geometry>

#include "drake/common/drake_throw.h"

namespace anzu {
namespace planning {

BodyShapeDescription::BodyShapeDescription(
    const drake::geometry::Shape& shape,
    const drake::math::RigidTransformd& X_BS,
    std::string model_instance_name,
    std::string body_name)
    : shape_(shape.Clone()),
      X_BS_(X_BS),
      model_instance_name_(std::move(model_instance_name)),
      body_name_(std::move(body_name)) {}

BodyShapeDescription MakeBodyShapeDescription(
    const drake::multibody::MultibodyPlant<double>& plant,
    const drake::systems::Context<double>& plant_context,
    const drake::geometry::GeometryId& geometry_id) {
  // Grab query object to get the inspector.
  const auto& query_object =
      plant.get_geometry_query_input_port()
          .template Eval<drake::geometry::QueryObject<double>>(plant_context);
  const auto& inspector = query_object.inspector();
  const auto frame_id = inspector.GetFrameId(geometry_id);
  // inspector gives us the shape's pose w.r.t. the parent frame F. We rely on
  // MbP registering the geometry frame F to the body B as X_BF = I.
  const drake::math::RigidTransformd& X_BS =
      inspector.GetPoseInFrame(geometry_id);
  const auto body = plant.GetBodyFromFrameId(frame_id);
  DRAKE_THROW_UNLESS(body != nullptr);
  const std::string& model_instance_name =
      plant.GetModelInstanceName(body->model_instance());
  return BodyShapeDescription(inspector.GetShape(geometry_id), X_BS,
                              model_instance_name, body->name());
}

}  // namespace planning
}  // namespace anzu
