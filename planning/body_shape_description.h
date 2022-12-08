#pragma once

#include <string>
#include <utility>

#include "drake/common/copyable_unique_ptr.h"
#include "drake/common/drake_copyable.h"
#include "drake/geometry/shape_specification.h"
#include "drake/multibody/plant/multibody_plant.h"

namespace anzu {
namespace planning {
/** %BodyShapeDescription captures all the information necessary to describe a
SceneGraph collision shape associated with a MultibodyPlant Body: a shape
S, the MultibodyPlant body B (identified by model instance and body names), and
the rigid pose of the shape S relative to the body B, X_BS.

Most clients should use the factory method MakeBodyShapeDescription() to
construct a valid %BodyShapeDescription; it will extract and verify the correct
information from a multibody plant and its context.

When default-constructed or moved-from, this models a null description and all
of the getter functions will throw. */
class BodyShapeDescription final {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(BodyShapeDescription);

  /** Constructs a description with the given attributes. Does not check or
  enforce correctness; callers are responsible for providing consistent
  input. */
  BodyShapeDescription(const drake::geometry::Shape& shape,
                       const drake::math::RigidTransformd& X_BS,
                       std::string model_instance_name, std::string body_name);

  /** @returns the shape passed at construction.
      @throws std::exception if this object was default constructed. */
  const drake::geometry::Shape& shape() const { return *shape_; }

  /** @returns the transform X_BS passed at construction.
      @throws std::exception if this object was default constructed. */
  const drake::math::RigidTransformd& pose_in_body() const { return X_BS_; }

  /** @returns the model instance name passed at construction.
      @throws std::exception if this object was default constructed. */
  const std::string& model_instance_name() const {
    return model_instance_name_;
  }

  /** @returns the body name passed at construction.
      @throws std::exception if this object was default constructed. */
  const std::string& body_name() const { return body_name_; }

 private:
  drake::copyable_unique_ptr<drake::geometry::Shape> shape_;
  drake::math::RigidTransformd X_BS_;
  std::string model_instance_name_;
  std::string body_name_;
};

/** Constructs a BodyShapeDescription by extracting the shape, pose, and names
associated with the provided geometry_id.

@throws std::exception if @p plant_context was not created for @p plant.
@throws std::exception if @p plant is not connected to a scene graph.
@throws std::exception if @p geometry_id  doesn't refer to a geometry rigidly
                       affixed to a body of @p plant. */
BodyShapeDescription MakeBodyShapeDescription(
    const drake::multibody::MultibodyPlant<double>& plant,
    const drake::systems::Context<double>& plant_context,
    const drake::geometry::GeometryId& geometry_id);

}  // namespace planning
}  // namespace anzu
