#pragma once

#include <string>
#include <utility>

#include "drake/common/copyable_unique_ptr.h"
#include "drake/geometry/shape_specification.h"
#include "drake/multibody/plant/multibody_plant.h"

namespace drake {
namespace planning {
/** %BodyShapeDescription captures all the information necessary to describe a
SceneGraph collision shape associated with a MultibodyPlant Body: a shape
S, the MultibodyPlant body B (identified by model instance and body names), and
the rigid pose of the shape S relative to the body B, X_BS.

Most clients should use the factory method MakeBodyShapeDescription() to
construct a valid %BodyShapeDescription; it will extract and verify the correct
information from a multibody plant and its context. */
class BodyShapeDescription final {
 public:
  /** @name     Allows copy semantics, but not move semantics. */
  //@{

  BodyShapeDescription(const BodyShapeDescription&) = default;
  BodyShapeDescription& operator=(const BodyShapeDescription&) = default;
  BodyShapeDescription(BodyShapeDescription&&) = delete;
  void operator=(BodyShapeDescription&&) = delete;
  //@}

  /** Constructs a description with the given attributes. Does not check or
  enforce correctness; callers are responsible for providing consistent
  input. */
  BodyShapeDescription(const geometry::Shape& shape,
                       const math::RigidTransformd& X_BS,
                       std::string model_instance_name, std::string body_name);

  /** @returns the shape passed at construction. */
  const geometry::Shape& shape() const {
    DRAKE_DEMAND(shape_ != nullptr);
    return *shape_;
  }

  /** @retval X_BS The pose passed at construction. */
  const math::RigidTransformd& pose_in_body() const { return X_BS_; }

  /** @returns the model instance name passed at construction. */
  const std::string& model_instance_name() const {
    return model_instance_name_;
  }

  /** @returns the body name passed at construction. */
  const std::string& body_name() const { return body_name_; }

 private:
  copyable_unique_ptr<geometry::Shape> shape_;
  math::RigidTransformd X_BS_;
  std::string model_instance_name_;
  std::string body_name_;
};

/** Constructs a BodyShapeDescription by extracting the shape, pose, and names
associated with the provided geometry_id.

@pre @p plant_context is compatible with @p plant.
@pre @p plant is connected to a scene graph.
@pre @p geometry_id refers to a geometry rigidly affixed to a body of @p
     plant. */
BodyShapeDescription MakeBodyShapeDescription(
    const multibody::MultibodyPlant<double>& plant,
    const systems::Context<double>& plant_context,
    const geometry::GeometryId& geometry_id);

}  // namespace planning
}  // namespace drake
