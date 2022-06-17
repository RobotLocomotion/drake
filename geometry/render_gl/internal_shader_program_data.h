#pragma once

#include <memory>
#include <utility>

#include "drake/common/copyable_unique_ptr.h"
#include "drake/common/identifier.h"
#include "drake/common/value.h"

namespace drake {
namespace geometry {
namespace render {
namespace internal {

/* Type used to identify unique shader programs in RenderEngineGl. */
using ShaderId = drake::Identifier<class ShaderTag>;

/* When a geometry instance is to be rendered into an image, it is associated
  with a shader. The instance will store the shader it uses and the per-instance
  data that that shader requires. See OpenGlInstance for more details.  */
class ShaderProgramData {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(ShaderProgramData)

  ShaderProgramData() = default;

  /* Constructs the data for the shader with the given `shader_id` and the given
   data (the data may be `nullptr`, but if the corresponding ShaderProgram
   was expecting actual data, this will lead to a run-time error).  */
  ShaderProgramData(ShaderId shader_id, std::unique_ptr<AbstractValue> value)
      : shader_id_(shader_id), value_(std::move(value)) {}

  ShaderId shader_id() const { return shader_id_; }
  const AbstractValue& value() const { return *value_; }

 private:
  /* The id of the shader to which this data belongs. */
  ShaderId shader_id_{};

  /* The type-erased data for the shader.  */
  copyable_unique_ptr<AbstractValue> value_{};
};
}  // namespace internal

}  // namespace render
}  // namespace geometry
}  // namespace drake
