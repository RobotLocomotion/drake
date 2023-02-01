#include "drake/planning/collision_checker_context.h"

#include <memory>
#include <utility>

namespace drake {
namespace planning {

using geometry::QueryObject;
using std::unique_ptr;
using systems::Context;

namespace {
template <typename T>
T* NonNull(T* pointer) {
  DRAKE_DEMAND(pointer != nullptr);
  return pointer;
}
}  // namespace

CollisionCheckerContext::CollisionCheckerContext(
    const RobotDiagram<double>* model)
    : CollisionCheckerContext(NonNull(model), model->CreateDefaultContext()) {
}

CollisionCheckerContext::~CollisionCheckerContext() = default;

const QueryObject<double>& CollisionCheckerContext::GetQueryObject() const {
  return model_.plant()
      .get_geometry_query_input_port()
      .template Eval<QueryObject<double>>(plant_context());
}

CollisionCheckerContext::CollisionCheckerContext(
    const CollisionCheckerContext& other)
    : CollisionCheckerContext(&other.model_, other.model_context_->Clone()) {
}

unique_ptr<CollisionCheckerContext> CollisionCheckerContext::DoClone() const {
  return unique_ptr<CollisionCheckerContext>(
      new CollisionCheckerContext(*this));
}

CollisionCheckerContext::CollisionCheckerContext(
    const RobotDiagram<double>* model,
    unique_ptr<Context<double>> model_context)
    : model_(*NonNull(model)),
      model_context_(std::move(model_context)),
      plant_context_(&model_.mutable_plant_context(model_context_.get())),
      scene_graph_context_(
          &model_.mutable_scene_graph_context(model_context_.get())) {
}

}  // namespace planning
}  // namespace drake
