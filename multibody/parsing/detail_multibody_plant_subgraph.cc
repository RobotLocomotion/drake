#include "drake/multibody/parsing/detail_multibody_plant_subgraph.h"

namespace drake {
namespace multibody {
namespace internal {
namespace {

template <typename Item, typename Container>
bool ContainedIn(const Item& item, const Container& container) {
  return std::find(container.begin(), container.end(), item) != container.end();
}

template <typename T, typename S, typename UnaryPredicate>
std::vector<T> CopyElementIf(const std::set<T, S>& c, UnaryPredicate pred) {
  std::vector<T> out;
  std::copy_if(c.begin(), c.end(), std::back_inserter(out), pred);
  return out;
}

template <typename T, typename GetFunction>
std::vector<const T*> GetPlantAggregate(
    int num_items, GetFunction get_func,
    std::optional<std::vector<ModelInstanceIndex>> model_instances =
        std::nullopt) {
  using IndexType = decltype(std::declval<T>().index());
  std::vector<const T*> items;
  items.reserve(num_items);
  for (IndexType index(0); index < num_items; ++index) {
    auto *item = get_func(index);
    if (!model_instances.has_value() ||
        ContainedIn(item->model_instance(), *model_instances)) {
      items.push_back(item);
    }
  }
  return items;
}
std::vector<const Frame<double>*> GetFrames(
    const MultibodyPlant<double>& plant,
    std::optional<std::vector<ModelInstanceIndex>> model_instances =
        std::nullopt) {
  return GetPlantAggregate<Frame<double>>(
      plant.num_frames(), [&](auto i) { return &plant.get_frame(i); },
      model_instances);
}

std::vector<const Frame<double>*> GetFramesAttachedTo(
    const MultibodyPlant<double>& plant,
    const SortedSet<const Body<double>*>& bodies) {
  auto frames = GetFrames(plant);
  auto new_end = std::remove_if(frames.begin(), frames.end(), [&](auto frame) {
    return bodies.find(&frame->body()) == bodies.end();
  });
  frames.erase(new_end, frames.end());
  return frames;
}

std::vector<const JointActuator<double>*> GetJointActuators(
    const MultibodyPlant<double>& plant,
    std::optional<std::vector<ModelInstanceIndex>> model_instances =
        std::nullopt) {
  return GetPlantAggregate<JointActuator<double>>(
      plant.num_actuators(),
      [&](auto i) { return &plant.get_joint_actuator(i); }, model_instances);
}

bool IsJointSolelyConnectedTo(const Joint<double>* joint,
                              const SortedSet<const Body<double>*>& bodies) {
  auto parent = bodies.find(&joint->parent_body());
  auto child = bodies.find(&joint->child_body());
  return (parent != bodies.end()) && (child != bodies.end());
}
std::vector<const Joint<double>*> GetJointsSolelyConnectedTo(
    const MultibodyPlant<double>& plant,
    const SortedSet<const Body<double>*>& bodies) {
  auto joints = GetJoints(plant);
  auto new_end = std::remove_if(joints.begin(), joints.end(), [&](auto joint) {
    return !IsJointSolelyConnectedTo(joint, bodies);
  });
  joints.erase(new_end, joints.end());
  return joints;
}

std::vector<const JointActuator<double>*> GetJointActuatorsAffectingJoints(
    const MultibodyPlant<double>& plant,
    const SortedSet<const Joint<double>*>& joints) {
  auto joint_actuators = GetJointActuators(plant);
  auto new_end = std::remove_if(
      joint_actuators.begin(), joint_actuators.end(), [&](auto joint_actuator) {
        return joints.find(&joint_actuator->joint()) == joints.end();
      });
  joint_actuators.erase(new_end, joint_actuators.end());
  return joint_actuators;
}

std::vector<geometry::GeometryId> GetGeometries(
    const MultibodyPlant<double>& plant,
    const geometry::SceneGraph<double>& scene_graph,
    const SortedSet<const Body<double>*>& bodies) {
  std::vector<geometry::GeometryId> geometry_ids;
  auto& inspector = scene_graph.model_inspector();
  for (auto* body : bodies) {
    auto frame_id = plant.GetBodyFrameIdOrThrow(body->index());
    auto body_geometry_ids = inspector.GetGeometries(frame_id);
    geometry_ids.insert(
        geometry_ids.end(), body_geometry_ids.begin(),
        body_geometry_ids.end());
  }
  // N.B. `inspector.GetGeometries` returns the ids in a consistent (sorted)
  // order, but we re-sort here just in case the geometries have been mutated.
  std::sort(geometry_ids.begin(), geometry_ids.end());
  return geometry_ids;
}
}  // namespace

class MultibodySubgraphElementAccessor
    : public DefaultElementMutableAccessor<double, double> {
 public:
  MultibodySubgraphElementAccessor(
      MultibodyPlantElementsMap* elements_map,
      ModelInstanceRemapFunction model_instance_remap,
      FrameNameRemapFunction frame_name_remap)
      : DefaultElementMutableAccessor(
            &elements_map->plant_destination()->mutable_tree()),
        elements_map_(elements_map),
        model_instance_remap_(model_instance_remap),
        frame_name_remap_(frame_name_remap) {}

 protected:
  const Frame<double>& DoGetFrame(const Frame<double>& element) const override {
    return *elements_map_->frames().at(&element);
  }
  std::string DoGetFrameNewName(const Frame<double>& element) const override {
    return frame_name_remap_(element);
  }

  const Joint<double>& DoGetJoint(const Joint<double>& element) const override {
    return *elements_map_->joints().at(&element);
  }

  ModelInstanceIndex DoGetFrameNewModelInstance(
      const Frame<double>& element) const override {
    return model_instance_remap_(element.model_instance());
  }

  ModelInstanceIndex DoGetBodyNewModelInstance(
      const Body<double>& element) const override {
    return model_instance_remap_(element.model_instance());
  }

  ModelInstanceIndex DoGetJointNewModelInstance(
      const Joint<double>& element) const override {
    return model_instance_remap_(element.model_instance());
  }

 private:
  MultibodyPlantElementsMap* elements_map_;
  ModelInstanceRemapFunction model_instance_remap_;
  FrameNameRemapFunction frame_name_remap_;
};

std::vector<ModelInstanceIndex> GetModelInstances(
    const MultibodyPlant<double>& plant) {
  std::vector<ModelInstanceIndex> items(plant.num_model_instances());
  std::iota(items.begin(), items.end(), ModelInstanceIndex(0));
  return items;
}

std::vector<const Body<double>*> GetBodies(
    const MultibodyPlant<double>& plant,
    std::optional<std::vector<ModelInstanceIndex>> model_instances) {
  return GetPlantAggregate<Body<double>>(
      plant.num_bodies(), [&](auto i) { return &plant.get_body(i); },
      model_instances);
}

std::vector<const Joint<double>*> GetJoints(
    const MultibodyPlant<double>& plant,
    std::optional<std::vector<ModelInstanceIndex>> model_instances) {
  return GetPlantAggregate<Joint<double>>(
      plant.num_joints(), [&](auto i) { return &plant.get_joint(i); },
      model_instances);
}

Eigen::VectorXd GetJointPositions(const MultibodyPlant<double>& plant,
                                  const systems::Context<double>& context,
                                  const Joint<double>& joint) {
  const Eigen::VectorXd& q = plant.GetPositions(context);
  auto start = joint.position_start();
  auto count = joint.num_positions();
  return q.segment(start, count);
}

void SetJointPositions(MultibodyPlant<double>* plant,
                       systems::Context<double>* context,
                       const Joint<double>& joint,
                       const Eigen::Ref<const VectorX<double>>& qj) {
  Eigen::VectorXd q = plant->GetPositions(*context);
  auto start = joint.position_start();
  auto count = joint.num_positions();
  q.segment(start, count) = qj;
  plant->SetPositions(context, q);
}

Eigen::VectorXd GetJointVelocities(const MultibodyPlant<double>& plant,
                                   const systems::Context<double>& context,
                                   const Joint<double>& joint) {
  const Eigen::VectorXd& q = plant.GetVelocities(context);
  auto start = joint.velocity_start();
  auto count = joint.num_velocities();
  return q.segment(start, count);
}

void SetJointVelocities(MultibodyPlant<double>* plant,
                       systems::Context<double>* context,
                       const Joint<double>& joint,
                       const Eigen::Ref<const VectorX<double>>& vj) {
  Eigen::VectorXd q = plant->GetVelocities(*context);
  auto start = joint.velocity_start();
  auto count = joint.num_velocities();
  q.segment(start, count) = vj;
  plant->SetVelocities(context, q);
}



MultibodyPlantElements MultibodyPlantElements::FromPlant(
    const MultibodyPlant<double>* plant,
    const geometry::SceneGraph<double>* scene_graph) {
  auto model_instances = GetModelInstances(*plant);

  return GetElementsFromBodies(plant, GetBodies(*plant), scene_graph,
                               model_instances);
}

MultibodyPlantElements MultibodyPlantElements::GetElementsFromBodies(
    const MultibodyPlant<double>* plant,
    const std::vector<const Body<double>*>& bodies,
    const geometry::SceneGraph<double>* scene_graph,
    std::optional<std::vector<ModelInstanceIndex>> model_instances) {
  MultibodyPlantElements elem(plant, scene_graph);
  elem.bodies_.insert(bodies.begin(), bodies.end());

  if (!model_instances.has_value()) {
    std::set<ModelInstanceIndex> model_instances_from_bodies;
    for (const auto& body : elem.bodies_) {
      model_instances_from_bodies.insert(body->model_instance());
    }
    elem.model_instances_ = model_instances_from_bodies;
  } else {
    elem.model_instances_.insert(model_instances->begin(),
                                 model_instances->end());
  }

  auto joints = GetJointsSolelyConnectedTo(*plant, elem.bodies_);
  elem.joints_.insert(joints.begin(), joints.end());

  auto joint_actuators = GetJointActuatorsAffectingJoints(*plant, elem.joints_);
  elem.joint_actuators_.insert(joint_actuators.begin(), joint_actuators.end());

  auto frames = GetFramesAttachedTo(*plant, elem.bodies_);
  elem.frames_.insert(frames.begin(), frames.end());

  for (const auto& frame : elem.frames_) {
    elem.model_instances_.insert(frame->model_instance());
  }

  if (scene_graph != nullptr) {
    auto geometries = GetGeometries(*plant, *scene_graph, elem.bodies_);
    elem.geometry_ids_.insert(geometries.begin(), geometries.end());

    elem.collision_filter_pairs_ =
        scene_graph->model_inspector().GetCollisionFilteredPairs();
  }
  return elem;
}

void MultibodyPlantElementsMap::CopyBody(
    const Body<double>* src, const MultibodySubgraphElementAccessor& handle) {
  if (builtins_src_.bodies().find(src) != builtins_src_.bodies().end()) {
    return;
  }
  auto body_src = dynamic_cast<const RigidBody<double>*>(src);
  DRAKE_DEMAND(body_src != nullptr);

  const ModelInstanceIndex model_instance_src = body_src->model_instance();
  const ModelInstanceIndex model_instance_dest =
      model_instances_[model_instance_src];
  DRAKE_DEMAND(model_instance_dest.is_valid());
  std::unique_ptr<Body<double>> body_clone = src->CloneToScalar(handle);
  const Body<double>* body_dest =
      &plant_dest_->AddBody(std::move(body_clone));
  bodies_.insert({body_src, body_dest});

  // Set default state.
  // TODO(azeey) Can we do without setting the pose here?
  const auto X_WB = plant_src_->GetDefaultFreeBodyPose(*body_src);
  plant_dest_->SetDefaultFreeBodyPose(*body_dest, X_WB);
  // Register body frame as a builtin.
  const Frame<double>* frame_src = &body_src->body_frame();
  const Frame<double>* frame_dest = &body_dest->body_frame();
  builtins_src_.frames().insert(frame_src);
  frames_.insert({frame_src, frame_dest});
}

void MultibodyPlantElementsMap::CopyFrame(
    const Frame<double>* src, const MultibodySubgraphElementAccessor& handle) {
  if (builtins_src_.frames().find(src) != builtins_src_.frames().end()) {
    return;
  }
  // BodyFrame's are handled by `CopyBody`, and are ignored by this method.
  DRAKE_DEMAND(dynamic_cast<const BodyFrame<double>*>(src) == nullptr);

  std::unique_ptr<Frame<double>> frame_clone =  src->CloneToScalar(handle);
  const Frame<double>* frame_dest =
      &plant_dest_->AddFrame(std::move(frame_clone));

  frames_.insert({src, frame_dest});
}

void MultibodyPlantElementsMap::CopyJoint(
    const Joint<double>* src, const MultibodySubgraphElementAccessor& handle) {
  const Joint<double>* joint_dest =
      &plant_dest_->AddJoint(src->CloneToScalar(handle));
  joints_.insert({src, joint_dest});
}

void MultibodyPlantElementsMap::CopyJointActuator(
    const JointActuator<double>* src,
    const MultibodySubgraphElementAccessor& handle) {
  const JointActuator<double>* joint_actuator_dest =
      &plant_dest_->AddJointActuator(src->CloneToScalar(handle));
  joint_actuators_.insert({src, joint_actuator_dest});
}

void MultibodyPlantElementsMap::CopyGeometryById(
    const geometry::GeometryId& geometry_id_src) {
  DRAKE_DEMAND(scene_graph_src_ != nullptr);

  const auto& inspector_src = scene_graph_src_->model_inspector();
  const auto frame_id_src = inspector_src.GetFrameId(geometry_id_src);
  const auto* body_src = plant_src_->GetBodyFromFrameId(frame_id_src);
  DRAKE_DEMAND(body_src != nullptr);
  const auto* body_dest = bodies_.at(body_src);
  auto geometry_instance_dest =
      inspector_src.CloneGeometryInstance(geometry_id_src);

  // Use new "scoped" name.
  std::string prefix_src;
  if (body_src->model_instance() != world_model_instance()) {
    prefix_src = plant_src_->GetModelInstanceName(body_src->model_instance());
  }

  std::string prefix_dest;
  if (body_dest->model_instance() != world_model_instance()) {
    prefix_dest =
        plant_dest_->GetModelInstanceName(body_dest->model_instance());
  }

  auto scoped_name_src =
      parsing::ParseScopedName(geometry_instance_dest->name());
  DRAKE_DEMAND(scoped_name_src.instance_name == prefix_src);

  auto scoped_name_dest =
      parsing::PrefixName(prefix_dest, scoped_name_src.name);
  geometry_instance_dest->set_name(scoped_name_dest);

  // TODO(eric.cousineau): How to relax this constraint? How can we
  // register with SceneGraph only?
  // See: https://github.com/RobotLocomotion/drake/issues/13445
  // TODO(eric.cousineau): Try Ale's potential fix here:
  // https://github.com/RobotLocomotion/drake/pull/13371
  geometry::GeometryId geometry_id_dest;
  // Register as normal.
  geometry_id_dest = plant_dest_->RegisterGeometry(
      *body_dest, std::move(geometry_instance_dest));
  geometry_ids_.insert({geometry_id_src, geometry_id_dest});
}

void MultibodyPlantElementsMap::CopyCollisionFilterPair(
    const std::pair<geometry::GeometryId, geometry::GeometryId>& filter_pair) {
  DRAKE_DEMAND(scene_graph_src_ != nullptr);

  auto [id_a, id_b] = filter_pair;
  auto id_a_dest = geometry_ids_[id_a];
  auto id_b_dest = geometry_ids_[id_b];

  plant_dest_->ExcludeCollisionGeometriesWithCollisionFilterGroupPair(
      {"a", geometry::GeometrySet(id_a_dest)},
      {"b", geometry::GeometrySet(id_b_dest)});
}

void MultibodyPlantElementsMap::CopyState(
    const systems::Context<double>& context_src,
    systems::Context<double>* context_dest) {
  for (const auto& [body_src, body_dest] : bodies_) {
    if (body_dest->is_floating()) {
      const auto X_WB = plant_src_->CalcRelativeTransform(
          context_src, plant_src_->world_frame(), body_src->body_frame());
      const auto V_WB =
          plant_src_->EvalBodySpatialVelocityInWorld(context_src, *body_src);
      plant_dest_->SetFreeBodyPose(context_dest, *body_dest, X_WB);
      plant_dest_->SetFreeBodySpatialVelocity(context_dest, *body_dest, V_WB);
    }
  }

  for (const auto& [joint_src, joint_dest] : joints_) {
    if (joint_src->type_name() != joint_dest->type_name()) {
      // This joint may have been welded (and part of a inverse map).
      // Skip.
      continue;
    }
    const auto qj = GetJointPositions(*plant_src_, context_src, *joint_src);
    SetJointPositions(plant_dest_, context_dest, *joint_dest, qj);
    const auto vj = GetJointVelocities(*plant_src_, context_src, *joint_src);
    SetJointVelocities(plant_dest_, context_dest, *joint_dest, vj);
  }
}

template <typename T, typename GetFunction>
void CheckPlantAggregate(GetFunction get_func, const T* item) {
  const T* other_item = get_func(item->index());
  DRAKE_THROW_UNLESS(other_item == item);
}

// Ensures that current elements / topology satisifes subgraph invariants.
void CheckSubgraphInvariants(const MultibodyPlantElements& elem) {
  const auto &plant = *elem.plant();

  auto plant_model_instances = GetModelInstances(plant);
  DRAKE_THROW_UNLESS(std::includes(
      plant_model_instances.begin(), plant_model_instances.end(),
      elem.model_instances().begin(), elem.model_instances().end()));

  // Check bodies.
  for (const auto* body : elem.bodies()) {
    CheckPlantAggregate([&](auto i) { return &plant.get_body(i); }, body);
    DRAKE_THROW_UNLESS(
        ContainedIn(body->model_instance(), elem.model_instances()));
  }

  // Check frames.
  for (const auto* frame : elem.frames()) {
    CheckPlantAggregate([&](auto i) { return &plant.get_frame(i); }, frame);
    DRAKE_THROW_UNLESS(ContainedIn(&frame->body(), elem.bodies()));
    DRAKE_THROW_UNLESS(
        ContainedIn(frame->model_instance(), elem.model_instances()));
  }

  // Check joints.
  for (const auto* joint : elem.joints()) {
    CheckPlantAggregate([&](auto i) { return &plant.get_joint(i); }, joint);
    IsJointSolelyConnectedTo(joint, elem.bodies());
    DRAKE_THROW_UNLESS(
        ContainedIn(joint->model_instance(), elem.model_instances()));
  }

  // Check actuators.
  for (const auto* joint_actuator : elem.joint_actuators()) {
    CheckPlantAggregate([&](auto i) { return &plant.get_joint_actuator(i); },
                        joint_actuator);
    DRAKE_THROW_UNLESS(ContainedIn(&joint_actuator->joint(), elem.joints()));
    DRAKE_THROW_UNLESS(
        ContainedIn(joint_actuator->model_instance(), elem.model_instances()));
  }

  // Check geometries.
  if (elem.scene_graph() != nullptr) {
    DRAKE_THROW_UNLESS(plant.geometry_source_is_registered());
    const auto &inspector = elem.scene_graph()->model_inspector();
    for (const auto geometry_id : elem.geometry_ids()) {
      auto frame_id = inspector.GetFrameId(geometry_id);
      auto body = plant.GetBodyFromFrameId(frame_id);
      DRAKE_THROW_UNLESS(ContainedIn(body, elem.bodies()));
    }
  } else {
    DRAKE_THROW_UNLESS(elem.geometry_ids().empty());
  }
}

ModelInstanceIndex GetOrCreateModelInstanceByName(
    MultibodyPlant<double>* plant, const std::string& model_name) {
  if (plant->HasModelInstanceNamed(model_name)) {
    return plant->GetModelInstanceByName(model_name);
  }
  return plant->AddModelInstance(model_name);
}

std::string FrameNameRenameSameName(const Frame<double>& frame) {
  return frame.name();
}

MultibodyPlantElementsMap MultibodyPlantSubgraph::AddTo(
    MultibodyPlant<double>* plant_dest,
    std::optional<ModelInstanceRemapFunction> opt_model_instance_remap,
    std::optional<FrameNameRemapFunction> opt_frame_name_remap) const {
  const auto* plant_src = elem_src_.plant();
  const auto* scene_graph_src = elem_src_.scene_graph();

  ModelInstanceRemapFunction model_instance_remap =
      opt_model_instance_remap.value_or(
          [&](ModelInstanceIndex model_instance_src) {
            const std::string name =
                plant_src->GetModelInstanceName(model_instance_src);
            return GetOrCreateModelInstanceByName(plant_dest, name);
          });

  FrameNameRemapFunction frame_name_remap =
      opt_frame_name_remap.value_or(FrameNameRenameSameName);

  MultibodyPlantElementsMap src_to_dest(plant_src, plant_dest, scene_graph_src);
  MultibodySubgraphElementAccessor handle(&src_to_dest, model_instance_remap,
                                          frame_name_remap);

  // Remap and register model instances.
  for (const auto& model_instance_src : elem_src_.model_instances()) {
    // auto model_instance_dest =
        // model_instance_remap(*plant_src, model_instance_src, plant_dest);
    auto model_instance_dest =
        model_instance_remap(model_instance_src);
    src_to_dest.RegisterModelInstance(model_instance_src, model_instance_dest);
  }

  // Register world body and frame if we're using that source model
  // instance.
  if (src_to_dest.model_instances().find(world_model_instance()) !=
      src_to_dest.model_instances().end()) {
    src_to_dest.RegisterWorldBodyAndFrame();
  }

  // Copy bodies
  for (const auto& body_src : elem_src_.bodies()) {
    src_to_dest.CopyBody(body_src, handle);
  }

  // Copy frames
  for (const auto& frame_src : elem_src_.frames()) {
    src_to_dest.CopyFrame(frame_src, handle);
  }

  for (const auto& joint_src : elem_src_.joints()) {
    src_to_dest.CopyJoint(joint_src, handle);
  }

  for (const auto& joint_actuator_src : elem_src_.joint_actuators()) {
    src_to_dest.CopyJointActuator(joint_actuator_src, handle);
  }

  // Copy geometries (if applicable)
  if (plant_dest->geometry_source_is_registered()) {
    for (const auto& geometry_id_src : elem_src_.geometry_ids()) {
      src_to_dest.CopyGeometryById(geometry_id_src);
    }

    for (const auto& filter_pair_src : elem_src_.collision_filter_pairs()) {
      src_to_dest.CopyCollisionFilterPair(filter_pair_src);
    }
  }

  // TODO(azeey) Apply policies
  return src_to_dest;
}

MultibodyPlantElementsMap MultibodyPlantSubgraph::AddTo(
    MultibodyPlant<double>* plant_dest, const std::string& model_instance_remap,
    FrameNameRemapFunction frame_name_remap) const {
  auto model_name_remap = [&model_instance_remap, plant_dest](auto) {
    return GetOrCreateModelInstanceByName(plant_dest, model_instance_remap);
  };
  return AddTo(plant_dest, model_name_remap, frame_name_remap);
}

MultibodyPlantElements MultibodyPlantSubgraph::MakeEmptyElements() {
  return MultibodyPlantElements(elem_src_.plant(), elem_src_.scene_graph());
}

MultibodyPlantElements MultibodyPlantSubgraph::RemoveModelInstance(
    ModelInstanceIndex model_instance) {
  MultibodyPlantElements elem_removed = MakeEmptyElements();
  elem_src_.model_instances().erase(model_instance);
  elem_removed.model_instances().insert(model_instance);
  for (const auto& body : GetBodies(*elem_src_.plant())) {
    if (body->model_instance() == model_instance) {
      elem_removed += RemoveBody(body);
    }
  }
  return elem_removed;
}

MultibodyPlantElements MultibodyPlantSubgraph::RemoveBody(
    const Body<double>* body, bool include_welded_bodies) {
  MultibodyPlantElements elem_removed = MakeEmptyElements();
  elem_src_.bodies().erase(body);
  elem_removed.bodies().insert(body);
  const auto frames = CopyElementIf(
      elem_src_.frames(), [&](auto& frame) { return &frame->body() == body; });

  for (const auto& frame : frames) {
    elem_removed += RemoveFrame(frame);
  }

  const auto joints = CopyElementIf(elem_src_.joints(), [&](auto& joint) {
    return &joint->parent_body() == body || &joint->child_body() == body;
  });
  for (const auto& joint : joints) {
    elem_removed += RemoveJoint(joint);
  }

  if (elem_src_.scene_graph() != nullptr) {
    const auto geometry_ids =
        GetGeometries(*elem_src_.plant(), *elem_src_.scene_graph(), {body});
    for (const auto& x : geometry_ids) {
      elem_removed += RemoveGeometryId(x);
    }
  }
  if (include_welded_bodies) {
    for (const auto& welded_body :
         elem_src_.plant()->GetBodiesWeldedTo(*body)) {
      if (ContainedIn(welded_body, elem_src_.bodies())) {
        elem_removed += RemoveBody(welded_body);
      }
    }
  }
  return elem_removed;
}

MultibodyPlantElements MultibodyPlantSubgraph::RemoveFrame(
    const Frame<double>* frame) {
  // N.B. Since most elements interface with bodies, not frames, then it is OK
  // to remove a frame and not really have other dependent elements.
  MultibodyPlantElements elem_removed = MakeEmptyElements();
  elem_src_.frames().erase(frame);
  elem_removed.frames().insert(frame);
  return elem_removed;
}

MultibodyPlantElements MultibodyPlantSubgraph::RemoveJoint(
    const Joint<double>* joint) {
  MultibodyPlantElements elem_removed = MakeEmptyElements();
  elem_src_.joints().erase(joint);
  elem_removed.joints().insert(joint);

  const auto joint_actuators =
      CopyElementIf(elem_src_.joint_actuators(),
                    [&](auto& x) { return &x->joint() == joint; });

  for (const auto x : joint_actuators) {
    elem_removed += RemoveJointActuator(x);
  }
  return elem_removed;
}

MultibodyPlantElements MultibodyPlantSubgraph::RemoveJointActuator(
    const JointActuator<double>* joint_actuator) {
  MultibodyPlantElements elem_removed = MakeEmptyElements();
  elem_src_.joint_actuators().erase(joint_actuator);
  elem_removed.joint_actuators().insert(joint_actuator);
  return elem_removed;
}

MultibodyPlantElements MultibodyPlantSubgraph::RemoveGeometryId(
    geometry::GeometryId geometry_id) {
  MultibodyPlantElements elem_removed = MakeEmptyElements();
  elem_src_.geometry_ids().erase(geometry_id);
  elem_removed.geometry_ids().insert(geometry_id);
  return elem_removed;
}
}  // namespace internal
}  // namespace multibody
}  // namespace drake
