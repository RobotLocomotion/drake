#include "drake/multibody/rigid_body_plant/rigid_body_plant_bridge.h"

#include <memory>
#include <string>
#include <utility>

#include "drake/common/drake_assert.h"
#include "drake/geometry/geometry_frame.h"
#include "drake/geometry/geometry_ids.h"
#include "drake/geometry/geometry_instance.h"
#include "drake/geometry/geometry_roles.h"
#include "drake/geometry/geometry_visualization.h"
#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/framework_common.h"

namespace drake {
namespace systems {

using geometry::Box;
using geometry::Cylinder;
using geometry::FrameId;
using geometry::FramePoseVector;
using geometry::GeometryFrame;
using geometry::GeometryId;
using geometry::GeometryInstance;
using geometry::IllustrationProperties;
using geometry::MakeDrakeVisualizerProperties;
using geometry::Mesh;
using geometry::ProximityProperties;
using geometry::SceneGraph;
using geometry::Shape;
using geometry::Sphere;

namespace {

// Utility function for extracting a shape from a visual element.
std::unique_ptr<Shape> ShapeFromElement(
    const DrakeShapes::Element& element) {
  using std::make_unique;

  std::unique_ptr<Shape> shape{nullptr};

  const DrakeShapes::Geometry& geometry = element.getGeometry();
  switch (element.getShape()) {
    case DrakeShapes::BOX: {
      const auto& box = dynamic_cast<const DrakeShapes::Box&>(geometry);
      shape = make_unique<Box>(box.size(0), box.size(1), box.size(2));
      break;
    }
    case DrakeShapes::SPHERE: {
      const auto& sphere =
          dynamic_cast<const DrakeShapes::Sphere&>(geometry);
      shape = make_unique<Sphere>(sphere.radius);
      break;
    }
    case DrakeShapes::CYLINDER: {
      const auto& cylinder =
          dynamic_cast<const DrakeShapes::Cylinder&>(geometry);
      shape = make_unique<Cylinder>(cylinder.radius, cylinder.length);
      break;
    }
    case DrakeShapes::MESH: {
      const auto& mesh = dynamic_cast<const DrakeShapes::Mesh&>(geometry);
      if (mesh.uri_.find("package://") == 0) {
        shape = make_unique<Mesh>(mesh.uri_);
      } else {
        shape = make_unique<Mesh>(mesh.resolved_filename_);
      }
      break;
    }
    default:
      drake::log()->warn("Only spheres, cylinders, boxes, and (limited) meshes"
                             "are supported by RigidBodyPlantBridge");
  }
  return shape;
}

}  // namespace

template <typename T>
RigidBodyPlantBridge<T>::RigidBodyPlantBridge(const RigidBodyTree<T>* tree,
                                              SceneGraph<T>* scene_graph)
    : tree_(tree) {
  DRAKE_THROW_UNLESS(tree_ != nullptr);
  DRAKE_THROW_UNLESS(scene_graph != nullptr);
  source_id_ = scene_graph->RegisterSource(this->get_name());

  // Declare the tree's pose input port -- don't need the index, it is always 0.
  const int vector_size =
      tree->get_num_positions() + tree->get_num_velocities();
  plant_state_port_ =
      this->DeclareInputPort(kVectorValued, vector_size).get_index();
  RegisterTree(scene_graph);

  // Now that the frames have been registered, instantiate the output port.
  // The first body id belongs to the world; we skip it by starting with the
  // second body id.
  std::vector<FrameId> dynamic_frames(body_ids_.begin() + 1, body_ids_.end());
  geometry_pose_port_ = this->DeclareAbstractOutputPort(
          FramePoseVector<T>(source_id_, dynamic_frames),
          &RigidBodyPlantBridge::CalcFramePoseOutput)
      .get_index();
}

template <typename T>
const OutputPort<T>& RigidBodyPlantBridge<T>::geometry_pose_output_port()
    const {
  return this->get_output_port(geometry_pose_port_);
}

template <typename T>
const InputPort<T>&
RigidBodyPlantBridge<T>::rigid_body_plant_state_input_port() const {
  return this->get_input_port(plant_state_port_);
}

template <typename T>
void RigidBodyPlantBridge<T>::RegisterTree(SceneGraph<T>* scene_graph) {
  // TODO(SeanCurtis-TRI): This treats all bodies in the tree as dynamic. Some
  // may be fixed to the world. In that case, the bodies should *not* be
  // registered, and the geometries should be registered as anchored.
  // This *may* lead to a correctness issues. If two bodies are both rigidly
  // fixed, then their geometries should *implicitly* not be considered for
  // collision. However, if both are included as dynamic bodies, penetrations
  // will be reported.

  using std::make_unique;

  // Process geometries attached to *all* bodies.
  body_ids_.reserve(tree_->get_bodies().size());
  // Iterate through unique pointers to bodies.
  for (const auto& body_ptr : tree_->get_bodies()) {
    const RigidBody<T>& body = *body_ptr;
    // TODO(SeanCurtis-TRI): Possibly account for the fact that some frames
    // may be rigidly affixed to other frames or frames without geometry
    // likewise wouldn't be registered. NOTE: We're registering a frame even
    // if the body has no geometry.

    // Default to the world body configuration.
    FrameId body_id = scene_graph->world_frame_id();
    if (body.get_body_index() != tree_->world().get_body_index()) {
      // All other bodies register a frame and (possibly) get a unique label.
      body_id = scene_graph->RegisterFrame(
          source_id_,
          GeometryFrame(body.get_name(), Isometry3<double>::Identity(),
                        body.get_model_instance_id()));
    }
    body_ids_.push_back(body_id);

    // TODO(SeanCurtis-TRI): Detect if equivalent shapes are used for visual
    // and collision and then simply assign it additional roles. This is an
    // optimization.
    int visual_count = 0;
    for (const auto& visual_element : body.get_visual_elements()) {
      std::unique_ptr<Shape> shape = ShapeFromElement(visual_element);
      if (shape) {
        const std::string name = "visual_" + std::to_string(visual_count++);
        Isometry3<double> X_FG = visual_element.getLocalTransform();
        GeometryId id = scene_graph->RegisterGeometry(
            source_id_, body_id,
            std::make_unique<GeometryInstance>(X_FG, std::move(shape), name));

        // Illustration properties -- simply pass the diffuse along.
        const Vector4<double>& diffuse = visual_element.getMaterial();
        scene_graph->AssignRole(source_id_, id,
                                MakeDrakeVisualizerProperties(diffuse));
      }
    }
    int collision_count = 0;
    for (const auto& collide_element_id : body.get_collision_element_ids()) {
      const multibody::collision::Element* collision_element =
          tree_->FindCollisionElement(collide_element_id);
      std::unique_ptr<Shape> shape = ShapeFromElement(*collision_element);
      if (shape) {
        const std::string name = "collision_" +
            std::to_string(collision_count++);
        Isometry3<double> X_FG = collision_element->getLocalTransform();
        GeometryId id = scene_graph->RegisterGeometry(
            source_id_, body_id,
            std::make_unique<GeometryInstance>(X_FG, std::move(shape), name));
        // TODO(SeanCurtis-TRI): Populate contact material from the element's
        // CompliantMaterial.
        scene_graph->AssignRole(source_id_, id, ProximityProperties());
      }
    }
  }
}

template <typename T>
void RigidBodyPlantBridge<T>::CalcFramePoseOutput(
    const MyContext& context, FramePoseVector<T>* poses) const {
  DRAKE_DEMAND(source_id_.is_valid());
  DRAKE_DEMAND(poses->size() == static_cast<int>(body_ids_.size() - 1));

  const BasicVector<T>& input_vector = *this->EvalVectorInput(context, 0);
  // Obtains the generalized positions from vector_base.
  const VectorX<T> q = input_vector.CopyToVector().head(
      tree_->get_num_positions());

  // Computes the poses of each body.
  KinematicsCache<T> cache = tree_->doKinematics(q);

  // Saves the poses of each body in the frame pose vector
  // NOTE: Body 0 is the world; we skip it.
  // TODO(SeanCurtis-TRI): When I start skipping rigidly fixed bodies, modify
  // this loop to account for them.
  const int world_body = 0;
  poses->clear();
  // NOTE: This relies on the definition that body i has its frame id at i - 1.
  // When we start skipping welded frames, or frames without geometry, this
  // mapping won't be so trivial.
  for (size_t i = 1; i < tree_->get_bodies().size(); ++i) {
    poses->set_value(body_ids_[i],
                     tree_->relativeTransform(cache, world_body, i));
  }
}

// Explicitly instantiates on the most common scalar types.
template class RigidBodyPlantBridge<double>;

}  // namespace systems
}  // namespace drake
