#pragma once

#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/geometry/frame_kinematics_vector.h"
#include "drake/geometry/geometry_ids.h"
#include "drake/geometry/render/render_label.h"
#include "drake/geometry/scene_graph.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace systems {

/** This class provides backwards compatibility between the old
 @ref systems::RigidBodyPlant "RigidBodyPlant" and the new SceneGraph. It
 consumes the plant's RigidBodyTree and, serving as a geometry source, registers
 the frames and geometries into SceneGraph. It is then responsible for
 updating the frame poses in SceneGraph by querying the RigidBodyPlant for
 its state and using the stored RigidBodyTree to evaluate kinematics values.

 <H2>Current functionality is limited in several ways</H2>

 <H3>What geometry types are read and what roles can they be used for?</H3>

 A RigidBody can have "visual" elements and "collision" elements. Which one is
 read and used in SceneGraph is indicated by the cell value (in the table
 below):

    - C : Collision element's geometry is used
    - V : Visual element's geometry is used (but without material values)
    - . : No geometry is used, shapes of this type are *ignored*

 The columns of the table indicate the SceneGraph roles.

   - Proximity: The shape is used in proximity queries (e.g., penetration,
                distance, ray-casting, etc.)
   - Visual:    The shape is displayed in drake_visualizer.
   - Render:    The shape is used in rendering queries (i.e., RGB images,
                depth images, label images, etc.)

  %Shape      | Proximity | Visual | Render
  ------------|:---------:|:------:|:------:
  %Box        | .         | .      | .
  %Capsule    | .         | .      | .
  %Cylinder   | V         | V      | .
  %Mesh       | .         | V      | .
  %MeshPoints | .         | .      | .
  %Plane      | V         | V      | .
  %Sphere     | V         | V      | .

 <h4>Table: Level of Support. Indication of what types of shapes (rows) are read
 from the RigidBodyTree, the role they played in the RigidBodyTree, collision or
 visual, (cell values "C", "V", or ".") and how they are used in SceneGraph
 roles (columns).</h4>

 The table maps a geometry instance in RigidBodyTree to the effect it has in
 SceneGraph. Things of particular note:

 1. %Box, %Capsule, and %MeshPoints are not supported at all. If found in the
    RigidBodyTree, as visual _or_ collision elements, they will be ignored.
 2. Collision elements are all completely ignored (no cell has the value "C").
    For those shapes that *are* supported, only the visual elements are
    included.
 3. The implication of the previous point is that for those shapes that are
    supported by the proximity queries, the _visual_ geometry will be used and
    not the declared collision geometry.
 4. Meshes are passed on to drake visualizer, but do not contribute to any
    queries.

 <H3>Distinction between anchored and dynamic geometry</H3>

 Currently, no distinction is made. All bodies are treated as "dynamic" and the
 geometry associated with the body is likewise dynamic. This *may* lead to
 errors in collision queries; two geometries attached to bodies rigidly fixed
 to the ground would be considered anchored and any penetration between them is
 ignored. However, in this case, that penetration will be reported. It can also
 contribute to some reduced efficiency if there are many anchored geometries.

 <H3>Geometry assigned to the world body is ignored</H3>

 Any geometries associated directly with the world body will not be registered.

 @tparam T The scalar type. Must be a valid Eigen scalar.

 Instantiated templates for the following kinds of T's are provided:

 - double
 - AutoDiffXd

 They are already available to link against in the containing library.
 No other values for T are currently supported.

 Because a RigidBodyPlantBridge contains a pointer to a RigidBodyTree, it cannot
 be _converted_ from one scalar type to another. It must be re-created using the
 SceneGraph and RigidBodyTree instantiated on the new scalar type. */
template <typename T>
class RigidBodyPlantBridge : public systems::LeafSystem<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(RigidBodyPlantBridge)

  using MyContext = systems::Context<T>;

  /** Constructor; reads the bodies and geometries in the rigid body tree and
   registers them with the given geometry system. Ultimately, this system must
   have its ports connected to the *same* geometry system and the RigidBodyPlant
   which owns the rigid body tree.

   `rigid_body_tree` is aliased internally; its life span must be longer than
   this. */
  RigidBodyPlantBridge(const RigidBodyTree<T>* rigid_body_tree,
                       geometry::SceneGraph<T>* scene_graph);

  ~RigidBodyPlantBridge() override {}

  geometry::SourceId source_id() const { return source_id_; }

  const systems::OutputPort<T>& geometry_pose_output_port() const;
  const systems::InputPort<T>& rigid_body_plant_state_input_port()
      const;

  /** Given a render label, reports the index of the RigidBody to which this
   label was assigned. The RenderLabel::kDontCare label maps to the index of the
   world body. The other reserved labels return a negative value.

   @throws std::logic_error If the label doesn't otherwise specify a body.  */
  int BodyForLabel(geometry::render::RenderLabel label) const;

  /** Reports the frame id for the given body. */
  geometry::FrameId FrameIdFromBody(const RigidBody<T>& body) const {
    return body_ids_[body.get_body_index()];
  }

 private:
  // Registers `this` system's tree's bodies and geometries to the given
  // geometry system.
  void RegisterTree(geometry::SceneGraph<T>* scene_graph);

  // Calculate the frame pose set output port value.
  void CalcFramePoseOutput(const MyContext& context,
                           geometry::FramePoseVector<T>* poses) const;

  // The tree used to populate SceneGraph and evaluate body kinematics.
  const RigidBodyTree<T>* const tree_{nullptr};

  // This system's source id with SceneGraph.
  geometry::SourceId source_id_;

  // Port handles
  int geometry_pose_port_{-1};
  int plant_state_port_{-1};

  // Maps from a generated render label to the index of the rigid body assigned
  // that label.
  std::unordered_map<geometry::render::RenderLabel, int> label_to_index_;

  // Registered frames. In this incarnation, body i's frame_id is stored in
  // element i. This is because *all* frames are currently being registered
  // (regardless of weldedness or whether it has geometry).
  std::vector<geometry::FrameId> body_ids_;
};
}  // namespace systems
}  // namespace drake
