#pragma once

#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/geometry/frame_kinematics_vector.h"
#include "drake/geometry/geometry_ids.h"
#include "drake/geometry/geometry_system.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace geometry {

/** This class provides backwards compatibility between the old
 @ref systems::RigidBodyPlant "RigidBodyPlant" and the new GeometrySystem. It
 consumes the plant's RigidBodyTree and, serving as a geometry source, registers
 the frames and geometries into GeometrySystem. It is then responsible for
 updating the frame poses in GeometrySystem by querying the RigidBodyPlant for
 its state and using the stored RigidBodyTree to evaluate kinematics values.

 <H2>Current functionality is limited in several ways</H2>

 <H3>What geometry types are read and what roles can they be used for?</H3>

  %Shape      | Proximity | Visual | Render
  ------------|:---------:|:------:|:------:
  %Box        | .         | .      | .
  %Capsule    | .         | .      | .
  %Cylinder   | V         | V      | .
  %Mesh       | .         | V      | .
  %MeshPoints | .         | .      | .
  %Plane      | V         | V      | .
  %Sphere     | V         | V      | .
<h4>Table: Level of Support. Indication of what types of shapes are read from
 the RigidBodyTree, what their origin is (collision or visual) and how they are
 used in GeometrySystem roles.</h4>

 Geometry can play two roles in a RigidBody: collision and visualization.
 The entry value indicates which RigidBody geometry role is used.

    - C : Collision geometry used
    - V : Visual geometry used (but without material values)
    - . : No geometry used, shapes of this type are *ignored*

 The columns indicate the GeometrySystem roles.
   - Proximity: The shape is used in proximity queries (e.g., penetration,
                distance, ray-casting, etc.)
   - Visual:    The shape is displayed in drake_visualizer.
   - Render:    The shape is used in rendering queries (i.e., RGB images,
                depth images, label images, etc.)


 @warning In the current version, collision geometry is ignored. So, if you
 perform penetration queries, you will be performing them on the visual
 elements.

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
 GeometrySystem and RigidBodyTree instantiated on the new scalar type. */
template <typename T>
class RigidBodyPlantBridge : public systems::LeafSystem<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(RigidBodyPlantBridge)

  using MyContext = systems::Context<T>;

  /** Constructor; reads the bodies and geometries in the rigid body tree and
   registers them with the given geometry system. Ultimately, this system must
   have its ports connected to the *same* geometry system and the RigidBodyPlant
   which owns the rigid body tree.   */
  RigidBodyPlantBridge(const RigidBodyTree<T>* rigid_body_tree,
                       GeometrySystem<T>* geometry_system);

  ~RigidBodyPlantBridge() override {}

  SourceId source_id() const { return source_id_; }

  const systems::OutputPort<T>& geometry_id_output_port() const;
  const systems::OutputPort<T>& geometry_pose_output_port() const;
  const systems::InputPortDescriptor<T>& rigid_body_plant_state_input_port()
      const;

 private:
  // Registers `this` system's tree's bodies and geometries  to the given
  // geometry system.
  void RegisterTree(GeometrySystem<T>* geometry_system);

  // This is nothing _but_ direct-feedthrough.
  optional<bool> DoHasDirectFeedthrough(int, int) const override {
    return true;
  }

  // Allocate the frame pose set output port value.
  FramePoseVector<T> AllocateFramePoseOutput(const MyContext& context) const;

  // Calculate the frame pose set output port value.
  void CalcFramePoseOutput(const MyContext& context,
                           FramePoseVector<T>* poses) const;

  // Allocate the id output.
  FrameIdVector AllocateFrameIdOutput(const MyContext& context) const;
  // Calculate the id output.
  void CalcFrameIdOutput(const MyContext& context, FrameIdVector* id_set) const;

  // The tree used to populate GeometrySystem and evaluate body kinematics.
  const RigidBodyTree<T>& tree_{};

  // This system's source id with GeometrySystem.
  SourceId source_id_;

  // Port handles
  int geometry_id_port_{-1};
  int geometry_pose_port_{-1};

  // Registered data
  std::vector<FrameId> body_ids_;
};
}  // namespace geometry
}  // namespace drake
