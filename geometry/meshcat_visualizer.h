#pragma once

#include <map>
#include <memory>
#include <optional>
#include <string>

#include "drake/geometry/geometry_roles.h"
#include "drake/geometry/meshcat.h"
#include "drake/geometry/meshcat_animation.h"
#include "drake/geometry/meshcat_visualizer_params.h"
#include "drake/geometry/rgba.h"
#include "drake/geometry/scene_graph.h"
#include "drake/systems/analysis/instantaneous_realtime_rate_calculator.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace geometry {

/** A system wrapper for Meshcat that publishes the current state of a
SceneGraph instance (whose QueryObject-valued output port is connected to this
system's input port).  While this system will add geometry to Meshcat, the
Meshcat instance is also available for users to add their own visualization
alongside the MeshcatVisualizer visualizations.  This can be enormously valuable
for impromptu visualizations.

 @system
 name: MeshcatVisualizer
 input_ports:
 - query_object
 @endsystem

The system uses the versioning mechanism provided by SceneGraph to detect
changes to the geometry so that a change in SceneGraph's data will propagate
to Meshcat.

By default, %MeshcatVisualizer visualizes geometries with the illustration role
(see @ref geometry_roles for more details). It can be configured to visualize
geometries with other roles. Only one role can be specified.  See
DrakeVisualizer which uses the same mechanisms for more details.

@warning MeshcatVisualizer does not support Context-per-thread parallelism.
This is because of limitations in both Meshcat and MeshcatVisualizer.
We may generalize this in the future if Meshcat limitations are removed.

Instances of %MeshcatVisualizer created by scalar-conversion will publish to the
same Meshcat instance.
@tparam_nonsymbolic_scalar
*/
template <typename T>
class MeshcatVisualizer final : public systems::LeafSystem<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(MeshcatVisualizer)

  /** Creates an instance of %MeshcatVisualizer.

   @param meshcat A Meshcat instance.  This class will assume shared ownership
                  for the lifetime of the object.
   @param params  The set of parameters to control this system's behavior.
   @throws std::exception if `params.publish_period <= 0`.
   @throws std::exception if `params.role == Role::kUnassigned`. */
  explicit MeshcatVisualizer(std::shared_ptr<Meshcat> meshcat,
                             MeshcatVisualizerParams params = {});

  /** Scalar-converting copy constructor. See @ref system_scalar_conversion.
   It should only be used to convert _from_ double _to_ other scalar types.
   */
  template <typename U>
  explicit MeshcatVisualizer(const MeshcatVisualizer<U>& other);

  /** Resets the realtime rate calculator. Calculation will resume on the next
   periodic publish event. This is useful for correcting the realtime rate after
   simulation is resumed from a paused state, etc. */
  void ResetRealtimeRateCalculator() const {
    realtime_rate_calculator_.Reset();
  }

  /** Calls Meshcat::Delete(std::string path), with the path set to
   MeshcatVisualizerParams::prefix.  Since this visualizer will only ever add
   geometry under this prefix, this will remove all geometry/transforms added
   by the visualizer, or by a previous instance of this visualizer using the
   same prefix.  Use MeshcatVisualizer::delete_on_initialization_event
   to determine whether this should be called on initialization. */
  void Delete() const;

  /** Sets a flag indicating that subsequent publish events should also be
  "recorded" into a MeshcatAnimation.  The data in these events will be
  combined with any frames previously added to the animation; if the same
  transform/property is set at the same time, then it will overwrite the
  existing frame in the animation.  Frames are added at the index
  MeshcatAnimation::frame(context.get_time()).

  @param set_transforms_while_recording if true, then each Publish will set the
  transform in Meshcat *and* record the transform in the animation.  Set to
  false to avoid updating the visualization during recording.  Note that
  animations do not support SetObject, so the objects must still be sent to the
  visualizer during the recording.

  @returns a mutable pointer to the current recording.  See
  get_mutable_recording().
  */
  MeshcatAnimation* StartRecording(bool set_transforms_while_recording = true);

  /** Sets a flag to pause/stop recording.  When stopped, publish events will
  not add frames to the animation. */
  void StopRecording();

  /** Sends the recording to Meshcat as an animation. The published animation
  only includes transforms and properties; the objects that they modify must be
  sent to the visualizer separately (e.g. by calling Publish()). */
  void PublishRecording() const;

  /** Deletes the current animation holding the recorded frames.  Animation
  options (autoplay, repetitions, etc) will also be reset, and any pointers
  obtained from get_mutable_recording() will be rendered invalid. This does
  *not* currently remove the animation from Meshcat. */
  void DeleteRecording();

  /** Returns a mutable pointer to this MeshcatVisualizer's unique
  MeshcatAnimation object in which the frames will be recorded. This pointer
  can be used to set animation properties (like autoplay, the loop mode, number
  of repetitions, etc), and can be passed to supporting visualizers (e.g.
  MeshcatPointCloudVisualizer and MeshcatContactVisualizer) so that they record
  into the same animation.

  The MeshcatAnimation object will only remain valid for the lifetime of `this`
  or until DeleteRecording() is called.

  @throws std::exception if meshcat does not have a recording.
  */
  MeshcatAnimation* get_mutable_recording();

  /** Returns the QueryObject-valued input port. It should be connected to
   SceneGraph's QueryObject-valued output port. Failure to do so will cause a
   runtime error when attempting to broadcast messages. */
  const systems::InputPort<T>& query_object_input_port() const {
    return this->get_input_port(query_object_input_port_);
  }

  /** Adds a MeshcatVisualizer and connects it to the given SceneGraph's
   QueryObject-valued output port. See
   MeshcatVisualizer::MeshcatVisualizer(MeshcatVisualizer*,
   MeshcatVisualizerParams) for details.
   The %MeshcatVisualizer's name (see systems::SystemBase::set_name) will be set
   to a sensible default value, unless the default name was already in use by
   another system. */
  static MeshcatVisualizer<T>& AddToBuilder(
      systems::DiagramBuilder<T>* builder, const SceneGraph<T>& scene_graph,
      std::shared_ptr<Meshcat> meshcat, MeshcatVisualizerParams params = {});

  /** Adds a MeshcatVisualizer and connects it to the given QueryObject-valued
   output port. See MeshcatVisualizer::MeshcatVisualizer(MeshcatVisualizer*,
   MeshcatVisualizerParams) for details.
   The %MeshcatVisualizer's name (see systems::SystemBase::set_name) will be set
   to a sensible default value, unless the default name was already in use by
   another system. */
  static MeshcatVisualizer<T>& AddToBuilder(
      systems::DiagramBuilder<T>* builder,
      const systems::OutputPort<T>& query_object_port,
      std::shared_ptr<Meshcat> meshcat, MeshcatVisualizerParams params = {});

 private:
  /* MeshcatVisualizer of different scalar types can all access each other's
   data. */
  template <typename>
  friend class MeshcatVisualizer;

  /* The periodic event handler. It tests to see if the last scene description
   is valid (if not, sends the objects) and then sends the transforms.  */
  systems::EventStatus UpdateMeshcat(const systems::Context<T>& context) const;

  /* Makes calls to Meshcat::SetObject to register geometry in SceneGraph. */
  void SetObjects(const SceneGraphInspector<T>& inspector) const;

  /* Makes calls to Meshcat::SetTransform to update the poses from SceneGraph.
   */
  void SetTransforms(const systems::Context<T>& context,
                     const QueryObject<T>& query_object) const;

  /* Makes calls to Meshcat::SetProperty to update geometry alphas. During
   initialization, it is necessary to explicitly configure each geometry
   individually due to race conditions between declaring the geometry and
   configuring it. Once the geometry is loaded, they can be updated en masse. */
  void SetAlphas(bool initializing) const;

  /* Handles the initialization event. */
  systems::EventStatus OnInitialization(const systems::Context<T>&) const;

  typename systems::LeafSystem<T>::GraphvizFragment DoGetGraphvizFragment(
      const typename systems::LeafSystem<T>::GraphvizFragmentParams& params)
      const final;

  /* The index of this System's QueryObject-valued input port. */
  int query_object_input_port_{};

  /* Meshcat is mutable because we must send messages (a non-const operation)
   from a const System (e.g. during simulation).  We use shared_ptr instead of
   unique_ptr to facilitate sharing ownership through scalar conversion;
   creating a new Meshcat object during the conversion is not a viable option.
   */
  mutable std::shared_ptr<Meshcat> meshcat_{};

  /* The version of the geometry that was last set in Meshcat by this
   instance. Because the underlying Meshcat is shared, this visualizer has no
   guarantees that the Meshcat state correlates with this value. If the version
   found on the input port differs from this value, SetObjects is called again
   before SetTransforms. This is intended to track the information in meshcat_,
   and is therefore also a mutable member variable (instead of declared state).
   */
  mutable std::optional<GeometryVersion> version_;

  /* A store of the dynamic frames and their path. It is coupled with the
   version_.  This is only for efficiency; it does not represent undeclared
   state. */
  mutable std::map<FrameId, std::string> dynamic_frames_{};

  /* A store of the geometries sent to Meshcat, so that they can be removed if a
   new geometry version appears that does not contain them. */
  mutable std::map<GeometryId, std::string> geometries_{};

  /* The last alpha value applied to the objects in geometries_; used to avoid
   unnecessary updates to geometry opacities. */
  mutable double alpha_value_{1.0};

  /* The parameters for the visualizer.  */
  MeshcatVisualizerParams params_;

  /* TODO(#16486): ideally this mutable state will go away once it is safe to
  run Meshcat multithreaded */
  mutable systems::internal::InstantaneousRealtimeRateCalculator
      realtime_rate_calculator_{};

  /* The name of the alpha slider, if any. */
  std::string alpha_slider_name_;
};

/** A convenient alias for the MeshcatVisualizer class when using the `double`
scalar type. */
using MeshcatVisualizerd = MeshcatVisualizer<double>;

}  // namespace geometry

// Define the conversion trait to *only* allow double -> AutoDiffXd conversion.
// Symbolic is not supported yet, and AutoDiff -> double doesn't "make sense".
namespace systems {
namespace scalar_conversion {
template <>
struct Traits<geometry::MeshcatVisualizer> : public NonSymbolicTraits {};
}  // namespace scalar_conversion
}  // namespace systems

}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::geometry::MeshcatVisualizer)
