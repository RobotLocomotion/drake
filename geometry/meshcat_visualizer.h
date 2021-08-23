#pragma once

#include <map>
#include <memory>
#include <string>

#include "drake/geometry/geometry_roles.h"
#include "drake/geometry/meshcat.h"
#include "drake/geometry/rgba.h"
#include "drake/geometry/scene_graph.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace geometry {

/** The set of parameters for configuring MeshcatVisualizer.  */
struct MeshcatVisualizerParams {
  /** The duration (in seconds) between calls to Meshcat::SetTransform() that
   update the poses of the scene's geometry.  */
  double publish_period{1 / 60.0};

  /** The role of the geometries to be sent to the visualizer.  */
  Role role{Role::kIllustration};

  /** The color to apply to any geometry that hasn't defined one.  */
  Rgba default_color{0.9, 0.9, 0.9, 1.0};

  /** A prefix to add to the path for all objects and transforms curated by the
   * MeshcatVisualizer.  It can be an absolute path or relative path.  See @ref
   * meshcat_path "Meshcat paths" for details. */
  std::string prefix{"visualizer"};

  /** Determines whether to send a Meschat::Delete("/prefix") message on an
  initialization event to remove any visualizations e.g. from a previous
  simulation. See @ref declare_initialization_events "Declare initialization
  events" for more information. */
  bool delete_prefix_on_initialization_event{true};
};

/** A system wrapper for Meshcat that publishes the current state of a
SceneGraph instance (whose QueryObject-valued output port is connected to this
system's input port).

 @system
 name: MeshcatVisualizer
 input_ports:
 - query_object
 @endsystem

The system uses the versioning mechanism provided by SceneGraph to detect
changes to the geometry so that a change in SceneGraph's data will propagate
to `drake_visualizer`.

By default, %MeshcatVisualizer visualizes geometries with the illustration role
(see @ref geometry_roles for more details). It can be configured to visualize
geometries with other roles. Only one role can be specified.  See
DrakeVisualizer which uses the same mechanisms for more details.

@tparam_nonsymbolic_scalar

Note that you must use the std::shared_pointer<Meschat> version of the
constructor to enable scalar conversion (e.g. ToAutoDiffXd()).
*/
template <typename T>
class MeshcatVisualizer final : public systems::LeafSystem<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(MeshcatVisualizer)

  /** Creates an instance of %MeshcatVisualizer.

   @param meshcat A Meshcat instance, which must remain valid for the lifetime
                  of this object. We do not offer to create an owned an
                  instance, since for Meshcat the visualizer can only be opened
                  during the lifetime of the Meshcat instance, which typically
                  exceeds the lifetime of a System (e.g. in a particular
                  simulation), and to avoid repeating any Meshcat options.
   @param params  The set of parameters to control this system's behavior.
   @throws std::exception if `params.publish_period <= 0`.
   @throws std::exception if `params.role == Role::kUnassigned`.  */
  explicit MeshcatVisualizer(Meshcat* meshcat,
                             MeshcatVisualizerParams params = {});

  /** Alternative constructor which takes a shared pointer to meshcat.  This
  version must be used in order to support scalar conversion (e.g.
  ToAutoDiffXd()), because otherwise the lifetime requirements on a raw Meshcat
  pointer would be too murky.  We use shared_ptr instead of unique_ptr because
  the most common workflow is to have a single Meshcat instance who's lifetime
  is longer than the life of any one Diagram. */
  explicit MeshcatVisualizer(const std::shared_ptr<Meshcat>& meshcat,
                             MeshcatVisualizerParams params = {});

  /** Scalar-converting copy constructor. See @ref system_scalar_conversion.
   It should only be used to convert _from_ double _to_ other scalar types.
   @throws std::exception if `other` does not *own* its Meshcats.
   */
  template <typename U>
  explicit MeshcatVisualizer(const MeshcatVisualizer<U>& other);

  /** Returns the Meshcat instance being used for visualization.  This can be
  used to e.g. add additional visualizations to the same scene. */
  Meshcat* meshcat() const { return meshcat_; }

  /** Calls Meschat::Delete(std::string path), with the path set to
   MeshcatVisualizerParams::prefix.  Since this visualizer will only ever add
   geometry under this prefix, this will remove all geometry/transforms added
   by the visualizer, or by a previous instance of this visualizer using the
   same prefix.  Use MeshcatVisualizer::delete_prefix_on_initialization_event
   to determine whether this should be called on initialization. */
  void DeletePrefix() const;

  /** Returns the QueryObject-valued input port. It should be connected to
   SceneGraph's QueryObject-valued output port. Failure to do so will cause a
   runtime error when attempting to broadcast messages.  */
  const systems::InputPort<T>& query_object_input_port() const {
    return this->get_input_port(query_object_input_port_);
  }

  /** Connects the newly added MeshcatVisualizer to the given SceneGraph's
   QueryObject-valued output port. See
   MeshcatVisualizer::MeshcatVisualizer(MeshcatVisualizer*,
   MeshcatVisualizerParams) for details. */
  static const MeshcatVisualizer<T>& AddToBuilder(
      systems::DiagramBuilder<T>* builder, const SceneGraph<T>& scene_graph,
      Meshcat* meshcat, MeshcatVisualizerParams params = {});

  /** Connects the newly added MeshcatVisualizer to the given QueryObject-valued
   output port. See MeshcatVisualizer::MeshcatVisualizer(MeshcatVisualizer*,
   MeshcatVisualizerParams) for details. */
  static const MeshcatVisualizer<T>& AddToBuilder(
      systems::DiagramBuilder<T>* builder,
      const systems::OutputPort<T>& query_object_port, Meshcat* meshcat,
      MeshcatVisualizerParams params = {});
  //@}

 private:
  /* Helper constructor which exists solely to give a useful error message for
   * the case of scalar conversion without an owned meshcat. */
  MeshcatVisualizer(Meshcat* meshcat,
                    const std::shared_ptr<Meshcat>& owned_meshcat,
                    bool scalar_conversion,
                    MeshcatVisualizerParams params = {});

  /* MeshcatVisualizer of different scalar types can all access each other's
   data. */
  template <typename>
  friend class MeshcatVisualizer;

  /* The periodic event handler. It tests to see if the last scene description
   is valid (if not, sends the objects) and then sends the transforms.  */
  systems::EventStatus UpdateMeshcat(const systems::Context<T>& context) const;

  /* Makes calls to Meshcat::SetObject to register geometry in SceneGraph.  */
  void SetObjects(const SceneGraphInspector<T>& inspector) const;

  /* Makes calls to Meshcat::SetTransform to update the poses from SceneGraph.
   */
  void SetTransforms(const QueryObject<T>& query_object) const;

  /* Handles the initialization event. */
  systems::EventStatus DeletePrefix(const systems::Context<T>&) const;

  /* The index of this System's QueryObject-valued input port.  */
  int query_object_input_port_{};

  // Meshcat is mutable because we must send messages (a non-const operation)
  // from a const System (e.g. during simulation).
  mutable Meshcat* meshcat_{};
  mutable std::shared_ptr<Meshcat> owned_meshcat_{};

  /* The version of the geometry that was last loaded (i.e., had a load message
   sent). If the version found on the input port differs from this value, a
   new load message will be sent prior to the "draw" message.  This is intended
   to track the information in meshcat_, and is therefore also a mutable member
   variable (instead of declared state). */
  mutable GeometryVersion version_;

  /* A store of the dynamic frames and their path. It is coupled with the
   version_.  This is only for efficiency; it does not represent undeclared
   state. */
  mutable std::map<FrameId, std::string> dynamic_frames_{};

  /* The parameters for the visualizer.  */
  MeshcatVisualizerParams params_;
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
