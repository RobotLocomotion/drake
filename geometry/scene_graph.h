#pragma once

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "drake/geometry/geometry_set.h"
#include "drake/geometry/geometry_state.h"
#include "drake/geometry/query_object.h"
#include "drake/geometry/query_results/penetration_as_point_pair.h"
#include "drake/geometry/scene_graph_inspector.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/rendering/pose_bundle.h"

namespace drake {

// Forward declarations to give LCM message publication appropriate access.
namespace lcm {
class DrakeLcmInterface;
}  // namespace lcm

namespace geometry {

class GeometryInstance;

template <typename T>
class QueryObject;

/** SceneGraph serves as the nexus for all geometry (and geometry-based
 operations) in a Diagram. Through SceneGraph, other systems that introduce
 geometry can _register_ that geometry as part of a common global domain,
 including it in geometric queries (e.g., cars controlled by one LeafSystem can
 be observed by a different sensor system). SceneGraph provides the
 interface for registering the geometry, updating its position based on the
 current context, and performing geometric queries.

 @system{SceneGraph,
   @input_port{source_pose{0}} @input_port{...} @input_port{source_pose{N-1}},
   @output_port{lcm_visualization} @output_port{query}
 }

 Only registered "geometry sources" can introduce geometry into %SceneGraph.
 Geometry sources will typically be other leaf systems, but, in the case of
 _anchored_ (i.e., stationary) geometry, it could also be some other block of
 code (e.g., adding a common ground plane with which all systems' geometries
 interact). For dynamic geometry (geometry whose pose depends on a Context), the
 geometry source must also provide pose values for all of the geometries the
 source owns, via a port connection on %SceneGraph. For N geometry sources,
 the %SceneGraph instance will have N pose input ports.

 The basic workflow for interacting with %SceneGraph is:

 - Register as a geometry source, acquiring a unique SourceId.
 - Register geometry (anchored and dynamic) with the system.
 - Connect source's geometry output ports to the corresponding %SceneGraph
   input ports.
   - Implement appropriate `Calc*` methods on the geometry output ports to
     update geometry pose values.

 @section geom_sys_inputs Inputs
 @cond
 In future versions, this will *also* include velocity and (possibly)
 acceleration ports.
 // TODO(SeanCurtis-TRI): Modify this to reflect the number of actual port
 // types.
 @endcond

 For each registered geometry source, there is one input port for each
 order of kinematics values (e.g., pose, velocity, and acceleration).
 If a source registers a frame, it must connect to these ports (although, in the
 current version, only pose is supported). Failure to connect to the port (or
 to provide valid kinematics values) will lead to runtime exceptions.

 __pose port__: An abstract-valued port providing an instance of
 FramePoseVector. For each registered frame, this "pose vector" maps the
 registered FrameId to a pose value. All registered frames must be accounted
 for and only frames registered by a source can be included in its output port.
 See the details in FrameKinematicsVector for details on how to provide values
 for this port.

 @section geom_sys_outputs Outputs

 %SceneGraph has two output ports:

 __query port__: An abstract-valued port containing an instance of QueryObject.
 It provides a "ticket" for downstream LeafSystem instances to perform geometric
 queries on the %SceneGraph. To perform geometric queries, downstream
 LeafSystem instances acquire the QueryObject from %SceneGraph's output port
 and provide it as a parameter to one of %SceneGraph's query methods (e.g.,
 SceneGraph::ComputeContact()). This assumes that the querying system has
 access to a const pointer to the connected %SceneGraph instance. Use
 get_query_output_port() to acquire the output port for the query handle.

 __lcm visualization port__: An abstract-valued port containing an instance of
 PoseBundle. This is a convenience port designed to feed LCM update messages to
 drake_visualizer for the purpose of visualizing the state of the world's
 geometry. Additional uses of this port are strongly discouraged; instead, use
 an appropriate geometric query to obtain the state of the world's geometry.

 @section geom_sys_workflow Working with SceneGraph

 LeafSystem instances can relate to SceneGraph in one of two ways: as a
 _consumer_ that performs queries, or as a _producer_ that introduces geometry
 into the shared world and defines its context-dependent kinematics values.
 It is reasonable for systems to perform either role singly, or both.

 __Consumer__

 Consumers perform geometric queries upon the world geometry. %SceneGraph
 _serves_ those queries. As indicated above, in order for a LeafSystem to act
 as a consumer, it must:
   1. define a QueryObject-valued input port and connect it to %SceneGraph's
   corresponding output port, and
   2. have a reference to the connected %SceneGraph instance.

 With those two requirements satisfied, a LeafSystem can perform geometry
 queries by:
   1. evaluating the QueryObject input port, and
   2. passing the returned query object into the appropriate query method on
   SceneGraph (e.g., SceneGraph::ComputeContact()).

 __Producer__

 All producers introduce geometry into the shared geometric world. This is
 called _registering_ geometry. Depending on what exactly has been registered,
 a producer may also have to _update kinematics_. Producers themselves must be
 registered with %SceneGraph as producers (a.k.a. _geometry sources_). They
 do this by acquiring a SourceId (via SceneGraph::RegisterSource()). The
 SourceId serves as a unique handle through which the producer's identity is
 validated and its ownership of its registered geometry is maintained.

 _Registering Geometry_

 %SceneGraph cannot know what geometry _should_ be part of the shared world.
 Other systems are responsible for introducing geometry into the world. This
 process (defining geometry and informing %SceneGraph) is called
 _registering_ the geometry. The source that registers the geometry "owns" the
 geometry; the source's unique SourceId is required to perform any operations
 on the geometry registered with that SourceId. Geometry can be registered as
 _anchored_ or _dynamic_.

 Dynamic geometry can move; more specifically, its kinematics (e.g., pose)
 depends on a system's Context. Particularly, dynamic geometry is
 _fixed_ to a _frame_ whose kinematics values depend on a context. As the frame
 moves, the geometries fixed to it move with it. Therefore, to register dynamic
 geometry a frame must be registered first. These registered frames serve as the
 basis for repositioning geometry in the shared world. The geometry source is
 responsible for providing up-to-date kinematics values for those registered
 frames upon request (via an appropriate output port on the source LeafSystem
 connecting to the appropriate input port on %SceneGraph). The work flow is
 as follows:
   1. A LeafSystem registers itself as a geometry source, acquiring a SourceId
      (RegisterSource()).
   2. The source registers a frame (GeometrySource::RegisterFrame()).
     - A frame always has a "parent" frame. It can implicitly be the world
     frame, _or_ another frame registered by the source.
   3. Register one or more geometries to a frame
   (GeometrySource::RegisterGeometry()).
     - The registered geometry is posed relative to the frame to which it is
     fixed.
     - The geometry can also be posed relative to another registered geometry.
     It will be affixed to _that_ geometry's frame.

 Anchored geometry is _independent_ of the context (i.e., it doesn't move).
 Anchored geometries are always affixed to the immobile world frame. As such,
 registering a frame is _not_ required for registering anchored geometry
 (see GeometrySource::RegisterAnchoredGeometry()). However, the source still
 "owns" the anchored geometry.

 _Updating Kinematics_

 Registering _dynamic_ geometry implies a contract between the geometry source
 and %SceneGraph. The geometry source must do the following:
   - It must provide, populate, and connect two output ports: the "id" port and
   the "pose" port.
   - The id port must contain _all_ the frame ids returned as a result of frame
   registration.
   - The pose port must contain one pose per registered frame; the pose value is
   expressed relative to the registered frame's _parent_ frame. As mentioned
   above, the iᵗʰ pose value should describe the frame indicated by the iᵗʰ id
   in the id output port.

 Failure to meet these requirements will lead to a run-time error.

 @section geom_model_vs_context Model versus Context

 Many (and eventually all) methods that configure the population of SceneGraph
 have two variants that differ by whether they accept a mutable Context or not.
 When no Context is provided, _this_ %SceneGraph instance's underlying model is
 modified. When the %SceneGraph instance allocates a context, its model is
 copied into that context.

 The second variant causes %SceneGraph to modify the data stored in the provided
 Context to be modified _instead of the internal model_.

 The two interfaces _can_ be used interchangeably. However, modifications to
 `this` %SceneGraph's underlying model will _not_ affect previously allocated
 Context instances. A new Context should be allocated after modifying the
 model.

 @note In this initial version, the only methods with the Context-modifying
 variant are those methods that _do not_ change the the semantics of the input
 or output ports. Modifications that make such changes must be coordinated
 across systems.
 <!-- TODO(SeanCurtis-TRI): Add context-modifying variants of all methods. -->

 @cond
 // TODO(SeanCurtis-TRI): Future work which will require add'l documentation:
 //   - velocity kinematics.
 //   - Finalizing API for topology changes at discrete events.
 @endcond

 @tparam_nonsymbolic_scalar
 @ingroup systems
 */
template <typename T>
class SceneGraph final : public systems::LeafSystem<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(SceneGraph)

  /** Constructs a default (empty) scene graph. */
  SceneGraph();

  /** Constructor used for scalar conversions. It should only be used to convert
   _from_ double _to_ other scalar types.  */
  template <typename U>
  explicit SceneGraph(const SceneGraph<U>& other);

  ~SceneGraph() override {}

  /** @name       Port management
   Access to SceneGraph's input/output ports. This topic includes
   registration of geometry sources because the input ports are mapped to
   registered geometry sources.

   A source that registers frames and geometries _must_ connect outputs to
   the inputs associated with that source. Failure to do so will be treated as
   a runtime error during the evaluation of %SceneGraph. %SceneGraph
   will detect that frames have been registered but no values have been
   provided.  */
  //@{

  /** Registers a new, named source to the geometry system. The caller must save
   the returned SourceId; it is the token by which all other operations on the
   geometry world are conducted.

   This source id can be used to register arbitrary _anchored_ geometry. But if
   dynamic geometry is registered (via RegisterGeometry/RegisterFrame), then
   the context-dependent pose values must be provided on an input port.
   See get_source_pose_port().

   This method modifies the underlying model and requires a new Context to be
   allocated.

   @param name          The optional name of the source. If none is provided
                        (or the empty string) a default name will be defined by
                        SceneGraph's logic.
   @throws std::logic_error if the name is not unique.  */
  SourceId RegisterSource(const std::string& name = "");

  /** Reports if the given source id is registered.
   @param id       The id of the source to query.  */
  bool SourceIsRegistered(SourceId id) const;

  /** Given a valid source `id`, returns a _pose_ input port associated
   with that `id`. This port is used to communicate _pose_ data for registered
   frames.
   @throws std::logic_error if the source_id is _not_ recognized.  */
  const systems::InputPort<T>& get_source_pose_port(SourceId id) const;

  /** Returns the output port which produces the PoseBundle for LCM
   communication to drake visualizer.  */
  const systems::OutputPort<T>& get_pose_bundle_output_port() const {
    return systems::System<T>::get_output_port(bundle_port_index_);
  }

  /** Returns the output port which produces the QueryObject for performing
   geometric queries.  */
  const systems::OutputPort<T>& get_query_output_port() const {
    return systems::System<T>::get_output_port(query_port_index_);
  }

  //@}

  /** @name             Topology Manipulation
   Topology manipulation consists of changing the data contained in the world.
   This includes registering a new geometry source, adding frames, adding or
   removing geometries, modifying geometry properties, etc.

   The work flow for adding geometry to the SceneGraph is as follows:

   - A geometry source registers itself with %SceneGraph (via RegisterSource()).
   - The geometry source can then immediately register "anchored" geometry --
     geometry that is affixed to the world frame. These geometries will never
     move.
   - For geometries that need to move based on the source's state, the
     geometry source must first register a GeometryFrame. In fact, geometries
     never move directly; it is the frames to which they are affixed that move.
     A geometry source can register a frame via the RegisterFrame() methods.
   - Once a frame has been registered, the geometry source can register
     geometries that are rigidly affixed to that frame (or, figuratively
     speaking, "hung" on that frame). The geometry is immovably posed in that
     frame and assigned various properties. The geometry is registered via calls
     to the RegisterGeometry() methods.

   %SceneGraph has a concept of "ownership" that is separate from the C++
   notion of ownership. In this case, %SceneGraph protects geometry and frames
   registered by one source from being modified by another source. All methods
   that change the world are associated with the SourceId of the geometry source
   requesting the change. One source cannot "hang" geometry onto a frame (or
   geometry) that belongs to another source. However, all sources have read
   access to all geometries in the world. For example, queries will return
   GeometryId values that span all sources and the properties of the associated
   geometries can be queried by arbitrary sources.

   That said, if one source _chooses_ to share its SourceId externally, then
   arbitrary code can use that SourceId to modify the geometry resources that
   are associated with that SourceId.

   @note There are no Context-modifying variants for source or frame
   registration yet, as these methods modify the port semantics.  */
  //@{

  /** Registers a new frame F for this source. This hangs frame F on the
   world frame (W). Its pose is defined relative to the world frame (i.e,
   `X_WF`). Returns the corresponding unique frame id.

   This method modifies the underlying model and requires a new Context to be
   allocated.

   @param source_id     The id for the source registering the frame.
   @param frame         The frame to register.
   @returns A unique identifier for the added frame.
   @throws std::logic_error  if a) the `source_id` does _not_ map to a
                             registered source, or
                             b) `frame` has an id that has already been
                             registered.  */
  FrameId RegisterFrame(SourceId source_id, const GeometryFrame& frame);

  /** Registers a new frame F for this source. This hangs frame F on another
   previously registered frame P (indicated by `parent_id`). The pose of the new
   frame is defined relative to the parent frame (i.e., `X_PF`).  Returns the
   corresponding unique frame id.

   This method modifies the underlying model and requires a new Context to be
   allocated.

   @param source_id    The id for the source registering the frame.
   @param parent_id    The id of the parent frame P.
   @param frame        The frame to register.
   @returns A unique identifier for the added frame.
   @throws std::logic_error  if a) the `source_id` does _not_ map to a
                             registered source,
                             b) the `parent_id` does _not_ map to a known
                             frame or does not belong to the source, or
                             c) `frame` has an id that has already been
                             registered.  */
  FrameId RegisterFrame(SourceId source_id, FrameId parent_id,
                        const GeometryFrame& frame);

  /** Registers a new geometry G for this source. This hangs geometry G on a
   previously registered frame F (indicated by `frame_id`). The pose of the
   geometry is defined in a fixed pose relative to F (i.e., `X_FG`).
   Returns the corresponding unique geometry id.

   Roles will be assigned to the registered geometry if the corresponding
   GeometryInstance `geometry` has had properties assigned.

   This method modifies the underlying model and requires a new Context to be
   allocated.

   @param source_id   The id for the source registering the geometry.
   @param frame_id    The id for the frame F to hang the geometry on.
   @param geometry    The geometry G to affix to frame F.
   @return A unique identifier for the added geometry.
   @throws std::logic_error  if a) the `source_id` does _not_ map to a
                             registered source,
                             b) the `frame_id` doesn't belong to the source,
                             c) the `geometry` is equal to `nullptr`, or
                             d) the geometry's name doesn't satisfy the
                             requirements outlined in GeometryInstance.  */
  GeometryId RegisterGeometry(SourceId source_id, FrameId frame_id,
                              std::unique_ptr<GeometryInstance> geometry);

  /** systems::Context-modifying variant of RegisterGeometry(). Rather than
   modifying %SceneGraph's model, it modifies the copy of the model stored in
   the provided context.  */
  GeometryId RegisterGeometry(systems::Context<T>* context, SourceId source_id,
                              FrameId frame_id,
                              std::unique_ptr<GeometryInstance> geometry) const;

  /** Registers a new geometry G for this source. This hangs geometry G on a
   previously registered geometry P (indicated by `geometry_id`). The pose of
   the geometry is defined in a fixed pose relative to geometry P (i.e.,
   `X_PG`). By induction, this geometry is effectively rigidly affixed to the
   frame that P is affixed to. Returns the corresponding unique geometry id.

   Roles will be assigned to the registered geometry if the corresponding
   GeometryInstance `geometry` has had properties assigned.

   This method modifies the underlying model and requires a new Context to be
   allocated.

   @param source_id    The id for the source registering the geometry.
   @param geometry_id  The id for the parent geometry P.
   @param geometry     The geometry G to add.
   @return A unique identifier for the added geometry.
   @throws std::logic_error if a) the `source_id` does _not_ map to a registered
                            source,
                            b) the `geometry_id` doesn't belong to the source,
                            c) the `geometry` is equal to `nullptr`, or
                            d) the geometry's name doesn't satisfy the
                            requirements outlined in GeometryInstance.  */
  GeometryId RegisterGeometry(SourceId source_id, GeometryId geometry_id,
                              std::unique_ptr<GeometryInstance> geometry);

  /** systems::Context-modifying variant of RegisterGeometry(). Rather than
   modifying %SceneGraph's model, it modifies the copy of the model stored in
   the provided context.  */
  GeometryId RegisterGeometry(systems::Context<T>* context, SourceId source_id,
                              GeometryId geometry_id,
                              std::unique_ptr<GeometryInstance> geometry) const;

  /** Registers a new _anchored_ geometry G for this source. This hangs geometry
   G from the world frame (W). Its pose is defined in that frame (i.e., `X_WG`).
   Returns the corresponding unique geometry id.

   Roles will be assigned to the registered geometry if the corresponding
   GeometryInstance `geometry` has had properties assigned.

   This method modifies the underlying model and requires a new Context to be
   allocated.

   @param source_id     The id for the source registering the frame.
   @param geometry      The anchored geometry G to add to the world.
   @return A unique identifier for the added geometry.
   @throws std::logic_error  if a) the `source_id` does _not_ map to a
                             registered source or
                             b) the geometry's name doesn't satisfy the
                             requirements outlined in GeometryInstance.  */
  GeometryId RegisterAnchoredGeometry(
      SourceId source_id, std::unique_ptr<GeometryInstance> geometry);

  /** Removes the given geometry G (indicated by `geometry_id`) from the given
   source's registered geometries. All registered geometries hanging from
   this geometry will also be removed.

   This method modifies the underlying model and requires a new Context to be
   allocated.

   @param source_id   The identifier for the owner geometry source.
   @param geometry_id The identifier of the geometry to remove (can be dynamic
                      or anchored).
   @throws std::logic_error  if a) the `source_id` does _not_ map to a
                             registered source,
                             b) the `geometry_id` does not map to a valid
                             geometry, or
                             c) the `geometry_id` maps to a geometry that does
                             not belong to the indicated source.  */
  void RemoveGeometry(SourceId source_id, GeometryId geometry_id);

  /** systems::Context-modifying variant of RemoveGeometry(). Rather than
   modifying %SceneGraph's model, it modifies the copy of the model stored in
   the provided context.  */
  void RemoveGeometry(systems::Context<T>* context, SourceId source_id,
                      GeometryId geometry_id) const;

  //@}

  /** @name     Managing RenderEngine instances      */
  //@{

  /** Adds a new render engine to this %SceneGraph. The %SceneGraph owns the
   render engine. The render engine's name should be referenced in the
   @ref render::CameraProperties "CameraProperties" provided in the render
   queries (see QueryObject::RenderColorImage() as an example).

   There is no restriction on when a renderer is added relative to geometry
   registration and role assignment. Given a representative sequence of
   registration and perception role assignment, the addition of the renderer
   can be introduced anywhere in the sequence and the end result would be
   the same.

   ```
   GeometryId id1 = scene_graph.RegisterGeometry(source_id, ...);
   scene_graph.AssignRole(source_id, id1, PerceptionProperties());
   GeometryId id2 = scene_graph.RegisterGeometry(source_id, ...);
   scene_graph.AssignRole(source_id, id2, PerceptionProperties());
   GeometryId id3 = scene_graph.RegisterGeometry(source_id, ...);
   scene_graph.AssignRole(source_id, id3, PerceptionProperties());
   ```

   @param name      The unique name of the renderer.
   @param renderer  The `renderer` to add.
   @throws std::logic_error if the name is not unique.  */
  void AddRenderer(std::string name,
                   std::unique_ptr<render::RenderEngine> renderer);

  /** Reports if this %SceneGraph has a renderer registered to the given name.
   */
  bool HasRenderer(const std::string& name) const;

  /** Reports the number of renderers registered to this %SceneGraph.  */
  int RendererCount() const;

  /** Reports the names of all registered renderers.  */
  std::vector<std::string> RegisteredRendererNames() const;

  //@}

  /** @name     Managing geometry roles

   Geometries _must_ be assigned one or more *roles* before they have an effect
   on SceneGraph computations (see @ref geometry_roles for details). These
   methods provide the ability to manage roles for a registered geometry.

   The `AssignRole()` methods provide the mechanism for initially assigning a
   role (via its corresponding properties) and, subsequently, modifying those
   properties.

   <h4>Assigning roles for the first time</h4>

   If a geometry has not had a particular role assigned to it, the role is
   assigned by the following call:

   @code
   scene_graph.AssignRole(source_id, geometry_id, properties);
   @endcode

   The role is inferred by the type of properties provided. An exception will
   be thrown if the geometry has already had the implied role assigned to it.

   <h4>Changing the properties for an assigned role</h4>

   If  the geometry has previously been assigned a role, the properties for
   that role can be modified with the following code (using ProximityProperties
   as an example):

   @code
   ProximityProperties props;
   props.AddProperty(....);  // Populate the properties.
   scene_graph.AssignRole(source_id, geometry_id, props, RoleAssign::kReplace);
   @endcode

   An exception will be thrown if the geometry _has not_ already had a role
   assigned.

   If the goal is to modify the properties that have already been assigned, we
   recommend the following (again, using ProximityProperties as an example):

   @code
   const ProximityProperties* old_props =
       scene_graph.model_inspector().GetProximityProperties(geometry_id);
   DRAKE_DEMAND(old_props);
   ProximityProperties new_props(*old_props);
   new_props.AddProperty(...);  // add additional properties.
   scene_graph.AssignRole(source_id, geometry_id, new_props,
                          RoleAssign::kReplace);
   @endcode

   Calling `AssignRole()` with an empty set of properties will *not* remove the
   role; it will simply eliminate possibly necessary properties. To remove
   the role completely, call `RemoveRole()`.

   @warning Currently, only __proximity__ properties can be updated via this
   mechanism. Updating illustration and perception will throw an exception (to
   be implemented in the near future).

   All invocations of `AssignRole()` will throw an exception if:

     - the source id is invalid.
     - the geometry id is invalid.
     - the geometry id is not owned by the given source id.
     - Another geometry with the same name, affixed to the same frame, already
       has the role.

   <h4>Removing roles</h4>

   Calling `RemoveRole()` will remove the properties and _role_ entirely.

   These methods include the model- and context-modifying variants.  */
  //@{

  /** Assigns the proximity role to the geometry indicated by `geometry_id`.
   @pydrake_mkdoc_identifier{proximity_direct}
   */
  void AssignRole(SourceId source_id, GeometryId geometry_id,
                  ProximityProperties properties,
                  RoleAssign assign = RoleAssign::kNew);

  /** systems::Context-modifying variant of
   @ref AssignRole(SourceId,GeometryId,ProximityProperties) "AssignRole()" for
   proximity properties. Rather than modifying %SceneGraph's model, it modifies
   the copy of the model stored in the provided context.
   @pydrake_mkdoc_identifier{proximity_context}
   */
  void AssignRole(systems::Context<T>* context, SourceId source_id,
                  GeometryId geometry_id, ProximityProperties properties,
                  RoleAssign assign = RoleAssign::kNew) const;

  /** Assigns the perception role to the geometry indicated by `geometry_id`.
   @pydrake_mkdoc_identifier{perception_direct}
   */
  void AssignRole(SourceId source_id, GeometryId geometry_id,
                  PerceptionProperties properties,
                  RoleAssign assign = RoleAssign::kNew);

  /** systems::Context-modifying variant of
   @ref AssignRole(SourceId,GeometryId,PerceptionProperties) "AssignRole()" for
   perception properties. Rather than modifying %SceneGraph's model, it modifies
   the copy of the model stored in the provided context.
   @pydrake_mkdoc_identifier{perception_context}
   */
  void AssignRole(systems::Context<T>* context, SourceId source_id,
                  GeometryId geometry_id, PerceptionProperties properties,
                  RoleAssign assign = RoleAssign::kNew) const;

  /** Assigns the illustration role to the geometry indicated by `geometry_id`.
   @pydrake_mkdoc_identifier{illustration_direct}
   */
  void AssignRole(SourceId source_id, GeometryId geometry_id,
                  IllustrationProperties properties,
                  RoleAssign assign = RoleAssign::kNew);

  /** systems::Context-modifying variant of
   @ref AssignRole(SourceId,GeometryId,IllustrationProperties) "AssignRole()"
   for illustration properties. Rather than modifying %SceneGraph's model, it
   modifies the copy of the model stored in the provided context.
   @pydrake_mkdoc_identifier{illustration_context}
   */
  void AssignRole(systems::Context<T>* context, SourceId source_id,
                  GeometryId geometry_id, IllustrationProperties properties,
                  RoleAssign assign = RoleAssign::kNew) const;

  /** Removes the indicated `role` from any geometry directly registered to the
   frame indicated by `frame_id` (if the geometry has the role).
   @returns The number of geometries affected by the removed role.
   @throws std::logic_error if a) `source_id` does not map to a registered
                            source,
                            b) `frame_id` does not map to a registered frame,
                            c) `frame_id` does not belong to `source_id`
                            (unless `frame_id` is the world frame id), or
                            d) the context has already been allocated.  */
  int RemoveRole(SourceId source_id, FrameId frame_id, Role role);

  /** systems::Context-modifying variant of
   @ref RemoveRole(SourceId,FrameId,Role) "RemoveRole()" for frames.
   Rather than modifying %SceneGraph's model, it modifies the copy of the model
   stored in the provided context.  */
  int RemoveRole(systems::Context<T>* context, SourceId source_id,
                  FrameId frame_id, Role role) const;

  /** Removes the indicated `role` from the geometry indicated by `geometry_id`.
   @returns One if the geometry had the role removed and zero if the geometry
            did not have the role assigned in the first place.
   @throws std::logic_error if a) `source_id` does not map to a registered
                            source,
                            b) `geometry_id` does not map to a registered
                            geometry,
                            c) `geometry_id` does not belong to `source_id`, or
                            d) the context has already been allocated.  */
  int RemoveRole(SourceId source_id, GeometryId geometry_id, Role role);

  /** systems::Context-modifying variant of
   @ref RemoveRole(SourceId,GeometryId,Role) "RemoveRole()" for individual
   geometries. Rather than modifying %SceneGraph's model, it modifies the copy
   of the model stored in the provided context.  */
  int RemoveRole(systems::Context<T>* context, SourceId source_id,
                  GeometryId geometry_id, Role role) const;

  //@}

  /** Reports the identifier for the world frame.  */
  static FrameId world_frame_id() {
    return internal::InternalFrame::world_frame_id();
  }

  /** Returns an inspector on the system's _model_ scene graph data.  */
  const SceneGraphInspector<T>& model_inspector() const;

  /** @name         Collision filtering
   @anchor scene_graph_collision_filtering
   The interface for limiting the scope of penetration queries (i.e., "filtering
   collisions").

   The scene graph consists of the set of geometry
   `G = D ⋃ A = {g₀, g₁, ..., gₙ}`, where D is the set of dynamic geometry and
   A is the set of anchored geometry (by definition `D ⋂ A = ∅`). Collision
   occurs between pairs of geometries (e.g., (gᵢ, gⱼ)). The set of collision
   candidate pairs is initially defined as `C = (G × G) - (A × A) - F - I`,
   where:
     - `G × G = {(gᵢ, gⱼ)}, ∀ gᵢ, gⱼ ∈ G` is the cartesian product of the set
       of %SceneGraph geometries.
     - `A × A` represents all pairs consisting only of anchored geometry;
       anchored geometry is never tested against other anchored geometry.
     - `F = (gᵢ, gⱼ)`, such that `frame(gᵢ) == frame(gⱼ)`; the pair where both
       geometries are rigidly affixed to the same frame. By implication,
       `gᵢ, gⱼ ∈ D` as only dynamic geometries are affixed to frames.
     - `I = {(g, g)}, ∀ g ∈ G` is the set of all pairs consisting of a geometry
        with itself; there is no collision between a geometry and itself.

   Only pairs contained in C will be tested as part of penetration queries.
   These filter methods essentially create new sets of pairs and then subtract
   them from the candidate set C. See each method for details.

   Modifications to C _must_ be performed before context allocation.  */
  //@{

  /** Excludes geometry pairs from collision evaluation by updating the
   candidate pair set `C = C - P`, where `P = {(gᵢ, gⱼ)}, ∀ gᵢ, gⱼ ∈ G` and
   `G = {g₀, g₁, ..., gₘ}` is the input `set` of geometries.

   If the set includes geometries which have _not_ been assigned a proximity
   role, those geometries will be ignored. If a proximity role is subsequently
   assigned, those geometries will _still_ not be part of any collision filters.
   Proximity roles should _generally_ be assigned prior to collision filter
   configuration.

   This method modifies the underlying model and requires a new Context to be
   allocated.

   @throws std::logic_error if the set includes ids that don't exist in the
                            scene graph.  */
  void ExcludeCollisionsWithin(const GeometrySet& set);

  /** systems::Context-modifying variant of ExcludeCollisionsWithin(). Rather
   than modifying %SceneGraph's model, it modifies the copy of the model stored
   in the provided context.  */
  void ExcludeCollisionsWithin(systems::Context<T>* context,
                               const GeometrySet& set) const;

  /** Excludes geometry pairs from collision evaluation by updating the
   candidate pair set `C = C - P`, where `P = {(a, b)}, ∀ a ∈ A, b ∈ B` and
   `A = {a₀, a₁, ..., aₘ}` and `B = {b₀, b₁, ..., bₙ}` are the input sets of
   geometries `setA` and `setB`, respectively. This does _not_ preclude
   collisions between members of the _same_ set.

   If the sets include geometries which have _not_ been assigned a proximity
   role, those geometries will be ignored. If a proximity role is subsequently
   assigned, those geometries will _still_ not be part of any collision filters.
   Proximity roles should _generally_ be assigned prior to collision filter
   configuration.

   This method modifies the underlying model and requires a new Context to be
   allocated.

   @throws std::logic_error if the groups include ids that don't exist in the
                            scene graph.   */
  void ExcludeCollisionsBetween(const GeometrySet& setA,
                                const GeometrySet& setB);

  /** systems::Context-modifying variant of ExcludeCollisionsBetween(). Rather
   than modifying %SceneGraph's model, it modifies the copy of the model stored
   in the provided context.  */
  void ExcludeCollisionsBetween(systems::Context<T>* context,
                                const GeometrySet& setA,
                                const GeometrySet& setB) const;
  //@}

 private:
  // Friend class to facilitate testing.
  friend class SceneGraphTester;

  // SceneGraph of different scalar types can all access each other's data.
  template <typename>
  friend class SceneGraph;

  // Give (at least temporarily) QueryObject access to the system API to
  // evaluate inputs on the context.
  friend class QueryObject<T>;

  // Helper class to register input ports for a source id.
  void MakeSourcePorts(SourceId source_id);

  // Allow the load dispatch to peek into SceneGraph.
  friend void DispatchLoadMessage(const SceneGraph<double>&,
                                  lcm::DrakeLcmInterface*, Role role);

  // Sets the context into the output port value so downstream consumers can
  // perform queries.
  void CalcQueryObject(const systems::Context<T>& context,
                       QueryObject<T>* output) const;

  // Constructs a PoseBundle of length equal to the concatenation of all inputs.
  // This is the method used by the allocator for the output port.
  systems::rendering::PoseBundle<T> MakePoseBundle() const;

  // Aggregates the input poses into the output PoseBundle, in the same order as
  // was used in allocation. Aborts if any inputs have a _different_ size than
  // expected.
  void CalcPoseBundle(const systems::Context<T>& context,
                      systems::rendering::PoseBundle<T>* output) const;

  // Collects all of the *dynamic* frames that have geometries with the given
  // role.
  std::vector<FrameId> GetDynamicFrames(const GeometryState<T>& g_state,
                                        Role role) const;

  // Refreshes the pose of the various engines which exploits the caching
  // infrastructure.
  void FullPoseUpdate(const systems::Context<T>& context) const {
    this->get_cache_entry(pose_update_index_).template Eval<int>(context);
  }

  // Updates the state of geometry world from *all* the inputs. This is the calc
  // method for the corresponding cache entry. The entry *value* (the int) is
  // strictly a dummy -- the value is unimportant; only the side effect matters.
  void CalcPoseUpdate(const systems::Context<T>& context, int*) const;

  // Asserts the given source_id is registered, throwing an exception whose
  // message is the given message with the source_id appended if not.
  void ThrowUnlessRegistered(SourceId source_id, const char* message) const;

  // Extracts a mutable reference to the underlying abstract geometry state from
  // the given context.
  GeometryState<T>& mutable_geometry_state(systems::Context<T>* context) const;

  // Extracts a reference to the underlying abstract geometry state from the
  // given context.
  const GeometryState<T>& geometry_state(
      const systems::Context<T>& context) const;

  // A struct that stores the port indices for a given source.
  // TODO(SeanCurtis-TRI): Consider making these TypeSafeIndex values.
  struct SourcePorts {
    int pose_port{-1};
  };

  // A mapping from added source identifier to the port indices associated with
  // that id.
  std::unordered_map<SourceId, SourcePorts> input_source_ids_;

  // The index of the output port with the PoseBundle abstract value.
  int bundle_port_index_{-1};

  // The index of the output port with the QueryObject abstract value.
  int query_port_index_{-1};

  // A raw pointer to the default geometry state (which serves as the model for
  // allocating contexts for this system). The instance is owned by
  // model_abstract_states_.
  GeometryState<T>* initial_state_{};

  SceneGraphInspector<T> model_inspector_;

  // The index of the geometry state in the context's abstract state.
  int geometry_state_index_{-1};

  // The cache index for the pose update cache entry.
  systems::CacheIndex pose_update_index_{};
};

}  // namespace geometry

// Define the conversion trait to *only* allow double -> AutoDiffXd conversion.
// Symbolic is not supported yet, and AutoDiff -> double doesn't "make sense".
namespace systems {
namespace scalar_conversion {
template <>
struct Traits<geometry::SceneGraph> {
  template <typename T, typename U>
  using supported =
      typename std::conditional<!std::is_same<T, symbolic::Expression>::value &&
                                    std::is_same<U, double>::value,
                                std::true_type, std::false_type>::type;
};
}  // namespace scalar_conversion
}  // namespace systems

}  // namespace drake
