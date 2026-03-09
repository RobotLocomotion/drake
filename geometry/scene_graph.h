#pragma once

#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

#include "drake/geometry/collision_filter_manager.h"
#include "drake/geometry/geometry_frame.h"
#include "drake/geometry/geometry_set.h"
#include "drake/geometry/kinematics_vector.h"
#include "drake/geometry/query_object.h"
#include "drake/geometry/query_results/penetration_as_point_pair.h"
#include "drake/geometry/scene_graph_config.h"
#include "drake/geometry/scene_graph_inspector.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace geometry {

#ifndef DRAKE_DOXYGEN_CXX
class GeometryInstance;
template <typename T>
class GeometryState;
template <typename T>
class QueryObject;
#endif

/** SceneGraph serves as the nexus for all geometry (and geometry-based
 operations) in a Diagram. Through SceneGraph, other systems that introduce
 geometry can _register_ that geometry as part of a common global domain,
 including it in geometric queries (e.g., cars controlled by one LeafSystem can
 be observed by a different sensor system). SceneGraph provides the
 interface for registering the geometry, updating its position based on the
 current context, and performing geometric queries.

 @system
 name: SceneGraph
 input_ports:
 - <em style="color:gray">(source name)</em>_pose
 - <em style="color:gray">(source name)</em>_configuration
 output_ports:
 - query
 @endsystem

 For each registered "geometry source", there is an input port whose name begins
 with <em style="color:gray">(source name)</em>.


 Only registered "geometry sources" can introduce geometry into %SceneGraph.
 Geometry sources will typically be other leaf systems, but, in the case of
 _anchored_ (i.e., stationary) geometry, it could also be some other block of
 code (e.g., adding a common ground plane with which all systems' geometries
 interact). For dynamic geometry (geometry whose pose depends on a Context), the
 geometry source must also provide pose/configuration values for all of the
 geometries the source owns, via a port connection on %SceneGraph. For N
 geometry sources, the %SceneGraph instance will have N pose/configuration input
 ports.

 The basic workflow for interacting with %SceneGraph is:

 - Register as a geometry source, acquiring a unique SourceId.
 - Register geometry (anchored and dynamic) with the system.
 - Connect source's geometry output ports to the corresponding %SceneGraph
   input ports.
   - Implement appropriate `Calc*` methods on the geometry output ports to
     update geometry pose/configuration values.

 @section geom_sys_inputs Inputs

 For each registered geometry source, there is one input port for each order
 of kinematics values (i.e., pose and configuration). If a source registers a
 frame or a deformable geometry, it must connect to the corresponding ports.
 Failure to connect to the ports (or to provide valid kinematics values) will
 lead to runtime exceptions.

 __pose port__: An abstract-valued port providing an instance of
 FramePoseVector. For each registered frame, this "pose vector" maps the
 registered FrameId to a pose value. All registered frames must be accounted
 for and only frames registered by a source can be included in its output port.
 See the details in KinematicsVector for details on how to provide values for
 this port.

 <!-- TODO(xuchenhan-tri): Consider adding some clarification about
  "configuration" as in "deformable vertex positions" compared to
  "configuration" as in articulated rigid-body configurations (and that we use
  the word to exclusively mean the former in SceneGraph). -->
 __configuration port__: An abstract-valued port providing an instance of
 GeometryConfigurationVector. For each registered deformable geometry, this
 "configuration vector" maps the registered GeometryId to its world space
 configuration (i.e. the vertex positions of its mesh representation in the
 world frame). All registered deformable geometries must be accounted for and
 only geometries registered by a source can be included in its output port.

 @section geom_sys_outputs Outputs

 %SceneGraph has one output port:

 __query port__: An abstract-valued port containing an instance of QueryObject.
 To perform geometric queries, downstream LeafSystem instances acquire the
 QueryObject from %SceneGraph's output port and invoke the appropriate methods
 on it. Use get_query_output_port() to acquire the output port for the query
 handle.

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
   1. evaluating the QueryObject input port, retrieving a `const QueryObject&`
   in return, and
   2. invoking the appropriate method on the QueryObject.

 __Producer__

 All producers introduce geometry into the shared geometric world. This is
 called _registering_ geometry. Depending on what exactly has been registered,
 a producer may also have to _update kinematics_. Producers themselves must be
 registered with %SceneGraph as producers (a.k.a. _geometry sources_). They
 do this by acquiring a SourceId (via SceneGraph::RegisterSource()). The
 SourceId serves as a unique handle through which the producer's identity is
 validated.

 _Registering Geometry_

 %SceneGraph cannot know what geometry _should_ be part of the shared world.
 Other systems are responsible for introducing geometry into the world. This
 process (defining geometry and informing %SceneGraph) is called
 _registering_ the geometry. Geometry can be registered as  _anchored_ or
 _dynamic_, and is always registered to (associated with) a SourceId.

 Dynamic geometry can move; more specifically, its kinematics (e.g., pose)
 depends on a system's Context. Particularly, a non-deformable dynamic geometry
 is _fixed_ to a _frame_ whose kinematics values depend on a context. As the
 frame moves, the geometries fixed to it move with it. On the other hand, a
 deformable dynamic geometry has a mesh representation whose vertices' positions
 can change and are expressed in the frame it is registered in. Therefore, to
 register dynamic geometry a frame must be registered first. These registered
 frames serve as the basis for repositioning geometry in the shared world. The
 geometry source is responsible for providing up-to-date kinematics values for
 those registered frames upon request (via an appropriate output port on the
 source LeafSystem connecting to the appropriate input port on %SceneGraph). The
 geometry source that registers deformable geometry is also responsible to
 provide the positions of the mesh vertices of the deformable geometry in the
 registered-in frame. The work flow is as follows:
   1. A LeafSystem registers itself as a geometry source, acquiring a SourceId
      (RegisterSource()).
   2. The source registers a frame (GeometrySource::RegisterFrame()).
     - A frame always has a "parent" frame. It can implicitly be the world
     frame, _or_ another frame registered by the source.
   3. Register one or more non-deformable geometries to a frame
      (RegisterGeometry()), and/or one or more deformable geometries to a frame
      (RegisterDeformableGeometry()).
     - A non-deformable geometry's pose is relative to the frame to which the
     geometry is fixed. For deformable geometries, the positions of their mesh
     vertices are expressed in the registered-in frame.
     - Rigid geometries can also be posed relative to another registered
     geometry. It will be affixed to _that_ geometry's frame.

 Anchored geometry is _independent_ of the context (i.e., it doesn't move).
 Anchored geometries are always affixed to the immobile world frame. As such,
 registering a frame is _not_ required for registering anchored geometry
 (see GeometrySource::RegisterAnchoredGeometry()). However, the source still
 "owns" the anchored geometry.

 _Updating Kinematics_

 Registering _dynamic_ non-deformable geometry implies a contract between the
 geometry source and %SceneGraph. The geometry source must do the following:
   - It must provide, populate, and connect two output ports: the "id" port and
   the "pose" port.
   - The id port must contain _all_ the frame ids returned as a result of frame
   registration.
   - The pose port must contain one pose per registered frame; the pose value is
   expressed relative to the registered frame's _parent_ frame. As mentioned
   above, the iᵗʰ pose value should describe the frame indicated by the iᵗʰ id
   in the id output port.

 Similarly, if it registers deformable geometries, the geometry source must
 provide, populate, and connect the "configuration" port. The configuration
 port must contain a vector of vertex positions per registered deformable
 geometry.

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
 variant are those methods that _do not_ change the semantics of the input or
 output ports. Modifications that make such changes must be coordinated across
 systems.
 <!-- TODO(SeanCurtis-TRI): Add context-modifying variants of all methods. -->

@section  scene_graph_versioning Detecting changes

 <!-- TODO(SeanCurtis-TRI) All references to APIs that modify versions should
  have cross links back to this section.  -->

 The geometry data associated with %SceneGraph is coarsely versioned. Consumers
 of the geometry can query for the version of the data and recognize if the
 data has been modified since last examined.

 The versioning is associated with geometry roles: proximity, illustration, and
 perception; each role has its own, independent version. Any operation
 that affects geometry with one of those roles will modify the corresponding
 version. For example:

 In C++:
 @code
 // Does *not* modify any version; no roles have been assigned.
 const GeometryId geometry_id = scene_graph.RegisterGeometry(
     source_id, frame_id, make_unique<GeometryInstance>(...));
 // Modifies the proximity version.
 scene_graph.AssignRole(source_id, geometry_id, ProximityProperties());
 // Modifies the illustration version.
 scene_graph.AssignRole(source_id, geometry_id, IllustrationProperties());
 // Modifies the perception version if there exists a renderer that accepts the
 // geometry.
 scene_graph.AssignRole(source_id, geometry_id, PerceptionProperties());
 // Modifies the illustration version.
 scene_graph.RemoveRole(source_id, geometry_id, Role::kIllustration);
 // Modifies proximity version and perception version if the geometry is
 // registered with any renderer.
 scene_graph.RemoveGeometry(source_id, geometry_id);
 @endcode

 In Python:
 @python_details_begin
 @code{.py}
 # Does *not* modify any version; no roles have been assigned.
 geometry_id = scene_graph.RegisterGeometry(
     source_id, frame_id, GeometryInstance(...))
 # Modifies the proximity version.
 scene_graph.AssignRole(source_id, geometry_id, ProximityProperties())
 # Modifies the illustration version.
 scene_graph.AssignRole(source_id, geometry_id, IllustrationProperties())
 # Modifies the perception version if there exists a renderer that accepts the
 # geometry.
 scene_graph.AssignRole(source_id, geometry_id, PerceptionProperties())
 # Modifies the illustration version.
 scene_graph.RemoveRole(source_id, geometry_id, Role.kIllustration)
 # Modifies proximity version and perception version if the geometry is
 # registered with any renderer.
 scene_graph.RemoveGeometry(source_id, geometry_id)
 @endcode
 @python_details_end

 Each copy of geometry data maintains its own set of versions.
 %SceneGraph's model has its own version, and that version is the same as the
 version in the Context provided by SceneGraph::CreateDefaultContext().
 Modifications to the geometry data contained in a Context modifies *that*
 data's version, but the original model data's version is unmodified,
 reflecting the unchanged model data.

 The geometry data's version is accessed via a SceneGraphInspector instance.
 model_inspector() will give access to %SceneGraph's model version. And
 QueryObject::inspector() will give access to the geometry data stored in a
 Context.

 Current versions can be compared against previously examined versions. If
 the versions match, then the geometry data is guaranteed to be the same.
 If they don't match, that indicates that the two sets of data underwent
 different revision processes. That, however, doesn't necessarily imply that the
 two sets of data are distinct. In other words, the versioning will report
 a difference unless it can guarantee equivalence.

 It is possible that two different contexts have different versions and a
 downstream system can be evaluated with each context alternatingly. If the
 system behavior depends on the geometry version, this will cause it to thrash
 whatever components depends on geometry version. The system should *clearly*
 document this fact.
 @cond
 // TODO(SeanCurtis-TRI): Future work which will require add'l documentation:
 //   - velocity kinematics.
 //   - Finalizing API for topology changes at discrete events.
 @endcond

 @tparam_default_scalar
 @ingroup systems
 */
template <typename T>
class SceneGraph final : public systems::LeafSystem<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(SceneGraph);

  /** Constructs a default (empty) scene graph. */
  SceneGraph();

  /** Constructs an empty scene graph with the provided configuration. */
  explicit SceneGraph(const SceneGraphConfig& config);

  /** Constructor used for scalar conversions. */
  template <typename U>
  explicit SceneGraph(const SceneGraph<U>& other);

  ~SceneGraph() final;

  /** @name      Configuration
   Allows configuration changes to scene graph systems.
   */
  //@{

  /** Sets the configuration. */
  void set_config(const SceneGraphConfig& config);

  /** @returns the current configuration. */
  const SceneGraphConfig& get_config() const;

  /** @returns the scene graph configuration from the given context.
   Note: there is no matching per-Context set_config() function. The context's
   scene graph configuration is copied from the main scene graph configuration
   at context construction time. Thereafter, the context's scene graph
   configuration is not mutable. */
  const SceneGraphConfig& get_config(const systems::Context<T>& context) const;

  //@}

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
   dynamic non-deformable geometry is registered
   (via RegisterGeometry/RegisterFrame), then the Context-dependent pose values
   must be provided on an input port. See get_source_pose_port().

   Similarly, if deformable geometry (always dynamic) is registered
   (via RegisterDeformableGeometry), then the Context-dependent configuration
   values must be provided on an input port. See
   get_source_configuration_port().

   This method modifies the underlying model and requires a new Context to be
   allocated.

   @param name          The optional name of the source. If none is provided
                        (or the empty string) a default name will be defined by
                        SceneGraph's logic.
   @throws std::exception if the name is not unique.  */
  SourceId RegisterSource(const std::string& name = "");

  /** Reports if the given source id is registered.
   @param id       The id of the source to query.  */
  bool SourceIsRegistered(SourceId id) const;

  /** Given a valid source `id`, returns a _pose_ input port associated
   with that `id`. This port is used to communicate _pose_ data for registered
   frames.
   @throws std::exception if the source_id is _not_ recognized.  */
  const systems::InputPort<T>& get_source_pose_port(SourceId id) const;

  /** Given a valid source `id`, returns a _configuration_ input port associated
   with that `id`. This port is used to communicate configuration data for
   registered deformable geometries.
   @throws std::exception if the source_id is _not_ recognized.  */
  const systems::InputPort<T>& get_source_configuration_port(SourceId id) const;

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
   - For non-deformable geometries that need to move based on the source's
     state, the geometry source must first register a GeometryFrame. In fact,
     non-deformable geometries never move directly; it is the frames to which
     they are affixed that move. A geometry source can register a frame via the
     RegisterFrame() methods.
   - Once a frame has been registered, the geometry source can register
     geometries that are rigidly affixed to that frame (or, figuratively
     speaking, "hung" on that frame). The geometry is immovably posed in that
     frame and assigned various properties. The geometry is registered via calls
     to the RegisterGeometry() methods.
   - (Experimental): Deformable geometries differ from non-deformable geometries
     in that it must have a meshed representation, and the vertices of the mesh
     can be moved in the frame the geometry is registered-in. Deformable
     geometries can be registered via calls to the (experimental)
     RegisterDeformableGeometry() methods.

   %SceneGraph has a concept of "ownership" that is separate from the C++ notion
   of ownership. All methods that *change* the world require passing the
   SourceId that was originally used to register that geometry. However,
   read-only operations on geometries do not require the SourceId, e.g., queries
   will return GeometryId values that span all sources and the properties of the
   associated geometries can be queried by arbitrary sources.

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
   @throws std::exception  if a) the `source_id` does _not_ map to a
                           registered source,
                           b) `frame` has an id that has already been
                           registered, or
                           c) there is already a frame with the
                           same name registered for the source.  */
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
   @throws std::exception  if a) the `source_id` does _not_ map to a
                           registered source,
                           b) the `parent_id` does _not_ map to a known
                           frame or does not belong to the source,
                           c) `frame` has an id that has already been
                           registered, or
                           d) there is already a frame with the
                           same name registered for the source.  */
  FrameId RegisterFrame(SourceId source_id, FrameId parent_id,
                        const GeometryFrame& frame);

  /** Renames the frame to `name`.

   This method modifies the underlying model and requires a new Context to be
   allocated. It does not modify the model versions (see @ref
   scene_graph_versioning).

   @param frame_id  The id of the frame to rename.
   @param name  The new name.
   @throws std::exception if a) the `frame_id` does not map to a valid frame,
                          or b) there is already a frame named `name` from
                          the same source. */
  void RenameFrame(FrameId frame_id, const std::string& name);

  // TODO(jwnimmer-tri) Deprecate and remove `source_id` argument, instead using
  // the source_id associated with the given `frame_id`.
  /** Registers a new rigid geometry G for this source. This hangs geometry G on
   a previously registered frame F (indicated by `frame_id`). The pose of the
   geometry is defined in a fixed pose relative to F (i.e., `X_FG`).
   Returns the corresponding unique geometry id.

   Roles will be assigned to the registered geometry if the corresponding
   GeometryInstance `geometry` has had properties assigned.

   This method modifies the underlying model and requires a new Context to be
   allocated. Potentially modifies proximity, perception, and illustration
   versions based on the roles assigned to the geometry (see @ref
   scene_graph_versioning).

   @param source_id   The id for the source registering the geometry.
   @param frame_id    The id for the frame F to hang the geometry on.
   @param geometry    The geometry G to affix to frame F.
   @return A unique identifier for the added geometry.
   @throws std::exception  if a) the `source_id` does _not_ map to a
                           registered source,
                           b) the `frame_id` doesn't belong to the source,
                           c) the `geometry` is equal to `nullptr`, or
                           d) the geometry's name doesn't satisfy the
                           requirements outlined in GeometryInstance.  */
  GeometryId RegisterGeometry(SourceId source_id, FrameId frame_id,
                              std::unique_ptr<GeometryInstance> geometry);

  // TODO(jwnimmer-tri) Deprecate and remove `source_id` argument, instead using
  // the source_id associated with the given `frame_id`.
  /** systems::Context-modifying variant of RegisterGeometry(). Rather than
   modifying %SceneGraph's model, it modifies the copy of the model stored in
   the provided context.  */
  GeometryId RegisterGeometry(systems::Context<T>* context, SourceId source_id,
                              FrameId frame_id,
                              std::unique_ptr<GeometryInstance> geometry) const;

  // TODO(jwnimmer-tri) Deprecate and remove `source_id` argument.
  /** Registers a new _anchored_ geometry G for this source. This hangs geometry
   G from the world frame (W). Its pose is defined in that frame (i.e., `X_WG`).
   Returns the corresponding unique geometry id.

   Roles will be assigned to the registered geometry if the corresponding
   GeometryInstance `geometry` has had properties assigned.

   This method modifies the underlying model and requires a new Context to be
   allocated. Potentially modifies proximity, perception, and illustration
   versions based on the roles assigned to the geometry (see @ref
   scene_graph_versioning).

   @param source_id     The id for the source registering the frame.
   @param geometry      The anchored geometry G to add to the world.
   @return A unique identifier for the added geometry.
   @throws std::exception  if a) the `source_id` does _not_ map to a
                           registered source or
                           b) the geometry's name doesn't satisfy the
                           requirements outlined in GeometryInstance.  */
  GeometryId RegisterAnchoredGeometry(
      SourceId source_id, std::unique_ptr<GeometryInstance> geometry);

  // TODO(jwnimmer-tri) Deprecate and remove `source_id` argument.
  // TODO(xuchenhan-tri): Consider allowing registering deformable geometries to
  // non-world frames.
  /** Registers a new deformable geometry G for this source. This registers
   geometry G on a frame F (indicated by `frame_id`). The registered geometry
   has a meshed representation. The positions of the vertices of this mesh
   representation are defined in the frame F (i.e., `q_FG`). Returns the
   corresponding unique geometry id.

   Roles will be assigned to the registered geometry if the corresponding
   GeometryInstance `geometry` has had properties assigned.

   This method modifies the underlying model and requires a new Context to be
   allocated. Potentially modifies proximity, perception, and illustration
   versions based on the roles assigned to the geometry (see @ref
   scene_graph_versioning).

   @experimental
   @param source_id        The id for the source registering the geometry.
   @param frame_id         The id for the frame F to put the geometry in.
   @param geometry         The geometry G to to be represented in frame F.
   @param resolution_hint  The parameter that guides the level of mesh
                           refinement of the deformable geometry. It has length
                           units (in meters) and roughly corresponds to a
                           typical edge length in the resulting mesh for a
                           primitive shape.
   @return A unique identifier for the added geometry.
   @pre resolution_hint > 0.
   @throws std::exception  if a) the `source_id` does _not_ map to a
                           registered source,
                           b) frame_id != world_frame_id(),
                           c) the `geometry` is equal to `nullptr`,
                           d) the geometry's name doesn't satisfy the
                           requirements outlined in GeometryInstance.  */
  GeometryId RegisterDeformableGeometry(
      SourceId source_id, FrameId frame_id,
      std::unique_ptr<GeometryInstance> geometry, double resolution_hint);

  // TODO(jwnimmer-tri) Deprecate and remove `source_id` argument.
  /** systems::Context-modifying variant of RegisterDeformableGeometry(). Rather
   than modifying %SceneGraph's model, it modifies the copy of the model stored
   in the provided context.
   @experimental  */
  GeometryId RegisterDeformableGeometry(
      systems::Context<T>* context, SourceId source_id, FrameId frame_id,
      std::unique_ptr<GeometryInstance> geometry, double resolution_hint) const;

  /** Renames the geometry to `name`.

   This method modifies the underlying model and requires a new Context to be
   allocated. It does not modify the model versions (see @ref
   scene_graph_versioning).

   @param geometry_id  The id of the geometry to rename.
   @param name  The new name.
   @throws std::exception if a) the `geometry_id` does not map to a valid
                          geometry, or b) `name` is not unique within any
                          assigned role of the geometry in its associated
                          frame. */
  void RenameGeometry(GeometryId geometry_id, const std::string& name);

  // TODO(jwnimmer-tri) Deprecate and remove `source_id` argument.
  /** Changes the `shape` of the geometry indicated by the given `geometry_id`.

   The geometry is otherwise unchanged -- same geometry_id, same assigned roles,
   same pose with respect to the parent (unless a new value for `X_FG` is
   given).

   This method modifies the underlying model and requires a new Context to be
   allocated. Potentially modifies proximity, perception, and illustration
   versions based on the roles assigned to the geometry (see @ref
   scene_graph_versioning).

   @param source_id    The id for the source modifying the geometry.
   @param geometry_id  The id for the geometry whose shape is being modified.
   @param shape        The new shape to use.
   @param X_FG         The (optional) new pose of the geometry in its frame. If
                       omitted, the old pose is used.

   @throws std::exception if a) the `source_id` does _not_ map to a
                           registered source,
                           b) the `geometry_id` does not map to a valid
                           geometry,
                           c) the `geometry_id` maps to a geometry that does
                           not belong to the indicated source, or
                           d) the geometry is deformable.
   @pydrake_mkdoc_identifier{model} */
  void ChangeShape(
      SourceId source_id, GeometryId geometry_id, const Shape& shape,
      std::optional<math::RigidTransform<double>> X_FG = std::nullopt);

  // TODO(jwnimmer-tri) Deprecate and remove `source_id` argument.
  /** systems::Context-modifying variant of ChangeShape(). Rather than modifying
   %SceneGraph's model, it modifies the copy of the model stored in the provided
   context.
   @pydrake_mkdoc_identifier{context} */
  void ChangeShape(
      systems::Context<T>* context, SourceId source_id, GeometryId geometry_id,
      const Shape& shape,
      std::optional<math::RigidTransform<double>> X_FG = std::nullopt);

  // TODO(jwnimmer-tri) Deprecate and remove `source_id` argument.
  /** Removes the given geometry G (indicated by `geometry_id`) from the given
   source's registered geometries. All registered geometries hanging from
   this geometry will also be removed.

   This method modifies the underlying model and requires a new Context to be
   allocated. Potentially modifies proximity, perception, and illustration
   versions based on the roles assigned to the geometry (see @ref
   scene_graph_versioning).

   @param source_id   The identifier for the owner geometry source.
   @param geometry_id The identifier of the geometry to remove (can be dynamic
                      or anchored).
   @throws std::exception  if a) the `source_id` does _not_ map to a
                           registered source,
                           b) the `geometry_id` does not map to a valid
                           geometry, or
                           c) the `geometry_id` maps to a geometry that does
                           not belong to the indicated source.  */
  void RemoveGeometry(SourceId source_id, GeometryId geometry_id);

  // TODO(jwnimmer-tri) Deprecate and remove `source_id` argument.
  /** systems::Context-modifying variant of RemoveGeometry(). Rather than
   modifying %SceneGraph's model, it modifies the copy of the model stored in
   the provided context.  */
  void RemoveGeometry(systems::Context<T>* context, SourceId source_id,
                      GeometryId geometry_id) const;

  //@}

  /** @name     Managing RenderEngine instances      */
  //@{

  /** Adds a new render engine to this %SceneGraph.
   The render engine's name should be referenced in the
   @ref render::ColorRenderCamera "ColorRenderCamera" or
   @ref render::DepthRenderCamera "DepthRenderCamera" provided in the render
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

   Modifies the perception version if `renderer` accepts any previously
   existing geometries (see @ref scene_graph_versioning).

   @param name      The unique name of the renderer.
   @param renderer  The `renderer` to add. (It will be copied / cloned, which
   means the lifetime of the argument need not extend past this call.)
   @throws std::exception if the name is not unique.  */
  void AddRenderer(std::string name, const render::RenderEngine& renderer);

  /** systems::Context-modifying variant of AddRenderer(). Rather than
   modifying %SceneGraph's model, it modifies the copy of the model stored in
   the provided context.  */
  void AddRenderer(systems::Context<T>* context, std::string name,
                   const render::RenderEngine& renderer) const;

  /** Non-copying variant of AddRenderer().
   The %SceneGraph takes ownership the render engine instead of copying it.
   The calling code must not retain a raw pointer to the renderer.
   @exclude_from_pydrake_mkdoc{Not bound in pydrake.} */
  void AddRenderer(std::string name,
                   std::unique_ptr<render::RenderEngine> renderer);

  /** Non-copying, context-modifying variant of AddRenderer().
   The %SceneGraph takes ownership the render engine instead of copying it.
   The calling code must not retain a raw pointer to the renderer.
   @exclude_from_pydrake_mkdoc{Not bound in pydrake.} */
  void AddRenderer(systems::Context<T>* context, std::string name,
                   std::unique_ptr<render::RenderEngine> renderer) const;

  /** Removes an existing renderer from this %SceneGraph
   @param name The unique name of the renderer to be removed.
   @throws std::exception if this %SceneGraph doesn't have a renderer with the
   specified name. */
  void RemoveRenderer(const std::string& name);

  /** systems::Context-modifying variant of RemoveRenderer(). Rather than
   modifying %SceneGraph's model, it modifies the copy of the model stored in
   the provided context.  */
  void RemoveRenderer(systems::Context<T>* context,
                      const std::string& name) const;

  /** Reports true if this %SceneGraph has a renderer registered with the given
   name. */
  bool HasRenderer(const std::string& name) const;

  /** systems::Context-query variant of HasRenderer(). Rather than querying
   %SceneGraph's model, it queries the copy of the model stored in the
   provided context.  */
  bool HasRenderer(const systems::Context<T>& context,
                   const std::string& name) const;

  /** Reports the type name for the RenderEngine registered with the given
   `name`.

   @returns the name of the RenderEngine's most derived type (as produced by
            NiceTypeName::Get()). An empty string if there is no RenderEngine
            registered with the given `name`. */
  std::string GetRendererTypeName(const std::string& name) const;

  /** systems::Context-query variant of GetRendererTypeName(). Rather than
   querying %SceneGraph's model, it queries the copy of the model stored in the
   provided context.  */
  std::string GetRendererTypeName(const systems::Context<T>& context,
                                  const std::string& name) const;

  /** Creates a Yaml-formatted string representing the named engine's
   parameters. The YAML will be prefixed with the paramater type's name, e.g:

       RenderEngineVtkParams:
         default_diffuse: [1, 1, 1]
         ...

   If no registered engine has the given `name`, the returned string is empty.
   */
  std::string GetRendererParameterYaml(const std::string& name) const;

  /** systems::Context-query variant of GetRendererParameterYaml(). Rather than
   querying %SceneGraph's model, it queries the copy of the model stored in the
   provided context.  */
  std::string GetRendererParameterYaml(const systems::Context<T>& context,
                                       const std::string& name) const;

  /** Reports the number of renderers registered to this %SceneGraph.  */
  int RendererCount() const;

  /** systems::Context-query variant of RendererCount(). Rather than querying
   %SceneGraph's model, it queries the copy of the model stored in the
   provided context.  */
  int RendererCount(const systems::Context<T>& context) const;

  /** Reports the names of all registered renderers.  */
  std::vector<std::string> RegisteredRendererNames() const;

  /** systems::Context-query variant of RegisteredRendererNames(). Rather than
   querying %SceneGraph's model, it queries the copy of the model stored in the
   provided context.  */
  std::vector<std::string> RegisteredRendererNames(
      const systems::Context<T>& context) const;

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

   If the geometry has previously been assigned a role, the properties for
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
   DRAKE_DEMAND(old_props != nullptr);
   ProximityProperties new_props(*old_props);
   // Add a new property.
   new_props.AddProperty("group", "new_prop_name", some_value);
   // Remove a property previously assigned.
   new_props.RemoveProperty("old_group", "old_name_1");
   // Update the *value* of an existing property (but enforce same type).
   new_props.UpdateProperty("old_group", "old_name_2", new_value);
   scene_graph.AssignRole(source_id, geometry_id, new_props,
                          RoleAssign::kReplace);
   @endcode

   Calling `AssignRole()` with an empty set of properties will *not* remove the
   role; it will simply eliminate possibly necessary properties. To remove
   the role completely, call `RemoveRole()`.

   @warning Currently, only __proximity__ and __illustration__ properties can be
   updated via this mechanism. Updating illustration properties has limitations
   (see @ref AssignRole(SourceId,GeometryId,IllustrationProperties,RoleAssign)
   "AssignRole(..., IllustrationProperties)" below). Attempting to update
   perception will throw an exception (to be implemented in the near future).

   All invocations of `AssignRole()` will throw an exception if:

     - the source id is invalid.
     - the geometry id is invalid.
     - the geometry id is not owned by the given source id.
     - Another geometry with the same name, affixed to the same frame, already
       has the role.

   <h4>Removing roles</h4>

   Calling `RemoveRole()` will remove the properties and _role_ entirely.

   These methods include the model- and context-modifying variants.    */
  //@{

  // TODO(jwnimmer-tri) Deprecate and remove `source_id` argument.
  /** Assigns the proximity role to the geometry indicated by `geometry_id`.
   Modifies the proximity version (see @ref scene_graph_versioning).
   @pydrake_mkdoc_identifier{proximity_direct}
   */
  void AssignRole(SourceId source_id, GeometryId geometry_id,
                  ProximityProperties properties,
                  RoleAssign assign = RoleAssign::kNew);

  // TODO(jwnimmer-tri) Deprecate and remove `source_id` argument.
  /** systems::Context-modifying variant of
   @ref AssignRole(SourceId,GeometryId,ProximityProperties) "AssignRole()" for
   proximity properties. Rather than modifying %SceneGraph's model, it modifies
   the copy of the model stored in the provided context.
   @pydrake_mkdoc_identifier{proximity_context}
   */
  void AssignRole(systems::Context<T>* context, SourceId source_id,
                  GeometryId geometry_id, ProximityProperties properties,
                  RoleAssign assign = RoleAssign::kNew) const;

  // TODO(jwnimmer-tri) Deprecate and remove `source_id` argument.
  /** Assigns the perception role to the geometry indicated by `geometry_id`.

   By default, a geometry with a perception role will be reified by all
   render::RenderEngine instances. This behavior can be changed. Renderers can
   be explicitly whitelisted via the ('renderer', 'accepting') perception
   property. Its type is std::set<std::string> and it contains the names of
   all the renderers that _may_ reify it. If no property is defined (or an
   empty set is given), then the default behavior of all renderers attempting
   to reify it will be restored.
   Modifies the perception version if the geometry is added to any renderer (see
   @ref scene_graph_versioning).
   @pydrake_mkdoc_identifier{perception_direct}
   */
  void AssignRole(SourceId source_id, GeometryId geometry_id,
                  PerceptionProperties properties,
                  RoleAssign assign = RoleAssign::kNew);

  // TODO(jwnimmer-tri) Deprecate and remove `source_id` argument.
  /** systems::Context-modifying variant of
   @ref AssignRole(SourceId,GeometryId,PerceptionProperties) "AssignRole()" for
   perception properties. Rather than modifying %SceneGraph's model, it modifies
   the copy of the model stored in the provided context.
   @pydrake_mkdoc_identifier{perception_context}
   */
  void AssignRole(systems::Context<T>* context, SourceId source_id,
                  GeometryId geometry_id, PerceptionProperties properties,
                  RoleAssign assign = RoleAssign::kNew) const;

  // TODO(jwnimmer-tri) Deprecate and remove `source_id` argument.
  /** Assigns the illustration role to the geometry indicated by `geometry_id`.
   Modifies the illustration version (see @ref scene_graph_versioning).

   @warning When changing illustration properties
   (`assign = RoleAssign::kReplace`), there is no guarantee that these changes
   will affect the visualization. The visualizer needs to be able to
   "initialize" itself after changes to properties that will affect how a
   geometry appears. If changing a geometry's illustration properties doesn't
   seem to be affecting the visualization, retrigger its initialization action.
   @pydrake_mkdoc_identifier{illustration_direct}
   */
  void AssignRole(SourceId source_id, GeometryId geometry_id,
                  IllustrationProperties properties,
                  RoleAssign assign = RoleAssign::kNew);

  // TODO(jwnimmer-tri) Deprecate and remove `source_id` argument.
  /** systems::Context-modifying variant of
   @ref AssignRole(SourceId,GeometryId,IllustrationProperties) "AssignRole()"
   for illustration properties. Rather than modifying %SceneGraph's model, it
   modifies the copy of the model stored in the provided context.

   @warning When changing illustration properties
   (`assign = RoleAssign::kReplace`), there is no guarantee that these changes
   will affect the visualization. The visualizer needs to be able to
   "initialize" itself after changes to properties that will affect how a
   geometry appears. If changing a geometry's illustration properties doesn't
   seem to be affecting the visualization, retrigger its initialization action.
   @pydrake_mkdoc_identifier{illustration_context}
   */
  void AssignRole(systems::Context<T>* context, SourceId source_id,
                  GeometryId geometry_id, IllustrationProperties properties,
                  RoleAssign assign = RoleAssign::kNew) const;

  // TODO(jwnimmer-tri) Deprecate and remove `source_id` argument.
  /** Removes the indicated `role` from any geometry directly registered to the
   frame indicated by `frame_id` (if the geometry has the role).
   Potentially modifies the proximity, perception, or illustration version based
   on the role being removed from the geometry (see @ref
   scene_graph_versioning).
   @returns The number of geometries affected by the removed role.
   @throws std::exception if a) `source_id` does not map to a registered
                          source,
                          b) `frame_id` does not map to a registered frame,
                          c) `frame_id` does not belong to `source_id`
                          (unless `frame_id` is the world frame id), or
                          d) the context has already been allocated.
   @pydrake_mkdoc_identifier{frame_direct}  */
  int RemoveRole(SourceId source_id, FrameId frame_id, Role role);

  // TODO(jwnimmer-tri) Deprecate and remove `source_id` argument.
  /** systems::Context-modifying variant of
   @ref RemoveRole(SourceId,FrameId,Role) "RemoveRole()" for frames.
   Rather than modifying %SceneGraph's model, it modifies the copy of the model
   stored in the provided context.
   @pydrake_mkdoc_identifier{frame_context}  */
  int RemoveRole(systems::Context<T>* context, SourceId source_id,
                 FrameId frame_id, Role role) const;

  // TODO(jwnimmer-tri) Deprecate and remove `source_id` argument.
  /** Removes the indicated `role` from the geometry indicated by `geometry_id`.
   Potentially modifies the proximity, perception, or illustration version based
   on the role being removed from the geometry (see @ref
   scene_graph_versioning).
   @returns One if the geometry had the role removed and zero if the geometry
            did not have the role assigned in the first place.
   @throws std::exception if a) `source_id` does not map to a registered
                          source,
                          b) `geometry_id` does not map to a registered
                          geometry,
                          c) `geometry_id` does not belong to `source_id`, or
                          d) the context has already been allocated.
   @pydrake_mkdoc_identifier{geometry_direct}  */
  int RemoveRole(SourceId source_id, GeometryId geometry_id, Role role);

  // TODO(jwnimmer-tri) Deprecate and remove `source_id` argument.
  /** systems::Context-modifying variant of
   @ref RemoveRole(SourceId,GeometryId,Role) "RemoveRole()" for individual
   geometries. Rather than modifying %SceneGraph's model, it modifies the copy
   of the model stored in the provided context.
   @pydrake_mkdoc_identifier{geometry_context}  */
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
   @anchor scene_graph_collision_filter_manager

   Control over "collision filtering" is handled by the CollisionFilterManager.
   %SceneGraph provides access to the manager. As with other geometry data,
   collision filters can be configured in %SceneGraph's *model* or in the copy
   stored in a particular Context. These methods provide access to the manager
   for the data stored in either location.

   %SceneGraph implicitly filters collisions between rigid geometries affixed to
   the same frame. This allows representation of complex shapes by providing a
   union of simpler shapes without producing spurious collisions between those
   overlapping shapes. %SceneGraph doesn't create *any* collision filters for
   deformable geometries automatically. Users can add filters to deformable
   geometries as they require after registration.

   Generally, it should be considered a bad practice to hang onto the instance
   of CollisionFilterManager returned by collision_filter_manager(). It is not
   immediately clear whether a particular CollisionFilterManager instance
   refers to the %SceneGraph model or the Context data and persisting the
   reference may lead to confusion. Keeping the reference for the duration of
   a function is appropriate, but allowing it to persist outside of the scope
   of acquisition is dangerous. Acquiring a new CollisionFilterManager is *very*
   cheap, so feel free to discard and reacquire.

   Simply acquiring an instance of CollisionFilterManager will advance the
   @ref scene_graph_versioning "proximity version" for the related geometry
   data (model or context).

   @note %SceneGraph does not track topology or semantic information of models,
   so decisions about *what* to filter belong in software layers that have the
   necessary information. For example, some automatic filtering is done in
   @ref mbp_finalize_stage "MultibodyPlant::Finalize()". Applications may
   need to add more filtering or adjust filters during simulation.
  */
  //@{

  /** Returns the collision filter manager for this %SceneGraph instance's
   *model*. */
  CollisionFilterManager collision_filter_manager();

  /** Returns the collision filter manager for data stored in `context`. The
   context must remain alive for at least as long as the returned manager.  */
  CollisionFilterManager collision_filter_manager(
      systems::Context<T>* context) const;
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

  // Writes the current version of the geometry data to the context's abstract
  // parameter.
  void SetDefaultParameters(const systems::Context<T>& context,
                            systems::Parameters<T>* parameters) const override;

  // Helper class to register input ports for a source id.
  void MakeSourcePorts(SourceId source_id);

  // Sets the context into the output port value so downstream consumers can
  // perform queries.
  void CalcQueryObject(const systems::Context<T>& context,
                       QueryObject<T>* output) const;

  // Collects all of the *dynamic* frames that have geometries with the given
  // role.
  std::vector<FrameId> GetDynamicFrames(const GeometryState<T>& g_state,
                                        Role role) const;

  // Refreshes the pose of the various engines which exploits the caching
  // infrastructure.
  void FullPoseUpdate(const systems::Context<T>& context) const {
    this->get_cache_entry(pose_update_index_).template Eval<int>(context);
  }

  // Refreshes the configuration of deformable geometries in various engines
  // which exploits the caching infrastructure.
  void FullConfigurationUpdate(const systems::Context<T>& context) const {
    this->get_cache_entry(configuration_update_index_)
        .template Eval<int>(context);
  }

  // Updates the state of geometry world from all pose inputs. This is the calc
  // method for the corresponding cache entry. The entry *value* (the int) is
  // strictly a dummy -- the value is unimportant; only the side effect matters.
  void CalcPoseUpdate(const systems::Context<T>& context, int*) const;

  // Updates the state of geometry world from all configuration inputs. This is
  // the calc method for the corresponding cache entry. The entry *value* (the
  // int) is strictly a dummy -- the value is unimportant; only the side effect
  // matters.
  void CalcConfigurationUpdate(const systems::Context<T>& context, int*) const;

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
    int configuration_port{-1};
  };

  // A mapping from added source identifier to the port indices associated with
  // that id.
  std::unordered_map<SourceId, SourcePorts> input_source_ids_;

  // The index of the output port with the QueryObject abstract value.
  int query_port_index_{-1};

  // Encapsulate some model detail to help enforce internal invariants.
  class Hub;
  std::unique_ptr<Hub> owned_hub_;
  class Hub& hub_;

  SceneGraphInspector<T> model_inspector_;

  // The geometry state is stored in the Context as a Parameter with this
  // index.
  int geometry_state_index_{-1};

  // The scene graph configuration from the time the context was created is
  // stored in the Context as a Parameter with this index.
  int scene_graph_config_index_{-1};

  // The cache indices for the pose and configuration update cache entries.
  systems::CacheIndex pose_update_index_{};
  systems::CacheIndex configuration_update_index_{};

  // (Testing only) a global count of calls to the scalar converting
  // constructor.
  static int64_t scalar_conversion_count_;
};

}  // namespace geometry
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::geometry::SceneGraph);
