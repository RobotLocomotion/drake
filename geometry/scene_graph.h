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
class GeometryContext;

template <typename T>
class QueryObject;

/** SceneGraph serves as the nexus for all geometry (and geometry-based
 operations) in a Diagram. Through SceneGraph, other systems that introduce
 geometry can _register_ that geometry as part of a common global domain,
 including it in geometric queries (e.g., cars controlled by one LeafSystem can
 be observed by a different sensor system). SceneGraph provides the
 interface for registering the geometry, updating its position based on the
 current context, and performing geometric queries.

 Only registered "geometry sources" can introduce geometry into %SceneGraph.
 Geometry sources will typically be other leaf systems, but, in the case of
 _anchored_ (i.e., stationary) geometry, it could also be some other block of
 code (e.g., adding a common ground plane with which all systems' geometries
 interact). For dynamic geometry (geometry whose pose depends on a Context), the
 geometry source must also provide pose values for all of the geometries the
 source owns, via a port connection on %SceneGraph.

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
 See the details in FrameKinematicsVector for details on how to allocate and
 calculate this port.

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

 @tparam T The scalar type. Must be a valid Eigen scalar.

 Instantiated templates for the following kinds of T's are provided:

 - double
 - AutoDiffXd

 They are already available to link against in the containing library.
 No other values for T are currently supported.

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

  /** Registers a new source to the geometry system. The caller must save the
   returned SourceId; it is the token by which all other operations on the
   geometry world are conducted.

   This source id can be used to register arbitrary _anchored_ geometry. But if
   dynamic geometry is registered (via RegisterGeometry/RegisterFrame), then
   the context-dependent pose values must be provided on an input port.
   See get_source_pose_port().
   @param name          The optional name of the source. If none is provided
                        (or the empty string) a unique name will be defined by
                        SceneGraph's logic.
   @throws std::logic_error if a context has already been allocated for this
                            %SceneGraph.
   @see GeometryState::RegisterNewSource()  */
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
   This includes registering a new geometry source, adding or
   removing frames, and adding or removing geometries.

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

  /** Registers a new frame F on for this source. This hangs frame F on the
   world frame (W). Its pose is defined relative to the world frame (i.e,
   `X_WF`). Returns the corresponding unique frame id.
   @param source_id     The id for the source registering the frame.
   @param frame         The definition of the frame to add.
   @returns  A newly allocated frame id.
   @throws std::logic_error  If the `source_id` does _not_ map to a registered
                             source or if a context has been allocated.  */
  FrameId RegisterFrame(SourceId source_id, const GeometryFrame& frame);

  /** Registers a new frame F for this source. This hangs frame F on another
   previously registered frame P (indicated by `parent_id`). The pose of the new
   frame is defined relative to the parent frame (i.e., `X_PF`).  Returns the
   corresponding unique frame id.
   @param source_id    The id for the source registering the frame.
   @param parent_id    The id of the parent frame P.
   @param frame        The frame to register.
   @returns  A newly allocated frame id.
   @throws std::logic_error  1. If the `source_id` does _not_ map to a
                             registered source,
                             2. If the `parent_id` does _not_ map to a known
                             frame or does not belong to the source, or
                             3. a context has been allocated.  */
  FrameId RegisterFrame(SourceId source_id, FrameId parent_id,
                        const GeometryFrame& frame);

  /** Registers a new geometry G for this source. This hangs geometry G on a
   previously registered frame F (indicated by `frame_id`). The pose of the
   geometry is defined in a fixed pose relative to F (i.e., `X_FG`).
   Returns the corresponding unique geometry id.

   Roles will be assigned to the geometry if the corresponding properties have
   been assigned to the instance.

   @param source_id   The id for the source registering the geometry.
   @param frame_id    The id for the frame F to hang the geometry on.
   @param geometry    The geometry G to affix to frame F.
   @return A unique identifier for the added geometry.
   @throws std::logic_error  1. the `source_id` does _not_ map to a registered
                             source,
                             2. the `frame_id` doesn't belong to the source,
                             3. the `geometry` is equal to `nullptr`,
                             4. a context has been allocated, or
                             5. the geometry's name doesn't satisfy the
                             requirements outlined in GeometryInstance.  */
  GeometryId RegisterGeometry(SourceId source_id, FrameId frame_id,
                              std::unique_ptr<GeometryInstance> geometry);

  /** systems::Context-modifying variant of RegisterGeometry(). Rather than
   modifying %SceneGraph's model, it modifies the copy of the model stored in
   the provided context.  */
  GeometryId RegisterGeometry(systems::Context<T>* context, SourceId source_id,
                              FrameId frame_id,
                              std::unique_ptr<GeometryInstance> geometry);

  /** Registers a new geometry G for this source. This hangs geometry G on a
   previously registered geometry P (indicated by `geometry_id`). The pose of
   the geometry is defined in a fixed pose relative to geometry P (i.e.,
   `X_PG`). By induction, this geometry is effectively rigidly affixed to the
   frame that P is affixed to. Returns the corresponding unique geometry id.

   Roles will be assigned to the geometry if the corresponding properties have
   been assigned to the instance.

   @param source_id    The id for the source registering the geometry.
   @param geometry_id  The id for the parent geometry P.
   @param geometry     The geometry G to add.
   @return A unique identifier for the added geometry.
   @throws std::logic_error 1. the `source_id` does _not_ map to a registered
                            source,
                            2. the `geometry_id` doesn't belong to the source,
                            3. the `geometry` is equal to `nullptr`,
                            4. a context has been allocated, or
                            5. the geometry's name doesn't satisfy the
                            requirements outlined in GeometryInstance.  */
  GeometryId RegisterGeometry(SourceId source_id, GeometryId geometry_id,
                              std::unique_ptr<GeometryInstance> geometry);

  /** systems::Context-modifying variant of RegisterGeometry(). Rather than
   modifying %SceneGraph's model, it modifies the copy of the model stored in
   the provided context.  */
  GeometryId RegisterGeometry(systems::Context<T>* context, SourceId source_id,
                              GeometryId geometry_id,
                              std::unique_ptr<GeometryInstance> geometry);

  /** Registers a new _anchored_ geometry G for this source. This hangs geometry
   G from the world frame (W). Its pose is defined in that frame (i.e., `X_WG`).
   Returns the corresponding unique geometry id.

   Roles will be assigned to the geometry if the corresponding properties have
   been assigned to the instance.

   @param source_id     The id for the source registering the frame.
   @param geometry      The anchored geometry G to add to the world.
   @returns The index for the added geometry.
   @throws std::logic_error  1. the `source_id` does _not_ map to a registered
                             source,
                             2. a context has been allocated, or
                             3. the geometry's name doesn't satisfy the
                             requirements outlined in GeometryInstance.  */
  GeometryId RegisterAnchoredGeometry(
      SourceId source_id, std::unique_ptr<GeometryInstance> geometry);

  /** Removes the given geometry G (indicated by `geometry_id`) from the given
   source's registered geometries. All registered geometries hanging from
   this geometry will also be removed.
   @param source_id   The identifier for the owner geometry source.
   @param geometry_id The identifier of the geometry to remove.
   @throws std::logic_error If:
                            1. The `source_id` is not a registered source,
                            2. the `geometry_id` doesn't belong to the source,
                               or
                            3. a context has been allocated.  */
  void RemoveGeometry(SourceId source_id, GeometryId geometry_id);

  /** systems::Context-modifying variant of RemoveGeometry(). Rather than
   modifying %SceneGraph's model, it modifies the copy of the model stored in
   the provided context.  */
  void RemoveGeometry(systems::Context<T>* context, SourceId source_id,
                      GeometryId geometry_id);

  //@}

  /** @name     Assigning roles to geometry

   Geometries must be assigned one or more *roles* before they have an effect
   on SceneGraph computations (see @ref geometry_roles for details). These
   methods provide the ability to assign a role after registering a geometry.

   The owner that registered the geometry provides its source id, the registered
   geometry id, and a collection of properties associated with the desired role.
   These methods will throw exceptions in any of the following circumstances:

     - The source id is invalid.
     - The geometry id is invalid.
     - The geometry id is not owned by the given source id.
     - The indicated role has already been assigned to the geometry.
     - A context has been allocated.
   */

  // TODO(SeanCurtis-TRI): Provide mechanism for modifying properties and/or
  // removing roles.

  //@{

  /** Assigns the proximity role to the given geometry.  */
  void AssignRole(SourceId source_id, GeometryId geometry_id,
                  ProximityProperties properties);

  /** Assigns the illustration role to the given geometry.  */
  void AssignRole(SourceId source_id, GeometryId geometry_id,
                  IllustrationProperties properties);

  //@}

  /** Reports the identifier for the world frame.  */
  static FrameId world_frame_id() {
    return internal::InternalFrame::world_frame_id();
  }

  /** Returns an inspector on the system's *model* scene graph data.
   @throws std::logic_error If a context has been allocated.*/
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

   Modifications to C _must_ be performed before context allocation.
   */
  //@{

  /** Excludes geometry pairs from collision evaluation by updating the
   candidate pair set `C = C - P`, where `P = {(gᵢ, gⱼ)}, ∀ gᵢ, gⱼ ∈ G` and
   `G = {g₀, g₁, ..., gₘ}` is the input `set` of geometries.

   If the set includes geometries which have _not_ been assigned a proximity
   role, those geometries will be ignored. If a proximity role is subsequently
   assigned, those geometries will _still_ not be part of any collision filters.
   Proximity roles should _generally_ be assigned prior to collision filter
   configuration.

   @throws std::logic_error if the set includes ids that don't exist in the
                            scene graph.  */
  void ExcludeCollisionsWithin(const GeometrySet& set);

  /** systems::Context-modifying variant of ExcludeCollisionsWithin(). Rather
   than modifying %SceneGraph's model, it modifies the copy of the model stored
   in the provided context.  */
  void ExcludeCollisionsWithin(systems::Context<T>* context,
                               const GeometrySet& set);

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

   @throws std::logic_error if the groups include ids that don't exist in the
                            scene graph.   */
  void ExcludeCollisionsBetween(const GeometrySet& setA,
                                const GeometrySet& setB);

  /** systems::Context-modifying variant of ExcludeCollisionsBetween(). Rather
   than modifying %SceneGraph's model, it modifies the copy of the model stored
   in the provided context.  */
  void ExcludeCollisionsBetween(systems::Context<T>* context,
                                const GeometrySet& setA,
                                const GeometrySet& setB);
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

  // The two output ports (bundle and query object) depend on all input
  // kinematics (more or less). This makes those relationships concrete and
  // official even if/when this class is made symbolic-compatible (or the
  // default non-symbolic-compatible behavior were to change).
  optional<bool> DoHasDirectFeedthrough(int, int) const override {
    return true;
  }

  // Helper class to register input ports for a source id.
  void MakeSourcePorts(SourceId source_id);

  // Allow the load dispatch to peek into SceneGraph.
  friend void DispatchLoadMessage(const SceneGraph<double>&,
                                  lcm::DrakeLcmInterface*);

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

  // Updates the state of geometry world from *all* the inputs.
  void FullPoseUpdate(const GeometryContext<T>& context) const;

  // Override of construction to account for
  //    - instantiating a GeometryContext instance (as opposed to LeafContext),
  //    - to detect allocation in support of the topology semantics described
  //      above.
  std::unique_ptr<systems::LeafContext<T>> DoMakeLeafContext() const override;

  // Helper method for throwing an exception if a context has *ever* been
  // allocated by this system. The invoking method should pass it's name so
  // that the error message can include that detail.
  void ThrowIfContextAllocated(const char* source_method) const;

  // Asserts the given source_id is registered, throwing an exception whose
  // message is the given message with the source_id appended if not.
  void ThrowUnlessRegistered(SourceId source_id, const char* message) const;

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

  // TODO(SeanCurtis-TRI): Get rid of this.
  mutable bool context_has_been_allocated_{false};

  // The index of the geometry state in the context's abstract state.
  int geometry_state_index_{-1};
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
