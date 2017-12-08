#pragma once

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "drake/geometry/geometry_state.h"
#include "drake/geometry/query_handle.h"
#include "drake/geometry/query_results/penetration_as_point_pair.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/rendering/pose_bundle.h"

namespace drake {
namespace geometry {

class GeometryInstance;

template <typename T> class GeometryContext;

/** GeometrySystem serves as a system-level wrapper for GeometryWorld. It serves
 as the nexus for all geometry (and geometry-based operations) in a Diagram.
 Through GeometrySystem, other systems that introduce geometry can _register_
 that geometry as part of a common global domain, including it in geometric
 queries (e.g., cars controlled by one LeafSystem can be observed by a different
 sensor system). GeometrySystem provides the interface for registering the
 geometry, updating its position based on the current context, and performing
 geometric queries.

 Only registered "geometry sources" can introduce geometry into %GeometrySystem.
 Geometry sources will typically be other leaf systems, but, in the case of
 _anchored_ (i.e., stationary) geometry, it could also be some other block of
 code (e.g., adding a common ground plane with which all systems' geometries
 interact). For dynamic geometry (geometry whose pose depends on a Context), the
 geometry source must also provide pose values for all of the geometries the
 source owns, via a port connection on %GeometrySystem.

 The basic workflow for interacting with %GeometrySystem is:
   - Register as a geometry source, acquiring a unique SourceId.
   - Register geometry (anchored and dynamic) with the system.
   - Connect source's geometry output ports to the corresponding %GeometrySystem
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

 For each registered geometry source, there are _two_ input ports: id and pose.
 Failing to connect to those ports or providing "bad" values on
 those ports will cause runtime errors to be thrown. The two ports work in
 tandem. Through these ports, the upstream source system communicates the
 poses of all of the _frames_ it has registered with %GeometrySystem (see
 RegisterFrame() for more details).

 __identifier port__: An abstract-valued port containing an instance of
 FrameIdVector. It should contain the FrameId of each frame registered by the
 upstream source exactly once. The _order_ of the ids is how the values in the
 pose port will be interpreted. Use get_source_frame_id_port() to acquire the
 port for a given source.

 __pose port__: An abstract-valued port containing an instance of
 FramePoseVector. There should be one pose value for each id in the the
 identifier port value. The iᵗʰ pose belongs to the iᵗʰ id. Use
 get_source_pose_port() to acquire the port for a given source.

 For source systems, there are some implicit assumptions regarding these input
 ports. Generally, we assume that the source system already has some logic for
 computing kinematics of the frames they've registered and an ordered data
 structure for organizing that data. These input ports rely on that. It is
 expected that the geometry source will define the frame identifiers in an order
 which matches the source's internal ordering (and never need to change that
 output value unless the topology changes). The values of the pose port can
 then simply be written by copying the ordered data from the internal ordering
 to the output ordering. This should facilitate translation from internal
 representation to GeometrySystem representation.

 @section geom_sys_outputs Outputs

 %GeometrySystem has two output ports:

 __query port__: An abstract-valued port containing an instance of QueryHandle.
 It provides a "ticket" for downstream LeafSystem instances to perform geometric
 queries on the %GeometrySystem. To perform geometric queries, downstream
 LeafSystem instances acquire the QueryHandle from %GeometrySystem's output port
 and provide it as a parameter to one of %GeometrySystem's query methods (e.g.,
 GeometrySystem::ComputeContact()). This assumes that the querying system has
 access to a const pointer to the connected %GeometrySystem instance. Use
 get_query_output_port() to acquire the output port for the query handle.

 __lcm visualization port__: An abstract-valued port containing an instance of
 PoseBundle. This is a convenience port designed to feed LCM update messages to
 drake_visualizer for the purpose of visualizing the state of the world's
 geometry. Additional uses of this port are strongly discouraged; instead, use
 an appropriate geometric query to obtain the state of the world's geometry.

 @section geom_sys_workflow Working with GeometrySystem

 LeafSystem instances can relate to GeometrySystem in one of two ways: as a
 _consumer_ that performs queries, or as a _producer_ that introduces geometry
 into the shared world and defines its context-dependent kinematics values.
 It is reasonable for systems to perform either role singly, or both.

 __Consumer__

 Consumers perform geometric queries upon the world geometry. %GeometrySystem
 _serves_ those queries. As indicated above, in order for a LeafSystem to act
 as a consumer, it must:
   1. define a QueryHandle-valued input port and connect it to %GeometrySystem's
   corresponding output port, and
   2. have a reference to the connected %GeometrySystem instance.

 With those two requirements satisfied, a LeafSystem can perform geometry
 queries by:
   1. evaluating the QueryHandle input port, and
   2. passing the returned handle into the appropriate query method on
   GeometrySystem (e.g., GeometrySystem::ComputeContact()).

 __Producer__

 All producers introduce geometry into the shared geometric world. This is
 called _registering_ geometry. Depending on what exactly has been registered,
 a producer may also have to _update kinematics_. Producers themselves must be
 registered with %GeometrySystem as producers (a.k.a. _geometry sources_). They
 do this by acquiring a SourceId (via GeometrySystem::RegisterSource()). The
 SourceId serves as a unique handle through which the producer's identity is
 validated and its ownership of its registered geometry is maintained.

 _Registering Geometry_

 %GeometrySystem cannot know what geometry _should_ be part of the shared world.
 Other systems are responsible for introducing geometry into the world. This
 process (defining geometry and informing %GeometrySystem) is called
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
 connecting to the appropriate input port on %GeometrySystem). The work flow is
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
 and %GeometrySystem. The geometry source must do the following:
   - It must provide, populate, and connect two output ports: the "id" port and
   the "pose" port.
   - The id port must contain _all_ the frame ids returned as a result of frame
   registration.
   - The pose port must contain one pose per registered frame; the pose value is
   expressed relative to the registered frame's _parent_ frame. As mentioned
   above, the iᵗʰ pose value should describe the frame indicated by the iᵗʰ id
   in the id output port.

 Failure to meet these requirements will lead to a run-time error.

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
 No other values for T are currently supported.  */
template <typename T>
class GeometrySystem final : public systems::LeafSystem<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(GeometrySystem)

  GeometrySystem();

  /** Constructor used for scalar conversions. It should only be used to convert
   _from_ double _to_ other scalar types. */
  template <typename U>
  explicit GeometrySystem(const GeometrySystem<U>& other);

  ~GeometrySystem() override {}

  /** @name       Port management
   Access to GeometrySystem's input/output ports. This topic includes
   registration of geometry sources because the input ports are mapped to
   registered geometry sources.

   A source that registers frames and geometries _must_ connect outputs to
   the inputs associated with that source. Failure to do so will be treated as
   a runtime error during the evaluation of %GeometrySystem. %GeometrySystem
   will detect that frames have been registered but no values have been
   provided. */
  //@{

  /** Registers a new source to the geometry system (see GeometryWorld for the
   discussion of "geometry source"). The caller must save the returned SourceId;
   it is the token by which all other operations on the geometry world are
   conducted.

   This source id can be used to register arbitrary _anchored_ geometry. But if
   dynamic geometry is registered (via RegisterGeometry/RegisterFrame), then
   the context-dependent pose values must be provided on an input port.
   See get_source_frame_id_port() and get_source_pose_port().
   @param name          The optional name of the source. If none is provided
                        (or the empty string) a unique name will be defined by
                        GeometrySystem's logic.
   @throws std::logic_error if a context has already been allocated for this
                            %GeometrySystem.
   @see GeometryState::RegisterNewSource() */
  SourceId RegisterSource(const std::string &name = "");

  /** Given a valid source `id`, returns the "frame id" input port associated
   with that `id`. This port's value is an ordered list of frame ids; it
   is used to provide an interpretation on the pose values provided on the
   pose port.
   @throws  std::logic_error if the source_id is _not_ recognized. */
  const systems::InputPortDescriptor<T>& get_source_frame_id_port(SourceId id);

  /** Given a valid source `id`, returns a _pose_ input port associated
   with that `id`. This port is used to communicate _pose_ data for registered
   frames.
   @throws  std::logic_error if the source_id is _not_ recognized. */
  const systems::InputPortDescriptor<T>& get_source_pose_port(SourceId id);

  /** Returns the output port which produces the PoseBundle for LCM
   communication to drake visualizer. */
  const systems::OutputPort<T>& get_pose_bundle_output_port() const {
    return systems::System<T>::get_output_port(bundle_port_index_);
  }

  /** Returns the output port which produces the QueryHandle for performing
   geometric queries. */
  const systems::OutputPort<T>& get_query_output_port() const {
    return systems::System<T>::get_output_port(query_port_index_);
  }

  //@}

  /** @name             Topology Manipulation
   Topology manipulation consists of changing the data contained in
   GeometryWorld. This includes registering a new geometry source, adding or
   removing frames, and adding or removing geometries.

   Currently, the topology can only be manipulated during initialization.
   Eventually, the API will expand to include modifications of the topology
   during discrete updates.

   The initialization phase begins with the instantiation of a %GeometrySystem
   and ends when a context is allocated by the %GeometrySystem instance. This is
   the only phase when geometry sources can be registered with GeometryWorld.
   Once a source is registered, it can register frames and geometries. Any
   frames and geometries registered during this phase become part of the
   _default_ context state for %GeometrySystem and calls to
   CreateDefaultContext() will produce identical contexts.

   Every geometry must ultimately be associated with a parent frame.
   The position of every geometry in the world depends on a hierarchy of frames
   between it and the world. The pose of a geometry is described relative
   to its parent (a frame or another geometry). That parent may, in turn, also
   have a parent. So, the position of a particular geometry in the world frame
   depends on all of its ancestors which lie between it and the world frame. The
   act of assigning a frame or geometry as a child to another frame or geometry
   (as appropriate) and defining its pose, is referred to colloquially has
   "hanging" it on the parent.

   Geometry sources can only hang frames or geometries onto other frames and/or
   geometries that it "owns".
   */
  //@{

  /** Registers a new frame F on for this source. This hangs frame F on the
   world frame (W). Its pose is defined relative to the world frame (i.e,
   `X_WF`). Returns the corresponding unique frame id.
   @param source_id     The id for the source registering the frame.
   @param frame         The definition of the frame to add.
   @returns  A newly allocated frame id.
   @throws std::logic_error  If the `source_id` does _not_ map to a registered
                             source or if a context has been allocated. */
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
                             3. a context has been allocated. */
  FrameId RegisterFrame(SourceId source_id, FrameId parent_id,
                        const GeometryFrame& frame);

  /** Registers a new geometry G for this source. This hangs geometry G on a
   previously registered frame F (indicated by `frame_id`). The pose of the
   geometry is defined in a fixed pose relative to F (i.e., `X_FG`).
   Returns the corresponding unique geometry id.
   @param source_id   The id for the source registering the geometry.
   @param frame_id    The id for the frame F to hang the geometry on.
   @param geometry    The geometry G to affix to frame F.
   @return A unique identifier for the added geometry.
   @throws std::logic_error  1. the `source_id` does _not_ map to a registered
                             source,
                             2. the `frame_id` doesn't belong to the source,
                             3. the `geometry` is equal to `nullptr`,
                             4. a context has been allocated. */
  GeometryId RegisterGeometry(SourceId source_id,
                              FrameId frame_id,
                              std::unique_ptr<GeometryInstance> geometry);

  /** Registers a new geometry G for this source. This hangs geometry G on a
   previously registered geometry P (indicated by `geometry_id`). The pose of
   the geometry is defined in a fixed pose relative to geometry P (i.e.,
   `X_PG`). By induction, this geometry is effectively rigidly affixed to the
   frame that P is affixed to. Returns the corresponding unique geometry id.

   @param source_id    The id for the source registering the geometry.
   @param geometry_id  The id for the parent geometry P.
   @param geometry     The geometry G to add.
   @return A unique identifier for the added geometry.
   @throws std::logic_error 1. the `source_id` does _not_ map to a registered
                            source,
                            2. the `geometry_id` doesn't belong to the source,
                            3. the `geometry` is equal to `nullptr`, or
                            4. a context has been allocated. */
  GeometryId RegisterGeometry(SourceId source_id,
                              GeometryId geometry_id,
                              std::unique_ptr<GeometryInstance> geometry);

  /** Registers a new _anchored_ geometry G for this source. This hangs geometry
   G from the world frame (W). Its pose is defined in that frame (i.e., `X_WG`).
   Returns the corresponding unique geometry id.
   @param source_id     The id for the source registering the frame.
   @param geometry      The anchored geometry G to add to the world.
   @returns The index for the added geometry.
   @throws std::logic_error  If the `source_id` does _not_ map to a registered
                             source or a context has been allocated. */
  GeometryId RegisterAnchoredGeometry(
      SourceId source_id,
      std::unique_ptr<GeometryInstance> geometry);

  /** Clears of all the registered frames and geometries from this source, but
   the source is still registered, allowing future registration of frames
   and geometries.
   @param source_id   The id of the source whose registered elements will be
                      cleared.
   @throws std::logic_error  If the `source_id` does _not_ map to a registered
                             source or if a context has been allocated. */
  void ClearSource(SourceId source_id);

  /** Removes the given frame F (indicated by `frame_id`) from the the given
   source's registered frames. All registered geometries connected to this frame
   will also be removed.
   @param source_id   The id for the owner geometry source.
   @param frame_id    The id of the frame to remove.
   @throws std::logic_error If:
                            1. The `source_id` is not a registered source,
                            2. the `frame_id` doesn't belong to the source, or
                            3. a context has been allocated. */
  void RemoveFrame(SourceId source_id, FrameId frame_id);

  /** Removes the given geometry G (indicated by `geometry_id`) from the the
   given source's registered geometries. All registered geometries hanging from
   this geometry will also be removed.
   @param source_id   The identifier for the owner geometry source.
   @param geometry_id The identifier of the geometry to remove.
   @throws std::logic_error If:
                            1. The `source_id` is not a registered source,
                            2. the `geometry_id` doesn't belong to the source,
                               or
                            3. a context has been allocated. */
  void RemoveGeometry(SourceId source_id, GeometryId geometry_id);

  //@}

  /** @name     System Queries
   These methods perform queries on the state of the geometry world including:
   proximity queries, contact queries, ray-casting queries, and look ups on
   geometry resources.

   These operations require a QueryHandle instance. The caller must acquire one
   from the %GeometrySystem by connecting to the output port that provides
   GeometryQuery instances. */
  //@{

  /** Reports the name for the given source id.
   @param handle   The QueryHandle produced by evaluating the connected
                   input port on the querying LeafSystem.
   @param id       The id of the source to query. */
  const std::string& get_source_name(const QueryHandle<T>& handle,
                                     SourceId id) const;

  /** Reports if the given source id is registered.
   @param id       The id of the source to query. */
  bool SourceIsRegistered(SourceId id) const;

  /** Reports the frame to which this geometry is registered.
   @param handle   The QueryHandle produced by evaluating the connected
                   input port on the querying LeafSystem. */
  FrameId GetFrameId(const QueryHandle<T>& handle,
                     GeometryId geometry_id) const;

  /** Determines penetrations across all pairs of geometries in GeometryWorld.
   @param handle   The QueryHandle produced by evaluating the connected
                   input port on the querying LeafSystem.
   @returns A vector populated with all detected penetrations characterized as
            point pairs. */
  std::vector<PenetrationAsPointPair<T>> ComputePenetration(
      const QueryHandle<T>& handle) const;

  // TODO(SeanCurtis-TRI): Flesh this out with the full set of queries.

  //@}

 private:
  // Friend class to facilitate testing.
  friend class GeometrySystemTester;

  // GeometrySystem of different scalar types can all access each other's data.
  template <typename>
  friend class GeometrySystem;

  // Helper class to register input ports for a source id.
  void MakeSourcePorts(SourceId source_id);

  // Allow the load dispatch to peek into GeometrySystem.
  friend void DispatchLoadMessage(const GeometrySystem<double>&);

  // Constructs a QueryHandle for OutputPort allocation.
  QueryHandle<T> MakeQueryHandle(const systems::Context<T>& context) const;

  // Sets the context into the output port value so downstream consumers can
  // perform queries.
  void CalcQueryHandle(const systems::Context<T>& context,
                      QueryHandle<T>* output) const;

  // Constructs a PoseBundle of length equal to the concatenation of all inputs.
  // This is the method used by the allocator for the output port.
  systems::rendering::PoseBundle<T> MakePoseBundle(
      const systems::Context<T>& context) const;

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
  std::unique_ptr<systems::LeafContext<T>> DoMakeContext() const override;

  // Helper method for throwing an exception if a context has *ever* been
  // allocated by this system. The invoking method should pass it's name so
  // that the error message can include that detail.
  void ThrowIfContextAllocated(const char* source_method) const;

  // Asserts the given source_id is registered, throwing an exception whose
  // message is the given message with the source_id appended if not.
  void ThrowUnlessRegistered(SourceId source_id, const char *message) const;

  // A struct that stores the port indices for a given source.
  // TODO(SeanCurtis-TRI): Consider making these TypeSafeIndex values.
  struct SourcePorts {
    int id_port{-1};
    int pose_port{-1};
  };

  // A mapping from added source identifier to the port indices associated with
  // that id.
  std::unordered_map<SourceId, SourcePorts> input_source_ids_;

  // The index of the output port with the PoseBundle abstract value.
  int bundle_port_index_{-1};

  // The index of the output port with the QueryHandle abstract value.
  int query_port_index_{-1};

  // A raw pointer to the default geometry state (which serves as the model for
  // allocating contexts for this system). The instance is owned by
  // model_abstract_states_. This pointer will only be non-null between
  // construction and context allocation. It serves a key role in enforcing the
  // property that source ids can only be added prior to context allocation.
  // This is mutable so that it can be cleared in the const method
  // AllocateContext().
  mutable GeometryState<T>* initial_state_;

  // The index of the geometry state in the context's abstract state.
  int geometry_state_index_{-1};
};

}  // namespace geometry

// Define the conversion trait to *only* allow double -> AutoDiffXd conversion.
// Symbolic is not supported yet, and AutoDiff -> double doesn't "make sense".
namespace systems {
namespace scalar_conversion {
template <>
struct Traits<geometry::GeometrySystem> {
  template <typename T, typename U>
  using supported = typename std::conditional<
      !std::is_same<T, symbolic::Expression>::value &&
          std::is_same<U, double>::value,
      std::true_type, std::false_type>::type;
};
}  // namespace scalar_conversion
}  // namespace systems

}  // namespace drake
