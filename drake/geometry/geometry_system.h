#pragma once

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "drake/geometry/geometry_ids.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace geometry {

template <typename T> class GeometryFrame;
template <typename T> class GeometryInstance;

// TODO(SeanCurtis-TRI): Introducing the API has been decomposed into two PRs.
// There is functionality alluded to in these comments (e.g., updating state
// and performing queries that will come in the follow-up PR.

/** GeometrySystem serves as a system-level wrapper for GeometryWorld. It serves
 as the nexus for all geometry (and geometry-based operations) in a Diagram.
 Through GeometrySystem, other systems that introduce geometry can _register_
 that geometry as part of a common global domain, including it in geometric
 queries (e.g., cars controlled by one LeafSystem can be observed by a different
 sensor system). GeometrySystem provides the interface for registering the
 geometry.

 Only registered "geometry sources" can introduce geometry into %GeometrySystem.
 Geometry sources will typically be other leaf systems, but, in the case of
 _anchored_ (i.e., stationary) geometry, it could also be some other block of
 code (e.g., adding a common ground plane with which all systems' geometries
 interact). For dynamic geometry (geometry whose pose depends on a Context), the
 geometry source must also provide pose values for all of the geometries the
 source owns, via a port connection on %GeometrySystem.

 The basic workflow for registering geometry with %GeometrySystem is:
   - Register as a geometry source, acquiring a unique SourceId.
   - Register geometry (anchored and dynamic) with the system.
   - @todo Document i/o connections when that API is introduced.

 @section geom_sys_workflow Working with GeometrySystem

 LeafSystem instances can relate to GeometrySystem in one of two ways: as a
 _consumer_ that performs queries, or as a _producer_ that introduces geometry
 into the shared world and defines its context-dependent kinematics values.
 It is reasonable for systems to perform either role singly, or both.

 __Consumer__

 @todo Document this when API is introduced.

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

 @todo Document this when the API has been introduced.

 @tparam T The underlying scalar type. Must be a valid Eigen scalar.
 */
template <typename T>
class GeometrySystem : public systems::LeafSystem<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(GeometrySystem)

  GeometrySystem() = default;
  ~GeometrySystem() override;

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
   @throws  std::logic_error if a context has already been allocated for this
                             %GeometrySystem.
   @see GeometryState::RegisterNewSource() */
  SourceId RegisterSource(const std::string &name = "");

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
  FrameId RegisterFrame(SourceId source_id, const GeometryFrame<T>& frame);

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
                        const GeometryFrame<T>& frame);

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
                              std::unique_ptr<GeometryInstance<T>> geometry);

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
                              std::unique_ptr<GeometryInstance<T>> geometry);

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
      std::unique_ptr<GeometryInstance<T>> geometry);

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
                            1. The `source_id` is not an active source,
                            2. the `frame_id` doesn't belong to the source, or
                            3. a context has been allocated. */
  void RemoveFrame(SourceId source_id, FrameId frame_id);

  /** Removes the given geometry G (indicated by `geometry_id`) from the the
   given source's registered geometries. All registered geometries hanging from
   this geometry will also be removed.
   @param source_id   The identifier for the owner geometry source.
   @param geometry_id The identifier of the geometry to remove.
   @throws std::logic_error If:
                            1. The `source_id` is not an active source,
                            2. the `geometry_id` doesn't belong to the source,
                               or
                            3. a context has been allocated. */
  void RemoveGeometry(SourceId source_id, GeometryId geometry_id);

  //@}

 private:
  // Friend class to facilitate testing.
  friend class GeometrySystemTester;

  // Override of construction to account for
  //    - instantiating a GeometryContext instance (as opposed to LeafContext),
  //    - modifying the state to prevent additional sources being added. */
  std::unique_ptr<systems::LeafContext<T>> DoMakeContext() const override;

  // Helper method for throwing an exception if a context has *ever* been
  // allocated by this system. The invoking method should pass it's name so
  // that the error message can include that detail.
  void ThrowIfContextAllocated(const char* source_method) const;

  mutable bool context_allocated_{false};
};

}  // namespace geometry
}  // namespace drake
