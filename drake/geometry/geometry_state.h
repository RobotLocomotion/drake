#pragma once

#include <memory>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/common/drake_optional.h"
#include "drake/geometry/geometry_ids.h"
#include "drake/geometry/internal_frame.h"

namespace drake {
namespace geometry {

// forward declarations
template <typename T> class GeometryFrame;
template <typename T> class GeometrySystem;

/** @name Structures for maintaining the entity relationships
 @{ */

/** Collection of unique frame ids. */
using FrameIdSet = std::unordered_set<FrameId>;

//@}

/**
 The context-dependent state of GeometryWorld. This serves as an AbstractValue
 in the context. GeometryWorld's time-dependent state includes more than just
 values; objects can be added to or removed from the world over time. Therefore,
 GeometryWorld's context-dependent state includes values and structure -- the
 topology of the world.

 @tparam T The underlying scalar type. Must be a valid Eigen scalar. */
template <typename T>
class GeometryState {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(GeometryState)

  /** Default constructor. */
  GeometryState();

  /** @name        State introspection.

   Various methods that allow reading the state's properties and values.
  @{ */

  /** Reports the number of registered sources -- whether they have frames or
   not. */
  int get_num_sources() const {
    return static_cast<int>(source_frame_id_map_.size());
  }

  /** Reports the total number of frames -- across all sources. */
  int get_num_frames() const { return static_cast<int>(frames_.size()); }

  /** Reports true if the given `source_id` references a registered source. */
  bool source_is_registered(SourceId source_id) const;

  /** Reports the source name for the given source id.
   @param id  The identifier of the source.
   @return The name of the source.
   @throws std::logic_error if the id does _not_ map to a registered source. */
  const std::string& get_source_name(SourceId id) const;
  //@}

  /** @name       Relationship queries

   Various methods that map identifiers for one type of entity to its related
   entities.
   @{ */

  /** Reports if the given frame id was registered to the given source id.
   @param frame_id      The query frame id.
   @param source_id     The query source id.
   @returns True if `frame_id` was registered on `source_id`.
   @throws std::logic_error  If the `frame_id` does _not_ map to a frame or the
                             identified source is not registered. */
  bool BelongsToSource(FrameId frame_id, SourceId source_id) const;

  /** Returns the set of frames registered to the given source.
   @param source_id     The identifier of the source to query.
   @return  The set of frames associated with the id.
   @throws std::logic_error If the `source_id` does _not_ map to a registered
                            source. */
  const FrameIdSet& GetFramesForSource(SourceId source_id) const;

  //@}

  /** @name        State management

   The methods that modify the state including: adding/removing entities from
   the state, modifying values in the state, etc.
   @{ */

  /** Registers a new, named source into the state.
    @param name          The optional name of the source. If none or the empty
                        string is provided it will be named "Source_##" where
                        the number is the value of the returned SourceId.
   @trhows std::logic_error is thrown if the name is _not_ unique. */
  SourceId RegisterNewSource(const std::string& name = "");

  /** Removes  all frames and geometry registered from the identified source.
   The source remains registered and further frames and geometry can be
   registered on it.
   @param source_id     The identifier for the source to clear.
   @throws std::logic_error  If the `source_id` does _not_ map to a registered
                             source. */
  void ClearSource(SourceId source_id);

  /** Removes the given frame from the the indicated source's frames. All
   registered geometries connected to this frame will also be removed from the
   world.
   @param source_id     The identifier for the owner geometry source.
   @param frame_id      The identifier of the frame to remove.
   @throws std::logic_error  1. If the `source_id` does _not_ map to a
                             registered source, or
                             2. the `frame_id` does not map to a valid frame, or
                             3. the `frame_id` maps to a frame that does not
                             belong to the indicated source. */
  void RemoveFrame(SourceId source_id, FrameId frame_id);

  /** Registers a new frame for the given source, the id of the new frame is
   returned.
   @param source_id    The id of the source for which this frame is allocated.
   @param frame        The frame to register.
   @returns  A newly allocated frame id.
   @throws std::logic_error  If the `source_id` does _not_ map to a registered
                             source. */
  FrameId RegisterFrame(SourceId source_id, const GeometryFrame<T>& frame);

  /** Registers a new frame for the given source as a child of a previously
   registered frame. The id of the new frame is returned.
   @param source_id    The id of the source for which this frame is allocated.
   @param parent_id    The id of the parent frame.
   @param frame        The frame to register.
   @returns  A newly allocated frame id.
   @throws std::logic_error  1. If the `source_id` does _not_ map to a
                             registered source, or
                             2. If the `parent_id` does _not_ map to a known
                             frame or does not belong to the source. */
  FrameId RegisterFrame(SourceId source_id, FrameId parent_id,
                        const GeometryFrame<T>& frame);

  //@}

 private:
  // Allow GeometrySystem unique access to the state members to perform queries.
  friend class GeometrySystem<T>;

  // Friend declaration so that the internals of the state can be confirmed in
  // unit tests.
  template <class U> friend class GeometryStateTester;

  // Gets the source id for the given frame id. Throws std::logic_error if the
  // frame belongs to no registered source.
  SourceId get_source_id(FrameId frame_id) const;

  // The origin from where an invocation of RemoveFrameUnchecked was called.
  // The origin changes the work that is required.
  enum class RemoveFrameOrigin {
    SOURCE,     // Invoked by ClearSource().
    FRAME,      // Invoked by RemoveFrame().
    RECURSE     // Invoked by recursive call in RemoveGeometryUnchecked.
  };

  // Performs the work necessary to remove the identified frame from
  // GeometryWorld. The amount of work depends on the context from which this
  // method is invoked:
  //
  //  - ClearSource(): ClearSource() is deleting *all* frames and geometries.
  //    It explicitly iterates through the frames (regardless of hierarchy).
  //    Thus, recursion is unnecessary, removal from parent references is
  //    likewise unnecessary (and actually wrong).
  //  - RemoveFrame(): The full removal is necessary; recursively remove child
  //    frames (and child geometries), removing references to this id from
  //    the source and its parent frame (if not the world).
  //   - RemoveFrameUnchecked(): This is the recursive call; it's parent
  //    is already slated for removal, so parent references can be left alone,
  //    but recursion is necessary.
  void RemoveFrameUnchecked(FrameId frame_id, RemoveFrameOrigin caller);

  // TODO(SeanCurtis-TRI): Several design issues on this:
  //  1. It should *ideally* be const.
  //  2. Can I guarantee that it's always 0?
  // The frame identifier for the world frame.
  FrameId kWorldFrame;

  // ---------------------------------------------------------------------
  // Maps representing the registered state of sources, frames and geometries,
  // and their relationships. This data should only change at major discrete
  // events where frames/geometries are introduced and removed. They do *not*
  // depend on time-dependent input values (e.g., System::InputPort).

  // The registered geometry sources and the frame ids that have been registered
  // on them.
  std::unordered_map<SourceId, FrameIdSet> source_frame_id_map_;

  // The registered geometry sources and the frame ids that have the world frame
  // as the parent frame. For a completely flat hierarchy, this contains the
  // same values as the corresponding entry in source_frame_id_map_.
  std::unordered_map<SourceId, FrameIdSet> source_root_frame_map_;

  // The registered geometry source names. Each name is unique and the keys in
  // this map should be identical to those in source_frame_id_map_ and
  // source_root_frame_map_.
  std::unordered_map<SourceId, std::string> source_names_;

  // The frame data, keyed on unique frame identifier.
  std::unordered_map<FrameId, internal::InternalFrame> frames_;
};
}  // namespace geometry
}  // namespace drake
