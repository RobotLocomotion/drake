#pragma once

#include <unordered_map>
#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/geometry/geometry_ids.h"

namespace drake {
namespace geometry {

// TODO(SeanCurtis-TRI): Extend documentation to give usage examples.
//  Usage notes:
//    0. *Set* semantics. It is represented by an ordered structure for
//       performance reasons. However, the frame ids in the set must be unique.
//      a. Debug build provides data validation and correctness (to the extent
//         that it can.)
//    1. Expectation of constructing once and updating as you go.

/** Represents an _ordered_ set of frame identifiers. Instances of this class
 work in conjunction with instances of FramePoseVector and FrameVelocityVector
 to communicate frame kinematics to GeometryWorld and GeometrySystem. Considered
 together, they represent a "struct-of-arrays" paradigm. The iᵗʰ value in the
 %FrameIdVector is the frame identifier whose position is specified by
 the iᵗʰ value in the corresponding FramePoseVector (and analogously for the
 FrameVelocityVector).

 The geometry source can use arbitrary logic to define the _order_ of the ids.
 However, all frames registered by the source must be included. Omitting a
 registered frame id is considered an error (and will cause an exception
 when GeometryWorld/GeometrySystem evaluates the data.

 @internal In future iterations, this will be relaxed to allow only
 communicating kinematics for frames that have _changed_. But in the initial
 version, this requirement is in place for pedagogical purposes to help users
 use GeometryWorld/GeometrySystem correctly. */
class FrameIdVector {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(FrameIdVector)

  typedef std::vector<FrameId>::const_iterator ConstIterator;

  /** Constructor for an _empty_ id vector.
   @param source_id   The id for the geometry source reporting frame kinematics.
   */
  explicit FrameIdVector(SourceId source_id);

  /** Constructor which initializes the frame ids by _copying_ the given `ids`.
   In Debug builds, the input `ids` will be tested for duplicates; an exception
   is thrown if duplicates are found.
   @param source_id   The id for the geometry source reporting frame kinematics.
   @param ids         The vector of ids which are copied into this vector.
   @throws std::logic_error (in Debug) if any of the ids are duplicated. */
  FrameIdVector(SourceId source_id, const std::vector<FrameId>& ids);

  /** Reports the source id for this data. */
  SourceId get_source_id() const { return source_id_; }

  /** Reports the number of ids stored in the vector. */
  int size() const { return static_cast<int>(id_index_map_.size()); }

  /** Returns the iᵗʰ frame id. */
  FrameId get_frame_id(int i) const { return index_id_map_[i]; }

  /** Returns the index of the given frame id.
   @throws std::logic_error if the frame id is not in the set. */
  int GetIndex(FrameId frame_id) const;

  /** Appends the given frame identifier to the set.
   @param frame_id    The frame identifier to add.
   @throws std::logic_error if the frame_id already exists in the vector.  */
  void AddFrameId(FrameId frame_id);

  /** Appends the given set of frame `ids` to the set. In Debug build, the
   resultant set is tested for duplicate frame ids.
   @param ids  The ordered set of frame ids to append to the current set.
   @throws std::logic_error if there are duplicate ids in the resultant set. */
  void AddFrameIds(const std::vector<FrameId>& ids);

  /** @name  Support for range-based loop iteration */
  //@{

  ConstIterator begin() const { return index_id_map_.cbegin(); }
  ConstIterator end() const { return index_id_map_.cend(); }

  //@}

 private:
  // Throws an exception if the given vector contains duplicates values. This
  // does not consider the contents of index_id_map_.
  static void ThrowIfContainsDuplicates(const std::vector<FrameId>& frame_ids);

  // Throws an exception if the given frame_id is already in id_index_map_.
  void ThrowIfContains(FrameId frame_id);

  // The id of the reporting geometry source.
  SourceId source_id_;

  // A mapping from frame_id to its index value. For N frames, the map should
  // span the range [0, N-1]. For a given frame id (f_id), the following should
  // hold:
  //   f_id == index_id_map_[id_index_map_[f_id]];
  std::unordered_map<FrameId, int> id_index_map_;

  // A mapping from index to frame id.
  std::vector<FrameId> index_id_map_;
};
}  // namespace geometry
}  // namespace drake
