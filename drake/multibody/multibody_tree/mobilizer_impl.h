#pragma once

#include <memory>
#include <vector>

#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/multibody/multibody_tree/frame.h"
#include "drake/multibody/multibody_tree/mobilizer.h"
#include "drake/multibody/multibody_tree/multibody_tree_element.h"
#include "drake/multibody/multibody_tree/multibody_tree_indexes.h"
#include "drake/multibody/multibody_tree/multibody_tree_topology.h"

namespace drake {
namespace multibody {

template <typename T, int  num_positions, int num_velocities>
class MobilizerImpl : public Mobilizer<T> {
 public:
  // static constexpr int i = 42; discouraged.
  // See answer in: http://stackoverflow.com/questions/37259807/static-constexpr-int-vs-old-fashioned-enum-when-and-why
  enum : int {nq = num_positions, nv = num_velocities};

  MobilizerImpl(const Frame<T>& inboard_frame,
                const Frame<T>& outboard_frame) :
      Mobilizer<T>(inboard_frame, outboard_frame) {}

  int get_num_positions() const final { return nq;}
  int get_num_velocities() const final { return nv;}

};

}  // namespace multibody
}  // namespace drake
