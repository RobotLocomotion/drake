#pragma once

#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <utility>
#include <vector>

#include <common_robotics_utilities/simple_knearest_neighbors.hpp>

#include "drake/common/parallelism.h"

namespace drake {
namespace planning {
namespace internal {
// Planner tree state for parallel RRT planner. This matches the necessary
// subset of the API presented by CRU's SimpleRRTPlannerState<StateType> with
// slight modifications to achieve the necessary thread safety - namely that
// GetValueImmutable must return by value, not by reference, as the underlying
// tree storage may be reallocated during the propagation step. Note that other
// non-const methods are *not* thread safe.
template <typename StateType>
class ParallelRRTPlannerState {
 public:
  ParallelRRTPlannerState(const StateType& value, int64_t parent_index)
      : value_(value), parent_index_(parent_index) {}

  bool IsInitialized() const { return true; }

  StateType GetValueImmutable() const { return value_; }

  // This may *only* be used in thread safe contexts where the parent tree is
  // guaranteed not to change.
  const StateType& GetValueImmutableUnsafe() const { return value_; }

  int64_t GetParentIndex() const { return parent_index_; }

  void AddChildIndex(const int64_t child_index) {
    for (size_t idx = 0; idx < child_indices_.size(); idx++) {
      if (child_indices_.at(idx) == child_index) {
        return;
      }
    }
    child_indices_.push_back(child_index);
  }

 private:
  StateType value_;
  std::vector<int64_t> child_indices_;
  int64_t parent_index_ = -1;
};

// Planner tree for parallel RRT planner. This matches the necessary subset
// of the API presented by CRU's SimpleRRTPlannerTree<StateType> with slight
// modifications to achieve the necessary thread safety - namely that
// GetNodeImmutable must return by value, not by reference, as the underlying
// tree storage may be reallocated during the propagation step. Note that other
// non-const methods are *not* thread safe.
template <typename StateType>
class ParallelRRTPlannerTree {
 public:
  explicit ParallelRRTPlannerTree(const int64_t anticipated_size) {
    nodes_ =
        std::make_shared<std::vector<ParallelRRTPlannerState<StateType>>>();
    nodes_->reserve(static_cast<size_t>(anticipated_size));
  }

  int64_t Size() const {
    std::lock_guard<std::mutex> nodes_lock(nodes_mutex_);
    return static_cast<int64_t>(nodes_->size());
  }

  bool Empty() const {
    std::lock_guard<std::mutex> nodes_lock(nodes_mutex_);
    return nodes_->empty();
  }

  ParallelRRTPlannerState<StateType> GetNodeImmutable(
      const int64_t index) const {
    std::lock_guard<std::mutex> nodes_lock(nodes_mutex_);
    return nodes_->at(static_cast<size_t>(index));
  }

  int64_t AddNode(const StateType& value) {
    return AddNodeAndConnect(value, -1);
  }

  int64_t AddNodeAndConnect(const StateType& value,
                            const int64_t parent_index) {
    // All node addition operations must be serialized. Within addition, only
    // modifications to nodes_ must be serialized with read operations.
    std::lock_guard<std::mutex> addition_lock(addition_mutex_);

    if (nodes_->size() < nodes_->capacity()) {
      // If adding the node will not reallocate the storage in nodes_, just add
      // the new node directly.
      std::lock_guard<std::mutex> nodes_lock(nodes_mutex_);

      nodes_->emplace_back(value, parent_index);
      const int64_t new_index = static_cast<int64_t>(nodes_->size() - 1);
      if (parent_index >= 0) {
        nodes_->at(static_cast<size_t>(parent_index)).AddChildIndex(new_index);
      }

      return new_index;
    } else {
      // If adding the node will cause a reallocation of the storage in nodes_,
      // make a copy with the larger size, add to the copy, then switch nodes_.
      auto new_nodes =
          std::make_shared<std::vector<ParallelRRTPlannerState<StateType>>>();

      // Reserve the same amount of space the reallocated vector would have.
      // Growing by a factor of 1.5 is better than growing by 2 as it allows
      // for more memory reuse.
      new_nodes->reserve(nodes_->size() + nodes_->size() / 2);

      // Copy the nodes over.
      new_nodes->insert(new_nodes->end(), nodes_->begin(), nodes_->end());

      // Add the new node.
      new_nodes->emplace_back(value, parent_index);
      const int64_t new_index = static_cast<int64_t>(new_nodes->size() - 1);
      if (parent_index >= 0) {
        new_nodes->at(static_cast<size_t>(parent_index))
            .AddChildIndex(new_index);
      }

      // Switch to the new nodes.
      std::lock_guard<std::mutex> nodes_lock(nodes_mutex_);

      nodes_ = new_nodes;

      return new_index;
    }
  }

  std::pair<std::shared_ptr<std::vector<ParallelRRTPlannerState<StateType>>>,
            size_t>
  GetViewOfNodes() const {
    std::lock_guard<std::mutex> nodes_lock(nodes_mutex_);
    return std::make_pair(nodes_, nodes_->size());
  }

 private:
  mutable std::mutex nodes_mutex_;
  mutable std::mutex addition_mutex_;
  std::shared_ptr<std::vector<ParallelRRTPlannerState<StateType>>> nodes_;
};

// Wrapper for ParallelRRTPlannerTree that presents a view of the tree with
// fixed size for nearest neighbors (even if the tree is growing
// simultaneously in another thread).
template <typename StateType>
class ViewOfParallelRRTPlannerTree {
 public:
  explicit ViewOfParallelRRTPlannerTree(
      const ParallelRRTPlannerTree<StateType>& tree) {
    std::tie(nodes_, nodes_size_) = tree.GetViewOfNodes();
  }

  size_t size() const { return nodes_size_; }

  const StateType& operator[](size_t index) const {
    // During the lifetime of this object, the nodes vector will never
    // reallocate, so we can use the unsafe accessor to avoid copying.
    return nodes_->operator[](index).GetValueImmutableUnsafe();
  }

 private:
  std::shared_ptr<std::vector<ParallelRRTPlannerState<StateType>>> nodes_;
  size_t nodes_size_ = 0;
};

/// Function type for a distance function between states.
template <typename StateType>
using StateDistanceFunction =
    std::function<double(const StateType&, const StateType&)>;

// Get nearest neighbor from the provided `tree` to the provided sampled state
// `sampled` using distance function `distance_fn`.
template <typename StateType>
int64_t GetParallelRRTNearestNeighbor(
    const ParallelRRTPlannerTree<StateType>& tree, const StateType& sampled,
    const StateDistanceFunction<StateType>& distance_fn,
    Parallelism parallelism) {
  const ViewOfParallelRRTPlannerTree<StateType> view_of_tree(tree);

  const auto neighbors = common_robotics_utilities::simple_knearest_neighbors::
      GetKNearestNeighbors(
          view_of_tree, sampled, distance_fn, 1,
          common_robotics_utilities::parallelism::DegreeOfParallelism(
              parallelism.num_threads()));

  return neighbors.at(0).Index();
}

}  // namespace internal
}  // namespace planning
}  // namespace drake
