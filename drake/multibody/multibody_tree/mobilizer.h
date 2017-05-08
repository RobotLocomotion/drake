#pragma once

#include <memory>
#include <vector>

#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/multibody/multibody_tree/frame.h"
#include "drake/multibody/multibody_tree/multibody_tree_element.h"
#include "drake/multibody/multibody_tree/multibody_tree_indexes.h"
#include "drake/multibody/multibody_tree/multibody_tree_topology.h"

namespace drake {
namespace multibody {

// Forward declarations.
template<typename T> class Body;
template<typename T> class BodyNode;

template <typename T>
class Mobilizer : public MultibodyTreeElement<Mobilizer<T>, MobilizerIndex> {
 public:
  /// Mobilizer constructor.
  Mobilizer(const Frame<T>& inboard_frame,
            const Frame<T>& outboard_frame) :
      inboard_frame_(inboard_frame), outboard_frame_(outboard_frame) {}

  // make pure virtual!!!
  virtual int get_num_positions() const {return 0;};

  virtual int get_num_velocities() const {return 0;};

  const Frame<T>& get_inboard_frame() const {
    return inboard_frame_;
  }

  const Frame<T>& get_outboard_frame() const {
    return outboard_frame_;
  }

  const Body<T>& get_inboard_body() const {
    return get_inboard_frame().get_body();
  }

  const Body<T>& get_outboard_body() const {
    return get_outboard_frame().get_body();
  }

  const MobilizerTopology& get_topology() const { return topology_;}

  //virtual std::unique_ptr<BodyNode<T>> CreateBodyNode(
  //    const BodyNodeTopology& topology,
  //    const Body<T>* body, const Mobilizer<T>* mobilizer) const = 0;

 private:
  // Implementation for MultibodyTreeElement::DoFinalize().
  // At MultibodyTree::Finalize() time, each mobilizer retrieves its topology
  // from the parent MultibodyTree.
  //void DoFinalize(const MultibodyTree<T>& tree) final {
  //  topology_ = tree.get_topology().mobilizers[this->get_index()];
  //}

  void DoSetTopology(const MultibodyTreeTopology& tree_topology) final {
    topology_ = tree_topology.mobilizers[this->get_index()];
  }

  const Frame<T>& inboard_frame_;
  const Frame<T>& outboard_frame_;
  MobilizerTopology topology_;
};

}  // namespace multibody
}  // namespace drake
