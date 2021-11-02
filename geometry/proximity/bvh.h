#pragma once

#include <array>
#include <memory>
#include <stack>
#include <utility>
#include <variant>
#include <vector>

#include "drake/common/copyable_unique_ptr.h"
#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/geometry/proximity/aabb.h"
#include "drake/geometry/proximity/obb.h"
#include "drake/geometry/proximity/triangle_surface_mesh.h"
#include "drake/geometry/proximity/volume_mesh.h"
#include "drake/geometry/utilities.h"
#include "drake/math/rigid_transform.h"

namespace drake {
namespace geometry {
namespace internal {

/* Forward declaration to enable Bvh for deformable meshes. */
template <typename> class BvhUpdater;

template <class MeshType>
struct MeshTraits;

template <typename T>
struct MeshTraits<TriangleSurfaceMesh<T>> {
  static constexpr int kMaxElementPerBvhLeaf = 3;
};

template <typename T>
struct MeshTraits<VolumeMesh<T>> {
  static constexpr int kMaxElementPerBvhLeaf = 1;
};

/* Node of the tree structure representing the Bvh.  */
template <class BvType, class MeshType>
class BvNode {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(BvNode)

  static constexpr int kMaxElementPerLeaf =
      MeshTraits<MeshType>::kMaxElementPerBvhLeaf;

  /* A leaf node can store as many as kMaxElementPerLeaf elements.
   The actual number of stored element indices is `num_index`. */
  struct LeafData {
    int num_index;
    std::array<int, kMaxElementPerLeaf> indices;
  };

  /* Constructor for leaf nodes consisting of multiple elements.
   @param bv    The bounding volume encompassing the elements.
   @param data  The indices of the mesh elements contained in the leaf. */
  BvNode(BvType bv, LeafData data)
      : bv_(std::move(bv)), child_(std::move(data)) {}

  /* Constructor for branch/internal nodes.
   @param bv The bounding volume encompassing the elements in child branches.
   @param left Unique pointer to the left child branch.
   @param right Unique pointer to the right child branch.
   @pre Both children must be distinct and not null.   */
  BvNode(BvType bv, std::unique_ptr<BvNode<BvType, MeshType>> left,
         std::unique_ptr<BvNode<BvType, MeshType>> right)
      : bv_(std::move(bv)),
        child_(NodeChildren(std::move(left), std::move(right))) {}

  /* Returns the bounding volume.  */
  const BvType& bv() const { return bv_; }

  /* Returns the number of element indices.
   @pre is_leaf() returns true. */
  int num_element_indices() const {
    return std::get<LeafData>(child_).num_index;
  }

  /* Returns the i-th element index in the leaf data.
   @pre is_leaf() returns true.
   @pre `i` is less than LeafData::num_index, and i >= 0. */
  int element_index(int i) const {
    DRAKE_ASSERT(0 <= i && i < std::get<LeafData>(child_).num_index);
    return std::get<LeafData>(child_).indices[i];
  }

  /* Returns the left child branch.
   @pre is_leaf() returns false.  */
  const BvNode<BvType, MeshType>& left() const {
    return *(std::get<NodeChildren>(child_).left);
  }

  /* Returns the right child branch.
   @pre is_leaf() returns false.  */
  const BvNode<BvType, MeshType>& right() const {
    return *(std::get<NodeChildren>(child_).right);
  }

  /* Returns whether this is a leaf node as opposed to a branch node.  */
  bool is_leaf() const {
    return std::holds_alternative<LeafData>(child_);
  }

  /* Compares this node with the given node in a strictly *topological* manner.
   For them to be considered "equal leaves", both nodes must be leaves and must
   contain the same indices.

   Because this test considers only the Bvh tree *topology* it can be used to
   compare nodes that have been constructed from meshes with different scalar
   types (e.g., TriangleSurfaceMesh<double> and TriangleSurfaceMesh<AutoDiffXd>)
   or meshes with different bounding volume types (e.g., Aabb vs Obb).

   @pre both nodes are leaves.  */
  template <typename OtherBvNode>
  bool EqualLeaf(const OtherBvNode& other_leaf) const {
    if constexpr (std::is_same_v<OtherBvNode, BvNode<BvType, MeshType>>) {
      if (this == &other_leaf) return true;
    }
    if (this->num_element_indices() != other_leaf.num_element_indices()) {
      return false;
    }
    for (int i = 0; i < this->num_element_indices(); ++i) {
      if (this->element_index(i) != other_leaf.element_index(i)) {
        return false;
      }
    }
    return true;
  }

 private:
  template <typename> friend class BvhUpdater;

  /* Provide disciplined access to BvhUpdater to a mutable child node. */
  BvNode<BvType, MeshType>& left() {
    return *(std::get<NodeChildren>(child_).left);
  }

  /* Provide disciplined access to BvhUpdater to a mutable child node. */
  BvNode<BvType, MeshType>& right() {
    return *(std::get<NodeChildren>(child_).right);
  }

  /* Provide disciplined access to BvhUpdater to a mutable bounding volume. */
  BvType& bv() { return bv_; }

  struct NodeChildren {
    DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(NodeChildren)

    NodeChildren(std::unique_ptr<BvNode> left_in,
                 std::unique_ptr<BvNode> right_in)
        : left(std::move(left_in)), right(std::move(right_in)) {
      DRAKE_DEMAND(left != nullptr);
      DRAKE_DEMAND(right != nullptr);
      DRAKE_DEMAND(left != right);
    }

    copyable_unique_ptr<BvNode> left;
    copyable_unique_ptr<BvNode> right;
  };

  BvType bv_;

  // If this is a leaf node then the child refers to indices into the mesh's
  // elements (i.e., triangles or tetrahedra) bounded by the node's bounding
  // volume. Otherwise, it refers to child nodes further down the tree.
  std::variant<LeafData, NodeChildren> child_;
};

/* Resulting instruction from performing the bounding volume tree traversal
 (BVTT) callback on two potentially colliding pairs. Note that this is not the
 mathematical result but information on how the traversal should proceed.  */
enum BvttCallbackResult {
  Continue,
  Terminate
};

/* Bounding volume tree traversal (BVTT) callback. Returns a BvttCallbackResult
 for further action, e.g. deciding whether to exit early. The parameters are
 an index into the elements of the *first* mesh followed by the index into
 the elements of the *second* mesh. */
using BvttCallback = std::function<BvttCallbackResult(int, int)>;

/* %Bvh is an acceleration structure for performing spatial queries against a
 collection of objects (in this case, triangles or tetrahedra). Specifically,
 for identifying those objects in or near a particular region of interest. It
 serves as a basis for culling objects that are trivially far from the region,
 reducing the number of "narrow-phase" calculations. The underlying structure
 is a binary tree of bounding volumes (bv) that encompass one or more
 mesh elements. The bounding volumes are all measured and expressed in this
 hierarchy's frame H. Leaf nodes contain element indices into elements of the
 mesh. The BVH needs a reference to the mesh in order to build the tree, but
 does not own the mesh.
 @pre    The mesh is not mutable. Modifications to the mesh after
         constructing the BVH will make the BVH invalid.
 @tparam BvType           The bounding volume type (e.g., Aabb, Obb).
 @tparam SourceMeshType   TriangleSurfaceMesh<T> or VolumeMesh<T>, T = double or
                          AutoDiffXd.  */
template <class BvType, class SourceMeshType>
class Bvh {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(Bvh)

  using MeshType = SourceMeshType;
  using NodeType = BvNode<BvType, MeshType>;

  explicit Bvh(const MeshType& mesh);

  const NodeType& root_node() const { return *root_node_; }

  /* Perform a query of this %Bvh's mesh elements (measured and expressed in
   Frame A) against the given %Bvh's mesh elements (measured and expressed in
   Frame B). The callback is invoked on every pair of elements that cannot
   conclusively be shown to be separated via bounding-volume comparisons (the
   unculled pairs).

   @param bvh_B           The bounding volume hierarchy to collide with.
   @param X_AB            The relative pose of the two hierarchies.
   @param callback        The callback to invoke on each unculled pair.
   @tparam OtherBvhType   The type of Bvh to collide against this.  */
  template <class OtherBvhType>
  void Collide(
      const OtherBvhType& bvh_B, const math::RigidTransformd& X_AB,
      BvttCallback callback) const {
    using NodePair =
        std::pair<const NodeType&, const typename OtherBvhType::NodeType&>;
    std::stack<NodePair, std::vector<NodePair>> node_pairs;
    node_pairs.emplace(root_node(), bvh_B.root_node());

    while (!node_pairs.empty()) {
      const auto& [node_a, node_b] = node_pairs.top();
      node_pairs.pop();

      // Check if the bounding volumes overlap.
      if (!BvType::HasOverlap(node_a.bv(), node_b.bv(), X_AB)) {
        continue;
      }

      // Run the callback on the pair if they are both leaf nodes, otherwise
      // check each branch.
      if (node_a.is_leaf() && node_b.is_leaf()) {
        const int num_a_elements = node_a.num_element_indices();
        const int num_b_elements = node_b.num_element_indices();
        for (int a = 0; a < num_a_elements; ++a) {
          for (int b = 0; b < num_b_elements; ++b) {
            const BvttCallbackResult result =
                callback(node_a.element_index(a), node_b.element_index(b));
            if (result == BvttCallbackResult::Terminate) return;
          }
        }
      } else if (node_b.is_leaf()) {
        node_pairs.emplace(node_a.left(), node_b);
        node_pairs.emplace(node_a.right(), node_b);
      } else if (node_a.is_leaf()) {
        node_pairs.emplace(node_a, node_b.left());
        node_pairs.emplace(node_a, node_b.right());
      } else {
        node_pairs.emplace(node_a.left(), node_b.left());
        node_pairs.emplace(node_a.right(), node_b.left());
        node_pairs.emplace(node_a.left(), node_b.right());
        node_pairs.emplace(node_a.right(), node_b.right());
      }
    }
  }

  /* Culls the nodes of the BVH based on the nodes' bounding volumes'
   relationships with a primitive object. This is different from the BVH-BVH
   Collide() method in that when a node is found to be overlapping the
   primitive, it triggers two new tests, one for each of the node's children;
   the BVH-BVH collision spawns four new tests.

   The callback function consumes only a single index -- the index of the leaf
   element whose box overlaps with the primitive; it is assumed that the
   provider for the primitive has perfect knowledge of that primitive.

   This BVH is defined in Frame H and the primitive is defined in frame P.

   @param primitive_P   The primitive defined in Frame P.
   @param X_PH          The relative pose between Frames P and H.
   @param callback      Function to process the elements in `this` BVH's
                        corresponding mesh whose bounding volumes intersect
                        the primitive.

   @note This method can be only used for a primitive type that has an overload
   for BvType::HasOverlap() defined.  */
  template <typename PrimitiveType>
  void Collide(const PrimitiveType& primitive_P,
               const math::RigidTransformd& X_PH,
               std::function<BvttCallbackResult(int)> callback) const {
    std::stack<const NodeType*> nodes;
    nodes.emplace(&root_node());
    while (!nodes.empty()) {
      const auto node = nodes.top();
      nodes.pop();

      if (!BvType::HasOverlap(node->bv(), primitive_P, X_PH)) {
        continue;
      }
      // Run the call back if `node` is a leaf.
      if (node->is_leaf()) {
        const int num_elements = node->num_element_indices();
        for (int i = 0; i < num_elements; ++i) {
          BvttCallbackResult result = callback(node->element_index(i));
          if (result == BvttCallbackResult::Terminate) return;  // Exit early.
        }
      } else {
        nodes.emplace(&node->left());
        nodes.emplace(&node->right());
      }
    }
  }

  /* Wrapper around `Collide` with a callback that accumulates each pair of
   collision candidates and returns them all.
   @return Vector of element index pairs whose elements are candidates for
   collision (index into *this* mesh's elements, index into bvh_B's mesh's
   elements).  */
  template <class OtherBvhType>
  std::vector<std::pair<int, int>> GetCollisionCandidates(
      const OtherBvhType& bvh_B, const math::RigidTransformd& X_AB) const {
    std::vector<std::pair<int, int>> result;
    BvttCallback callback = [&result](int a, int b) -> BvttCallbackResult {
      result.emplace_back(a, b);
      return BvttCallbackResult::Continue;
    };
    Collide(bvh_B, X_AB, callback);
    return result;
  }

  /* Compares the two Bvh instances for exact equality down to the last bit.
   Assumes that the quantities are measured and expressed in the same frame. */
  template <typename OtherBvhType>
  bool Equal(const OtherBvhType& other) const {
    if constexpr (std::is_same_v<OtherBvhType, Bvh<BvType, SourceMeshType>>) {
      if (this == &other) return true;
    }
    return EqualTrees(this->root_node(), other.root_node());
  }

 private:
  // Convenience class for testing.
  friend class BvhTester;

  template <typename> friend class BvhUpdater;

  NodeType& mutable_root_node() { return *root_node_; }

  using CentroidPair = std::pair<int, Vector3<double>>;

  static std::unique_ptr<NodeType> BuildBvTree(
      const MeshType& mesh,
      const typename std::vector<CentroidPair>::iterator& start,
      const typename std::vector<CentroidPair>::iterator& end);

  static BvType ComputeBoundingVolume(
      const MeshType& mesh,
      const typename std::vector<CentroidPair>::iterator& start,
      const typename std::vector<CentroidPair>::iterator& end);

  // TODO(tehbelinda): Move this function into TriangleSurfaceMesh/VolumeMesh
  // directly and rename to CalcElementCentroid(int element_index).
  // Computes the centroid of the ith element of the given mesh.
  static Vector3<double> ComputeCentroid(const MeshType& mesh, int i);

  // Tests that two trees, rooted at nodes a and b, respectively, are equal
  // in the sense that they have identical node structure and equal bounding
  // volumes (see BvType::Equal()). The two hierarchies must be built from the
  // same bounding volume type, and the same mesh type, but the mesh scalar can
  // differ.
  template <typename OtherNodeType>
  static bool EqualTrees(const NodeType& a, const OtherNodeType& b) {
    if constexpr (std::is_same_v<NodeType, OtherNodeType>) {
      if (&a == &b) return true;
    }

    if (!a.bv().Equal(b.bv())) return false;

    if (a.is_leaf()) {
      if (!b.is_leaf()) {
        return false;
      }
      return a.EqualLeaf(b);
    } else {
      if (b.is_leaf()) {
        return false;
      }
      return EqualTrees(a.left(), b.left()) && EqualTrees(a.right(), b.right());
    }
  }

  static constexpr int kElementVertexCount = MeshType::kVertexPerElement;

  copyable_unique_ptr<NodeType> root_node_;
};

}  // namespace internal
}  // namespace geometry
}  // namespace drake
