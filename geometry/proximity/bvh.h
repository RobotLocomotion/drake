#pragma once

#include <array>
#include <memory>
#include <stack>
#include <utility>
#include <variant>
#include <vector>

#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/geometry/proximity/obb.h"
#include "drake/geometry/proximity/surface_mesh.h"
#include "drake/geometry/proximity/volume_mesh.h"
#include "drake/geometry/utilities.h"
#include "drake/math/rigid_transform.h"

namespace drake {
namespace geometry {
namespace internal {

template <class MeshType>
struct MeshTraits;

template <typename T>
struct MeshTraits<SurfaceMesh<T>> {
  static constexpr int kMaxElementPerBvhLeaf = 3;
};

template <typename T>
struct MeshTraits<VolumeMesh<T>> {
  static constexpr int kMaxElementPerBvhLeaf = 1;
};

/* Node of the tree structure representing the Bvh.  */
template <class MeshType>
class BvNode {
 public:
  static constexpr int kMaxElementPerLeaf =
      MeshTraits<MeshType>::kMaxElementPerBvhLeaf;

  /* A leaf node can store as many as kMaxElementPerLeaf elements.
   The actual number of stored element indices is `num_index`. */
  struct LeafData {
    int num_index;
    std::array<typename MeshType::ElementIndex, kMaxElementPerLeaf> indices;
  };

  /* Constructor for leaf nodes consisting of multiple elements.
   @param obb The bounding volume encompassing the elements.
   @param data contains indices into the mesh for retrieving the elements. */
  BvNode(Obb bv, LeafData data)
      : bv_(std::move(bv)), child_(std::move(data)) {}

  /* Constructor for branch/internal nodes.
   @param bv The bounding volume encompassing the elements in child branches.
   @param left Unique pointer to the left child branch.
   @param right Unique pointer to the right child branch.
   @pre Both children must be distinct and not null.   */
  BvNode(Obb bv, std::unique_ptr<BvNode<MeshType>> left,
         std::unique_ptr<BvNode<MeshType>> right)
      : bv_(std::move(bv)),
        child_(NodeChildren(std::move(left), std::move(right))) {}

  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(BvNode)

  /* Returns the bounding volume.  */
  const Obb& bv() const { return bv_; }

  /* Returns the number of element indices.
   @pre is_leaf() returns true. */
  int num_element_indices() const {
    return std::get<LeafData>(child_).num_index;
  }

  /* Returns the i-th element index in the leaf data.
   @pre is_leaf() returns true.
   @pre `i` is less than LeafData::num_index, and i >= 0. */
  typename MeshType::ElementIndex element_index(int i) const {
    DRAKE_ASSERT(0 <= i && i < std::get<LeafData>(child_).num_index);
    return std::get<LeafData>(child_).indices[i];
  }

  /* Returns the left child branch.
   @pre is_leaf() returns false.  */
  const BvNode<MeshType>& left() const {
    return *(std::get<NodeChildren>(child_).left);
  }

  /* Returns the right child branch.
   @pre is_leaf() returns false.  */
  const BvNode<MeshType>& right() const {
    return *(std::get<NodeChildren>(child_).right);
  }

  /* Returns whether this is a leaf node as opposed to a branch node.  */
  bool is_leaf() const {
    return std::holds_alternative<LeafData>(child_);
  }

  /* Compares element indices only. Bounding volumes are not checked.
   @pre both nodes are leaves.  */
  bool EqualLeaf(const BvNode<MeshType>& other_leaf) const {
    if (this == &other_leaf) return true;
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
  struct NodeChildren {
    std::unique_ptr<BvNode<MeshType>> left;
    std::unique_ptr<BvNode<MeshType>> right;

    NodeChildren(std::unique_ptr<BvNode<MeshType>> left_in,
                 std::unique_ptr<BvNode<MeshType>> right_in)
        : left(std::move(left_in)), right(std::move(right_in)) {
      DRAKE_DEMAND(left != nullptr);
      DRAKE_DEMAND(right != nullptr);
      DRAKE_DEMAND(left != right);
    }

    NodeChildren(const NodeChildren& other)
        : NodeChildren{std::make_unique<BvNode<MeshType>>(*other.left),
                       std::make_unique<BvNode<MeshType>>(*other.right)} {}
  };

  Obb bv_;

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
 for further action, e.g. deciding whether to exit early.  */
template <class MeshType, class OtherMeshType>
using BvttCallback = std::function<BvttCallbackResult(
    typename MeshType::ElementIndex, typename OtherMeshType::ElementIndex)>;

/* %Bvh is an acceleration structure for performing spatial queries against a
 collection of objects (in this case, triangles or tetrahedra). Specifically,
 for identifying those objects in or near a particular region of interest. It
 serves as a basis for culling objects that are trivially far from the region,
 reducing the number of "narrow-phase" calculations. The underlying structure
 is a binary tree of oriented bounding boxes (Obb) that encompass one or more
 mesh elements. The bounding volumes are all measured and expressed in this
 hierarchy's frame H. Leaf nodes contain element indices into elements of the
 mesh. The BVH needs a reference to the mesh in order to build the tree, but
 does not own the mesh.
 @pre    The mesh is not mutable. Modifications to the mesh after
         constructing the BVH will make the BVH invalid.
 @tparam MeshType SurfaceMesh<double> or VolumeMesh<double> (Exotic types like
         SurfaceMesh<AutoDiffXd> are not supported).  */
template <class MeshType>
class Bvh {
 public:
  using IndexType = typename MeshType::ElementIndex;

  explicit Bvh(const MeshType& mesh);

  Bvh(const Bvh& bvh) { *this = bvh; }

  Bvh& operator=(const Bvh& bvh) {
    if (&bvh == this) return *this;

    root_node_ = std::make_unique<BvNode<MeshType>>(*bvh.root_node_);
    return *this;
  }

  Bvh(Bvh&&) = default;
  Bvh& operator=(Bvh&&) = default;

  const BvNode<MeshType>& root_node() const { return *root_node_; }

  /* Perform a query of this bvh's mesh elements against the given bvh's
   mesh elements and runs the callback for each unculled pair.  */
  template <class OtherMeshType>
  void Collide(const Bvh<OtherMeshType>& bvh, const math::RigidTransformd& X_AB,
               BvttCallback<MeshType, OtherMeshType> callback) const {
    using NodePair =
        std::pair<const BvNode<MeshType>&, const BvNode<OtherMeshType>&>;
    std::stack<NodePair, std::vector<NodePair>> node_pairs;
    node_pairs.emplace(root_node(), bvh.root_node());

    while (!node_pairs.empty()) {
      const auto& [node_a, node_b] = node_pairs.top();
      node_pairs.pop();

      // Check if the bounding volumes overlap.
      if (!Obb::HasOverlap(node_a.bv(), node_b.bv(), X_AB)) {
        continue;
      }

      // Run the callback on the pair if they are both leaf nodes, otherwise
      // check each branch.
      if (node_a.is_leaf() && node_b.is_leaf()) {
        const int num_a_elements = node_a.num_element_indices();
        const int num_b_elements = node_b.num_element_indices();
        for (int a = 0; a < num_a_elements; ++a) {
          for (int b = 0; b < num_b_elements; ++b) {
            BvttCallbackResult result =
                callback(node_a.element_index(a), node_b.element_index(b));
            if (result == BvttCallbackResult::Terminate) return;  // Exit early.
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
   for Obb::HasOverlap() defined.   */
  template <typename PrimitiveType>
  void Collide(
      const PrimitiveType& primitive_P, const math::RigidTransformd& X_PH,
      std::function<BvttCallbackResult(typename MeshType::ElementIndex)>
          callback) const {
    std::stack<const BvNode<MeshType>*> nodes;
    nodes.emplace(&root_node());
    while (!nodes.empty()) {
      const auto node = nodes.top();
      nodes.pop();

      if (!Obb::HasOverlap(node->bv(), primitive_P, X_PH)) {
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
   collision.  */
  template <class OtherMeshType>
  std::vector<std::pair<IndexType, typename OtherMeshType::ElementIndex>>
  GetCollisionCandidates(const Bvh<OtherMeshType>& bvh,
                         const math::RigidTransformd& X_AB) const {
    std::vector<std::pair<IndexType, typename OtherMeshType::ElementIndex>>
        result;
    auto callback =
        [&result](
            IndexType a,
            typename OtherMeshType::ElementIndex b) -> BvttCallbackResult {
      result.emplace_back(a, b);
      return BvttCallbackResult::Continue;
    };
    Collide(bvh, X_AB, callback);
    return result;
  }

  /* Compares the two Bvh instances for exact equality down to the last bit.
   Assumes that the quantities are measured and expressed in the same frame. */
  bool Equal(const Bvh<MeshType>& other) const {
    if (this == &other) return true;
    return EqualTrees(this->root_node(), other.root_node());
  }

 private:
  // Convenience class for testing.
  friend class BvhTester;

  using CentroidPair = std::pair<IndexType, Vector3<double>>;

  static std::unique_ptr<BvNode<MeshType>> BuildBvTree(
      const MeshType& mesh,
      const typename std::vector<CentroidPair>::iterator& start,
      const typename std::vector<CentroidPair>::iterator& end);

  static Obb ComputeBoundingVolume(
      const MeshType& mesh,
      const typename std::vector<CentroidPair>::iterator& start,
      const typename std::vector<CentroidPair>::iterator& end);

  // TODO(tehbelinda): Move this function into SurfaceMesh/VolumeMesh directly
  // and rename to CalcElementCentroid(ElementIndex).
  static Vector3<double> ComputeCentroid(const MeshType& mesh,
                                         IndexType i);

  // Tests that the two hierarchy trees, rooted at nodes a and b, are equal in
  // the sense that they have identical structure and equal bounding volumes
  // (see Obb::Equal()).
  static bool EqualTrees(const BvNode<MeshType>& a, const BvNode<MeshType>& b);

  static constexpr int kElementVertexCount = MeshType::kDim + 1;

  std::unique_ptr<BvNode<MeshType>> root_node_;
};

}  // namespace internal
}  // namespace geometry
}  // namespace drake

extern template class drake::geometry::internal::Bvh<
    drake::geometry::SurfaceMesh<double>>;
extern template class drake::geometry::internal::Bvh<
    drake::geometry::VolumeMesh<double>>;

// TODO(SeanCurtis-TRI): Remove support for building a Bvh on an AutoDiff-valued
//  mesh after we've cleaned up the scalar types in hydroelastics. Specifically,
//  this is here to support the unit tests in mesh_intersection_test.cc. Also
//  the calls to convert_to_double should be removed.
//  See issue #14136.
extern template class drake::geometry::internal::Bvh<
    drake::geometry::SurfaceMesh<drake::AutoDiffXd>>;
extern template class drake::geometry::internal::Bvh<
    drake::geometry::VolumeMesh<drake::AutoDiffXd>>;
