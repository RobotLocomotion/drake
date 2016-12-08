#pragma once

#include <map>
#include <memory>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include "drake/common/eigen_stl_types.h"
#include "drake/multibody/rigid_body_tree.h"

// The minimum set of required methods is here: rigidBodyTreeMexFunctions.h

/** This is the type-erased base class for instantiated classes. **/
class InstantiatedObject {
 public:
  virtual ~InstantiatedObject() {}
};

/** For a System instantiated with the fundamental scalar type `double` this
class contains a registry of the other available instantiations of the
same System. For any other type this struct is empty. **/
template <typename T>
class AlternateInstantiations {
 public:
  // Default implementation is empty.
  int get_num_alternates() const { return 0; }
};

/** Specialization for double. **/
template<>
class AlternateInstantiations<double> {
 public:
  void RegisterAlternate(const std::type_index& index,
                         std::unique_ptr<InstantiatedObject> alt) {
    auto entry = type_to_instantiation_.insert(
        std::pair<std::type_index, size_t>(index, instantiations.size()));
    if (!entry.second) return;  // This alternate is already present.
    instantiations.push_back(std::move(alt));
  }

  const InstantiatedObject& get_alternate(const std::type_index& index) const {
    auto entry = type_to_instantiation_.find(index);
    assert(entry != type_to_instantiation_.end());
    return *instantiations[entry->second];
  }

  int get_num_alternates() const { return (int)instantiations.size(); }

 private:
  std::vector<std::unique_ptr<InstantiatedObject>> instantiations;

  // Here is how you find the right instantiation.
  std::unordered_map<std::type_index, size_t> type_to_instantiation_;
};

template <typename T>
class RigidBodyTreeWithAlternates : public InstantiatedObject {
 public:
  explicit RigidBodyTreeWithAlternates(
      const RigidBodyTreeWithAlternates<double>& fundamental) {
    // tree_ = fundamental.CloneToRigidBodyTreeWithScalarType<T>();
    // tree_ = std::make_unique<RigidBodyTree<T>>(fundamental);
  }

  /** Create an alternate instantiation of the fundamental System and
   * register this one with it. **/
  static void AddAlternate(RigidBodyTreeWithAlternates<double>& fundamental) {
    std::unique_ptr<RigidBodyTreeWithAlternates<T>>
        alternate(new RigidBodyTreeWithAlternates<T>(fundamental));
    // TODO(sherm) Make alternate mirror fundamental.
    fundamental.get_mutable_alternates()->RegisterAlternate(
        std::type_index(typeid(T)),
        std::unique_ptr<InstantiatedSystem>(alternate.release()));
  }

  template <class TT>
  const RigidBodyTreeWithAlternates<TT>& get_alternate() const {
    const InstantiatedSystem& alt =
        alternates_.get_alternate(std::type_index(typeid(TT)));
    return dynamic_cast<const RigidBodyTreeWithAlternates<TT>&>(alt);
  }

  int get_num_alternates() const { return alternates_.get_num_alternates(); }

  AlternateInstantiations<T>* get_mutable_alternates() { return &alternates_; }

  // For debugging purposes, report the name of the scalar type here.
  const char* type() const { return typeid(T).name(); }

  const MBTree<T>& get_rigid_body_tree() const { return *tree_; }

 private:
  // Let all other instantiations of this class be friends with this one.
  template <typename TT>
  friend class RigidBodyTreeWithAlternates;

  std::unique_ptr<RigidBodyTree<T>> tree_;

  AlternateInstantiations<T> alternates_;
};

class MatlabRigidBodyTree {
 public:
  // In constructModelmex.cpp use RigidBodyTree<double> as the IR that then
  // is used by MatlabRigidBodyTree to get constructed.
  // Also take ownership of the <double> version so you don't need to clone it.
  MatlabRigidBodyTree(std::unique_ptr<RigidBodyTree<double>> tree);

  ~MatlabRigidBodyTree();

  template <typename Scalar>
  Eigen::Matrix<Scalar, drake::kSpaceDimension, 1>
  centerOfMassJacobianDotTimesV(
      // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
      KinematicsCache<Scalar>& cache,
      const std::set<int>& model_instance_id_set =
      default_model_instance_id_set) const;

 private:
  std::unique_ptr<RigidBodyTree<double>> tree_double_;
  //std::unique_ptr<RigidBodyTree<AutoDiffXd>> tree_dynamic_autodiff_;

  // Use a map? so that I don't need to create a lot of overloaded methods?
  //std::map<some_sort_of_type_id, void*>
};
