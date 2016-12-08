#pragma once

#include <cassert>
#include <memory>
#include <typeindex>
#include <typeinfo>
#include <unordered_map>
#include <utility>
#include <vector>

/** This is the type-erased base class for instantiated `ObjectWithAlternates`
 * classes. **/
class ObjectWithAlternatesBase {
 public:
  virtual ~ObjectWithAlternatesBase() {}
};

/** For a ObjectWithAlternates instantiated with the fundamental scalar type `double` this
class contains a registry of the other available instantiations of the
same ObjectWithAlternates. For any other type this struct is empty. **/
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
                         std::unique_ptr<ObjectWithAlternatesBase> alt) {
    auto entry = type_to_instantiation_.insert(
        std::pair<std::type_index, size_t>(index, instantiations.size()));
    if (!entry.second) return;  // This alternate is already present.
    instantiations.push_back(std::move(alt));
  }

  const ObjectWithAlternatesBase& get_alternate(const std::type_index& index) const {
    auto entry = type_to_instantiation_.find(index);
    assert(entry != type_to_instantiation_.end());
    return *instantiations[entry->second];
  }

  int get_num_alternates() const { return (int)instantiations.size(); }

 private:
  std::vector<std::unique_ptr<ObjectWithAlternatesBase>> instantiations;

  // Here is how you find the right instantiation.
  std::unordered_map<std::type_index, size_t> type_to_instantiation_;
};

/** This is a ObjectWithAlternates instantiated for a concrete ObjectWithAlternates type DerivedObjectWithAlternates, which
must be derived from `ObjectWithAlternates<MyObjectWithAlternates, T>` for an Eigen Scalar type T. **/
template <template <typename> class DerivedObjectWithAlternates, typename T>
class ObjectWithAlternates : public ObjectWithAlternatesBase {
 public:
  using ObjectWithAlternatesT = ObjectWithAlternates;

  /** Default constructor creates an empty ObjectWithAlternates with no alternative
  instantiations. **/
  ObjectWithAlternates() {}

  /** Construction from fundamental instantiation. **/
  explicit ObjectWithAlternates(const DerivedObjectWithAlternates<double>& fundamental) {

  }


  /** Create an alternate instantiation of the fundamental ObjectWithAlternates and
  register this one with it. **/
  static void AddAlternate(DerivedObjectWithAlternates<double>& fundamental) {
    std::unique_ptr<DerivedObjectWithAlternates<T>> alternate(new DerivedObjectWithAlternates<T>(fundamental));
    // TODO(sherm) Make alternate mirror fundamental.
    fundamental.get_mutable_alternates()->RegisterAlternate(
        std::type_index(typeid(T)),
        std::unique_ptr<ObjectWithAlternatesBase>(alternate.release()));
  }

  template <class TT>
  const DerivedObjectWithAlternates<TT>& get_alternate() const {
    const ObjectWithAlternatesBase& alt =
        alternates_.get_alternate(std::type_index(typeid(TT)));
    return dynamic_cast<const DerivedObjectWithAlternates<TT>&>(alt);
  }

  int get_num_alternates() const { return alternates_.get_num_alternates(); }
  AlternateInstantiations<T>* get_mutable_alternates() { return &alternates_; }

 protected:

 private:
  AlternateInstantiations<T> alternates_;
};
