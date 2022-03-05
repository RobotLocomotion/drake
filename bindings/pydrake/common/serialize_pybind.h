#pragma once

#include <climits>

#include "pybind11/pybind11.h"
#include <fmt/format.h>

#include "drake/common/drake_copyable.h"
#include "drake/common/drake_throw.h"

namespace drake {
namespace pydrake {
namespace internal {

// Helper for DefAttributesUsingSerialize.
template <typename PyClass, typename Docs>
class DefAttributesArchive {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(DefAttributesArchive)

  using CxxClass = typename PyClass::type;

  // @param ppy_class A pointer to the `py::class_` to add the properties to.
  // @param prototype A pointer to the instance that will be visited.
  // @param docs A pointer to ...
  DefAttributesArchive(
      PyClass* ppy_class, CxxClass* prototype, const Docs* cls_docs)
      : ppy_class_(ppy_class), prototype_(prototype), cls_docs_(cls_docs) {
    DRAKE_DEMAND(ppy_class != nullptr);
    DRAKE_DEMAND(prototype != nullptr);
  }

  // Creates a class property for one instance field, akin to def_readwrite
  // but using the CxxClass's Serialize function to iterate over the fields.
  template <typename NameValuePair>
  void Visit(const NameValuePair& nvp) {
    // In the prototype CxxClass instance, we're visiting the field named `name`
    // which stored at address `prototype_value` which sits at `offset` bytes
    // inside of the CxxClass.
    const char* const name = nvp.name();
    using T = typename NameValuePair::value_type;
    const T* const prototype_value = nvp.value();
    const int offset = CalcClassOffset(prototype_value);

    // Define property functions to get and set this particular field.
    pybind11::cpp_function getter(
        [offset](const CxxClass& self) -> const T& {
          return *reinterpret_cast<const T*>(
              reinterpret_cast<const char*>(&self) + offset);
        },
        pybind11::is_method(*ppy_class_));
    pybind11::cpp_function setter(
        [offset](CxxClass& self, const T& value) {
          *reinterpret_cast<T*>(reinterpret_cast<char*>(&self) + offset) =
              value;
        },
        pybind11::is_method(*ppy_class_));

    // Fetch the docstring.
    const char* const doc = [this, &name]() {
      if constexpr (std::is_same_v<Docs, void>) {
        return "";
      } else {
        for (const auto& member : cls_docs_->__all()) {
          if (member.first == name) {
            return member.second;
          }
        }
      }
      throw std::runtime_error(fmt::format("Missing docstring for {}", name));
    }();

    // Add the binding.
    ppy_class_->def_property(name, getter, setter, doc,
        pybind11::return_value_policy::reference_internal);
  }

  // Return the offset (in bytes) of `address` within our `prototype_` object.
  // Fails if the address does not fall within the `prototype_` object.
  int CalcClassOffset(const void* address) {
    static_assert(sizeof(CxxClass) < INT_MAX);
    const char* begin = reinterpret_cast<const char*>(prototype_);
    const char* end = begin + sizeof(CxxClass);
    DRAKE_DEMAND(address >= begin);
    DRAKE_DEMAND(address < end);
    return reinterpret_cast<const char*>(address) - begin;
  }

 private:
  PyClass* const ppy_class_;
  CxxClass* const prototype_;
  const Docs* const cls_docs_;
};

}  // namespace internal

/// Binds the attributes from a Serialize function. This function only works for
/// classes with a trivial Serialize function that uses DRAKE_NVP of each of its
/// member fields. Serialize functions that use DRAKE_NVP on stack variables
/// will fail.
template <typename PyClass, typename Docs>
void DefAttributesUsingSerialize(PyClass* ppy_class, const Docs& cls_docs) {
  using CxxClass = typename PyClass::type;
  CxxClass prototype{};
  internal::DefAttributesArchive archive(ppy_class, &prototype, &cls_docs);
  prototype.Serialize(&archive);
}

/// An overload that doesn't bind docstrings.
template <typename PyClass>
void DefAttributesUsingSerialize(PyClass* ppy_class) {
  using CxxClass = typename PyClass::type;
  CxxClass prototype{};
  internal::DefAttributesArchive<PyClass, void> archive(
      ppy_class, &prototype, nullptr);
  prototype.Serialize(&archive);
}

}  // namespace pydrake
}  // namespace drake
