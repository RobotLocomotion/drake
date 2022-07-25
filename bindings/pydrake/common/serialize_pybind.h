#pragma once

#include <climits>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

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
  // @param docs A pointer to the mkdoc struct for this ppy_class; can be null
  //   iff `Docs` is `void`, in which case docstrings will not be added.
  DefAttributesArchive(
      PyClass* ppy_class, CxxClass* prototype, const Docs* cls_docs)
      : ppy_class_(ppy_class), prototype_(prototype), cls_docs_(cls_docs) {
    DRAKE_DEMAND(ppy_class != nullptr);
    DRAKE_DEMAND(prototype != nullptr);
    DRAKE_DEMAND((std::is_same_v<Docs, void>) == (cls_docs == nullptr));
  }

  // Creates a class property for one instance field, akin to def_readwrite
  // but using `nvp` created by the CxxClass's Serialize function to specify
  // the field to be bound.
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
          const T* const field_in_self = reinterpret_cast<const T*>(
              reinterpret_cast<const char*>(&self) + offset);
          return *field_in_self;
        },
        pybind11::is_method(*ppy_class_));
    pybind11::cpp_function setter(
        [offset](CxxClass& self, const T& value) {
          T* const field_in_self =
              reinterpret_cast<T*>(reinterpret_cast<char*>(&self) + offset);
          *field_in_self = value;
        },
        pybind11::is_method(*ppy_class_));

    // Fetch the docstring (or the empty string, if we aren't using docstrings).
    const char* doc = "";
    if constexpr (!std::is_same_v<Docs, void>) {
      bool matched = false;
      for (const auto& member : cls_docs_->Serialize__fields()) {
        if (member.first == name) {
          matched = true;
          doc = member.second;
          break;
        }
      }
      if (!matched) {
        throw std::runtime_error(fmt::format("Missing docstring for {}", name));
      }
    }

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

/// Binds the attributes visited by a C++ class Serialize function as readwrite
/// on properties its ppy_class. This function only works for classes with a
/// trivial Serialize function that uses DRAKE_NVP on each of its member fields;
/// Serialize functions that use DRAKE_NVP on temporary stack variables are not
/// supported. The class also must be default-constructible.
///
/// @internal TODO(eric.cousineau): Investigate exposing struct classes as a
/// `dataclass`.
template <typename PyClass, typename Docs>
void DefAttributesUsingSerialize(PyClass* ppy_class, const Docs& cls_docs) {
  using CxxClass = typename PyClass::type;
  CxxClass prototype{};
  internal::DefAttributesArchive archive(ppy_class, &prototype, &cls_docs);
  prototype.Serialize(&archive);
}

/// (Advanced) An overload that doesn't bind docstrings. We expect that pydrake
/// bindings should always pass a Docs class (i.e., use the other overload),
/// in some cases (especially downstream projects) that might not be possible.
template <typename PyClass>
void DefAttributesUsingSerialize(PyClass* ppy_class) {
  using CxxClass = typename PyClass::type;
  CxxClass prototype{};
  internal::DefAttributesArchive<PyClass, void> archive(
      ppy_class, &prototype, nullptr);
  prototype.Serialize(&archive);
}

namespace internal {

// Helper for DefReprUsingSerialize.
class DefReprArchive {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(DefReprArchive)
  DefReprArchive() = default;

  // Appends the visited item's name to the list of names.
  template <typename NameValuePair>
  void Visit(const NameValuePair& nvp) {
    const char* const name = nvp.name();
    property_names_.push_back(name);
  }

  std::vector<std::string>& property_names() { return property_names_; }

 private:
  std::vector<std::string> property_names_;
};

}  // namespace internal

/// Binds __repr__ using a C++ class Serialize function.
/// The class must be default-constructible.
template <typename PyClass>
void DefReprUsingSerialize(PyClass* ppy_class) {
  using CxxClass = typename PyClass::type;
  internal::DefReprArchive archive;
  CxxClass prototype{};
  prototype.Serialize(&archive);
  ppy_class->def("__repr__",
      [names = std::move(archive.property_names())](pybind11::object self) {
        std::ostringstream result;
        result << pybind11::str(self.attr("__class__").attr("__name__"));
        result << "(";
        bool first = true;
        for (const std::string& name : names) {
          if (!first) {
            result << ", ";
          }
          result << name << "=" << pybind11::repr(self.attr(name.c_str()));
          first = false;
        }
        result << ")";
        return result.str();
      });
}

}  // namespace pydrake
}  // namespace drake
