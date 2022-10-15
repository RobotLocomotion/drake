#pragma once

#include <climits>
#include <map>
#include <optional>
#include <sstream>
#include <string>
#include <type_traits>
#include <utility>
#include <variant>
#include <vector>

#include "pybind11/eigen.h"
#include "pybind11/pybind11.h"
#include "pybind11/stl.h"
#include <fmt/format.h>

#include "drake/common/drake_copyable.h"
#include "drake/common/drake_throw.h"
#include "drake/common/eigen_types.h"

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

    // Memorize the field name and type for later use by Finished().
    auto field = pybind11::module::import("types").attr("SimpleNamespace")();
    pybind11::setattr(field, "name", pybind11::str(name));
    pybind11::setattr(field, "type", CalcSchemaType(prototype_value));
    fields_.append(field);
  }

  // To be called after Serialize() is complete; binds any members that are
  // scoped to the entire struct (rather than one field at a time).
  void Finished() {
    ppy_class_->def_property_readonly_static(
        "__fields__", [fields_tuple = pybind11::tuple(fields_)](
                          pybind11::object /* self */) {  // BR
          return fields_tuple;
        });
  }

 private:
  // Returns the offset (in bytes) of `address` within our `prototype_` object.
  // Fails if the address does not fall within the `prototype_` object.
  int CalcClassOffset(const void* address) {
    static_assert(sizeof(CxxClass) < INT_MAX);
    const char* begin = reinterpret_cast<const char*>(prototype_);
    const char* end = begin + sizeof(CxxClass);
    DRAKE_DEMAND(address >= begin);
    DRAKE_DEMAND(address < end);
    return reinterpret_cast<const char*>(address) - begin;
  }

  // Returns the python type-annotation corresponding to the given T.
  template <typename T>
  static pybind11::object CalcSchemaType(const T*) {
    // When the type is registered with pybind11, we can use type::of.
    if constexpr (std::is_base_of_v<pybind11::detail::type_caster_generic,
                      pybind11::detail::make_caster<T>>) {
      return pybind11::type::of<T>();
    } else if constexpr (std::is_same_v<T, bool>) {
      return pybind11::type::of(pybind11::bool_());
    } else if constexpr (std::is_integral_v<T>) {
      return pybind11::type::of(pybind11::int_());
    } else if constexpr (std::is_floating_point_v<T>) {
      return pybind11::type::of(pybind11::float_());
    } else if constexpr (std::is_same_v<T, std::string>) {
      return pybind11::type::of(pybind11::str());
    } else if constexpr (is_eigen_type<T>::value) {
      return pybind11::module::import("numpy").attr("ndarray");
    } else {
      return CannotIdentifySchemaType<T>();
    }
  }

  template <typename U>
  static pybind11::object CalcSchemaType(const std::optional<U>*) {
    auto u_type = CalcSchemaType(static_cast<U*>(nullptr));
    return pybind11::module::import("typing").attr("Optional")[u_type];
  }

  template <typename U>
  static pybind11::object CalcSchemaType(const std::vector<U>*) {
    auto u_type = CalcSchemaType(static_cast<U*>(nullptr));
    return pybind11::module::import("typing").attr("List")[u_type];
  }

  template <typename U, typename V>
  static pybind11::object CalcSchemaType(const std::map<U, V>*) {
    auto u_type = CalcSchemaType(static_cast<U*>(nullptr));
    auto v_type = CalcSchemaType(static_cast<V*>(nullptr));
    auto inner_types = pybind11::make_tuple(u_type, v_type);
    return pybind11::module::import("typing").attr("Mapping")[inner_types];
  }

  template <typename... Types>
  static pybind11::object CalcSchemaType(const std::variant<Types...>*) {
    auto inner_types = pybind11::make_tuple(  // BR
        CalcSchemaType(static_cast<Types*>(nullptr))...);
    return pybind11::module::import("typing").attr("Union")[inner_types];
  }

  // When there is no match found for the schema type, this function will
  // produce a compile-time error that's at least somewhat readable.
  template <typename T>
  static pybind11::object CannotIdentifySchemaType(T*) {
    // N.B. This static_assert will always fail, but with a nice message.
    static_assert(std::is_same_v<T, void>,
        "DefAttributesUsingSerialize() could not understand a field type");
    return pybind11::none();
  }

  PyClass* const ppy_class_;
  CxxClass* const prototype_;
  const Docs* const cls_docs_;

  // As we visit each field, we'll accumulate a list of [{name=, type=}, ...]
  // to bind later as the `__fields__` static propery.
  pybind11::list fields_;
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
  archive.Finished();
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
