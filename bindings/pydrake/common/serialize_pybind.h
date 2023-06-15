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

#include <fmt/format.h>

#include "drake/bindings/pydrake/common/cpp_template_pybind.h"
#include "drake/bindings/pydrake/pydrake_pybind.h"
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
    py::cpp_function getter(
        [offset](const CxxClass& self) -> const T& {
          const T* const field_in_self = reinterpret_cast<const T*>(
              reinterpret_cast<const char*>(&self) + offset);
          return *field_in_self;
        },
        py::is_method(*ppy_class_));
    py::cpp_function setter(
        [offset](CxxClass& self, const T& value) {
          T* const field_in_self =
              reinterpret_cast<T*>(reinterpret_cast<char*>(&self) + offset);
          *field_in_self = value;
        },
        py::is_method(*ppy_class_));

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
    ppy_class_->def_property(
        name, getter, setter, doc, py::return_value_policy::reference_internal);

    // Remember the field's name and type for later use by Finished().
    auto field = py::module::import("types").attr("SimpleNamespace")();
    py::setattr(field, "name", py::str(name));
    py::setattr(field, "type", CalcSchemaType(prototype_value));
    fields_.append(field);
  }

  // To be called after Serialize() is complete; binds any members that are
  // scoped to the entire struct (rather than one field at a time).
  void Finished() {
    ppy_class_->def_property_readonly_static("__fields__",
        [fields_tuple = py::tuple(fields_)](py::object /* self */) {  // BR
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

  // Returns the python type annotation corresponding to the given T.
  //
  // Our goal here is to align with what pybind11::type_caster produces when we
  // bind this struct's C++ member fields as Python attributes, e.g., for a C++
  // type `int16_t` we'll return the Python type `int` not `np.int16`. For any
  // compound types (list, dict, Union, etc.) we'll use Python's conventional
  // generic types (either the builtins or via `typing`) but via the cpp_param
  // logic to ensure that alias types are always canonicalized.
  //
  // This logic is similar to GetPyParam<T> (in cpp_param_pybind.h) but with
  // the important difference that in our case the primitive types follow the
  // type_caster<> rules (C++ int16_t => Python int) whereas GetPyParam uses
  // the numpy sizes (C++ int16_t => Python np.int16). For the purposes of
  // DefAttributesUsingSerialize we want the return type of property getters
  // (which use the type_caster<>) to match the schema type.
  //
  // Note that the argument pointer is ignored (can be nullptr).
  // Note that this template function is specialized below for container types.
  //
  // @tparam T the C++ type for which we'll return the Python type.
  template <typename T>
  static py::object CalcSchemaType(const T*) {
    // Pybind11 doesn't support type::of<> for primitive types, so we must
    // match them manually. See https://github.com/pybind/pybind11/issues/2486.
    if constexpr (std::is_same_v<T, bool>) {
      return py::type::of(py::bool_());
    } else if constexpr (std::is_integral_v<T>) {
      return py::type::of(py::int_());
    } else if constexpr (std::is_floating_point_v<T>) {
      return py::type::of(py::float_());
    } else if constexpr (std::is_same_v<T, std::string>) {
      return py::type::of(py::str());
    } else if constexpr (is_eigen_type<T>::value) {
      // TODO(jwnimmer-tri) Perhaps we can use numpy.typing here some day?
      return py::module::import("numpy").attr("ndarray");
    } else {
      // Anything that remains should be a registered C++ type.
      constexpr bool is_registered_type =
          std::is_base_of_v<py::detail::type_caster_generic,
              py::detail::make_caster<T>>;
      if constexpr (is_registered_type) {
        return py::type::of<T>();
      } else {
        return CannotIdentifySchemaType<T>();
      }
    }
  }

  // Partial specialization for List.
  template <typename U>
  static py::object CalcSchemaType(const std::vector<U>*) {
    auto u_type = CalcSchemaType(static_cast<U*>(nullptr));
    return GetTemplateClass("List")[u_type];
  }

  // Partial specialization for Dict.
  template <typename U, typename V>
  static py::object CalcSchemaType(const std::map<U, V>*) {
    auto u_type = CalcSchemaType(static_cast<U*>(nullptr));
    auto v_type = CalcSchemaType(static_cast<V*>(nullptr));
    auto inner_types = py::make_tuple(u_type, v_type);
    return GetTemplateClass("Dict")[inner_types];
  }

  // Partial specialization for Optional.
  template <typename U>
  static py::object CalcSchemaType(const std::optional<U>*) {
    auto u_type = CalcSchemaType(static_cast<U*>(nullptr));
    return GetTemplateClass("Optional")[u_type];
  }

  // Partial specialization for Union.
  template <typename... Types>
  static py::object CalcSchemaType(const std::variant<Types...>*) {
    auto inner_types = py::make_tuple(  // BR
        CalcSchemaType(static_cast<Types*>(nullptr))...);
    return GetTemplateClass("Union")[inner_types];
  }

  // Returns the cpp_param template class for the given name (e.g., "List",
  // "Dict", etc.).
  static py::object GetTemplateClass(const char* name) {
    return py::module::import("pydrake.common.cpp_param").attr(name);
  }

  // When there is no match found for the schema type, this function will
  // produce a compile-time error that's at least somewhat readable.
  template <typename T>
  static py::object CannotIdentifySchemaType(T*) {
    // N.B. This static_assert will always fail, but with a nice message.
    static_assert(std::is_same_v<T, void>,
        "DefAttributesUsingSerialize() could not understand a field type");
    return py::none();
  }

  PyClass* const ppy_class_;
  CxxClass* const prototype_;
  const Docs* const cls_docs_;

  // As we visit each field, we'll accumulate a list of [{name=, type=}, ...]
  // to bind later as the `__fields__` static property.
  py::list fields_;
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
  archive.Finished();
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
      [names = std::move(archive.property_names())](py::object self) {
        std::ostringstream result;
        result << py::str(internal::PrettyClassName(self.attr("__class__")));
        result << "(";
        bool first = true;
        for (const std::string& name : names) {
          if (!first) {
            result << ", ";
          }
          result << name << "=" << py::repr(self.attr(name.c_str()));
          first = false;
        }
        result << ")";
        return result.str();
      });
}

}  // namespace pydrake
}  // namespace drake
