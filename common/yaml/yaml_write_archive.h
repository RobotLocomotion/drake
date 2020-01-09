#pragma once

#include <array>
#include <map>
#include <optional>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <variant>
#include <vector>

#include <Eigen/Core>
#include <fmt/format.h>
#include "yaml-cpp/emitfromevents.h"
#include "yaml-cpp/yaml.h"

#include "drake/common/drake_copyable.h"
#include "drake/common/name_value.h"

namespace drake {
namespace yaml {

/// Saves data from a C++ structure into a YAML file, using the Serialize /
/// Archive pattern.
///
/// Sample code:
/// @code{cpp}
/// struct MyData {
///   double foo{NAN};
///   std::vector<double> bar;
///
///   template <typename Archive>
///   void Serialize(Archive* a) {
///     a->Visit(DRAKE_NVP(foo));
///     a->Visit(DRAKE_NVP(bar));
///   }
/// };
///
/// std::string SaveData(const MyData& data) {
///   common::YamlWriteArchive archive;
///   archive.Accept(data);
///   return archive.EmitString();
/// }
///
/// int main() {
///   MyData data{1.0, {2.0, 3.0}};
///   std::cout << SaveData(data);
///   return 0;
/// }
/// @endcode
///
/// Output:
/// @code{yaml}
/// root:
///   foo: 1.0
///   bar: [2.0, 3.0]
/// @endcode
///
/// Structures can be arbitrarily nested, as long as each `struct` has a
/// `Serialize` method.  Many common built-in types (int, double, std::string,
/// std::vector, std::array, std::map, std::unordered_map, std::optional,
/// std::variant, Eigen::Matrix) may also be used.
///
/// The EmitString output is always deterministic, even for unordered datatypes
/// like std::unordered_map.
///
/// For inspiration and background, see:
/// https://www.boost.org/doc/libs/release/libs/serialization/doc/tutorial.html
class YamlWriteArchive final {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(YamlWriteArchive)

  /// Creates an archive.  See the %YamlWriteArchive class overview for
  /// details.
  YamlWriteArchive() {}

  /// Copies the contents of `serializable` into the YAML object associated
  /// with this archive.  See the %YamlWriteArchive class overview for details.
  template <typename Serializable>
  void Accept(const Serializable& serializable) {
    auto* serializable_mutable = const_cast<Serializable*>(&serializable);
    root_ = {};
    visit_order_.clear();
    DoAccept(this, serializable_mutable, static_cast<int32_t>(0));
    if (!visit_order_.empty()) {
      YAML::Node key_order(YAML::NodeType::Sequence);
      for (const std::string& key : visit_order_) {
        key_order.push_back(key);
      }
      root_[kKeyOrderName] = std::move(key_order);
    }
  }

  /// Returns the YAML string for whatever Serializable was most recently
  /// passed into Accept.  The returned document will be a single Map node
  /// named using `root_name` with the Serializable's visited fields as
  /// key-value entries within it.
  std::string EmitString(const std::string& root_name = "root") const {
    std::string result;
    if (root_.IsNull()) {
      result = root_name + ":\n";
    } else {
      YAML::Node document;
      document[root_name] = root_;
      result = YamlDumpWithSortedMaps(document) + "\n";
    }
    return result;
  }

  /// (Advanced.)  Copies the value pointed to by `nvp.value()` into the YAML
  /// object.  Most users should should call Accept, not Visit.
  template <typename NameValuePair>
  void Visit(const NameValuePair& nvp) {
    // Use int32_t for the final argument to prefer the specialized overload.
    this->visit_order_.push_back(nvp.name());
    this->DoVisit(nvp, *nvp.value(), static_cast<int32_t>(0));
  }

 private:
  static constexpr const char* const kKeyOrderName = "__key_order";

  // Helpers for EmitString.
  static std::string YamlDumpWithSortedMaps(const YAML::Node&);
  static void RecursiveEmit(const YAML::Node&, YAML::EmitFromEvents*);

  // N.B. In the private details below, we use "NVP" to abbreviate the
  // "NameValuePair" template concept.

  // --------------------------------------------------------------------------
  // @name Overloads for the Accept() implementation

  // This version applies when Serialize is member method.
  template <typename Archive, typename Serializable>
  auto DoAccept(Archive* a, Serializable* serializable, int32_t) ->
      decltype(serializable->Serialize(a)) {
    return serializable->Serialize(a);
  }

  // This version applies when Serialize is an ADL free function.
  template <typename Archive, typename Serializable>
  void DoAccept(Archive* a, Serializable* serializable, int64_t) {
    Serialize(a, serializable);
  }

  // --------------------------------------------------------------------------
  // @name Overloads for the Visit() implementation

  // This version applies when the type has a Serialize member function.
  template <typename NVP, typename T>
  auto DoVisit(const NVP& nvp, const T&, int32_t) ->
      decltype(nvp.value()->Serialize(
          static_cast<YamlWriteArchive*>(nullptr))) {
    return this->VisitSerializable(nvp);
  }

  // This version applies when the type has an ADL Serialize function.
  template <typename NVP, typename T>
  auto DoVisit(const NVP& nvp, const T&, int32_t) ->
      decltype(Serialize(
          static_cast<YamlWriteArchive*>(nullptr),
          nvp.value())) {
    return this->VisitSerializable(nvp);
  }

  // For std::vector.
  template <typename NVP, typename T>
  void DoVisit(const NVP& nvp, const std::vector<T>&, int32_t) {
    std::vector<T>& data = *nvp.value();
    this->VisitArrayLike<T>(nvp.name(), data.size(),
                            data.empty() ? nullptr : &data.at(0));
  }

  // For std::array.
  template <typename NVP, typename T, std::size_t N>
  void DoVisit(const NVP& nvp, const std::array<T, N>&, int32_t) {
    this->VisitArrayLike<T>(nvp.name(), N, nvp.value()->data());
  }

  // For std::map.
  template <typename NVP, typename K, typename V, typename C>
  void DoVisit(const NVP& nvp, const std::map<K, V, C>&, int32_t) {
    this->VisitMap<K, V>(nvp);
  }

  // For std::unordered_map.
  template <typename NVP, typename K, typename V, typename C>
  void DoVisit(const NVP& nvp, const std::unordered_map<K, V, C>&, int32_t) {
    this->VisitMap<K, V>(nvp);
  }

  // For std::optional.
  template <typename NVP, typename T>
  void DoVisit(const NVP& nvp, const std::optional<T>&, int32_t) {
    this->VisitOptional(nvp);
  }

  // For std::variant.
  template <typename NVP, typename... Types>
  void DoVisit(const NVP& nvp, const std::variant<Types...>&, int32_t) {
    this->VisitVariant(nvp);
  }

  // For Eigen::Vector.
  template <typename NVP, typename T, int Rows>
  void DoVisit(const NVP& nvp, const Eigen::Matrix<T, Rows, 1>&, int32_t) {
    Eigen::Matrix<T, Rows, 1>& value = *nvp.value();
    const bool empty = value.size() == 0;
    this->VisitArrayLike<T>(nvp.name(), value.size(),
                            empty ? nullptr : &value.coeffRef(0));
  }

  // For Eigen::Matrix.
  template <typename NVP, typename T, int Rows, int Cols>
  void DoVisit(const NVP& nvp, const Eigen::Matrix<T, Rows, Cols>&, int32_t) {
    this->VisitMatrix<T>(nvp.name(), nvp.value());
  }

  // If no other DoVisit matched, we'll treat the value as a scalar.
  template <typename NVP, typename T>
  void DoVisit(const NVP& nvp, const T&, int64_t) {
    this->VisitScalar(nvp);
  }

  // --------------------------------------------------------------------------
  // @name Implementations of Visit() once the shape is known

  template <typename NVP>
  void VisitSerializable(const NVP& nvp) {
    YamlWriteArchive sub_archive;
    using T = typename NVP::value_type;
    const T& value = *nvp.value();
    sub_archive.Accept(value);
    root_[nvp.name()] = std::move(sub_archive.root_);
  }

  template <typename NVP>
  void VisitScalar(const NVP& nvp) {
    using T = typename NVP::value_type;
    const T& value = *nvp.value();
    root_[nvp.name()] = fmt::format("{}", value);
  }

  template <typename NVP>
  void VisitOptional(const NVP& nvp) {
    // Bail out if the optional was unset.
    if (!nvp.value()->has_value()) {
      // Since we are not going to add ourselves to root_, we should not list
      // our name in the visit_order either.  This undoes the addition of our
      // name that was performed by the Visit() call on the optional<T> value.
      this->visit_order_.pop_back();
      return;
    }

    // Visit the unpacked optional as if it weren't wrapped in optional<>.
    using T = typename NVP::value_type::value_type;
    T& storage = nvp.value()->value();
    this->Visit(drake::MakeNameValue(nvp.name(), &storage));

    // The above call to Visit() for the *unwrapped* value pushed our name onto
    // the visit_order a second time, duplicating work performed by the Visit()
    // for the *wrapped* value.  We'll undo that duplication now.
    this->visit_order_.pop_back();
}

  template <typename NVP>
  void VisitVariant(const NVP& nvp) {
    auto& variant = *nvp.value();
    if (variant.index() != 0) {
      throw std::runtime_error(fmt::format(
          "Cannot YamlWriteArchive the variant field {} "
          "with a non-zero type index {}",
          nvp.name(), variant.index()));
    }

    // Visit the unpacked variant as if it weren't wrapped in variant<>.
    auto& unboxed = std::get<0>(variant);
    this->Visit(drake::MakeNameValue(nvp.name(), &unboxed));

    // The above call to Visit() for the *unwrapped* value pushed our name onto
    // the visit_order a second time, duplicating work performed by the Visit()
    // for the *wrapped* value.  We'll undo that duplication now.
    this->visit_order_.pop_back();
  }

  template <typename T>
  void VisitArrayLike(const char* name, size_t size, T* data) {
    YAML::Node sub_node(YAML::NodeType::Sequence);
    for (size_t i = 0; i < size; ++i) {
      T& item = data[i];
      YamlWriteArchive sub_archive;
      sub_archive.Visit(drake::MakeNameValue("i", &item));
      sub_node.push_back(sub_archive.root_["i"]);
    }
    root_[name] = std::move(sub_node);
  }

  template <typename T, int Rows, int Cols>
  void VisitMatrix(const char* name, Eigen::Matrix<T, Rows, Cols>* matrix) {
    YAML::Node sub_node(YAML::NodeType::Sequence);
    for (int i = 0; i < matrix->rows(); ++i) {
      Eigen::Matrix<T, Cols, 1> row = matrix->row(i);
      YamlWriteArchive sub_archive;
      sub_archive.Visit(drake::MakeNameValue("i", &row));
      sub_node.push_back(sub_archive.root_["i"]);
    }
    root_[name] = std::move(sub_node);
  }


  template <typename Key, typename Value, typename NVP>
  void VisitMap(const NVP& nvp) {
    static_assert(std::is_same<Key, std::string>::value,
                  "Map keys must be strings");
    YAML::Node sub_node(YAML::NodeType::Map);
    // N.B. For std::unordered_map, this iteration order is non-deterministic,
    // but because YamlDumpWithSortedMaps sorts the keys anyway, it doesn't
    // matter what order we insert them here.
    for (auto& map_key_value_pair : *nvp.value()) {
      const std::string& key = map_key_value_pair.first;
      auto& value = map_key_value_pair.second;
      YamlWriteArchive sub_archive;
      sub_archive.Visit(drake::MakeNameValue(key.c_str(), &value));
      sub_node[key] = sub_archive.root_[key];
    }
    root_[nvp.name()] = std::move(sub_node);
  }

  YAML::Node root_;
  std::vector<std::string> visit_order_;
};

}  // namespace yaml
}  // namespace drake
