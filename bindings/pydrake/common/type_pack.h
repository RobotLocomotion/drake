#pragma once

/// @file
/// Basic meta-programming utilities for types, focused on template parameter
/// packs.

#include <cstddef>
#include <type_traits>
#include <typeindex>
#include <typeinfo>
#include <utility>

namespace drake {

template <typename... Ts>
struct type_pack;

namespace internal {

// Provides type at given index.
template <size_t N, size_t K, typename T, typename... Ts>
struct type_at_impl {
  using type = typename type_at_impl<N, K + 1, Ts...>::type;
};

// Base case.
template <size_t N, typename T, typename... Ts>
struct type_at_impl<N, N, T, Ts...> {
  using type = T;
};

// Visits a type given a VisitWith mechanism, templated to permit
// conditional execution.
template <typename VisitWith, typename Visitor>
struct type_visit_impl {
  template <typename T, bool execute>
  struct runner {
    inline static void run(Visitor&& visitor) {
      VisitWith::template run<T>(std::forward<Visitor>(visitor));
    }
  };
  template <typename T>
  struct runner<T, false> {
    inline static void run(Visitor&&) {}
  };
};

// Catches non-template types explicitly.
template <typename T>
struct type_pack_extract_impl {
  // Defer to show that this is a bad instantiation.
  static_assert(!std::is_same_v<T, T>, "Wrong template");
};

template <template <typename... Ts> class Tpl, typename... Ts>
struct type_pack_extract_impl<Tpl<Ts...>> {
  using type = type_pack<Ts...>;
};

// Provides type for pack expansion into an initializer list for
// deterministic execution order.
using DummyList = bool[];

template <typename T>
struct assert_default_constructible {
  static_assert(
      std::is_default_constructible_v<T>, "Must be default constructible");
};

}  // namespace internal

/// Extracts the Ith type from a sequence of types.
template <size_t I, typename... Ts>
struct type_at {
  static_assert(I >= 0 && I < sizeof...(Ts), "Invalid type index");
  using type = typename internal::type_at_impl<I, 0, Ts...>::type;
};

/// Provides a tag to pass a type for ease of inference.
template <typename T>
struct type_tag {
  using type = T;
};

/// Provides a tag for single-parameter templates.
/// @note Single-parameter is specialized because `using` aliases are picky
/// about how template parameters are passed.
/// @see https://stackoverflow.com/a/33131008/7829525
/// @note The above issues can be worked around if either (a) inheritance
/// rather than aliasing is used, or (b) the alias uses the *exact* matching
/// form of expansion.
template <template <typename> class Tpl>
struct template_single_tag {
  template <typename T>
  using type = Tpl<T>;
};

/// Provides a tag to pass a parameter packs for ease of inference.
template <typename... Ts>
struct type_pack {
  /// Number of template parameters.
  static constexpr int size = sizeof...(Ts);

  /// Rebinds parameter pack to a given template.
  template <template <typename...> class Tpl>
  using bind = Tpl<Ts...>;

  /// Extracts the Ith type from this sequence.
  template <size_t I>
  using type_at = typename drake::type_at<I, Ts...>::type;
};

/// Returns an expression (only to be used in `decltype`) for inferring
/// and binding a parameter pack to a template.
template <template <typename...> class Tpl, typename... Ts>
Tpl<Ts...> type_bind(type_pack<Ts...>);

/// Extracts the inner template arguments (typename only) for a typename which
/// is a template instantiation.
template <typename T>
using type_pack_extract = typename internal::type_pack_extract_impl<T>::type;

/// Visit a type by constructing its default value.
/// Useful for iterating over `type_tag`, `type_pack`, `std::integral_constant`,
/// etc.
struct type_visit_with_default {
  template <typename T, typename Visitor>
  inline static void run(Visitor&& visitor) {
    // TODO(eric.cousineau): Figure out how to make this the only error, without
    // wasting more function calls.
    (void)internal::assert_default_constructible<T>{};
    visitor(T{});
  }
};

/// Visits a type by construct a template tag's default value.
template <template <typename> class Tag = type_tag>
struct type_visit_with_tag {
  template <typename T, typename Visitor>
  inline static void run(Visitor&& visitor) {
    visitor(Tag<T>{});
  }
};

/// Provides a check which will return true for any type.
template <typename T>
using type_check_always_true = std::true_type;

/// Provides a check which returns whether `T` is different than `U`.
template <typename T>
struct type_check_different_from {
  template <typename U>
  using type = std::negation<std::is_same<T, U>>;
};

/// Visits each type in a type pack. This effectively implements a
// `constexpr for` loops. See `type_pack_test.cc` for usages.
/// @tparam VisitWith
///   Visit helper. @see `type_visit_with_default`, `type_visit_with_tag`.
/// @tparam Predicate Predicate operating on the type dictated by `VisitWith`.
/// @param visitor Lambda or functor for visiting a type.
template <class VisitWith = type_visit_with_default,
    template <typename> class Predicate = type_check_always_true,
    typename Visitor = void, typename... Ts>
void type_visit(Visitor&& visitor, type_pack<Ts...> = {},
    template_single_tag<Predicate> = {}) {
  // clang-format off
  (void)internal::DummyList{(
      internal::type_visit_impl<VisitWith, Visitor>::
          template runner<Ts, Predicate<Ts>::value>::
              run(std::forward<Visitor>(visitor)),
      true)...};
  // clang-format on
}

/// Provides short-hand for hashing a type.
template <typename T>
constexpr size_t type_hash() {
  return std::type_index(typeid(T)).hash_code();
}

}  // namespace drake
