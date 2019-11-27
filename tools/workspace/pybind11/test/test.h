#pragma once

#define DUMMY_MACRO(DummyClassname)      \
  DummyClassname(const DummyClassname&) = delete;                 \
  void operator=(const DummyClassname&) = delete;            \
  DummyClassname(DummyClassname&&) = delete;                      \
  void operator=(DummyClassname&&) = delete;

namespace namespace_1 {
namespace namespace_2 {
/** Quam odio at est.
 *
 * Proin eleifend nisi et nibh. Maecenas a lacus. Mauris porta quam non massa
 * molestie scelerisque. Nulla sed ante at lorem suscipit rutrum. Nam quis
 * tellus. Cras elit nisi, ornare a, condimentum vitae, rutrum sit amet, tellus.
 * Maecenas
 */

struct Struct1 {
  /// Et, ornare sagittis, tellus. Fusce felis.
  int var_1{};
  /// Nulla a augue. Pellentesque sed est.
  int var_2{};
  /// Imperdiet tristique, interdum a, dolor.
  double var_3{};
  /// Tempor lobortis turpis. Sed tellus velit, ullamcorper.
  double var_4{};
  /// Id, rutrum auctor, ullamcorper sed, orci. In.
  double var_5{};
  /// Fames ac turpis egestas. Sed vitae eros. Nulla.
  double var_6{};
  /// Condimentum. Donec arcu quam, dictum accumsan, convallis.
  double var_7{};
  /// Volutpat. Donec non tortor. Vivamus posuere nisi mollis.
  double var_8{};
};

struct Struct2 {};
struct Struct3 {};
struct Struct4 {};
struct Struct5 {};
struct Struct6{};

class DummyClass {
 public:
  DUMMY_MACRO(DummyClass)

  /// Ligula. Nunc turpis. Mauris vitae sapien. Nunc.
  using Details = Struct1;

  DummyClass();
  ~DummyClass();

  /// @Litgn Phasellus in odio. Duis lobortis, metus eu.
  //@{
  static Struct2 struct_2();
  static bool static_function_1();
  static bool static_function_2(const Struct3&);
  //@}

 private:
  void DoSolve(const Struct3&, const Struct4&,
               const Struct5&, Struct6*) const;
};
}  // namespace namespace_2

namespace dev {
// Must be ignored
struct IgnoredStruct {
  int var_1{};
  int var_2{};
  double var_3{};
};

}
}  // namespace namespace_1
