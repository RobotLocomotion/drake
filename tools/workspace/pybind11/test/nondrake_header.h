#pragma once

/// @dir
/// Directory docs ignored.

/// @file
/// File documentation is ignored.

/// @defgroup first_group Ignored.

template <typename ... Args>
void unused(const Args& ...) {}
#define DEPRECATED(removal_date, message)               \
  [[deprecated(                                         \
  "\nDEPRECATED: " message                              \
  "\nThe deprecated code will be removed"               \
  " on or after " removal_date ".")]]

#define DRAKE_NO_COPY_NO_MOVE(DummyClassname)      \
  DummyClassname(const DummyClassname&) = delete;                 \
  void operator=(const DummyClassname&) = delete;            \
  DummyClassname(DummyClassname&&) = delete;                      \
  void operator=(DummyClassname&&) = delete;


#define NO_COPY_NO_MOVE_NO_ASSIGN(Classname)      \
  Classname(const Classname&) = delete;                 \
  void operator=(const Classname&) = delete;            \
  Classname(Classname&&) = delete;                      \
  void operator=(Classname&&) = delete;

#define DEFAULT_COPY_AND_MOVE_AND_ASSIGN(Classname)       \
  Classname(const Classname&) = default;                        \
  Classname& operator=(const Classname&) = default;             \
  Classname(Classname&&) = default;                             \
  Classname& operator=(Classname&&) = default;

/// @def PREPROCESSOR_DEFINITION
/// Preprocessor definitions are ignored.
#define PREPROCESSOR_DEFINITION "Nisl purus in mollis nunc sed id."

/// Root-level symbol.
struct RootLevelSymbol {};

/// @namespace drake
/// Namespaces are ignored.
namespace drake {

/**
 * MidLevelSymbol.
 */
struct MidLevelSymbol {};

namespace mkdoc_test {

// A forward-declaration is ignored.
class Class;

/** Function. */
void func();
/// Function, overload 1.
void func(int* param);
/// Function, template overload.
template <typename T>
void func(T tee);

struct SimpleStruct { int value; };

template <typename T>
struct TemplateStruct { T value; };

/// Code.
class Class {
 public:
  DEFAULT_COPY_AND_MOVE_AND_ASSIGN(Class)

  /// @name Name. Stuff and things.
  /// @{

  /// Using alias.
  using UsingAlias = int;

  /// Typedef alias.
  typedef int TypedefAlias;

  /// @}

  /// @addtogroup second_group Second group.
  /// @{

  /// Custom constructor 1.
  Class() {}

  /// Custom constructor 2.
  explicit Class(int param) {
    unused(param);
  }

  /// @}

  /// Custom constructor 3.
  explicit Class(int param1, TemplateStruct<int> param2) {
    unused(param1);
    unused(param2);
  }

  void PublicMethod() {}

  // Private documentation.
  template <typename T>
  void PublicTemplateMethod() {}

  /// More things.
  static void PublicStatic() {}

  /// This one takes an int.
  void overloaded_method(int alpha);

  /// This one takes a double.
  void overloaded_method(double bravo);

  /// This one takes an int and a double.
  void overloaded_method(int charlie, double delta);

  /// This one takes the road less traveled.
  void overloaded_method(double, int);

  /// This one takes a non-primitive type.
  void overloaded_method(const SimpleStruct&);

  /// Different overload with same doc.
  void overloaded_with_same_doc();

  /// Different overload with same doc.
  void overloaded_with_same_doc(int);

  /// Overloaded only by its const-ness.
  void get_foo();

  /// The const one.
  void get_foo() const;

  /// Docstring 1.
  /// @pydrake_mkdoc_identifier{stuff_1}
  void do_stuff(double);

  /// Docstring 2.
  /// @pydrake_mkdoc_identifier{stuff_2}
  template <typename T>
  void do_stuff(T);

 protected:
  /// Protected method.
  int ProtectedMethod() {
    return 0;
  }

  /// Protected nested class.
  class Nested {};

  int protected_member_{};

 private:
  /// Private method, public documentation.
  void PrivateMethod() {}

  // Private member, private documentation.
  int private_member_{};
};

/// Struct.
struct Struct {
  /// Field 1.
  int field_1;
  /// Field 2.
  TemplateStruct<int> field_2;
};

/// Template class.
template <typename T>
class TemplateClass {
 public:
  DEFAULT_COPY_AND_MOVE_AND_ASSIGN(TemplateClass)

  /// Default constructor.
  TemplateClass();

  /// Single argument int constructor.
  explicit TemplateClass(int i);

  /// Scalar-converting copy constructor.
  template <typename U>
  explicit TemplateClass(const TemplateClass<U>&);
};

// Out-of-line definition.
template <typename T>
TemplateClass<T>::TemplateClass() {}

// Out-of-line definition.
template <typename T>
TemplateClass<T>::TemplateClass(int i) {}

// Out-of-line definition.
template <typename T>
template <typename U>
TemplateClass<T>::TemplateClass(const TemplateClass<U>&)
    : TemplateClass<T>() {}

/// Specialize.
template <>
class TemplateClass<int> {
};

/// Enumeration.
enum Enum {
  /// Enumeration constant.
  EnumConstant,
};

/// Enumeration class.
enum EnumClass {
  EnumClassConstant,  ///< Enumeration class constant. Vestibulum mattis.
};

/// Anonymous values are ignored.
enum {
  /// Anonymous enum's constant.
  AnonymousConstant,
};

/// I am measurably old.
class DEPRECATED("2038-01-19", "Use MyNewClass instead.")
DrakeDeprecatedClass {
 public:
  DEPRECATED("2038-01-19",
    "f() is slow; use g() instead. "
    "Also, I like hats.")
  int f(int arg);

  DEPRECATED("2038-01-19",
    "f() now requires an int.")
  int f();

  /// Ideally this overview would still appear, but it does not yet.
  DEPRECATED("2038-01-19", "a() is slow; use b() instead.")
  int a();
};

/// I am symbolically old.
template <typename T>
class DEPRECATED("2038-01-19", "Templates rule!")
DrakeDeprecatedTemplateClass {};

}  // namespace mkdoc_test
}  // namespace drake

namespace namespace_1 {
namespace namespace_2 {
/** Quam odio at est.
 *
 * Proin eleifend nisi et nibh.
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
  NO_COPY_NO_MOVE_NO_ASSIGN(DummyClass)

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

// Must be ignored
namespace dev {
struct IgnoredStruct {
  int var_1{};
};

}

// Must be ignored
namespace internal {
struct IgnoredStruct {
  int var_1{};
};

}
}  // namespace namespace_1
