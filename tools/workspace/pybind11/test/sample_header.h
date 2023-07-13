#pragma once

/// @dir
/// Directory documentation is ignored. Sit amet massa vitae tortor. Pulvinar
/// pellentesque habitant morbi tristique senectus et. Lacus sed turpis
/// tincidunt id.

/// @file
/// File documentation is ignored. Sit amet nisl purus in mollis nunc sed id
/// semper. Risus nec feugiat in fermentum posuere urna nec tincidunt praesent.
/// Suscipit tellus mauris a diam.

/// @defgroup first_group Elementum pulvinar etiam non quam lacus.
/// Ultrices in iaculis nunc sed augue lacus viverra. Dolor sit amet
/// consectetur adipiscing elit duis tristique.

#include <string>
#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/common/drake_deprecated.h"
#include "drake/common/unused.h"

/// @def PREPROCESSOR_DEFINITION
/// Preprocessor definitions are ignored. In nibh mauris cursus mattis
/// molestie a. Non arcu risus quis varius quam quisque id.
// (Test) Ensure this comment does not ruin docstring association.
#define PREPROCESSOR_DEFINITION "Nisl purus in mollis nunc sed id."

/// Root-level symbol. Magna fermentum iaculis eu non diam phasellus
/// vestibulum.
// (Test) Ensure this comment does not ruin docstring association.
struct RootLevelSymbol {};

/// @namespace drake
/// Namespaces are ignored. Enim blandit volutpat maecenas volutpat blandit. Eu
/// feugiat pretium nibh ipsum consequat.
// (Test) Ensure this comment does not ruin docstring association.
namespace drake {

/**
 * 1. Begin first ordered list element. Rutrum quisque non tellus orci ac
 *    auctor. End first ordered list element.
 * 2. Begin second ordered list element. Ipsum faucibus vitae aliquet nec.
 *    Ligula ullamcorper malesuada proin libero. End second ordered list
 *    element.
 * 3. Begin third ordered list element. Dictum sit amet justo donec enim.
 *    Pharetra convallis posuere morbi leo urna molestie. End third ordered
 *    list element.
 *
 * Senectus et netus et malesuada fames ac. Tincidunt lobortis feugiat vivamus
 * at augue eget arcu dictum varius.
 */
 // (Test) Ensure this comment does not ruin docstring association.
struct MidLevelSymbol {};

namespace mkdoc_test {

// A forward-declaration is ignored.
class Class;

/** Function. Mi sit amet mauris commodo quis. */
// (Test) Ensure this comment does not ruin docstring association.
void func();
/// Function, overload 1. Velit ut tortor pretium viverra suspendisse potenti
/// nullam ac tortor.
// (Test) Ensure this comment does not ruin docstring association.
void func(int* param);
/// Function, template overload. Pellentesque diam volutpat commodo sed egestas
/// egestas fringilla phasellus faucibus.
// (Test) Ensure this comment does not ruin docstring association.
template <typename T>
void func(T tee);

/// * Begin first unordered list element. Volutpat blandit aliquam etiam erat
///   velit scelerisque. End first unordered list element.
/// * Begin second unordered list element. Eget est lorem ipsum dolor sit amet.
///   Ipsum dolor sit amet consectetur adipiscing. End second unordered list
///   element.
/// * Begin third unordered list element. Hac habitasse platea dictumst quisque
///   sagittis purus sit. End third unordered list element.
// (Test) Ensure this comment does not ruin docstring association.
class Class {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(Class)

  /// @name Name. Augue neque gravida in fermentum et.
  /// @{

  /// Using alias. Sit amet nisl purus in mollis nunc sed id
  /// semper.
  // (Test) Ensure this comment does not ruin docstring association.
  using UsingAlias = std::vector<Class>;

  /// @typedef std::vector<Class> TypedefAlias
  /// Typedef alias. Risus nec feugiat in fermentum posuere urna nec
  /// tincidunt praesent.
  // (Test) Ensure this comment does not ruin docstring association.
  typedef std::vector<Class> TypedefAlias;

  /// @}

  /// @addtogroup second_group Luctus venenatis lectus magna fringilla urna.
  /// @{

  /// Custom constructor 1.
  // (Test) Ensure this comment does not ruin docstring association.
  Class() {}

  /// Custom constructor 2. Ut tristique et egestas quis ipsum
  /// suspendisse ultrices gravida. Suscipit tellus mauris a
  /// diam. Maecenas accumsan lacus vel facilisis volutpat est.
  ///
  /// Ut consequat semper viverra nam libero.
  // (Test) Ensure this comment does not ruin docstring association.
  explicit Class(int param) {
    unused(param);
  }

  /// @}

  /// Custom constructor 3. Integer quis auctor elit sed vulputate
  /// mi sit.
  // (Test) Ensure this comment does not ruin docstring association.
  explicit Class(int param1, std::vector<int> param2) {
    unused(param1);
    unused(param2);
  }

  void PublicMethod() {}

  // Private documentation.
  template <typename T>
  void PublicTemplateMethod() {}

  /// Sed faucibus turpis in eu mi bibendum neque egestas.
  /// @pre Begin precondition. Cras fermentum odio eu feugiat pretium nibh.
  /// Ornare suspendisse sed nisi lacus sed viverra tellus. End precondition.
  /// @post Begin postcondition. Tortor id aliquet lectus proin nibh nisl
  /// condimentum id. End postcondition.
  // (Test) Ensure this comment does not ruin docstring association.
  static void PublicStatic() {}

  /// This one takes an int.
  // (Test) Ensure this comment does not ruin docstring association.
  void overloaded_method(int alpha);

  /// This one takes a double.
  // (Test) Ensure this comment does not ruin docstring association.
  void overloaded_method(double bravo);

  /// This one takes an int and a double.
  // (Test) Ensure this comment does not ruin docstring association.
  void overloaded_method(int charlie, double delta);

  /// This one takes the road less traveled.
  // (Test) Ensure this comment does not ruin docstring association.
  void overloaded_method(double, int);

  /// This one takes a non-primitive type.
  // (Test) Ensure this comment does not ruin docstring association.
  void overloaded_method(const std::string&);

  /// Different overload with same doc.
  // (Test) Ensure this comment does not ruin docstring association.
  void overloaded_with_same_doc();

  /// Different overload with same doc.
  // (Test) Ensure this comment does not ruin docstring association.
  void overloaded_with_same_doc(int);

  /// Overloaded only by its const-ness.
  // (Test) Ensure this comment does not ruin docstring association.
  void get_foo();

  /// The const one.
  // (Test) Ensure this comment does not ruin docstring association.
  void get_foo() const;

  /// Docstring 1.
  /// @pydrake_mkdoc_identifier{stuff_1}
  // (Test) Ensure this comment does not ruin docstring association.
  void do_stuff(double);

  /// Docstring 2.
  /// @pydrake_mkdoc_identifier{stuff_2}
  // (Test) Ensure this comment does not ruin docstring association.
  template <typename T>
  void do_stuff(T);

 protected:
  /// Protected method. Nibh sed pulvinar proin gravida hendrerit.
  /// Orci phasellus egestas tellus rutrum tellus pellentesque eu.
  // (Test) Ensure this comment does not ruin docstring association.
  int ProtectedMethod() {
    return 0;
  }

  /// Protected nested class. Sed turpis tincidunt id aliquet.
  /// Egestas sed sed risus pretium.
  // (Test) Ensure this comment does not ruin docstring association.
  class Nested {};

  int protected_member_{};

 private:
  /// Private method, public documentation. Senectus et netus et malesuada
  /// fames ac.
  // (Test) Ensure this comment does not ruin docstring association.
  void PrivateMethod() {}

  // Private member, private documentation.
  int private_member_{};
};

/// Struct. Sed elementum tempus egestas sed sed risus pretium. Vel pharetra
/// vel turpis nunc.
/// @deprecated Begin deprecated. Est pellentesque elit ullamcorper dignissim
/// cras tincidunt lobortis. End deprecated.
// (Test) Ensure this comment does not ruin docstring association.
struct Struct {
  /// See @ref implementing_serialize "Implementing Serialize".
  // (Test) Ensure this comment does not ruin docstring association.
  template <typename Archive>
  void Serialize(Archive* a);
  /// Field 1. Sit amet cursus sit amet dictum sit amet. Id leo in vitae turpis
  /// massa sed elementum tempus.
  // (Test) Ensure this comment does not ruin docstring association.
  int field_1;
  /// Field 2. Consectetur libero id faucibus nisl tincidunt eget nullam non
  /// nisi.
  // (Test) Ensure this comment does not ruin docstring association.
  std::vector<int> field_2;
};

/// Template class. Mauris pharetra et ultrices neque ornare aenean euismod
/// elementum.
// (Test) Ensure this comment does not ruin docstring association.
template <typename T>
class TemplateClass {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(TemplateClass)

  /// Default constructor. Condimentum mattis pellentesque id nibh tortor id.
  /// Nisl rhoncus mattis rhoncus urna neque.
  // (Test) Ensure this comment does not ruin docstring association.
  TemplateClass();

  /// Single argument int constructor.
  // (Test) Ensure this comment does not ruin docstring association.
  explicit TemplateClass(int i);

  /// Scalar-converting copy constructor.
  // (Test) Ensure this comment does not ruin docstring association.
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

/// Specialize. Nisl pretium fusce id velit ut tortor pretium viverra. Quis
/// ipsum suspendisse ultrices gravida dictum fusce ut.
// (Test) Ensure this comment does not ruin docstring association.
template <>
class TemplateClass<int> {
};

/// Enumeration. Feugiat scelerisque varius morbi enim.  Facilisis leo vel
/// fringilla est ullamcorper eget nulla facilisi.
// (Test) Ensure this comment does not ruin docstring association.
enum Enum {
  /// Enumeration constant.
  EnumConstant,
};

/// Enumeration class. Malesuada fames ac turpis egestas integer eget aliquet
/// nibh praesent.
// (Test) Ensure this comment does not ruin docstring association.
enum EnumClass {
  EnumClassConstant,  ///< Enumeration class constant. Vestibulum mattis.
};

/// Anonymous values are ignored.
// (Test) Ensure this comment does not ruin docstring association.
enum {
  /// Anonymous enum's constant.
  // (Test) Ensure this comment does not ruin docstring association.
  AnonymousConstant,
};

// Warning: Injecting a comment // between DRAKE_DEPRECATED and the given
// symbol does break the docstring association.

/// I am measurably old.
// (Test) Ensure this comment does not ruin docstring association.
class DRAKE_DEPRECATED("2038-01-19", "Use MyNewClass instead.")
DrakeDeprecatedClass {
 public:
  DRAKE_DEPRECATED("2038-01-19",
    "f() is slow; use g() instead. "
    "Also, I like hats.")
  int f(int arg);

  DRAKE_DEPRECATED("2038-01-19",
    "f() now requires an int.")
  int f();

  /// Ideally this overview would still appear, but it does not yet.
  DRAKE_DEPRECATED("2038-01-19", "a() is slow; use b() instead.")
  int a();
};

/// I am symbolically old.
// (Test) Ensure this comment does not ruin docstring association.
template <typename T>
class DRAKE_DEPRECATED("2038-01-19", "Templates rule!")
DrakeDeprecatedTemplateClass {};

}  // namespace mkdoc_test
}  // namespace drake

#define DRAKE_NO_COPY_NO_MOVE(DummyClassname)      \
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
// (Test) Ensure this comment does not ruin docstring association.

struct Struct1 {
  /// Et, ornare sagittis, tellus. Fusce felis.
  // (Test) Ensure this comment does not ruin docstring association.
  int var_1{};
  /// Nulla a augue. Pellentesque sed est.
  // (Test) Ensure this comment does not ruin docstring association.
  int var_2{};
  /// Imperdiet tristique, interdum a, dolor.
  // (Test) Ensure this comment does not ruin docstring association.
  double var_3{};
  /// Tempor lobortis turpis. Sed tellus velit, ullamcorper.
  // (Test) Ensure this comment does not ruin docstring association.
  double var_4{};
  /// Id, rutrum auctor, ullamcorper sed, orci. In.
  // (Test) Ensure this comment does not ruin docstring association.
  double var_5{};
  /// Fames ac turpis egestas. Sed vitae eros. Nulla.
  // (Test) Ensure this comment does not ruin docstring association.
  double var_6{};
  /// Condimentum. Donec arcu quam, dictum accumsan, convallis.
  // (Test) Ensure this comment does not ruin docstring association.
  double var_7{};
  /// Volutpat. Donec non tortor. Vivamus posuere nisi mollis.
  // (Test) Ensure this comment does not ruin docstring association.
  double var_8{};
};

struct Struct2 {};
struct Struct3 {};
struct Struct4 {};
struct Struct5 {};
struct Struct6{};

class DummyClass {
 public:
  DRAKE_NO_COPY_NO_MOVE(DummyClass)

  /// Ligula. Nunc turpis. Mauris vitae sapien. Nunc.
  // (Test) Ensure this comment does not ruin docstring association.
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

// N.B. Breaking comment styles will cause parsing to break (#14498).
/// My simple system.
///
/// @system
/// name: Wooh
/// input_ports:
/// - u
/// output_ports:
/// - y
/// @endsystem
// (Test) Ensure this comment does not ruin docstring association.
class MySimpleSystem {};

}  // namespace namespace_1
