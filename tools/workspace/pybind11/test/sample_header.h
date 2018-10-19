#pragma once

/// @file
/// Sit amet nisl purus in mollis nunc sed id semper. Risus nec feugiat in
/// fermentum posuere urna nec tincidunt praesent. Suscipit tellus mauris a
/// diam.

#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/common/unused.h"

/// Root-level symbol. Magna fermentum iaculis eu non diam phasellus
/// vestibulum.
struct RootLevelSymbol {};

namespace drake {

/**
 * Mid-level symbol. Ut enim ad minim veniam, quis nostrud exercitation
 * ullamco laboris nisi ut aliquip ex ea commodo consequat.
 *
 * 1. Begin first ordered list element. Rutrum quisque non tellus orci ac
 *    auctor. End first ordered list element.
 * 2. Begin second ordered list element. Ipsum faucibus vitae aliquet nec.
 *    Ligula ullamcorper malesuada proin libero. End second ordered list
 *    element.
 * 3. Begin third ordered list element. Dictum sit amet justo donec enim.
 *    Pharetra convallis posuere morbi leo urna molestie. End third ordered
 *    list element.
 *
 * @version 1.0.
 */
struct MidLevelSymbol {};

namespace mkdoc_test {

/** Function. Mi sit amet mauris commodo quis. */
void func();
/// Function, overload 1. Velit ut tortor pretium viverra suspendisse potenti
/// nullam ac tortor.
/// @throws std::exception Begin raises. Morbi tincidunt augue interdum velit
/// euismod. Justo nec ultrices dui sapien eget mi proin sed libero. End
/// raises.
void func(int* param);
/// Function, template overload. Pellentesque diam volutpat commodo sed egestas
/// egestas fringilla phasellus faucibus.
/// @param[in,out] param Begin input/output parameter. Morbi enim nunc faucibus
/// a pellentesque sit. End input/output parameter.
template <typename T>
void func(T);

/// Class.
/// Lorem ipsum dolor sit amet, consectetur adipiscing elit, sed do eiusmod
/// tempor incididunt ut labore et dolore magna aliqua.
///
/// * Begin first unordered list element. Volutpat blandit aliquam etiam erat
///   velit scelerisque. End first unordered list element.
/// * Begin second unordered list element. Eget est lorem ipsum dolor sit amet.
///   Ipsum dolor sit amet consectetur adipiscing. End second unordered list
///   element.
/// * Begin third unordered list element. Hac habitasse platea dictumst quisque
///   sagittis purus sit. End third unordered list element.
///
/// Quisque sagittis purus sit amet volutpat.
/// @sa Struct
class Class {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(Class)

  // TODO(eric.cousineau): Make `mkdoc` recognize documentation for aliases. At
  // present, neither `UsingAlias` nor `TypedefAlias` show up.

  /// Using alias. @a Italics. Sit amet nisl purus in mollis nunc sed id
  /// semper.
  using UsingAlias = std::vector<Class>;

  /// Typedef alias. @b Bold. Risus nec feugiat in fermentum posuere urna nec
  /// tincidunt praesent.
  typedef std::vector<Class> TypedefAlias;

  /// Custom constructor 1.
  Class() {}

  /// Custom constructor 2. @e Italics. Ut tristique et egestas quis ipsum
  /// suspendisse ultrices gravida. @c Typewriter. Suscipit tellus mauris a
  /// diam. Maecenas accumsan lacus vel facilisis volutpat est.
  /// @param param Begin parameter. Dignissim diam quis enim lobortis
  /// scelerisque fermentum dui faucibus. End parameter.
  ///
  /// Ut consequat semper viverra nam libero.
  /// @code{.cpp}
  /// Class class();
  /// class.PublicMethod();
  /// @endcode
  explicit Class(int param) {
    unused(param);
  }

  /// Custom constructor 3. @em Italics. Integer quis auctor elit sed vulputate
  /// mi sit.
  /// @param[in] param1 Begin first input parameter. Mollis nunc sed id semper
  /// risus in hendrerit gravida rutrum. End first input parameter.
  /// @param[in] param2 Begin second input parameter. Tristique senectus et
  /// netus et malesuada fames ac turpis. End second input parameter.
  explicit Class(int param1, std::vector<int> param2) {
    unused(param1);
    unused(param2);
  }

  /// Public method. @f$ A = \pi r^2 @f$. Condimentum id venenatis a
  /// condimentum vitae sapien pellentesque habitant morbi.
  /// @cond
  /// Private documentation. Tortor vitae purus faucibus ornare suspendisse.
  /// Orci dapibus ultrices in iaculis.
  /// @endcond
  void PublicMethod() {}

  // Private documentation.
  /// Public template method. @f[ a^2 + b^2 = c^2. @f] Sagittis id consectetur
  /// purus ut faucibus pulvinar.
  /// @tparam T Begin template parameter. acilisi etiam dignissim diam quis. Ut
  /// pharetra sit amet aliquam. End template parameter.
  template <typename T>
  void PublicTemplateMethod() {}

  /// Static method. @p Typewriter. Sed faucibus turpis in eu mi bibendum neque
  /// egestas.
  /// @invariant Begin invariant. Odio euismod lacinia at quis risus sed
  /// vulputate odio. End invariant.
  /// @pre Begin precondition. Cras fermentum odio eu feugiat pretium nibh.
  /// Ornare suspendisse sed nisi lacus sed viverra tellus. End precondition.
  /// @post Begin postcondition. Tortor id aliquet lectus proin nibh nisl
  /// condimentum id. End postcondition.
  static void PublicStatic() {}

 protected:
  /// Protected method. Nibh sed pulvinar proin gravida hendrerit. Orci
  /// phasellus egestas tellus rutrum tellus pellentesque eu.
  /// @returns Begin returns. Faucibus interdum posuere lorem ipsum dolor.
  /// Malesuada proin libero nunc consequat interdum varius sit amet. End
  /// returns.
  int ProtectedMethod() {
    return 0;
  }

  /// Protected nested class. Sed turpis tincidunt id aliquet. Egestas sed sed
  /// risus pretium.
  /// @bug Begin bug report. Cras pulvinar mattis nunc sed blandit libero. Eget
  /// est lorem ipsum dolor sit amet consectetur. End bug report.
  /// @test Begin test case. Sem integer vitae justo eget magna fermentum.
  /// Convallis posuere morbi leo urna molestie at elementum eu facilisis. End
  /// test case.
  class Nested {};

  /// Protected member. Porttitor eget dolor morbi non arcu risus quis varius
  /// quam.
  int protected_member_{};

 private:
  /// Private method, public documentation. Senectus et netus et malesuada
  /// fames ac.
  /// @todo Begin TODO. Ut tellus elementum sagittis vitae et leo duis ut diam.
  /// Et malesuada fames ac turpis. End TODO.
  void PrivateMethod() {}

  // Private member, private documentation.
  int private_member_{};
};

/// Struct. Sed elementum tempus egestas sed sed risus pretium. Vel pharetra
/// vel turpis nunc.
/// @deprecated Begin deprecated. Est pellentesque elit ullamcorper dignissim
/// cras tincidunt lobortis. End deprecated.
/// @see Class
struct Struct {
  /// Field 1. Sit amet cursus sit amet dictum sit amet. Id leo in vitae turpis
  /// massa sed elementum tempus.
  /// @attention Begin attention. Ultricies lacus sed turpis tincidunt id
  /// aliquet risus feugiat.  End attention.
  int field_1;
  /// Field 2. Consectetur libero id faucibus nisl tincidunt eget nullam non
  /// nisi.
  /// @note Begin note. Consectetur a erat nam at lectus. Consequat ac felis
  /// donec et odio pellentesque diam. End note.
  std::vector<int> field_2;
};

/// Template class. Mauris pharetra et ultrices neque ornare aenean euismod
/// elementum.
/// @tparam T Begin template parameter. Feugiat scelerisque varius morbi enim
/// nunc faucibus a. End template parameter.
template <typename T>
class TemplateClass {
 public:
  /// Default constructor. Condimentum mattis pellentesque id nibh tortor id.
  /// Nisl rhoncus mattis rhoncus urna neque.
  /// @remarks Begin remarks. Ante metus dictum at tempor commodo. Nec feugiat
  /// in fermentum posuere urna nec. End remarks.
  TemplateClass() {}
};

/// Specialize. Nisl pretium fusce id velit ut tortor pretium viverra. Quis
/// ipsum suspendisse ultrices gravida dictum fusce ut.
/// @since Begin since. Nullam eget felis eget nunc lobortis mattis aliquam
/// faucibus. End since.
template <>
class TemplateClass<int> {
};

/// Enumeration. Feugiat scelerisque varius morbi enim.  Facilisis leo vel
/// fringilla est ullamcorper eget nulla facilisi.
enum Enum {
  /// Enumeration constant.
  EnumConstant,
};

/// Enumeration class. Malesuada fames ac turpis egestas integer eget aliquet
/// nibh praesent.
enum EnumClass {
  /// Enumeration class constant. Vestibulum mattis ullamcorper velit sed
  /// ullamcorper.
  EnumClassConstant,
};

/// Anonymous values are ignored.
enum {
  /// Anonymous enum's constant.
  AnonymousConstant,
};

}  // namespace mkdoc_test
}  // namespace drake
