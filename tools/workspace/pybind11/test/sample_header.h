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
#include "drake/common/unused.h"

/// @def PREPROCESSOR_DEFINITION
/// Preprocessor definitions are ignored. In nibh mauris cursus mattis
/// molestie a. Non arcu risus quis varius quam quisque id.
#define PREPROCESSOR_DEFINITION "Nisl purus in mollis nunc sed id."

/// Root-level symbol. Magna fermentum iaculis eu non diam phasellus
/// vestibulum.
struct RootLevelSymbol {};

/// @namespace drake
/// Namespaces are ignored. Enim blandit volutpat maecenas volutpat blandit. Eu
/// feugiat pretium nibh ipsum consequat.
namespace drake {

/**
 * @struct MidLevelSymbol
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
 * @anchor anchor
 * Senectus et netus et malesuada fames ac. Tincidunt lobortis feugiat vivamus
 * at augue eget arcu dictum varius.
 * @internal
 * Begin ignored internal section. Est ante in nibh mauris cursus mattis
 * molestie. Morbi tristique senectus et netus et malesuada. Magnis dis
 * parturient montes nascetur ridiculus mus mauris. End ignored internal
 * section.
 * @endinternal
 *
 * @section first_level_heading First level heading
 * Cursus in hac habitasse platea dictumst quisque sagittis purus sit. Et
 * malesuada fames ac turpis.
 * @subsection second_level_heading Second level heading
 * Adipiscing diam donec adipiscing tristique risus nec feugiat. Condimentum
 * vitae sapien pellentesque habitant.
 * @subsubsection third_level_heading Third level heading
 * Fermentum odio eu feugiat pretium nibh. Sed nisi lacus sed viverra.  Ut
 * ornare lectus sit amet est.
 * @version 1.0.
 */
struct MidLevelSymbol {};

namespace mkdoc_test {

// A forward-declaration is ignored.
class Class;

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
void func(T tee);

/// @class Class
/// Class. Lorem ipsum dolor sit amet, consectetur adipiscing elit, sed do
/// eiusmod tempor incididunt ut labore et dolore magna aliqua.
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
///
/// # First level heading #
/// Aliquet nec ullamcorper sit amet risus nullam eget felis. __Bold__. Hac
/// habitasse platea dictumst quisque sagittis purus sit.
///
/// ## Second level heading ##
/// Donec massa sapien faucibus et molestie ac feugiat sed lectus. _Italics_.
/// Amet justo donec enim diam vulputate ut pharetra sit.
///
/// ### Third level heading ###
/// Orci eu lobortis elementum nibh. `Typewriter`. Luctus venenatis lectus
/// magna fringilla urna porttitor rhoncus dolor.
///
/// #### Fourth level heading ####
/// Orci eu lobortis elementum nibh. Luctus venenatis lectus magna fringilla
///  urna porttitor rhoncus dolor.
///
/// Tortor id aliquet lectus proin nibh. [Link](https://example.org). Cras
/// semper auctor neque vitae tempus quam pellentesque nec.
/// @sa Struct
class Class {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(Class)

  /// @name Name. Augue neque gravida in fermentum et.
  /// @{

  /// @public
  /// Using alias. @a Italics. Sit amet nisl purus in mollis nunc sed id
  /// semper.
  using UsingAlias = std::vector<Class>;

  /// @typedef std::vector<Class> TypedefAlias
  /// Typedef alias. @b Bold. Risus nec feugiat in fermentum posuere urna nec
  /// tincidunt praesent.
  typedef std::vector<Class> TypedefAlias;

  /// @}

  /// @addtogroup second_group Luctus venenatis lectus magna fringilla urna.
  /// @{

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

  /// @}

  /// Custom constructor 3. @em Italics. Integer quis auctor elit sed vulputate
  /// mi sit.
  /// @param[in] param1 Begin first input parameter. Mollis nunc sed id semper
  /// risus in hendrerit gravida rutrum. End first input parameter.
  /// @param[in] param2 Begin second input parameter. Tristique senectus et
  /// netus et malesuada fames ac turpis. End second input parameter.
  /// @ingroup first_group second_group
  explicit Class(int param1, std::vector<int> param2) {
    unused(param1);
    unused(param2);
  }

  /// @fn void PublicMethod()
  /// Public method. @f$ A = \pi r^2 @f$. Condimentum id venenatis a
  /// condimentum vitae sapien pellentesque habitant morbi.
  /// @cond TEST
  /// Begin ignored conditional section. Tortor vitae purus faucibus ornare
  /// suspendisse. Orci dapibus ultrices in iaculis. End ignored conditional
  /// section.
  /// @endcond
  /// @callgraph
  /// @callergraph
  void PublicMethod() {}

  // Private documentation.
  /// Public template method. @f[ a^2 + b^2 = c^2. @f] Sagittis id consectetur
  /// purus ut faucibus pulvinar.
  /// @tparam T Begin template parameter. acilisi etiam dignissim diam quis. Ut
  /// pharetra sit amet aliquam. End template parameter.
  /// @hidecallgraph
  /// @hidecallergraph
  template <typename T>
  void PublicTemplateMethod() {}

  /// @static
  /// @brief Static method. @p Typewriter. Sed faucibus turpis in eu mi
  /// bibendum neque egestas.
  /// @details Vitae sapien pellentesque habitant morbi tristique senectus.
  /// Iaculis eu non diam phasellus vestibulum.
  /// @invariant Begin invariant. Odio euismod lacinia at quis risus sed
  /// vulputate odio. End invariant.
  /// @pre Begin precondition. Cras fermentum odio eu feugiat pretium nibh.
  /// Ornare suspendisse sed nisi lacus sed viverra tellus. End precondition.
  /// @post Begin postcondition. Tortor id aliquet lectus proin nibh nisl
  /// condimentum id. End postcondition.
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
  void overloaded_method(const std::string&);

  /// Overloaded only by its const-ness.
  void get_foo();

  /// The const one.
  void get_foo() const;

 protected:
  /// @protected
  /// Protected method. **Bold**. Nibh sed pulvinar proin gravida hendrerit.
  /// Orci phasellus egestas tellus rutrum tellus pellentesque eu.
  /// @returns Begin returns. Faucibus interdum posuere lorem ipsum dolor.
  /// Malesuada proin libero nunc consequat interdum varius sit amet. End
  /// returns.
  int ProtectedMethod() {
    return 0;
  }

  /// Protected nested class. *Italics*. Sed turpis tincidunt id aliquet.
  /// Egestas sed sed risus pretium.
  /// @bug Begin bug report. Cras pulvinar mattis nunc sed blandit libero. Eget
  /// est lorem ipsum dolor sit amet consectetur. End bug report.
  /// @test Begin test case. Sem integer vitae justo eget magna fermentum.
  /// Convallis posuere morbi leo urna molestie at elementum eu facilisis. End
  /// test case.
  class Nested {};

  /// @var int protected_member_
  /// Protected member. ``Typewriter``. Porttitor eget dolor morbi non arcu
  /// risus quis varius quam.
  /// @hideinitializer
  int protected_member_{};

 private:
  /// @private
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
  /// aliquet risus feugiat. End attention.
  /// @showinitializer
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
/// @relatesalso Struct
template <typename T>
class TemplateClass {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(TemplateClass)

  /// Default constructor. Condimentum mattis pellentesque id nibh tortor id.
  /// Nisl rhoncus mattis rhoncus urna neque.
  /// @remarks Begin remarks. Ante metus dictum at tempor commodo. Nec feugiat
  /// in fermentum posuere urna nec. End remarks.
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

/// Specialize. Nisl pretium fusce id velit ut tortor pretium viverra. Quis
/// ipsum suspendisse ultrices gravida dictum fusce ut.
/// @since Begin since. Nullam eget felis eget nunc lobortis mattis aliquam
/// faucibus. End since.
template <>
class TemplateClass<int> {
};

/// @enum Enum
/// Enumeration. Feugiat scelerisque varius morbi enim.  Facilisis leo vel
/// fringilla est ullamcorper eget nulla facilisi.
/// @relates Class
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
