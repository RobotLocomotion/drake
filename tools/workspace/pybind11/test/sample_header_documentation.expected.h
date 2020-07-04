#pragma once

// <GENERIC MARKER SCRUBBED FOR REVIEWABLE>
// This file contains docstrings for the Python bindings that were
// automatically extracted by mkdoc.py.

#if defined(__GNUG__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-variable"
#endif

// #include "drake/tools/workspace/pybind11/test/sample_header.h"

// Symbol: sample_header_doc
constexpr struct /* sample_header_doc */ {
  // Symbol: RootLevelSymbol
  struct /* RootLevelSymbol */ {
    // Source: drake/tools/workspace/pybind11/test/sample_header.h:31
    const char* doc =
R"""(Root-level symbol. Magna fermentum iaculis eu non diam phasellus
vestibulum.)""";
  } RootLevelSymbol;
  // Symbol: drake
  struct /* drake */ {
    // Symbol: drake::MidLevelSymbol
    struct /* MidLevelSymbol */ {
      // Source: drake/tools/workspace/pybind11/test/sample_header.h:73
      const char* doc =
R"""(Mid-level symbol. Ut enim ad minim veniam, quis nostrud exercitation
ullamco laboris nisi ut aliquip ex ea commodo consequat.

1. Begin first ordered list element. Rutrum quisque non tellus orci ac
   auctor. End first ordered list element.
2. Begin second ordered list element. Ipsum faucibus vitae aliquet nec.
   Ligula ullamcorper malesuada proin libero. End second ordered list
   element.
3. Begin third ordered list element. Dictum sit amet justo donec enim.
   Pharetra convallis posuere morbi leo urna molestie. End third ordered
   list element.

Senectus et netus et malesuada fames ac. Tincidunt lobortis feugiat
vivamus at augue eget arcu dictum varius.

First level heading
===================

Cursus in hac habitasse platea dictumst quisque sagittis purus sit. Et
malesuada fames ac turpis.

Second level heading
--------------------

Adipiscing diam donec adipiscing tristique risus nec feugiat.
Condimentum vitae sapien pellentesque habitant.

**Third level heading**

Fermentum odio eu feugiat pretium nibh. Sed nisi lacus sed viverra. Ut
ornare lectus sit amet est.

Version:
    1.0.)""";
    } MidLevelSymbol;
    // Symbol: drake::mkdoc_test
    struct /* mkdoc_test */ {
      // Symbol: drake::mkdoc_test::AnonymousConstant
      struct /* AnonymousConstant */ {
        // Source: drake/tools/workspace/pybind11/test/sample_header.h:382
        const char* doc = R"""(Anonymous enum's constant.)""";
      } AnonymousConstant;
      // Symbol: drake::mkdoc_test::Class
      struct /* Class */ {
        // Source: drake/tools/workspace/pybind11/test/sample_header.h:136
        const char* doc =
R"""(Class. Lorem ipsum dolor sit amet, consectetur adipiscing elit, sed do
eiusmod tempor incididunt ut labore et dolore magna aliqua.

* Begin first unordered list element. Volutpat blandit aliquam etiam erat
  velit scelerisque. End first unordered list element.
* Begin second unordered list element. Eget est lorem ipsum dolor sit amet.
  Ipsum dolor sit amet consectetur adipiscing. End second unordered list
  element.
* Begin third unordered list element. Hac habitasse platea dictumst quisque
  sagittis purus sit. End third unordered list element.

Quisque sagittis purus sit amet volutpat.

.. raw:: html

   <table align=center cellpadding=0 cellspacing=0><tr align=center><td style="vertical-align:middle"><table cellspacing=0 cellpadding=0><tr><td align=right style="padding:5px 0px 5px 0px">lead&rarr;</td></tr></table></td><td align=center style="border:solid;padding-left:20px;padding-right:20px;vertical-align:middle" bgcolor=#F0F0F0>Alchemist</td><td style="vertical-align:middle"><table cellspacing=0 cellpadding=0><tr><td align=left style="padding:5px 0px 5px 0px">&rarr; gold</td></tr></table></td></tr></table>

First level heading
===================

Aliquet nec ullamcorper sit amet risus nullam eget felis. **Bold**.
Hac habitasse platea dictumst quisque sagittis purus sit.

Second level heading
--------------------

Donec massa sapien faucibus et molestie ac feugiat sed lectus.
*Italics*. Amet justo donec enim diam vulputate ut pharetra sit.

**Third level heading**

Orci eu lobortis elementum nibh. ``Typewriter``. Luctus venenatis
lectus magna fringilla urna porttitor rhoncus dolor.

*Fourth level heading*

Orci eu lobortis elementum nibh. Luctus venenatis lectus magna
fringilla urna porttitor rhoncus dolor.

Tortor id aliquet lectus proin nibh. `Link <https://example.org>`_.
Cras semper auctor neque vitae tempus quam pellentesque nec.

See also:
    Struct)""";
        // Symbol: drake::mkdoc_test::Class::Class
        struct /* ctor */ {
          // Source: drake/tools/workspace/pybind11/test/sample_header.h:159
          const char* doc_0args = R"""(Custom constructor 1.)""";
          // Source: drake/tools/workspace/pybind11/test/sample_header.h:172
          const char* doc_1args =
R"""(Custom constructor 2. *Italics*. Ut tristique et egestas quis ipsum
suspendisse ultrices gravida. ``Typewriter``. Suscipit tellus mauris a
diam. Maecenas accumsan lacus vel facilisis volutpat est.

Parameter ``param``:
    Begin parameter. Dignissim diam quis enim lobortis scelerisque
    fermentum dui faucibus. End parameter.

Ut consequat semper viverra nam libero.


::

    Class class();
    class.PublicMethod();)""";
          // Source: drake/tools/workspace/pybind11/test/sample_header.h:185
          const char* doc_2args =
R"""(Custom constructor 3. *Italics*. Integer quis auctor elit sed
vulputate mi sit.

Parameter ``param1``:
    Begin first input parameter. Mollis nunc sed id semper risus in
    hendrerit gravida rutrum. End first input parameter.

Parameter ``param2``:
    Begin second input parameter. Tristique senectus et netus et
    malesuada fames ac turpis. End second input parameter.)""";
        } ctor;
        // Symbol: drake::mkdoc_test::Class::Nested
        struct /* Nested */ {
          // Source: drake/tools/workspace/pybind11/test/sample_header.h:279
          const char* doc =
R"""(Protected nested class. *Italics*. Sed turpis tincidunt id aliquet.
Egestas sed sed risus pretium.

Bug report:
    Begin bug report. Cras pulvinar mattis nunc sed blandit libero.
    Eget est lorem ipsum dolor sit amet consectetur. End bug report.

Test case:
    Begin test case. Sem integer vitae justo eget magna fermentum.
    Convallis posuere morbi leo urna molestie at elementum eu
    facilisis. End test case.)""";
        } Nested;
        // Symbol: drake::mkdoc_test::Class::ProtectedMethod
        struct /* ProtectedMethod */ {
          // Source: drake/tools/workspace/pybind11/test/sample_header.h:268
          const char* doc =
R"""(Protected method. **Bold**. Nibh sed pulvinar proin gravida hendrerit.
Orci phasellus egestas tellus rutrum tellus pellentesque eu.

Returns:
    Begin returns. Faucibus interdum posuere lorem ipsum dolor.
    Malesuada proin libero nunc consequat interdum varius sit amet.
    End returns.)""";
        } ProtectedMethod;
        // Symbol: drake::mkdoc_test::Class::PublicMethod
        struct /* PublicMethod */ {
          // Source: drake/tools/workspace/pybind11/test/sample_header.h:200
          const char* doc =
R"""(Public method. :math:`A = \pi r^2`. Condimentum id venenatis a
condimentum vitae sapien pellentesque habitant morbi.)""";
        } PublicMethod;
        // Symbol: drake::mkdoc_test::Class::PublicStatic
        struct /* PublicStatic */ {
          // Source: drake/tools/workspace/pybind11/test/sample_header.h:223
          const char* doc =
R"""(Static method. ``Typewriter``. Sed faucibus turpis in eu mi bibendum
neque egestas.

Vitae sapien pellentesque habitant morbi tristique senectus. Iaculis
eu non diam phasellus vestibulum.

Invariant:
    Begin invariant. Odio euismod lacinia at quis risus sed vulputate
    odio. End invariant.

Precondition:
    Begin precondition. Cras fermentum odio eu feugiat pretium nibh.
    Ornare suspendisse sed nisi lacus sed viverra tellus. End
    precondition.

Postcondition:
    Begin postcondition. Tortor id aliquet lectus proin nibh nisl
    condimentum id. End postcondition.)""";
        } PublicStatic;
        // Symbol: drake::mkdoc_test::Class::PublicTemplateMethod
        struct /* PublicTemplateMethod */ {
          // Source: drake/tools/workspace/pybind11/test/sample_header.h:210
          const char* doc =
R"""(Public template method.

.. math:: a^2 + b^2 = c^2.

Sagittis id consectetur purus ut faucibus pulvinar.

Template parameter ``T``:
    Begin template parameter. acilisi etiam dignissim diam quis. Ut
    pharetra sit amet aliquam. End template parameter.)""";
        } PublicTemplateMethod;
        // Symbol: drake::mkdoc_test::Class::TypedefAlias
        struct /* TypedefAlias */ {
          // Source: drake/tools/workspace/pybind11/test/sample_header.h:151
          const char* doc =
R"""(Typedef alias. **Bold**. Risus nec feugiat in fermentum posuere urna
nec tincidunt praesent.)""";
        } TypedefAlias;
        // Symbol: drake::mkdoc_test::Class::UsingAlias
        struct /* UsingAlias */ {
          // Source: drake/tools/workspace/pybind11/test/sample_header.h:146
          const char* doc =
R"""(Using alias. *Italics*. Sit amet nisl purus in mollis nunc sed id
semper.)""";
        } UsingAlias;
        // Symbol: drake::mkdoc_test::Class::do_stuff
        struct /* do_stuff */ {
          // Source: drake/tools/workspace/pybind11/test/sample_header.h:254
          const char* doc_stuff_1 = R"""(Docstring 1.)""";
          // Source: drake/tools/workspace/pybind11/test/sample_header.h:259
          const char* doc_stuff_2 = R"""(Docstring 2.)""";
        } do_stuff;
        // Symbol: drake::mkdoc_test::Class::get_foo
        struct /* get_foo */ {
          // Source: drake/tools/workspace/pybind11/test/sample_header.h:247
          const char* doc_0args_nonconst = R"""(Overloaded only by its const-ness.)""";
          // Source: drake/tools/workspace/pybind11/test/sample_header.h:250
          const char* doc_0args_const = R"""(The const one.)""";
        } get_foo;
        // Symbol: drake::mkdoc_test::Class::overloaded_method
        struct /* overloaded_method */ {
          // Source: drake/tools/workspace/pybind11/test/sample_header.h:226
          const char* doc_1args_alpha = R"""(This one takes an int.)""";
          // Source: drake/tools/workspace/pybind11/test/sample_header.h:229
          const char* doc_1args_bravo = R"""(This one takes a double.)""";
          // Source: drake/tools/workspace/pybind11/test/sample_header.h:232
          const char* doc_2args_charlie_delta = R"""(This one takes an int and a double.)""";
          // Source: drake/tools/workspace/pybind11/test/sample_header.h:235
          const char* doc_2args_double_int = R"""(This one takes the road less traveled.)""";
          // Source: drake/tools/workspace/pybind11/test/sample_header.h:238
          const char* doc_1args_conststdstring = R"""(This one takes a non-primitive type.)""";
        } overloaded_method;
        // Symbol: drake::mkdoc_test::Class::overloaded_with_same_doc
        struct /* overloaded_with_same_doc */ {
          // Source: drake/tools/workspace/pybind11/test/sample_header.h:241
          const char* doc = R"""(Different overload with same doc.)""";
        } overloaded_with_same_doc;
        // Symbol: drake::mkdoc_test::Class::protected_member_
        struct /* protected_member_ */ {
          // Source: drake/tools/workspace/pybind11/test/sample_header.h:285
          const char* doc =
R"""(Protected member. ``Typewriter``. Porttitor eget dolor morbi non arcu
risus quis varius quam.)""";
        } protected_member_;
      } Class;
      // Symbol: drake::mkdoc_test::DrakeDeprecatedClass
      struct /* DrakeDeprecatedClass */ {
        // Source: drake/tools/workspace/pybind11/test/sample_header.h:387
        const char* doc_deprecated =
R"""(I am measurably old. (Deprecated.)

Deprecated:
    Use MyNewClass instead. This will be removed from Drake on or
    after 2038-01-19.)""";
        // Symbol: drake::mkdoc_test::DrakeDeprecatedClass::a
        struct /* a */ {
          // Source: drake/tools/workspace/pybind11/test/sample_header.h:400
          const char* doc_deprecated =
R"""((Deprecated.)

Deprecated:
    a() is slow; use b() instead. This will be removed from Drake on
    or after 2038-01-19.)""";
        } a;
        // Symbol: drake::mkdoc_test::DrakeDeprecatedClass::f
        struct /* f */ {
          // Source: drake/tools/workspace/pybind11/test/sample_header.h:392
          const char* doc_deprecated_1args =
R"""((Deprecated.)

Deprecated:
    f() is slow; use g() instead. Also, I like hats. This will be
    removed from Drake on or after 2038-01-19.)""";
          // Source: drake/tools/workspace/pybind11/test/sample_header.h:396
          const char* doc_deprecated_0args =
R"""((Deprecated.)

Deprecated:
    f() now requires an int. This will be removed from Drake on or
    after 2038-01-19.)""";
        } f;
      } DrakeDeprecatedClass;
      // Symbol: drake::mkdoc_test::DrakeDeprecatedTemplateClass
      struct /* DrakeDeprecatedTemplateClass */ {
        // Source: drake/tools/workspace/pybind11/test/sample_header.h:406
        const char* doc_deprecated =
R"""(I am symbolically old. (Deprecated.)

Deprecated:
    Templates rule! This will be removed from Drake on or after
    2038-01-19.)""";
      } DrakeDeprecatedTemplateClass;
      // Symbol: drake::mkdoc_test::Enum
      struct /* Enum */ {
        // Source: drake/tools/workspace/pybind11/test/sample_header.h:368
        const char* doc =
R"""(Enumeration. Feugiat scelerisque varius morbi enim. Facilisis leo vel
fringilla est ullamcorper eget nulla facilisi.)""";
        // Symbol: drake::mkdoc_test::Enum::EnumConstant
        struct /* EnumConstant */ {
          // Source: drake/tools/workspace/pybind11/test/sample_header.h:370
          const char* doc = R"""(Enumeration constant.)""";
        } EnumConstant;
      } Enum;
      // Symbol: drake::mkdoc_test::EnumClass
      struct /* EnumClass */ {
        // Source: drake/tools/workspace/pybind11/test/sample_header.h:375
        const char* doc =
R"""(Enumeration class. Malesuada fames ac turpis egestas integer eget
aliquet nibh praesent.)""";
        // Symbol: drake::mkdoc_test::EnumClass::EnumClassConstant
        struct /* EnumClassConstant */ {
          // Source: drake/tools/workspace/pybind11/test/sample_header.h:376
          const char* doc =
R"""(Enumeration class constant. Vestibulum mattis.)""";
        } EnumClassConstant;
      } EnumClass;
      // Symbol: drake::mkdoc_test::Struct
      struct /* Struct */ {
        // Source: drake/tools/workspace/pybind11/test/sample_header.h:304
        const char* doc_deprecated =
R"""(Struct. Sed elementum tempus egestas sed sed risus pretium. Vel
pharetra vel turpis nunc.

Deprecated:
    Begin deprecated. Est pellentesque elit ullamcorper dignissim cras
    tincidunt lobortis. End deprecated.

See also:
    Class)""";
        // Symbol: drake::mkdoc_test::Struct::field_1
        struct /* field_1 */ {
          // Source: drake/tools/workspace/pybind11/test/sample_header.h:310
          const char* doc =
R"""(Field 1. Sit amet cursus sit amet dictum sit amet. Id leo in vitae
turpis massa sed elementum tempus.

Attention:
    Begin attention. Ultricies lacus sed turpis tincidunt id aliquet
    risus feugiat. End attention.)""";
        } field_1;
        // Symbol: drake::mkdoc_test::Struct::field_2
        struct /* field_2 */ {
          // Source: drake/tools/workspace/pybind11/test/sample_header.h:315
          const char* doc =
R"""(Field 2. Consectetur libero id faucibus nisl tincidunt eget nullam non
nisi.

Note:
    Begin note. Consectetur a erat nam at lectus. Consequat ac felis
    donec et odio pellentesque diam. End note.)""";
        } field_2;
      } Struct;
      // Symbol: drake::mkdoc_test::TemplateClass
      struct /* TemplateClass */ {
        // Source: drake/tools/workspace/pybind11/test/sample_header.h:324
        const char* doc_was_unable_to_choose_unambiguous_names =
R"""(Template class. Mauris pharetra et ultrices neque ornare aenean
euismod elementum.

Template parameter ``T``:
    Begin template parameter. Feugiat scelerisque varius morbi enim
    nunc faucibus a. End template parameter.)""";
        // Symbol: drake::mkdoc_test::TemplateClass::TemplateClass<T>
        struct /* ctor */ {
          // Source: drake/tools/workspace/pybind11/test/sample_header.h:332
          const char* doc_0args =
R"""(Default constructor. Condimentum mattis pellentesque id nibh tortor
id. Nisl rhoncus mattis rhoncus urna neque.

Remark:
    Begin remarks. Ante metus dictum at tempor commodo. Nec feugiat in
    fermentum posuere urna nec. End remarks.)""";
          // Source: drake/tools/workspace/pybind11/test/sample_header.h:335
          const char* doc_1args = R"""(Single argument int constructor.)""";
          // Source: drake/tools/workspace/pybind11/test/sample_header.h:339
          const char* doc_copyconvert = R"""(Scalar-converting copy constructor.)""";
        } ctor;
      } TemplateClass;
      // Symbol: drake::mkdoc_test::func
      struct /* func */ {
        // Source: drake/tools/workspace/pybind11/test/sample_header.h:81
        const char* doc_0args =
R"""(Function. Mi sit amet mauris commodo quis.)""";
        // Source: drake/tools/workspace/pybind11/test/sample_header.h:87
        const char* doc_1args_param =
R"""(Function, overload 1. Velit ut tortor pretium viverra suspendisse
potenti nullam ac tortor.

Raises:
    RuntimeError Begin raises. Morbi tincidunt augue interdum velit
    euismod. Justo nec ultrices dui sapien eget mi proin sed libero.
    End raises.)""";
        // Source: drake/tools/workspace/pybind11/test/sample_header.h:93
        const char* doc_1args_T =
R"""(Function, template overload. Pellentesque diam volutpat commodo sed
egestas egestas fringilla phasellus faucibus.

Parameter ``param``:
    Begin input/output parameter. Morbi enim nunc faucibus a
    pellentesque sit. End input/output parameter.)""";
      } func;
    } mkdoc_test;
  } drake;
  // Symbol: namespace_1
  struct /* namespace_1 */ {
    // Symbol: namespace_1::namespace_2
    struct /* namespace_2 */ {
      // Symbol: namespace_1::namespace_2::DummyClass
      struct /* DummyClass */ {
        // Source: drake/tools/workspace/pybind11/test/sample_header.h:452
        const char* doc = R"""()""";
        // Symbol: namespace_1::namespace_2::DummyClass::Details
        struct /* Details */ {
          // Source: drake/tools/workspace/pybind11/test/sample_header.h:457
          const char* doc =
R"""(Ligula. Nunc turpis. Mauris vitae sapien. Nunc.)""";
        } Details;
        // Symbol: namespace_1::namespace_2::DummyClass::DummyClass
        struct /* ctor */ {
          // Source: drake/tools/workspace/pybind11/test/sample_header.h:454
          const char* doc = R"""()""";
        } ctor;
        // Symbol: namespace_1::namespace_2::DummyClass::static_function_1
        struct /* static_function_1 */ {
          // Source: drake/tools/workspace/pybind11/test/sample_header.h:465
          const char* doc = R"""()""";
        } static_function_1;
        // Symbol: namespace_1::namespace_2::DummyClass::static_function_2
        struct /* static_function_2 */ {
          // Source: drake/tools/workspace/pybind11/test/sample_header.h:466
          const char* doc = R"""()""";
        } static_function_2;
        // Symbol: namespace_1::namespace_2::DummyClass::struct_2
        struct /* struct_2 */ {
          // Source: drake/tools/workspace/pybind11/test/sample_header.h:464
          const char* doc = R"""()""";
        } struct_2;
      } DummyClass;
      // Symbol: namespace_1::namespace_2::Struct1
      struct /* Struct1 */ {
        // Source: drake/tools/workspace/pybind11/test/sample_header.h:427
        const char* doc =
R"""(Quam odio at est.

Proin eleifend nisi et nibh. Maecenas a lacus. Mauris porta quam non
massa molestie scelerisque. Nulla sed ante at lorem suscipit rutrum.
Nam quis tellus. Cras elit nisi, ornare a, condimentum vitae, rutrum
sit amet, tellus. Maecenas)""";
        // Symbol: namespace_1::namespace_2::Struct1::var_1
        struct /* var_1 */ {
          // Source: drake/tools/workspace/pybind11/test/sample_header.h:429
          const char* doc =
R"""(Et, ornare sagittis, tellus. Fusce felis.)""";
        } var_1;
        // Symbol: namespace_1::namespace_2::Struct1::var_2
        struct /* var_2 */ {
          // Source: drake/tools/workspace/pybind11/test/sample_header.h:431
          const char* doc = R"""(Nulla a augue. Pellentesque sed est.)""";
        } var_2;
        // Symbol: namespace_1::namespace_2::Struct1::var_3
        struct /* var_3 */ {
          // Source: drake/tools/workspace/pybind11/test/sample_header.h:433
          const char* doc = R"""(Imperdiet tristique, interdum a, dolor.)""";
        } var_3;
        // Symbol: namespace_1::namespace_2::Struct1::var_4
        struct /* var_4 */ {
          // Source: drake/tools/workspace/pybind11/test/sample_header.h:435
          const char* doc =
R"""(Tempor lobortis turpis. Sed tellus velit, ullamcorper.)""";
        } var_4;
        // Symbol: namespace_1::namespace_2::Struct1::var_5
        struct /* var_5 */ {
          // Source: drake/tools/workspace/pybind11/test/sample_header.h:437
          const char* doc =
R"""(Id, rutrum auctor, ullamcorper sed, orci. In.)""";
        } var_5;
        // Symbol: namespace_1::namespace_2::Struct1::var_6
        struct /* var_6 */ {
          // Source: drake/tools/workspace/pybind11/test/sample_header.h:439
          const char* doc =
R"""(Fames ac turpis egestas. Sed vitae eros. Nulla.)""";
        } var_6;
        // Symbol: namespace_1::namespace_2::Struct1::var_7
        struct /* var_7 */ {
          // Source: drake/tools/workspace/pybind11/test/sample_header.h:441
          const char* doc =
R"""(Condimentum. Donec arcu quam, dictum accumsan, convallis.)""";
        } var_7;
        // Symbol: namespace_1::namespace_2::Struct1::var_8
        struct /* var_8 */ {
          // Source: drake/tools/workspace/pybind11/test/sample_header.h:443
          const char* doc =
R"""(Volutpat. Donec non tortor. Vivamus posuere nisi mollis.)""";
        } var_8;
      } Struct1;
      // Symbol: namespace_1::namespace_2::Struct2
      struct /* Struct2 */ {
        // Source: drake/tools/workspace/pybind11/test/sample_header.h:446
        const char* doc = R"""()""";
      } Struct2;
      // Symbol: namespace_1::namespace_2::Struct3
      struct /* Struct3 */ {
        // Source: drake/tools/workspace/pybind11/test/sample_header.h:447
        const char* doc = R"""()""";
      } Struct3;
      // Symbol: namespace_1::namespace_2::Struct4
      struct /* Struct4 */ {
        // Source: drake/tools/workspace/pybind11/test/sample_header.h:448
        const char* doc = R"""()""";
      } Struct4;
      // Symbol: namespace_1::namespace_2::Struct5
      struct /* Struct5 */ {
        // Source: drake/tools/workspace/pybind11/test/sample_header.h:449
        const char* doc = R"""()""";
      } Struct5;
      // Symbol: namespace_1::namespace_2::Struct6
      struct /* Struct6 */ {
        // Source: drake/tools/workspace/pybind11/test/sample_header.h:450
        const char* doc = R"""()""";
      } Struct6;
    } namespace_2;
  } namespace_1;
} sample_header_doc;

#if defined(__GNUG__)
#pragma GCC diagnostic pop
#endif
