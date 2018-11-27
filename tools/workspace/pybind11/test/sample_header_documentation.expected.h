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
    // Source: drake/tools/workspace/pybind11/test/sample_header.h:29
    const char* doc =
R"""(Root-level symbol. Magna fermentum iaculis eu non diam phasellus
vestibulum.)""";
  } RootLevelSymbol;
  // Symbol: drake
  struct /* drake */ {
    // Symbol: drake::MidLevelSymbol
    struct /* MidLevelSymbol */ {
      // Source: drake/tools/workspace/pybind11/test/sample_header.h:71
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
        // Source: drake/tools/workspace/pybind11/test/sample_header.h:312
        const char* doc = R"""(Anonymous enum's constant.)""";
      } AnonymousConstant;
      // Symbol: drake::mkdoc_test::Class
      struct /* Class */ {
        // Source: drake/tools/workspace/pybind11/test/sample_header.h:123
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
          // Source: drake/tools/workspace/pybind11/test/sample_header.h:125
          const char* doc_copy = R"""()""";
          // Source: drake/tools/workspace/pybind11/test/sample_header.h:125
          const char* doc_move = R"""()""";
          // Source: drake/tools/workspace/pybind11/test/sample_header.h:146
          const char* doc_0args = R"""(Custom constructor 1.)""";
          // Source: drake/tools/workspace/pybind11/test/sample_header.h:159
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
          // Source: drake/tools/workspace/pybind11/test/sample_header.h:172
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
        // Symbol: drake::mkdoc_test::Class::DRAKE_COPYABLE_DEMAND_COPY_CAN_COMPILE
        struct /* DRAKE_COPYABLE_DEMAND_COPY_CAN_COMPILE */ {
          // Source: drake/tools/workspace/pybind11/test/sample_header.h:125
          const char* doc = R"""()""";
        } DRAKE_COPYABLE_DEMAND_COPY_CAN_COMPILE;
        // Symbol: drake::mkdoc_test::Class::Nested
        struct /* Nested */ {
          // Source: drake/tools/workspace/pybind11/test/sample_header.h:230
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
          // Source: drake/tools/workspace/pybind11/test/sample_header.h:219
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
          // Source: drake/tools/workspace/pybind11/test/sample_header.h:187
          const char* doc =
R"""(Public method. :math:`A = \pi r^2`. Condimentum id venenatis a
condimentum vitae sapien pellentesque habitant morbi.)""";
        } PublicMethod;
        // Symbol: drake::mkdoc_test::Class::PublicStatic
        struct /* PublicStatic */ {
          // Source: drake/tools/workspace/pybind11/test/sample_header.h:210
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
          // Source: drake/tools/workspace/pybind11/test/sample_header.h:197
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
          // Source: drake/tools/workspace/pybind11/test/sample_header.h:138
          const char* doc =
R"""(Typedef alias. **Bold**. Risus nec feugiat in fermentum posuere urna
nec tincidunt praesent.)""";
        } TypedefAlias;
        // Symbol: drake::mkdoc_test::Class::UsingAlias
        struct /* UsingAlias */ {
          // Source: drake/tools/workspace/pybind11/test/sample_header.h:133
          const char* doc =
R"""(Using alias. *Italics*. Sit amet nisl purus in mollis nunc sed id
semper.)""";
        } UsingAlias;
        // Symbol: drake::mkdoc_test::Class::protected_member_
        struct /* protected_member_ */ {
          // Source: drake/tools/workspace/pybind11/test/sample_header.h:236
          const char* doc =
R"""(Protected member. ``Typewriter``. Porttitor eget dolor morbi non arcu
risus quis varius quam.)""";
        } protected_member_;
      } Class;
      // Symbol: drake::mkdoc_test::Enum
      struct /* Enum */ {
        // Source: drake/tools/workspace/pybind11/test/sample_header.h:296
        const char* doc =
R"""(Enumeration. Feugiat scelerisque varius morbi enim. Facilisis leo vel
fringilla est ullamcorper eget nulla facilisi.)""";
        // Symbol: drake::mkdoc_test::Enum::EnumConstant
        struct /* EnumConstant */ {
          // Source: drake/tools/workspace/pybind11/test/sample_header.h:298
          const char* doc = R"""(Enumeration constant.)""";
        } EnumConstant;
      } Enum;
      // Symbol: drake::mkdoc_test::EnumClass
      struct /* EnumClass */ {
        // Source: drake/tools/workspace/pybind11/test/sample_header.h:303
        const char* doc =
R"""(Enumeration class. Malesuada fames ac turpis egestas integer eget
aliquet nibh praesent.)""";
        // Symbol: drake::mkdoc_test::EnumClass::EnumClassConstant
        struct /* EnumClassConstant */ {
          // Source: drake/tools/workspace/pybind11/test/sample_header.h:306
          const char* doc =
R"""(Enumeration class constant. Vestibulum mattis ullamcorper velit sed
ullamcorper.)""";
        } EnumClassConstant;
      } EnumClass;
      // Symbol: drake::mkdoc_test::Struct
      struct /* Struct */ {
        // Source: drake/tools/workspace/pybind11/test/sample_header.h:255
        const char* doc =
R"""(Struct. Sed elementum tempus egestas sed sed risus pretium. Vel
pharetra vel turpis nunc.

Deprecated:
    Begin deprecated. Est pellentesque elit ullamcorper dignissim cras
    tincidunt lobortis. End deprecated.

See also:
    Class)""";
        // Symbol: drake::mkdoc_test::Struct::field_1
        struct /* field_1 */ {
          // Source: drake/tools/workspace/pybind11/test/sample_header.h:261
          const char* doc =
R"""(Field 1. Sit amet cursus sit amet dictum sit amet. Id leo in vitae
turpis massa sed elementum tempus.

Attention:
    Begin attention. Ultricies lacus sed turpis tincidunt id aliquet
    risus feugiat. End attention.)""";
        } field_1;
        // Symbol: drake::mkdoc_test::Struct::field_2
        struct /* field_2 */ {
          // Source: drake/tools/workspace/pybind11/test/sample_header.h:266
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
        // Source: drake/tools/workspace/pybind11/test/sample_header.h:275
        const char* doc =
R"""(Template class. Mauris pharetra et ultrices neque ornare aenean
euismod elementum.

Template parameter ``T``:
    Begin template parameter. Feugiat scelerisque varius morbi enim
    nunc faucibus a. End template parameter.)""";
        // Source: drake/tools/workspace/pybind11/test/sample_header.h:289
        const char* doc_2 =
R"""(Specialize. Nisl pretium fusce id velit ut tortor pretium viverra.
Quis ipsum suspendisse ultrices gravida dictum fusce ut.

Since:
    Begin since. Nullam eget felis eget nunc lobortis mattis aliquam
    faucibus. End since.)""";
        // Symbol: drake::mkdoc_test::TemplateClass::TemplateClass<T>
        struct /* ctor */ {
          // Source: drake/tools/workspace/pybind11/test/sample_header.h:281
          const char* doc =
R"""(Default constructor. Condimentum mattis pellentesque id nibh tortor
id. Nisl rhoncus mattis rhoncus urna neque.

Remark:
    Begin remarks. Ante metus dictum at tempor commodo. Nec feugiat in
    fermentum posuere urna nec. End remarks.)""";
        } ctor;
      } TemplateClass;
      // Symbol: drake::mkdoc_test::func
      struct /* func */ {
        // Source: drake/tools/workspace/pybind11/test/sample_header.h:76
        const char* doc =
R"""(Function. Mi sit amet mauris commodo quis.)""";
        // Source: drake/tools/workspace/pybind11/test/sample_header.h:82
        const char* doc_2 =
R"""(Function, overload 1. Velit ut tortor pretium viverra suspendisse
potenti nullam ac tortor.

Raises:
    RuntimeError Begin raises. Morbi tincidunt augue interdum velit
    euismod. Justo nec ultrices dui sapien eget mi proin sed libero.
    End raises.)""";
        // Source: drake/tools/workspace/pybind11/test/sample_header.h:88
        const char* doc_3 =
R"""(Function, template overload. Pellentesque diam volutpat commodo sed
egestas egestas fringilla phasellus faucibus.

Parameter ``param``:
    Begin input/output parameter. Morbi enim nunc faucibus a
    pellentesque sit. End input/output parameter.)""";
      } func;
    } mkdoc_test;
  } drake;
} sample_header_doc;

#if defined(__GNUG__)
#pragma GCC diagnostic pop
#endif
