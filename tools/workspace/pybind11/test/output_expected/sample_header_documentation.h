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
    const char* doc = R"""(Root-level symbol.)""";
  } RootLevelSymbol;
  // Symbol: drake
  struct /* drake */ {
    // Symbol: drake::MidLevelSymbol
    struct /* MidLevelSymbol */ {
      const char* doc = R"""(MidLevelSymbol.)""";
    } MidLevelSymbol;
    // Symbol: drake::mkdoc_test
    struct /* mkdoc_test */ {
      // Symbol: drake::mkdoc_test::AnonymousConstant
      struct /* AnonymousConstant */ {
        const char* doc = R"""(Anonymous enum's constant.)""";
      } AnonymousConstant;
      // Symbol: drake::mkdoc_test::Class
      struct /* Class */ {
        const char* doc = R"""(Code.)""";
        // Symbol: drake::mkdoc_test::Class::Class
        struct /* ctor */ {
          const char* doc_0args = R"""(Custom constructor 1.)""";
          const char* doc_1args = R"""(Custom constructor 2.)""";
          const char* doc_2args = R"""(Custom constructor 3.)""";
        } ctor;
        // Symbol: drake::mkdoc_test::Class::Nested
        struct /* Nested */ {
          const char* doc = R"""(Protected nested class.)""";
        } Nested;
        // Symbol: drake::mkdoc_test::Class::ProtectedMethod
        struct /* ProtectedMethod */ {
          const char* doc = R"""(Protected method.)""";
        } ProtectedMethod;
        // Symbol: drake::mkdoc_test::Class::PublicMethod
        struct /* PublicMethod */ {
          const char* doc = R"""(Public method that used to get hidden.)""";
        } PublicMethod;
        // Symbol: drake::mkdoc_test::Class::PublicStatic
        struct /* PublicStatic */ {
          const char* doc = R"""(More things.)""";
        } PublicStatic;
        // Symbol: drake::mkdoc_test::Class::PublicTemplateMethod
        struct /* PublicTemplateMethod */ {
          const char* doc = R"""()""";
        } PublicTemplateMethod;
        // Symbol: drake::mkdoc_test::Class::TypedefAlias
        struct /* TypedefAlias */ {
          const char* doc = R"""(Typedef alias.)""";
        } TypedefAlias;
        // Symbol: drake::mkdoc_test::Class::UsingAlias
        struct /* UsingAlias */ {
          const char* doc = R"""(Using alias.)""";
        } UsingAlias;
        // Symbol: drake::mkdoc_test::Class::do_stuff
        struct /* do_stuff */ {
          const char* doc_stuff_1 = R"""(Docstring 1.)""";
          const char* doc_stuff_2 = R"""(Docstring 2.)""";
        } do_stuff;
        // Symbol: drake::mkdoc_test::Class::get_foo
        struct /* get_foo */ {
          const char* doc_0args_nonconst = R"""(Overloaded only by its const-ness.)""";
          const char* doc_0args_const = R"""(The const one.)""";
        } get_foo;
        // Symbol: drake::mkdoc_test::Class::overloaded_method
        struct /* overloaded_method */ {
          const char* doc_1args_alpha = R"""(This one takes an int.)""";
          const char* doc_1args_bravo = R"""(This one takes a double.)""";
          const char* doc_2args_charlie_delta = R"""(This one takes an int and a double.)""";
          const char* doc_2args_double_int = R"""(This one takes the road less traveled.)""";
          const char* doc_1args_constdrakemkdoctestSimpleStruct = R"""(This one takes a non-primitive type.)""";
        } overloaded_method;
        // Symbol: drake::mkdoc_test::Class::overloaded_with_same_doc
        struct /* overloaded_with_same_doc */ {
          const char* doc = R"""(Different overload with same doc.)""";
        } overloaded_with_same_doc;
        // Symbol: drake::mkdoc_test::Class::protected_member_
        struct /* protected_member_ */ {
          const char* doc = R"""()""";
        } protected_member_;
      } Class;
      // Symbol: drake::mkdoc_test::DrakeDeprecatedClass
      struct /* DrakeDeprecatedClass */ {
        const char* doc = R"""(I am measurably old.)""";
        // Symbol: drake::mkdoc_test::DrakeDeprecatedClass::a
        struct /* a */ {
          const char* doc = R"""()""";
        } a;
        // Symbol: drake::mkdoc_test::DrakeDeprecatedClass::f
        struct /* f */ {
          const char* doc = R"""()""";
        } f;
      } DrakeDeprecatedClass;
      // Symbol: drake::mkdoc_test::DrakeDeprecatedTemplateClass
      struct /* DrakeDeprecatedTemplateClass */ {
        const char* doc = R"""(I am symbolically old.)""";
      } DrakeDeprecatedTemplateClass;
      // Symbol: drake::mkdoc_test::Enum
      struct /* Enum */ {
        const char* doc = R"""(Enumeration.)""";
        // Symbol: drake::mkdoc_test::Enum::EnumConstant
        struct /* EnumConstant */ {
          const char* doc = R"""(Enumeration constant.)""";
        } EnumConstant;
      } Enum;
      // Symbol: drake::mkdoc_test::EnumClass
      struct /* EnumClass */ {
        const char* doc = R"""(Enumeration class.)""";
        // Symbol: drake::mkdoc_test::EnumClass::EnumClassConstant
        struct /* EnumClassConstant */ {
          const char* doc =
R"""(Enumeration class constant. Vestibulum mattis.)""";
        } EnumClassConstant;
      } EnumClass;
      // Symbol: drake::mkdoc_test::SimpleStruct
      struct /* SimpleStruct */ {
        const char* doc = R"""()""";
        // Symbol: drake::mkdoc_test::SimpleStruct::value
        struct /* value */ {
          const char* doc = R"""()""";
        } value;
      } SimpleStruct;
      // Symbol: drake::mkdoc_test::Struct
      struct /* Struct */ {
        const char* doc = R"""(Struct.)""";
        // Symbol: drake::mkdoc_test::Struct::field_1
        struct /* field_1 */ {
          const char* doc = R"""(Field 1.)""";
        } field_1;
        // Symbol: drake::mkdoc_test::Struct::field_2
        struct /* field_2 */ {
          const char* doc = R"""(Field 2.)""";
        } field_2;
      } Struct;
      // Symbol: drake::mkdoc_test::TemplateClass
      struct /* TemplateClass */ {
        const char* doc_was_unable_to_choose_unambiguous_names = R"""(Template class.)""";
        // Symbol: drake::mkdoc_test::TemplateClass::TemplateClass<T>
        struct /* ctor */ {
          const char* doc_0args = R"""(Default constructor.)""";
          const char* doc_1args = R"""(Single argument int constructor.)""";
          const char* doc_copyconvert = R"""(Scalar-converting copy constructor.)""";
        } ctor;
      } TemplateClass;
      // Symbol: drake::mkdoc_test::TemplateStruct
      struct /* TemplateStruct */ {
        const char* doc = R"""()""";
        // Symbol: drake::mkdoc_test::TemplateStruct::value
        struct /* value */ {
          const char* doc = R"""()""";
        } value;
      } TemplateStruct;
      // Symbol: drake::mkdoc_test::func
      struct /* func */ {
        const char* doc_0args = R"""(Function.)""";
        const char* doc_1args_param = R"""(Function, overload 1.)""";
        const char* doc_1args_T =
R"""(Function, template overload. Note that the argument name ``tee`` will
*not* show up in the docstring name.)""";
      } func;
    } mkdoc_test;
  } drake;
  // Symbol: namespace_1
  struct /* namespace_1 */ {
    // Symbol: namespace_1::namespace_2
    struct /* namespace_2 */ {
      // Symbol: namespace_1::namespace_2::CrtpBase
      struct /* CrtpBase */ {
        const char* doc = R"""(Curious.)""";
        // Symbol: namespace_1::namespace_2::CrtpBase::derived
        struct /* derived */ {
          const char* doc = R"""()""";
        } derived;
      } CrtpBase;
      // Symbol: namespace_1::namespace_2::CrtpChild
      struct /* CrtpChild */ {
        const char* doc = R"""(And recurring.)""";
      } CrtpChild;
      // Symbol: namespace_1::namespace_2::CrtpFunction
      struct /* CrtpFunction */ {
        const char* doc_1args_constCrtpBase =
R"""(And a pattern. Note that the argument name ``instance`` will *not*
show up in the docstring name.)""";
        const char* doc_1args_value = R"""(With an overload.)""";
      } CrtpFunction;
      // Symbol: namespace_1::namespace_2::DummyClass
      struct /* DummyClass */ {
        const char* doc = R"""()""";
        // Symbol: namespace_1::namespace_2::DummyClass::Details
        struct /* Details */ {
          const char* doc =
R"""(Ligula. Nunc turpis. Mauris vitae sapien. Nunc.)""";
        } Details;
        // Symbol: namespace_1::namespace_2::DummyClass::DummyClass
        struct /* ctor */ {
          const char* doc = R"""()""";
        } ctor;
        // Symbol: namespace_1::namespace_2::DummyClass::static_function_1
        struct /* static_function_1 */ {
          const char* doc = R"""()""";
        } static_function_1;
        // Symbol: namespace_1::namespace_2::DummyClass::static_function_2
        struct /* static_function_2 */ {
          const char* doc = R"""()""";
        } static_function_2;
        // Symbol: namespace_1::namespace_2::DummyClass::struct_2
        struct /* struct_2 */ {
          const char* doc = R"""()""";
        } struct_2;
      } DummyClass;
      // Symbol: namespace_1::namespace_2::Struct1
      struct /* Struct1 */ {
        const char* doc =
R"""(Quam odio at est.

Proin eleifend nisi et nibh.)""";
        // Symbol: namespace_1::namespace_2::Struct1::var_1
        struct /* var_1 */ {
          const char* doc =
R"""(Et, ornare sagittis, tellus. Fusce felis.)""";
        } var_1;
        // Symbol: namespace_1::namespace_2::Struct1::var_2
        struct /* var_2 */ {
          const char* doc = R"""(Nulla a augue. Pellentesque sed est.)""";
        } var_2;
        // Symbol: namespace_1::namespace_2::Struct1::var_3
        struct /* var_3 */ {
          const char* doc = R"""(Imperdiet tristique, interdum a, dolor.)""";
        } var_3;
        // Symbol: namespace_1::namespace_2::Struct1::var_4
        struct /* var_4 */ {
          const char* doc =
R"""(Tempor lobortis turpis. Sed tellus velit, ullamcorper.)""";
        } var_4;
        // Symbol: namespace_1::namespace_2::Struct1::var_5
        struct /* var_5 */ {
          const char* doc =
R"""(Id, rutrum auctor, ullamcorper sed, orci. In.)""";
        } var_5;
        // Symbol: namespace_1::namespace_2::Struct1::var_6
        struct /* var_6 */ {
          const char* doc =
R"""(Fames ac turpis egestas. Sed vitae eros. Nulla.)""";
        } var_6;
        // Symbol: namespace_1::namespace_2::Struct1::var_7
        struct /* var_7 */ {
          const char* doc =
R"""(Condimentum. Donec arcu quam, dictum accumsan, convallis.)""";
        } var_7;
        // Symbol: namespace_1::namespace_2::Struct1::var_8
        struct /* var_8 */ {
          const char* doc =
R"""(Volutpat. Donec non tortor. Vivamus posuere nisi mollis.)""";
        } var_8;
      } Struct1;
      // Symbol: namespace_1::namespace_2::Struct2
      struct /* Struct2 */ {
        const char* doc = R"""()""";
      } Struct2;
      // Symbol: namespace_1::namespace_2::Struct3
      struct /* Struct3 */ {
        const char* doc = R"""()""";
      } Struct3;
      // Symbol: namespace_1::namespace_2::Struct4
      struct /* Struct4 */ {
        const char* doc = R"""()""";
      } Struct4;
      // Symbol: namespace_1::namespace_2::Struct5
      struct /* Struct5 */ {
        const char* doc = R"""()""";
      } Struct5;
      // Symbol: namespace_1::namespace_2::Struct6
      struct /* Struct6 */ {
        const char* doc = R"""()""";
      } Struct6;
    } namespace_2;
  } namespace_1;
  // Symbol: unused
  struct /* unused */ {
    const char* doc =
R"""(@defgroup first_group This should be ignored, but clang associates it
with the ``unused`` symbol.)""";
  } unused;
} sample_header_doc;

#if defined(__GNUG__)
#pragma GCC diagnostic pop
#endif
