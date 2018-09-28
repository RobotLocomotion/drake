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
    // Source: drake/tools/workspace/pybind11/test/sample_header.h:9
    const char* doc = R"""(Root-level symbol.)""";
  } RootLevelSymbol;
  // Symbol: drake
  struct /* drake */ {
    // Symbol: drake::MidLevelSymbol
    struct /* MidLevelSymbol */ {
      // Source: drake/tools/workspace/pybind11/test/sample_header.h:18
      const char* doc =
R"""(Mid-level symbol. Ut enim ad minim veniam, quis nostrud exercitation
ullamco laboris nisi ut aliquip ex ea commodo consequat.)""";
    } MidLevelSymbol;
    // Symbol: drake::mkdoc_test
    struct /* mkdoc_test */ {
      // Symbol: drake::mkdoc_test::AnonymousConstant
      struct /* AnonymousConstant */ {
        // Source: drake/tools/workspace/pybind11/test/sample_header.h:119
        const char* doc = R"""(Anonymous enum's constant.)""";
      } AnonymousConstant;
      // Symbol: drake::mkdoc_test::Class
      struct /* Class */ {
        // Source: drake/tools/workspace/pybind11/test/sample_header.h:33
        const char* doc =
R"""(Class. Lorem ipsum dolor sit amet, consectetur adipiscing elit, sed do
eiusmod tempor incididunt ut labore et dolore magna aliqua.)""";
        // Symbol: drake::mkdoc_test::Class::Class
        struct /* ctor */ {
          // Source: drake/tools/workspace/pybind11/test/sample_header.h:35
          const char* doc = R"""()""";
          // Source: drake/tools/workspace/pybind11/test/sample_header.h:35
          const char* doc_2 = R"""()""";
          // Source: drake/tools/workspace/pybind11/test/sample_header.h:47
          const char* doc_3 = R"""(Custom constructor 1.)""";
          // Source: drake/tools/workspace/pybind11/test/sample_header.h:50
          const char* doc_4 = R"""(Custom constructor 2.)""";
          // Source: drake/tools/workspace/pybind11/test/sample_header.h:53
          const char* doc_5 = R"""(Custom constructor 3.)""";
        } ctor;
        // Symbol: drake::mkdoc_test::Class::DRAKE_COPYABLE_DEMAND_COPY_CAN_COMPILE
        struct /* DRAKE_COPYABLE_DEMAND_COPY_CAN_COMPILE */ {
          // Source: drake/tools/workspace/pybind11/test/sample_header.h:35
          const char* doc = R"""()""";
        } DRAKE_COPYABLE_DEMAND_COPY_CAN_COMPILE;
        // Symbol: drake::mkdoc_test::Class::Nested
        struct /* Nested */ {
          // Source: drake/tools/workspace/pybind11/test/sample_header.h:70
          const char* doc = R"""(Protected nested class.)""";
        } Nested;
        // Symbol: drake::mkdoc_test::Class::ProtectedMethod
        struct /* ProtectedMethod */ {
          // Source: drake/tools/workspace/pybind11/test/sample_header.h:67
          const char* doc = R"""(Protected method.)""";
        } ProtectedMethod;
        // Symbol: drake::mkdoc_test::Class::PublicMethod
        struct /* PublicMethod */ {
          // Source: drake/tools/workspace/pybind11/test/sample_header.h:56
          const char* doc = R"""(Public method.)""";
        } PublicMethod;
        // Symbol: drake::mkdoc_test::Class::PublicStatic
        struct /* PublicStatic */ {
          // Source: drake/tools/workspace/pybind11/test/sample_header.h:63
          const char* doc = R"""(Static method.)""";
        } PublicStatic;
        // Symbol: drake::mkdoc_test::Class::PublicTemplateMethod
        struct /* PublicTemplateMethod */ {
          // Source: drake/tools/workspace/pybind11/test/sample_header.h:60
          const char* doc = R"""(Public template method.)""";
        } PublicTemplateMethod;
        // Symbol: drake::mkdoc_test::Class::protected_member_
        struct /* protected_member_ */ {
          // Source: drake/tools/workspace/pybind11/test/sample_header.h:73
          const char* doc = R"""(Protected member.)""";
        } protected_member_;
      } Class;
      // Symbol: drake::mkdoc_test::Enum
      struct /* Enum */ {
        // Source: drake/tools/workspace/pybind11/test/sample_header.h:105
        const char* doc = R"""(Enumeration.)""";
        // Symbol: drake::mkdoc_test::Enum::EnumConstant
        struct /* EnumConstant */ {
          // Source: drake/tools/workspace/pybind11/test/sample_header.h:107
          const char* doc = R"""(Enumeration constant.)""";
        } EnumConstant;
      } Enum;
      // Symbol: drake::mkdoc_test::EnumClass
      struct /* EnumClass */ {
        // Source: drake/tools/workspace/pybind11/test/sample_header.h:111
        const char* doc = R"""(Enumeration class.)""";
        // Symbol: drake::mkdoc_test::EnumClass::EnumClassConstant
        struct /* EnumClassConstant */ {
          // Source: drake/tools/workspace/pybind11/test/sample_header.h:113
          const char* doc = R"""(Enumeration class constant.)""";
        } EnumClassConstant;
      } EnumClass;
      // Symbol: drake::mkdoc_test::Struct
      struct /* Struct */ {
        // Source: drake/tools/workspace/pybind11/test/sample_header.h:84
        const char* doc = R"""(Struct.)""";
        // Symbol: drake::mkdoc_test::Struct::field_1
        struct /* field_1 */ {
          // Source: drake/tools/workspace/pybind11/test/sample_header.h:86
          const char* doc = R"""(Field 1.)""";
        } field_1;
        // Symbol: drake::mkdoc_test::Struct::field_2
        struct /* field_2 */ {
          // Source: drake/tools/workspace/pybind11/test/sample_header.h:88
          const char* doc = R"""(Field 2.)""";
        } field_2;
      } Struct;
      // Symbol: drake::mkdoc_test::TemplateClass
      struct /* TemplateClass */ {
        // Source: drake/tools/workspace/pybind11/test/sample_header.h:93
        const char* doc = R"""(Template class.)""";
        // Source: drake/tools/workspace/pybind11/test/sample_header.h:101
        const char* doc_2 = R"""(Specialize.)""";
        // Symbol: drake::mkdoc_test::TemplateClass::TemplateClass<T>
        struct /* ctor */ {
          // Source: drake/tools/workspace/pybind11/test/sample_header.h:96
          const char* doc = R"""(Default constructor.)""";
        } ctor;
      } TemplateClass;
      // Symbol: drake::mkdoc_test::func
      struct /* func */ {
        // Source: drake/tools/workspace/pybind11/test/sample_header.h:23
        const char* doc = R"""(Function.)""";
        // Source: drake/tools/workspace/pybind11/test/sample_header.h:25
        const char* doc_2 = R"""(Function, overload 1.)""";
        // Source: drake/tools/workspace/pybind11/test/sample_header.h:28
        const char* doc_3 = R"""(Function, template overload.)""";
      } func;
    } mkdoc_test;
  } drake;
} sample_header_doc;

#if defined(__GNUG__)
#pragma GCC diagnostic pop
#endif
