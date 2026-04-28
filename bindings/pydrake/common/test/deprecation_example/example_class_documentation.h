#pragma once

// GENERATED FILE DO NOT EDIT
// This file contains docstrings for the Python bindings that were
// automatically extracted by mkdoc.py.

#include <array>
#include <utility>

#if defined(__GNUG__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-variable"
#endif

// #include "drake/bindings/pydrake/common/test/deprecation_example/example_class.h"

// Symbol: pydrake_doc
constexpr struct /* pydrake_doc */ {
  // Symbol: drake
  struct /* drake */ {
    // Symbol: drake::example_class
    struct /* example_class */ {
      // Symbol: drake::example_class::ExampleCppClass
      struct /* ExampleCppClass */ {
        // Source: drake/bindings/pydrake/common/test/deprecation_example/example_class.h
        const char* doc = R"""(Example class.)""";
        // Symbol: drake::example_class::ExampleCppClass::DeprecatedMethod
        struct /* DeprecatedMethod */ {
          // Source: drake/bindings/pydrake/common/test/deprecation_example/example_class.h
          const char* doc_deprecated =
R"""((Deprecated.)

Deprecated:
    Do not use DeprecatedMethod. This will be removed from Drake on or
    after 2038-01-19.)""";
        } DeprecatedMethod;
        // Symbol: drake::example_class::ExampleCppClass::ExampleCppClass
        struct /* ctor */ {
          // Source: drake/bindings/pydrake/common/test/deprecation_example/example_class.h
          const char* doc_0args = R"""(Good constructor.)""";
          // Source: drake/bindings/pydrake/common/test/deprecation_example/example_class.h
          const char* doc_deprecated_deprecated_1args_x =
R"""((Deprecated.)

Deprecated:
    Do not use ExampleCppClass(int). This will be removed from Drake
    on or after 2038-01-19.)""";
          // Source: drake/bindings/pydrake/common/test/deprecation_example/example_class.h
          const char* doc_deprecated_deprecated_1args_y =
R"""((Deprecated.)

Deprecated:
    Do not use ExampleCppClass(double). This will be removed from
    Drake on or after 2038-01-19.)""";
        } ctor;
        // Symbol: drake::example_class::ExampleCppClass::FunctionWithArgumentName
        struct /* FunctionWithArgumentName */ {
          // Source: drake/bindings/pydrake/common/test/deprecation_example/example_class.h
          const char* doc = R"""()""";
        } FunctionWithArgumentName;
        // Symbol: drake::example_class::ExampleCppClass::ParallelWork
        struct /* ParallelWork */ {
          // Source: drake/bindings/pydrake/common/test/deprecation_example/example_class.h
          const char* doc_deprecated =
R"""((Deprecated.)

Deprecated:
    Do not use ParallelWork. This will be removed from Drake on or
    after 2038-01-19.)""";
        } ParallelWork;
        // Symbol: drake::example_class::ExampleCppClass::overload
        struct /* overload */ {
          // Source: drake/bindings/pydrake/common/test/deprecation_example/example_class.h
          const char* doc = R"""(Good overload.)""";
          // Source: drake/bindings/pydrake/common/test/deprecation_example/example_class.h
          const char* doc_deprecated =
R"""((Deprecated.)

Deprecated:
    Do not use overload(int). This will be removed from Drake on or
    after 2038-01-19.)""";
        } overload;
        // Symbol: drake::example_class::ExampleCppClass::prop
        struct /* prop */ {
          // Source: drake/bindings/pydrake/common/test/deprecation_example/example_class.h
          const char* doc = R"""(Good property.)""";
        } prop;
      } ExampleCppClass;
      // Symbol: drake::example_class::ExampleCppStruct
      struct /* ExampleCppStruct */ {
        // Source: drake/bindings/pydrake/common/test/deprecation_example/example_class.h
        const char* doc_deprecated =
R"""(Serves as an example for binding (and deprecating) a simple struct.
This allows the struct to be constructed with ParamInit and deprecated
using the corresponding DeprecatedParamInit. (Deprecated.)

Deprecated:
    Do not use ExampleCppStruct This will be removed from Drake on or
    after 2038-01-19.)""";
        // Symbol: drake::example_class::ExampleCppStruct::i
        struct /* i */ {
          // Source: drake/bindings/pydrake/common/test/deprecation_example/example_class.h
          const char* doc = R"""()""";
        } i;
        // Symbol: drake::example_class::ExampleCppStruct::j
        struct /* j */ {
          // Source: drake/bindings/pydrake/common/test/deprecation_example/example_class.h
          const char* doc = R"""()""";
        } j;
      } ExampleCppStruct;
    } example_class;
  } drake;
} pydrake_doc;

#if defined(__GNUG__)
#pragma GCC diagnostic pop
#endif
