---
title: VS Code IDE setup
---

This guide describes how to edit the Drake codebase using VS Code.

If you have tips that would help out other developers, drop us a line and we
can add them here!

# Setting up VS Code

There is nothing really special to do.
Be sure you've installed the C/C++ extension pack from Microsoft.

## C++ paths

Because VS Code wants to locate all included headers, but Bazel does
not provide those headers in a single consistent location, it is
common to get "red squiggles" for included headers, particularly for
externals like LCM.  So far this does not seem to be avoidable; the
`"C_Cpp.errorSquiggles": "enabledIfIncludesResolve"` setting does not
work to solve this problem.

## C++ code formatting

The [Visual Studio docs](https://code.visualstudio.com/docs/cpp/cpp-ide#_code-formatting)
on code formatting work well. Take note of "Format Document",
"Format Selection", "Format on save", and "Format on type".

In the VS Code Options configuration, check that the option for
``C_Cpp: Clang_format_path`` is set to Drake's preferred value
``/path/to/drake/bazel-bin/tools/lint/clang-format``.
