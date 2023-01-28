---
title: VS Code IDE setup
---

This guide describes how to edit the Drake codebase using VS Code.

If you have tips that would help out other developers, drop us a line and we
can add them here!

# Setting up VS Code

There is nothing really special to do.
Be sure you've installed the C/C++ extension pack from Microsoft.

## C++ Code formatting

The [Visual Studio docs](https://code.visualstudio.com/docs/cpp/cpp-ide#_code-formatting)
on code formatting work well. Take note of "Format Document",
"Format Selection", "Format on save", and "Format on type".

In the VS Code Options configuration, check that the option for
``C_Cpp: Clang_format_path`` is set to Drake's preferred value
``clang-format-12``.
