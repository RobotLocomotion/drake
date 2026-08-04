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

## C++ code semantic analysis / autocompletion
VScode relies on the language server `clangd` and the json file [compile_commands.json](https://clang.llvm.org/docs/JSONCompilationDatabase.html) for code semantic analysis / autocompletion. To enable this feature, take the following steps
1. Install `clangd` on your system.
On Ubuntu, do
   ```
   $ sudo apt install clangd
   ```
   On mac, do
   ```
   brew install llvm
   ```
2. In VSCode, add `clangd` extension. If you have IntelliSense, disable it through
   ```
   // .vscode/settings.json
   {
     "C_Cpp.intelliSenseEngine": "disabled"
   }
   ```
3. To generate `compile_commands.json` file, take the following steps
   - Checkout this [branch](https://github.com/hongkai-dai/bazel-compile-commands-extractor/tree/enable_drake) of `bazel-compile-commands-extractor` (the upstream version doesn't work with Drake)
      ```
      $ git clone git@github.com:hongkai-dai/bazel-compile-commands-extractor.git
      $ git checkout enable_drake
      ```
   - Then add this to your `drake/user.bazelrc`
      ```
      common --inject_repository=hedron_compile_commands=PATH_TO_BAZEL_COMPILE_COMMANDS_EXTRACTOR
      ```
      where `PATH_TO_BAZEL_COMPILE_COMMANDS_EXTRACTOR` is the directory where you checked out `bazel-compile-commands-extractor`.
   - In `drake` folder, run
      ```
      $ bazel run @hedron_compile_commands//:refresh_all
      ```
      You should see a `compile_commands.json` file in your `drake` folder.
      Note: this command might take a while (like 5 mins) to run. You probably will see some error messages like
      ```
      bazel-out/k8-opt/bin/external/+internal_repositories+fcl_internal/drake_hdr/fcl/common/types.h:130:27: error: missing binary operator before token "("
      130 | #if EIGEN_VERSION_AT_LEAST(3,2,9)
      ```
      you can ignore these error messages.
4. In VSCode, you should see that you can go to the definition of any C++ object, and the autocompletion is semantic-based.
5. Everytime Drake Bazel build files (including `BUILD.bazel`) are changed, you will need to rerun `bazel run @hedron_compile_commands//:refresh_all` to update the `compilation_commands.json` file.

These instructions are adapted from [Hedronvision's bazel-compile-commands-extractor](https://github.com/hedronvision/bazel-compile-commands-extractor) project.
