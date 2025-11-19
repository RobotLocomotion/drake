---
title: Tools for Code Style Compliance
---

This section provides a list of tools that some have found useful for ensuring
their code abides by [Drake's Code Style Guide](/code_style_guide.html).
The list is by no means comprehensive.
If your favorite tools or methodologies are not listed, we would be delighted
to learn about them. Please document your trick and submit a pull request!


# Automated style checks

Code style tests are run by default during ``bazel test`` and the results are
cached so that only edited files are re-checked.  In other words, no special
action is required by a developer.

However, you may still invoke code style checks directly if desired, as
follows:

```
cd /path/to/drake
bazel test --config lint //...         # Only run style checks; don't build or test anything else.
bazel test --config lint //common/...  # Check common/ and its child subdirectories.
```

User manuals for the style-checking tools are as follows:

* C/C++: See the cpplint ``USAGE`` string at
  [https://github.com/cpplint/cpplint/blob/develop/cpplint.py](https://github.com/cpplint/cpplint/blob/develop/cpplint.py).
  * In particular, note the ``// NOLINT(foo/bar)`` syntax to disable a warning.
* Python: Drake uses `ruff` both for linting (`check` mode) and auto-formatting
  (`format --check` mode). See the `ruff` manual at
  [https://docs.astral.sh/ruff/linter/#error-suppression](https://docs.astral.sh/ruff/linter/#error-suppression)
  and
  [https://docs.astral.sh/ruff/formatter/#format-suppression](https://docs.astral.sh/ruff/formatter/#format-suppression).
  * The syntax ``# noqa`` can be used to quiet the warning about an overly-long
    line.
* Bazel: Uses the buildifier tool as described in
  [Updating BUILD files](/bazel.html#updating-build-files).

To opt-out of all linting (e.g., when committing vendored copies of third-party
external files into Drake's workspace), add `tags = ["nolint"]` to the
`BUILD.bazel` rule(s) for the copied code.

# Manual style fixups

## C/C++: Clang-Format

To run ``clang-format``:

```
cd drake
bazel run //tools/lint:clang-format -- -i -style=file [file name]
```

Using ``clang-format`` will modify the entire file that is specified.

### IDE integration

Most IDEs can run ``clang-format`` automatically.
We have some tips for specific IDEs:

* [CLion](/clion.html#formatting-files)
* [Emacs](/emacs.html#c-code-formatting)
* [VS Code](/vscode.html#c-code-formatting)

## Python: Ruff Format

To run ``ruff`` auto-formatter:

```
cd drake
bazel run -- //tools/lint:ruff format path/to/my_file.py
```
