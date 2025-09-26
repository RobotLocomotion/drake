---
title: Emacs IDE setup
---

This guide describes how to edit the Drake codebase using Emacs.

If you have tips that would help out other developers, drop us a line and we
can add them here!

# Setting up Emacs

There is nothing really special to do.
On Ubuntu ``sudo apt install emacs``.

## Bazel

We recommend [emacs-bazel-mode](https://github.com/bazelbuild/emacs-bazel-mode).
If you have MELPA configured, use ``M-x package-install bazel``.

## Git

We recommend ``magit``.
On Ubuntu ``sudo apt install elpa-magit``.

## C++ code formatting

Use ``(require 'clang-format)`` to enable the ``M-x clang-format-...`` family of
functions. Also check that the customize variable ``clang-format-executable`` is
set to Drake's preferred value
``/path/to/drake/bazel-bin/tools/lint/clang-format``.

