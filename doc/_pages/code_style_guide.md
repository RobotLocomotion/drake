---
title: Code Style Guide
---

This section defines a style guide which should be followed by all code that is
written in Drake. Being consistent with this style will make the code easier to
read, debug, and maintain. To ensure your code is style compliant, consider
using [tools for complying with coding style](/code_style_tools.html).

See also the brief
[Code Review Checklist](/code_review_checklist.html),
where a list of the most frequent problems are collected.

Note: Many of the files in the repository were written before this style guide,
or did not follow it precisely.  If you find style errors, go ahead and change
it and submit a pull request.

# C++ Style

The Drake C++ style guide (which is derived from the Google C++ style guide)
can be found
[on its own page](https://drake.mit.edu/styleguide/cppguide.html).
Its source lives in
[the styleguide repository](https://github.com/RobotLocomotion/styleguide).

# Python Style

Drake strictly follows [PEP 8 -- Style Guide for Python Code](
https://www.python.org/dev/peps/pep-0008/) except for the specific
clarifications, exceptions, and additional rules noted below. Since PEP 8
incorporates [PEP 257 -- Docstring Conventions](
https://www.python.org/dev/peps/pep-0257/), Drake follows its
recommendations as well.

Drake also follows the
[RobotLocomotion fork](https://drake.mit.edu/styleguide/pyguide.html)
of the Google Python Style Guide. Please refer to that page for the sections
that are adhered to, and the relevant exceptions.

See [tools for complying with coding style](/code_style_tools.html) for details
about the automated style checks.

{% comment %}
TODO(eric.cousineau): Move these clarifications and exceptions to styleguide
   repo.
{% endcomment %}

## Clarifications

* External, third-party, and auto-generated source files are not to be checked
  for style.
* Always prefer long, human-readable variable/method/class names to short
  acronyms.

## Exceptions

* Lines containing a long URL may be longer than 80 columns if necessary to
  avoid splitting the URL.

## Additional Rules

* When importing in-tree modules, always use absolute import paths; explicit
  relative import paths are disallowed. See the [PEP 8 discussion of imports](
  https://www.python.org/dev/peps/pep-0008/#imports) for more detail.
* Sometimes ``__init__.py`` files are necessary, for Python's ``import``
  mechanism; these files should be non-empty (via a copyright notice, for
  example). Rationale: 0-byte files can be mistakenly perceived as the result
  of some error or accident.
* When using the ``logging`` module, avoid its lazy-formatting
  syntax. Rationale: exceptions raised in lazy formatting get printed to
  ``stderr``, but are otherwise ignored, and thus may escape notice.
* Executable Python files should be limited to *only* scripts which are not run
  via Bazel-generated Python proxy scripts. (The proxy scripts are those
  run via ``bazel run``, ``bazel test``, or ``./bazel-bin/...``.)
  If a script qualifies, use the following "shebang" line:<br/>
  ``#!/usr/bin/env python3``<br/>
  Rationale: ``/usr/bin/env`` enables a ``PATH`` search for the Python 3
  executable. This is also recommended by
  [PEP 394](https://www.python.org/dev/peps/pep-0394/).

# Java Style

We also strictly follow the [Google Java Style Guide](
https://google.github.io/styleguide/javaguide.html).
Here are some additional comments:

* Every class and method should have a brief `_javadoc_` associated with it.
* All Java classes should be in packages relative to the Drake root,
   e.g.: package drake.examples.Pendulum

# LCM Style

* LCM types are under_scored with a leading `lcmt_` added. If the type is
  specific to a particular robot, then it begins with `lcmt_robotname_`.
* Variable names in LCM types follow the rules above.

# Shell Script Style

We follow the [Google Shell Style Guide](
https://google.github.io/styleguide/shell.xml).

# Git Commit Message Style

For commit messages we require:

* An empty line between the first line and any subsequent lines. (For
  further insight, see
  [git's commit discussion](https://git-scm.com/docs/git-commit#_discussion).)
