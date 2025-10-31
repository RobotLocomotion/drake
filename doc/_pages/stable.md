---
title: Drake Stability Guidelines
---

We want to move fast and bring out exciting new ideas quickly. We also
appreciate users' need for a solid, stable foundation to build on. This
document explains what kind of stability the Drake project aims to provide,
and what you can do to minimize the impact of the inevitable changes to Drake
on your own work.

# Rolling releases

Drake ships a new stable version approximately once per month. The versions are
numbered "1.x.y", where "x" increases by one for each stable version, and "y"
reflects the patch version (typically zero). Drake does **not** follow
[SemVer](https://semver.org/). Instead, we use a rolling deprecation window.
In each release, new deprecations might be announced, and then those changes
will be finalized 3+ minor versions later, without changing the major version
number.

Deprecation announcements appear in the
[release notes](/release_notes/release_notes.html) for each stable release.
For C++ and Python API changes, we will also use the language mechanism to
highlight the change when possible (`[[deprecated]]` for C++; `warnings.warn`
for Python). However, those mechanisms will not always trigger in every case,
and some deprecations will reach beyond what those tools can denote.
Therefore, we still recommend that you monitor the release notes as the final
arbiter of deprecation announcements.

Due to our 3-month announcement window, we recommend upgrading your pinned
version of Drake at least that frequently. If you wish to upgrade less
frequently, you still might benefit from rolling forward a few minor versions
at a time, in order to obtain the built-in C++ and Python deprecation
suggestions to guide you.

In general, we do not plan to backport fixes into prior stable releases.

# Stable API

We define a large portion of Drake as our "Stable API" that you can rely
on. For the Stable API, we aim to give at least 3 months of notice ahead of any
disruption (via deprecation announcements in Drake's
[release notes](/release_notes/release_notes.html), and with compile-time or
run-time warnings when feasible). Any element of Drake that is not explicitly
documented to be part of the Stable API is deemed "unstable" and is subject to
change or removal without prior notice.

The sub-headings below explain the Stable API for various facets of Drake.

Specific code or tools beyond what is listed here can also opt-in to be part of
the Stable API in its local documentation. Those opt-ins are not enumerated
here.

On very rare occasions it is impractical, too expensive, or logically unsound
to meet the full three month deprecation window. In these cases the change will
be announced in the "Breaking changes" section of the release notes.

## C++

The C++ Stable API covers all C++ library code within `namespace drake` whose
header files are distributed in a
[Drake pre-compiled binary image](/from_binary.html#stable-releases).

* Excluding the `drake/examples/...` directory tree.
* Excluding code within `namespace internal`.
* Excluding code documented as "internal use only".
* Excluding code documented with an "experimental" warning.

In particular, note that any
[directory named "dev"](https://drake.mit.edu/directory_structure.html)
is necessarily excluded, because its header files are not installed.

## Python

The Python Stable API covers all Python library code under `import drake` or
`import pydrake` that is distributed in a
[Drake PyPI wheel](/pip.html#stable-releases):

* Excluding the `pydrake.examples...` module tree.
* Excluding names that begin with a leading underscore, which denotes "internal
  use only" by convention of
  [PEP-8](https://www.python.org/dev/peps/pep-0008/#descriptive-naming-styles).
* Excluding code documented as "internal use only".
* Excluding code documented with an "experimental" warning.
* Excluding the "all" modules such as `pydrake.all`, `pydrake.systems.all`,
  etc. These are intended to be shortcuts for temporary hacking only, and so
  are not Stable.
  * However, the existence of the `pydrake.all` module is part of the Stable
    API (we will not remove it without notice), but the specific list of items
    contained within it is not Stable.

## Build system

The Bazel Stable API covers all Drake Bazel targets (e.g., `@drake//common`)
with public visibility that refer to C++ or Python library code that is itself
part of the "Stable API":

* This **includes**
  <a href="https://docs.bazel.build/versions/main/build-ref.html#dependencies">depending on</a>
  those labels (e.g., `deps = ["@drake//common"]`).
* This **excludes**
  <a href="https://docs.bazel.build/versions/main/build-ref.html#load">loading</a>
  Drake's bzl macros (e.g., `load("@drake//foo:bar.bzl", "bar_macro")`.

Additionally, Drake's Bazel configuration flags at `@drake//tools/flags:*` are
part of the Stable API.

For Drake's dependencies:

* The extension module
  `use_extension("@drake//tools/workspace:default.bzl", "drake_dep_repositories")`
  is part of the Stable API, including the names of the repositories it offers
  as extensions (e.g., `"eigen"`).
  * For any repository provided by the extension, we will deprecate it prior to
    removing it.

We may upgrade any of our dependencies to a newer version without prior notice.
If you require an older version, you will need to rebuild Drake from source and
customize your own `MODULE.bazel` file to refer to the older version of the
dependency.

We may add new dependencies without prior notice. All of our dependencies will
either be installed via the host system via our `install_prereqs` scripts,
and/or downloaded at build-time via our `add_default_...` macros (when not
using `bzlmod`) or our `MODULE.bazel` file (when using bzlmod), and/or
specified via packaging metadata in the case of `apt` or `pip`.

## LCM messages

Our Stable API covers all of Drake's
[LCM message definitions](https://github.com/RobotLocomotion/drake/tree/master/lcmtypes):

* Excluding messages with "internal" as part of the message name.
* Excluding messages with "experimental" as part of the message name.
* Excluding messages documented as "internal use only".
* Excluding messages documented as "experimental".

Regarding changes to message definitions:

* LCM messages provide no mechanism for schema evolution or versioning.
* If we need to change a message definition (e.g., `lcmt_schunk_wsg_command`),
  we will fork it into a newly-named `lcmt_schunk_wsg_command2` with the
  change, and deprecate the original `lcmt_schunk_wsg_command`.
* Drake code that uses that message type (e.g., `SchunkWsgCommandSender`) will
  immediately switch to use the revised message, without deprecation.
* This means that independent, direct use of the LCM message by users will have
  a smooth transition like any other library code, but any dependency on
  Drake's sending or receiving a particular schema is not part of our "Stable
  API" promise.

## Behavioral changes

Even if Drake does not change its API, sometimes its implementation may evolve
in ways that affect your use. In the limit,
[any implementation change can be a breaking change](https://xkcd.com/1172/).

Our goal is to keep our implementation within its documented API promises
([C++](https://drake.mit.edu/doxygen_cxx/),
[Python](https://drake.mit.edu/pydrake/)).
Any significant changes to the API documentation should be announced via
deprecation with 3 months of advance notice.

Sometimes our software will choose a default value when nothing more specific
is provided. Those defaults may change as we find better heuristics, without
prior notice. Similarly, our numerical algorithms may change in ways that alter
the calculated results. There is no guarantee that Drake's output is
bitwise-identical from one version to the next.

## Exemptions

Any new API will have these requirements waived for the first version where
that API appears. (In other words, the second release of the API is the one
where the Stable guarantees come into effect.) However, we will make a best
effort to adhere to these guidelines as we resolve any issues with the new API,
even in the first release.

Drake's nightly builds are exempt from any stability guarantees.

Within the Stable API, we also expect you to meet certain requirements in order
to obtain the stability guarantee:

* Include what you use.
  * Drake's `#include` graph will change over time; always `#include` the
    header that matches each Drake class that you use.
  * Relatedly, do not include headers that you don't use. Our C++ or Python
    deprecation warnings typically only trigger on **call** into the API, not
    mere inclusion or importing.
* For Bazel users, `deps` what you use.
  * Drake's `deps` graph will change over time; always list out your Drake
  library dependencies (to match what you `#include` or `import`); do not rely
  on transitive dependencies to bring them in.
* In C++, do not forward-declare anything from Drake.
* In C++, do not depend on the exact signature of Drake functions, as we may
  add new, defaulted arguments without prior notice (i.e., do not take the
  address of any Drake function, or assign it to a `std::function`).
* In C++, do not depend on the exact order of ``struct`` fields.
  While the _relative_ order of struct fields is guaranteed, new member fields
  might be inserted between existing fields. Consequently, when using
  [aggregate initialization](https://en.cppreference.com/w/cpp/language/aggregate_initialization)
  always use designated initializers (i.e., field names) when referring to
  struct fields.

### Model files

Note in particular that any data files provided by Drake (e.g., SDFormat or
URDF models) are not incorporated into the "Stable API", even though some of
those files are distributed in our stable releases for demonstration
purposes. We may alter the models (e.g., changing the collision geometry or
joint limits) or remove a model entirely, without prior notice.

# ABI

Drake does not offer any stable ABI
([Application Binary Interface](https://en.wikipedia.org/wiki/Application_binary_interface)).
We expect any C++ code linked against Drake within the same program to use the
identical source revision and build flags of Drake as all other code within
that program.

Refer to [Installation and Quickstart](/installation.html) for the current
details.

# OS Support

Drake intends to support the two most recent versions of Ubuntu LTS and macOS
on an ongoing basis. That generally means that your OS must be no more than
~2-4 years old for Ubuntu, or ~2 years old for macOS.

Refer to [Installation and Quickstart](/installation.html) for the current
details, and [End of support releases](/release_notes/end_of_support.html)
for historical details.

# Python support

On Ubuntu when installing from PyPI, Drake intends to support many versions of
Python, up to and including the most recent version at the time of our release.
The *oldest* version of Python we intend to support is the penultimate Ubuntu
LTS's default Python version at the time of our release. This is consistent with
[NEP-29](https://numpy.org/neps/nep-0029-deprecation_policy.html). We hope that
our `manylinux` wheels are able to install and run on non-Ubuntu Linux, but it
is not tested (and thus, unsupported).

On Ubuntu when installing a binary package (`*.tar.gz`), Drake intends to
support only Ubuntu's default version of Python (at ``/usr/bin/python3``).

On macOS when installing from PyPI, Drake intends to support the most recent two
versions Python at the time of our release. This range is shorter than NEP-29's
recommended window, but is the best we can do for now. (This may improve once
[#23683](https://github.com/RobotLocomotion/drake/issues/23683) is finished.)

On macOS when installing a binary package (`*.tar.gz`), Drake intends to
support only Homebrew's newest version of Python at the time of our release.

Refer to [Installation and Quickstart](/installation.html) for the current
details, and [End of support releases](/release_notes/end_of_support.html)
for historical details.
