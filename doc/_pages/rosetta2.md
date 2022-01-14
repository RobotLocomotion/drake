---
title: Running Drake on macOS ARM hardware via Rosetta 2
---

# Overview

Building or running Drake directly on arm64 is not yet supported; plans
for any future arm64 support on macOS and/or Ubuntu are discussed in
[issue #13514](https://github.com/RobotLocomotion/drake/issues/13514).

In the meantime, running Drake using Apple's x86_64 emulation should work.

# Instructions

To use Drake on Apple's newer ARM hardware, **you must do all of your work in
an x86_64 shell**:

1. Use ``arch -x86_64 /bin/bash`` to obtain a shell.
1. In that shell, run ``arch`` (with no arguments).
   * Expect to see ``i386`` as the output, which confirms that the shell is
     running as x86_64.
1. Make sure there is not an ARM version of brew
   (i.e., ``/opt/homebrew/bin/brew``) in your ``$PATH``.
   * You can use ``echo $PATH`` to see what's there.
   * You can use ``which -a brew`` to check which brew will be run.
1. Install Homebrew as usual from the x86_64 shell.
1. Install Drake as usual from the x86_64 shell.

# Additional information

The ``arch`` command serves two purposes; to report the "active" architecture
(when given no arguments), or to run a selected architecture of a universal
binary (e.g., ``/bin/bash``, ``/usr/bin/python``).

While the "active" architecture defaults to ARM, having any x86_64 process as
an ancestor appears sufficient to change the "active" architecture, so long as
``arch -arm64`` is not used. Many programs will respect the reported
architecture, but ``brew`` is an exception, and will always attempt to install
software for the architecture of the ``brew`` executable that is executed; this
is why it is important to ensure that ``/opt/homebrew/bin`` is not in your
``$PATH`` when attempting to install or run Drake using x86_64 emulation on
ARM. Otherwise, once in x86_64 mode, things will tend to stay that way, such
that accidentally switching back to ARM mode should be rare.
