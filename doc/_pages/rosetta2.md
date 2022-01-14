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

``arch`` is the "magic Rosetta command" that does two things: reports the
current "arch environment" (no arguments) and selects which version of a
multi-arch binary (e.g., ``/bin/bash``, ``/usr/bin/python``) to run.

When not run via arch, macOS defaults to running in x86_64 mode if any parent
process is an x86_64 process. For example, running ``/opt/homebrew/bin/bash``
once one is "in" x86_64 mode won't switch back to ARM mode.  Also, once one ha
``/usr/local/bin/bash``, it is sufficient to just run that (no arch required).

Bottom line, once arch reports "i386", it is difficult to accidentally do
anything in ARM mode, with the exception of running ``/opt/homebrew/bin/brew``,
as brew seems to hard-code what arch it wants to be.
