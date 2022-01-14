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
1. Make sure there is not an ARM version of brew
   (i.e., ``/opt/homebrew/bin/brew``) in your ``$PATH``.
   * You can use ``echo $PATH`` to see what's there.
1. Install Homebrew as usual from the "x86" shell.
1. Install Drake as usual from the "x86" shell.
