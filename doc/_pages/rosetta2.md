---
title: Running Drake on macOS ARM hardware via Rosetta 2
---

# Instructions

<div class="warning" markdown="1">
**The advice on this page is no longer necessary as of current Drake.**

For older versions of Drake, users running on Apple's newer arm64 hardware were
required to run Drake's x86_64 binaries using Apple's Rosetta 2 emulation. That
is no longer the case as of v1.9.0. The page serves as a historical record for
how to run Drake x86_64 under Rosetta 2 emulation. Most users should ignore it
and follow one of the typical [installation](/installation.html) instructions
instead.
</div>

To run macOS x86_64 pre-compiled Drake binaries on Apple's newer arm64 hardware,
you must do all of your work in an x86_64 shell:

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
