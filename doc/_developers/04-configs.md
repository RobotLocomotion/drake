---
title: Supported Configurations
slug: supported-configurations
links: 
    - title: Configuration Management Non-Determinism 
      slug: configuration-management-non-determination
    - title: Binary Packages
      slug: binary-packages

---

The following table shows the configurations and platforms that Drake
officially supports. Supported configurations are tested in continuous
integration. All other configurations are provided on a best-effort basis.

Drake requires a compiler running in C++17 mode.

| Operating System     | Bazel         | CMake | C/C++ Compiler         | Java | Python |
|----------------------|---------------|-------|------------------------|------|--------|
| Ubuntu 18.04 LTS (Bionic Beaver) | 3.7 | 3.10 | GCC 7.5 (default) Clang 9 | OpenJDK 11 | 3.6 |
| Ubuntu 20.04 LTS (Focal Fossa) |  | 3.16 | GCC 9.3 (default) Clang 9 | | 3.8 |  
| macOS Catalina (10.15) |  | 3.18 | Apple LLVM 12.0.0 (Xcode 12.2) | AdoptOpenJDK 15 (HotSpot JVM) | | 

CPython is the only Python implementation supported. On Ubuntu, amd64
(i.e., x86_64) is the only supported architecture. On macOS, x86_64 is the only
supported architecture and running Drake under Rosetta 2 emulation on arm64 is
not supported. Plans for any future arm64 support on macOS and/or Ubuntu are 
discussed in [issue #13514](https://github.com/RobotLocomotion/drake/issues/13514).


#### {{page.links[0].title}} {#{{page.links[0].slug}}}

The indicated versions for build systems and languages are recorded after
having been tested on Continuous Integration.

Due to how the Debian ``apt`` and Homebrew package managers work, you may not
have these exact versions on your system when (re)running
``install_prereqs.sh``. In general, later minor versions for more stable
packages (e.g. CMake, compilers) should not prove to be too much of an issue.

For less stable packages, such as Bazel, later minor versions may cause
breakages. If you are on Ubuntu, please rerun ``install_prereqs.sh`` as it can
downgrade Bazel. If on Mac, there is no easy mechanism to downgrade with
Homebrew; however, we generally try to stay on top of Bazel versions.

If you have tried and are unable to configure your system by
[following the instructions](/from_source.html), please do not hesitate
to [ask for help](/getting_help.html).


#### {{page.links[1].title}} {#{{page.links[1].slug}}}

The binary releases of Drake are built with GCC 7.5 on Ubuntu 18.04 (Bionic),
GCC 9.3 on Ubuntu 20.04 (Focal), and Apple LLVM 12.0.0 on macOS Catalina
(10.15).

The links for these packages are listed in [Binary installation (macOS, Ubuntu)](/from_binary.html).