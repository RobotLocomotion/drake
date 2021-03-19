---
title: Source installation (macOS, Ubuntu)
---

# Getting Drake

We recommend that you [setup SSH access to GitHub.com](https://help.github.com/articles/adding-a-new-ssh-key-to-your-github-account/)
to avoid needing to type your password each time you access it. The following
instructions assume you have uploaded your public SSH key to your GitHub
account.

Now run:

```
git clone git@github.com:RobotLocomotion/drake.git
```

Note: we suggest you keep the default clone directory name (``drake``) and not
rename it (such as ``drake2``).  The CLion integration will suffer if the
checkout directory is not named ``drake``.  (See [CLion IDE setup](clion.html) for details.)

Note: the build process may encounter problems if you have unusual characters
like parentheses in the absolute path to the drake directory
(see [#394](https://github.com/RobotLocomotion/drake/issues/394)).

The above ``git clone`` command will configure Drake's primary repository as a
remote called ``origin``. We recommend that you configure your fork of Drake's
primary repository as the ``origin`` remote and Drake's primary repository as
the ``upstream`` remote. This can be done by executing the following commands:

```
cd drake
git remote set-url origin git@github.com:[your github user name]/drake.git
git remote add upstream git@github.com:RobotLocomotion/drake.git
git remote set-url --push upstream no_push
```

# Mandatory platform specific instructions

Before running the build, you must follow some one-time platform-specific
setup steps:

* [macOS](/mac.html)
* [Ubuntu](/ubuntu.html)

See [supported configurations](/developers.html#supported-configurations)
for the configurations and platforms that Drake officially supports.
All else being equal, we would recommend developers use Ubuntu Bionic.

# Build with Bazel

For instructions, jump to [Using Bazel](/bazel.html#developing-drake-using-bazel), or check out the
full details at:

* [Bazel build system](/bazel.html)

# Historical Note

Older releases were built around substantial MATLAB support, and are
described on [this release notes page](/release_notes/older_releases.html).
