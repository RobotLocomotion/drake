---
title: Introduction
slug: introduction
---

If you have improvements to Drake, send us your pull requests!

Our standard workflow is to fork [Drake's official GitHub repository](
https://github.com/RobotLocomotion/drake/) into your
own GitHub account and then push your changes into a branch on your fork. Once
you believe your code is ready to be merged into Drake's primary repository,
open a [pull request](https://help.github.com/articles/using-pull-requests/)
via the GitHub website. Your code will then undergo an interactive review
process and [Continuous Integration (CI)](/developers.html#continuous-integration-notes)
tests before it is merged into
[Drake's primary repository](https://github.com/RobotLocomotion/drake).

Drake's [CI service](/developers.html#continuous-integration-notes) runs on all pull requests each time they are
submitted and updated. Pull requests cannot be merged into master unless all
unit tests pass on all
[supported platform configurations](/developers.html#supported-configurations).

Drake's CI server also runs continuously on
[Drake’s primary master branch](https://github.com/RobotLocomotion/drake)
using an even more comprehensive set of unit tests.
If problems are detected on this branch, the build cop will
[revert the PRs that most likely caused the problem](/buildcop.html).
To increase the likelihood that your pull requests pass CI tests and are not
reverted, you can run the unit tests locally. Instructions for how to do that
are provided [here](/unit_testing_instructions.html). Note, however, that there are
many computationally-demanding tests and running the entire test suite can take
several hours depending on your machine.

We would like to hear about your success stories if you've used Drake in your
own projects.  Please consider contributing to our [Drake Gallery](/gallery.html) by editing
``doc/gallery.rst`` and submitting a pull request with the update!
