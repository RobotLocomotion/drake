---
title: Model Version Control
---


# Adding Model Artifacts in Pull Requests

Model artifacts have the potential to be very large, and we should avoid
committing large files directly to Git.

If your model's files are small enough (<100KB in a given directory), then you
may add them directly to Git in a PR to master.

Otherwise, you should add the large files to
[RobotLocomotion/models](https://github.com/RobotLocomotion/models).

Before you decide to submit models, please ensure that you have tests that
will need them. Do not submit a PR that adds models but has zero intent to use
them, as Drake is not a model repository.

See below for the suggested workflow.

## Develop Changes Locally

1. Clone ``RobotLocomotion/models`` locally.
2. Create a Git branch in your local checkouts of *both* ``models`` and
   ``drake``.
3. Update ``drake/tools/workspace/drake_models/repository.bzl`` to point to your
   ``models`` checkout using
   ``github_archive(..., local_repository_override = <path>)``.
4. Incorporate the new ``models`` files into a Drake ``BUILD.bazel`` file where
   they are needed by adding ``data = ["@drake_models//:some_subdir"]``
   to the demo program or benchmark that uses them.
   1. Do not ask the build system to "install" any files from ``drake_models``;
      they are already available via remote fetching. In particular, don't add
      any ``install()`` rule for them, nor add them to any ``filegroup()`` that
      is already being installed.
5. Ensure your tests pass under ``bazel test``.

## Submit Changes in a Pull Request

1. Push your changes to your fork of ``RobotLocomotion/models``. Make a PR.
2. Update ``drake/tools/workspace/drake_models/repository.bzl`` to use the
   commit you pushed.
3. Submit a PR to Drake, and add a self-blocking discussion thread, such as
   ``"Working temporary SHA1 until the models PR <LINK> is merged."``,
   where ``<LINK>`` references your ``models`` PR.
4. Get review on your Drake PR first. Once it is generally approved, then
   request review for your ``models`` PR.
5. Once both PRs are approved:
   1. Merge your ``models`` PR.
   2. Update ``drake/tools/workspace/drake_models/repository.bzl`` to the latest
      merge commit on ``master`` for ``RobotLocomotion/models``.
   3. Merge your ``drake`` PR.
