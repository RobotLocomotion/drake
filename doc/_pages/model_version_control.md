---
title: Model Version Control
---


# Adding Model Artifacts in Pull Requests

Model artifacts have the potential to be very large, and we should avoid
committing large files directly to Git.

If your model's files are small enough (<100KB in a given directory), then you
may add them directly to Git in a PR to master.

Otherwise, you should add the large files to
[RobotLocomotion/models](https://github.com/RobotLocomotion/models). Please do
not commit files that are generally small, like ``*.sdf`` or ``*.urdf`` files,
in ``RobotLocomotion/models``; instead, please commit those directly.

Before you decide to submit models, please ensure that you have tests that
will need them. Do not submit a PR that adds models but has zero intent to use
them, as Drake is not a model repository.

See below for the suggested workflow.

## Develop Changes Locally

1. Clone ``RobotLocomotion/models`` locally
2. Create a Git branch in your local checkouts of *both* ``models`` and
   ``drake``.
3. Update ``drake/tools/workspace/models/repository.bzl`` to point to your
   ``models`` checkout using
   ``github_archive(..., local_repository_override = <path>)``.
4. Update ``drake/tools/workspace/models/files.bzl`` to incorporate the models
   you want.
5. Update ``drake/tools/workspace/models/package.BUILD.bazel`` to export the
   models.
6. Ensure that you use ``forward_files`` to make the files available inside
   the Drake bazel workspace. For an example, see
   [drake/manipulation/models/ycb/BUILD.bazel](https://github.com/RobotLocomotion/drake/blob/master/manipulation/models/ycb/BUILD.bazel).
7. Ensure your tests pass under ``bazel test``.

## Submit Changes in a Pull Request

1. Push your changes to your fork of ``RobotLocomotion/models``. Make a PR.
2. Update ``drake/tools/workspace/models/models/repository.bzl`` to use the
   commit you pushed.
3. Submit a PR to Drake, and add a self-blocking discussion thread, such as
   ``"Working temporary SHA1 until the models PR <LINK> is merged."``,
   where ``<LINK>`` references your ``models`` PR.
4. Get review on your Drake PR first. Once it is generally approved, then
   request review for your ``models`` PR.
5. Once both PRs are approved:
   1. Merge your ``models`` PR.
   2. Update ``drake/tools/workspace/models/repository.bzl`` to the latest
      merge commit on ``master`` for ``RobotLocomotion/models``.
   3. Merge your ``drake`` PR.
