.. model_version_control:

*********************
Model Version Control
*********************

Adding Model Artifacts in Pull Requests
=======================================

If you are adding model artifacts, such as textures or meshes, that are small
enough (<100KB), and there are <10MB of such files in the source tree at any
time, then add them directly to Git.

Otherwise, you should add your models to
`RobotLocomotion/models <https://github.com/RobotLocomotion/models>`_.

These artifacts SHOULD be things that are large and should change infrequently,
e.g. ``*.obj``, ``*.stl``, ``*.png``.

These should NOT be things that are small, e.g. ``*.sdf``, ``*.urdf``.

See below for the suggested workflow.

Develop Changes Locally
-----------------------

#. Clone ``RobotLocomotion/models`` locally
#. Update ``tools/workspace/models/repository.bzl`` to point to your checkout
   using ``github_archive(..., local_repository_override = <path>)``.
#. Update ``tools/workspace/models/files.bzl`` to incorporate the models you
   want.
#. Ensure that you use ``forward_files`` to make the files available inside
   the Drake bazel workspace. For an example, see
   ``manipulation/models/ycb/BUILD.bazel``.
#. Write your unittest that uses these files. Do NOT submit PRs that only adds
   models but has zero intent to use them; Drake is not a model repository!

Submit Changes in a Pull Request
--------------------------------

#. Push your changes to your fork of ``RobotLocomotion/models``. Make a PR.
#. Update ``.../models/repository.bzl`` to use the commit you pushed.
#. Submit a PR to Drake, label it as ``do not merge``. Reference your ``models``
   PR in this PR.
#. Get review on your Drake PR first. Once it is generally approved, then
   request review for your ``models`` PR.
#. Once both PRs are approved:

   #) Merge your ``models`` PR.
   #) Update Drake's ``.../models/repository.bzl`` to the latest merge commit on
      ``master`` for ``RobotLocomotion/models``.
   #) Ensure your PR fully passes CI, then have your PR merged.
