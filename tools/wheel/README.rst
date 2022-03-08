Drake Wheels
============

This document briefly explains the process of building Drake wheels, focusing
primarily on the build-wheels script. The process for publishing wheels is
described at https://drake.mit.edu/release_playbook.html.

Basic Usage
-----------

Wheels are built in dedicated Docker containers, such that any host capable of
executing the necessary containers may be used, with no local environment setup
required. However, it is recommended to run the script on the most recent
version of Ubuntu LTS that is supported by Drake.

The script takes a single, required positional argument, which is used to
specify the version with which the wheels should be tagged. The version number
should match the version of Drake being released, and must conform to
`PEP 440 <https://www.python.org/dev/peps/pep-0440/>`_.

The script builds the Drake code which is currently checked out, *including*
any local, uncommitted changes. Most untracked files will also be included.
This simplifies the process of testing changes locally.

On successful completion, the requested set of wheels will be written to the
specified output directory (by default, the current working directory, unless
overridden by ``--output-dir``), unless ``--no-extract`` was specified.

Cleanup
-------

To reclaim disk space used by wheel builds, run the following commands::

  docker rmi $(docker image ls --filter=reference='pip-drake:*' -q)
  docker builder prune -f

Optional Arguments
------------------

Except for ``--test`` (``-t``) and ``--output-dir`` (``-o``), most of the
available options exist for debugging purposes and should not be needed in
ordinary use.

``-t``, ``--test``
    Run some basic tests on the wheels after extracting them. For obvious
    reasons, this is incompatible with ``--no-extract``.

    Tests should always be run prior to publishing wheels. However, while
    debugging build issues, it may be expedient to skip the tests.

``-o``, ``--output-dir``
    Specifies where extracted wheels should be written. The default is the
    current working directory (i.e. ``.``).

``-n``, ``--no-extract``
    Do not extract the wheels after building. This is useful if the goal is
    simply to freshen the tags (``--tag-stages``) or while debugging build
    issues. When used, the script itself will not write to the file system.
    (The Docker cache and tagged images may still be altered.)

``-s``, ``--tag-stages``
    Build each stage independently and, on completion, assign it a permanent
    tag. This serves two functions; first, assignment of permanent tags ensures
    that Docker's cache won't expire early build stages. Second, it allows the
    individual stages contents to be examined, which can be useful for
    debugging, especially if later stages are failing.

``-k``, ``--keep-containers``
    Do not delete intermediate containers in case of a build failure (i.e. do
    not pass ``--force-rm`` to ``docker build``). This has no effect unless
    the build fails. In case of a failed build, the container from the failed
    build step will be left, allowing for post-mortem examination. Don't forget
    to manually delete the container later.

Implementation Details
----------------------

Wheels are built using Docker, in a number of stages. Rather than encoding
everything about the build in the ``Dockerfile``, build instructions are split
out into a number of helper scripts. In some cases, these in turn pull in
additional files to specify things such as build arguments or packages to be
installed. Where possible, earlier steps are designed such that the resulting
images can be used for multiple wheels, in order to save space and reduce build
times.

Starting with a "bare" image, the first step is to provision the environment
by installing common tools and dependencies, starting with those provided by
the operating system. Because wheels must distribute all libraries which they
use aside from a very limited set, most dependencies are built as static
libraries during this process, rather than using system packages. (A small
number of exceptions exist due to licensing reasons.)

These provisioning stages change infrequently. After provisioning, the Drake
sources are injected into the image, resulting in the ``clean`` stage. This
stage is probably the most useful for debugging failing Drake builds. Finally,
Drake's build is executed, followed by the wheel packaging.

When ``--tag-stages`` is not used, the completed build is assigned a temporary
tag, which is used (unless ``--no-extract`` was specified) to extract the
built wheel from the image, and is then untagged.

Testing is also performed using Docker. A different set of provisioning steps
is used to install only the minimal prerequisites in a clean environment,
after which the wheel is added to the image and installed, followed by the
execution of a script which performs some basic tests to ensure that the wheel
was successfully installed.
