---
title: GitHub PR Interaction with Jenkins
---

When a new pull request is opened in the project and the author of the pull
request is not a member of the RobotLocomotion GitHub organization, the Jenkins
GitHub Pull Request Builder @drake-jenkins-bot will not automatically schedule
builds.

To allow the pull request to be tested, a member of the RobotLocomotion
organization may comment:

* ``@drake-jenkins-bot ok to test`` to accept this pull request for testing.
* ``@drake-jenkins-bot test this please`` for a one time test run.

If the build fails for other various reasons you can rebuild:

* ``@drake-jenkins-bot retest this please`` to start a new build.

You can also view the [Jenkins UI](https://drake-jenkins.csail.mit.edu/)
directly.

# Rebuilding via Reviewable

When posting a ``@drake-jenkins-bot ... please`` comment in Reviewable,
never use the large green "Publish" button in the upper right corner.

Instead, write the bot comment in the "Review discussion" box immediately below
the "File Matrix" widget **and** use the "single message send" button to post
it, in the lower-right corner of the "Review discussion" box.

![Jenkins Bot Reviewable Comment](/images/jenkins_bot_reviewable_comment.png)

(For details, see
[Reviewable#576](https://github.com/Reviewable/Reviewable/issues/576).)

# Scheduling an On-Demand Build

There are a number of Jenkins builds that do not normally run pre-merge, but do
run post-merge, nightly, or weekly. These builds include lower-priority
platforms (e.g., macOS), and specialized options (e.g.,
[UndefinedBehaviorSanitizer](https://releases.llvm.org/6.0.0/tools/clang/docs/UndefinedBehaviorSanitizer.html)).
Members of the RobotLocomotion organization can manually schedule these builds
on pull requests that have not yet been merged, or on arbitrary commits in the
``RobotLocomotion/drake`` repository.

To schedule a build of an open pull request merged with master, comment:

* ``@drake-jenkins-bot <job-name> please``

where ``<job-name>`` is the name of an
[experimental job](https://drake-jenkins.csail.mit.edu/view/Experimental/).

For example:

* ``@drake-jenkins-bot mac-big-sur-clang-bazel-experimental-release please``
* ``@drake-jenkins-bot linux-bionic-clang-bazel-experimental-valgrind-memcheck please``

## Scheduling Builds via the Jenkins User Interface

Alternatively, to schedule a build of an open pull request or arbitrary commit
in the ``RobotLocomotion/drake`` repository:

1. **Log in** to [Jenkins](https://drake-jenkins.csail.mit.edu/) using GitHub OAuth.
   (Make sure that you see your name the upper-right corner, *not* the words "Log in".)
2. Go to the list of [experimental builds](https://drake-jenkins.csail.mit.edu/view/Experimental/).
3. Click on the specific build you want to schedule.
4. Click on "Build with Parameters" in the left menu.
5. Enter ``pr/XYZ/head`` (HEAD of pull request), ``pr/XYZ/merge`` (pull request
   merged with master), or the desired commit SHA in the ``sha1`` field.
6. Click ``Build``.

The list of experimental builds includes builds that automatically run on opened
and updated pull requests, as well as numerous other builds for on-demand use.
To help identify the on-demand build you want to run, you can consult the lists
of [continuous](https://drake-jenkins.csail.mit.edu/view/Continuous/),
[nightly](https://drake-jenkins.csail.mit.edu/view/Nightly/), and
[weekly](https://drake-jenkins.csail.mit.edu/view/Weekly/) builds,
but you should not schedule continuous, nightly, or weekly builds directly.

## Updating Installation Prerequisites

Installation prerequisites are packages that are not pulled in Bazel, but
instead installed on the OS itself using a package manager like ``apt``,
Homebrew, or ``pip`` (only on Mac). They are installed via the scripts under
``setup/``, and are split between ``binary_distribution`` (dependencies that
are necessary for [binary installation](/installation.html)) and
``source_distribution`` (dependencies, in addition to those in
``binary_distribution``, necessary for
[source installation](/from_source.html)). Since
``source_distribution`` will also install prerequisites in
``binary_distribution``, you do not need to duplicate binary prerequisites in
``source_distribution``.

Prerequisites of the ``source_distribution`` are further split into three
parts: those that are needed for building and running the ``//:install`` target
using ``bazel`` (``bazel run //:install``), those additional dependencies for
building and running tests (``bazel test ...``), and those additional
dependencies for running select maintainer scripts (e.g., ``mirror_to_s3.py``
and ``new_release.py``). Again, it is expected that a given prerequisite will
only appear in one of these lists.

When updating prerequisites with these scripts, the normal experimental CI will
most likely fail. To test new prerequisites, you should first request
unprovisioned experimental builds, e.g.:

* ``@drake-jenkins-bot linux-bionic-unprovisioned-gcc-bazel-experimental-release please``
* ``@drake-jenkins-bot linux-focal-unprovisioned-gcc-bazel-experimental-release please``
* ``@drake-jenkins-bot mac-big-sur-unprovisioned-clang-bazel-experimental-release please``

After this has passed, go through normal review. Once normal review is done,
add `@BetsyMcPhail` for review and request that the provisioned instances be
updated. She will then respond on when it is appropriate to merge the PR.

## Building Binary Packages on Demand

To schedule an "experimental" build of the [binary packages](/from_binary.html),
comment on an open pull request as follows:

* ``@drake-jenkins-bot linux-bionic-unprovisioned-gcc-bazel-experimental-snopt-packaging please``
* ``@drake-jenkins-bot linux-focal-unprovisioned-gcc-bazel-experimental-snopt-packaging please``
* ``@drake-jenkins-bot mac-big-sur-unprovisioned-clang-bazel-experimental-snopt-packaging please``

or follow the [instructions above](#scheduling-builds-via-the-jenkins-user-interface)
to schedule a build of one of the following jobs from the Jenkins user
interface:

* linux-bionic-unprovisioned-gcc-bazel-experimental-snopt-packaging
* linux-focal-unprovisioned-gcc-bazel-experimental-snopt-packaging
* mac-big-sur-unprovisioned-clang-bazel-experimental-snopt-packaging

The URL from which to download the built package will be indicated in the
Jenkins console log for the completed build, for example:

```
-- Uploading package archive 1 of 1 to AWS S3...

upload: drake-<yyymmddhhmmss>-<commit>-<platform>.tar.gz to s3://drake-packages/drake/experimental/drake-<yyymmddhhmmss>-<commit>-<platform>.tar.gz
-- Package URL 1 of 1: https://drake-packages.csail.mit.edu/drake/experimental/drake-<yyymmddhhmmss>-<commit>-<platform>.tar.gz
-- Uploading package archive checksum 1 of 1 to AWS S3...

upload: drake-<yyymmddhhmmss>-<commit>-<platform>.tar.gz.sha512 to s3://drake-packages/drake/experimental/drake-<yyymmddhhmmss>-<commit>-<platform>.tar.gz.sha512
```

where ``<platform>`` is ``bionic``, ``focal``, or ``mac``.
