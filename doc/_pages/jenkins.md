---
title: Continuous Integration with GitHub Pull Requests
---

# Scheduling an On-Demand Build

There are a number of Jenkins builds that do not normally run pre-merge, but do
run post-merge or nightly. These builds include lower-priority
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

* ``@drake-jenkins-bot mac-arm-sequoia-clang-bazel-experimental-release please``
* ``@drake-jenkins-bot linux-noble-clang-bazel-experimental-valgrind-memcheck please``

A list of Jenkins bot commands for experimental builds that covers the full
set of continuous and nightly production jobs is available
[here](https://github.com/RobotLocomotion/drake/blob/jenkins-jobs-experimental/request-jobs-experimental.txt).
Both provisioned and unprovisioned jobs are listed. A subset of this list which
excludes the jobs that normally run pre-merge is available
[here](https://github.com/RobotLocomotion/drake/blob/jenkins-jobs-experimental/request-jobs-experimental-extra.txt).

To rerun all regular builds on an open pull request (if the previous build(s)
failed for various reasons), comment:

* ``@drake-jenkins-bot retest this please``

**Note:** Immediately after opening a pull request, it can take up to 10
minutes for the branches of all experimental jobs in Jenkins to be created.
Requesting special build flavors with the ``@drake-jenkins-bot ... please``
comments will _not_ work until this occurs. In general, it's safe to wait until
the statuses of all required pre-merge jobs change from "expected" to "pending"
before requesting any additional builds.

## Rebuilding via Reviewable

When posting a ``@drake-jenkins-bot ... please`` comment in Reviewable,
never use the large green "Publish" button in the upper right corner.

Instead, write the bot comment in the "Review discussion" box immediately below
the "File Matrix" widget **and** use the "single message send" button to post
it, in the lower-right corner of the "Review discussion" box.

![Jenkins Bot Reviewable Comment](/images/jenkins_bot_reviewable_comment.png)

(For details, see
[Reviewable#576](https://github.com/Reviewable/Reviewable/issues/576).)

## Scheduling Builds via the Jenkins User Interface

To schedule a build of an open pull request in the ``RobotLocomotion/drake``
repository from the [Jenkins UI](https://drake-jenkins.csail.mit.edu/),

1. **Sign in** to [Jenkins](https://drake-jenkins.csail.mit.edu/) using GitHub
   OAuth. (Make sure that you see a profile picture in the upper-right corner,
   *not* the words "Sign in".)
2. Go to the list of
   [experimental builds](https://drake-jenkins.csail.mit.edu/view/Experimental/).
3. Click on the specific build you want to schedule.
4. Click on "Pull Requests" towards the top, and select your pull request.
5. Click on "Build with Parameters" in the left menu.
6. (Optional) If you need to test your changes alongside a pull request or
   branch of the ``RobotLocomotion/drake-ci`` repository, enter ``pr/XYZ/head``
   (HEAD of pull request), ``pr/XYZ/merge`` (pull request merged with master)
   for ``ciSha``. Otherwise, leave it set to "main."
7. Click ``Build``.

The list of experimental builds includes builds that automatically run on opened
and updated pull requests, as well as numerous other builds for on-demand use.
To help identify the on-demand build you want to run, you can consult the lists
of [continuous](https://drake-jenkins.csail.mit.edu/view/Continuous/), and
[nightly](https://drake-jenkins.csail.mit.edu/view/Nightly/)
but you should not schedule continuous or nightly builds directly.

## Testing Pull Requests from External Contributors

When a new pull request is opened in the project and the author of the pull
request is not a member of the RobotLocomotion GitHub organization, Jenkins
will not automatically schedule builds. To test the pull request, a member
of the RobotLocomotion organization should comment:

* ``@drake-jenkins-bot test this please`` for a one time test run.
* ``@drake-jenkins-bot retest this please`` to start a new build, if the
previous build fails for various reasons.

You can also view the [Jenkins UI](https://drake-jenkins.csail.mit.edu/)
directly.

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
most likely fail. To test new prerequisites on Linux, you should request
unprovisioned experimental builds. A list of Jenkins bot commands for
experimental unprovisioned builds that covers the full set of corresponding
continuous and nightly production jobs (including provisioned) is available
[here](https://github.com/RobotLocomotion/drake/blob/jenkins-jobs-experimental/request-jobs-unprovisioned.txt).

Testing changes to the source distribution prerequisites for macOS is a work
in progress as there are no longer unprovisioned builds.
Contact `@BetsyMcPhail` for guidance on testing these changes.

After this has passed, go through normal review. Once normal review is done,
add `@BetsyMcPhail` for review and request that the provisioned instances be
updated. She will then respond on when it is appropriate to merge the PR.

## Building Packages on Demand

### Binary or Debian

To schedule an "experimental" build of a [binary package](/from_binary.html)
or [debian package](/apt.html), comment on an open pull request using one or
more of these commands:

* ``@drake-jenkins-bot linux-noble-unprovisioned-gcc-cmake-experimental-packaging please``
* ``@drake-jenkins-bot mac-arm-sequoia-clang-cmake-experimental-packaging please``

or follow the [instructions above](#scheduling-builds-via-the-jenkins-user-interface)
to schedule a build of one of the [Packaging](https://drake-jenkins.csail.mit.edu/view/Packaging/)
jobs with **experimental** in its name.

To download the built package, open the Jenkins console log for the completed
build (click on "Details" for a packaging build in the pull request's
list of checks, then "Console Output") and search for the text "Artifacts
uploaded to AWS" to find the download URL (usually about a screen's-worth of
text above the end of the log).  For example:

```
...
[2:30:23 PM]  -- Artifacts uploaded to AWS:
[2:30:23 PM]  https://drake-packages.csail.mit.edu/drake/experimental/drake-0.0.20250530.180720%2Bgit3468349f-noble.tar.gz
[2:30:23 PM]  https://drake-packages.csail.mit.edu/drake/experimental/drake-dev_0.0.20250530.180720%2Bgit3468349f-1_amd64-noble.deb
...
```

(In some cases, it may be necessary to click the "Full Log" and search for the
text "Upload complete", particularly if you wish to also find the checksum
URLs.)

To download the package, simply click the link or use your favorite HTTP
retrieval tool (e.g. ``wget`` or ``curl``).

### Wheel

To schedule an "experimental" build of a [wheel package](/pip.html),
comment on an open pull request using one or more of these commands:

* ``@drake-jenkins-bot linux-noble-unprovisioned-gcc-wheel-experimental-release please``
* ``@drake-jenkins-bot mac-arm-sequoia-clang-wheel-experimental-release please``

or follow the [instructions above](#scheduling-builds-via-the-jenkins-user-interface)
to schedule a build of one of the [Wheel](https://drake-jenkins.csail.mit.edu/view/Wheel/)
jobs with **experimental** in its name.

To download or install the built wheel, open the Jenkins console log for the
completed build (click on "Details" for a wheel build in the pull request's
list of checks, then "Console Output") and search for the text "Artifacts
uploaded to AWS" to find the download URL (usually about a screen's-worth of
text above the end of the log).  For example:

```
...
[2:17:49 PM]  -- Artifacts uploaded to AWS:
[2:17:49 PM]  https://drake-packages.csail.mit.edu/drake/experimental/drake-0.0.20250521.172625%2Bgitbbcde5ab-cp310-cp310-manylinux_2_34_x86_64.whl
[2:17:49 PM]  https://drake-packages.csail.mit.edu/drake/experimental/drake-0.0.20250521.172625%2Bgitbbcde5ab-cp311-cp311-manylinux_2_34_x86_64.whl
[2:17:49 PM]  https://drake-packages.csail.mit.edu/drake/experimental/drake-0.0.20250521.172625%2Bgitbbcde5ab-cp312-cp312-manylinux_2_34_x86_64.whl
[2:17:49 PM]  https://drake-packages.csail.mit.edu/drake/experimental/drake-0.0.20250521.172625%2Bgitbbcde5ab-cp313-cp313-manylinux_2_34_x86_64.whl
...
```

Note that there might be multiple wheel files uploaded for different versions
of Python. Be sure to match the Python ``M.NN`` version you will be using to
the ``-cpMNN-`` substring in the URL.

(In some cases, it may be necessary to click the "Full Log" and search for the
text "Upload complete", particularly if you wish to also find the checksum
URLs.)

To download the wheel, simply click the link or use your favorite HTTP
retrieval tool (e.g. ``wget`` or ``curl``).

Wheels may also be installed locally for testing without downloading the wheel
as a separate step:

```bash
python3 -m venv env
env/bin/pip install --upgrade pip
env/bin/pip install <url-of-experimental-wheel>
source env/bin/activate
```

# Disabling Builds on Pull Requests

For draft pull requests that may have frequent updates to the remote branch,
it can be useful to disable the builds that run automatically. This can be done
by adding the label ``status: defer ci``. Jobs will automatically be reported
back to the pull request as failures, but won't run the actual build. Since
these jobs are required to merge, this label will eventually need to be removed.
Comment ``@drake-jenkins-bot retest this please`` after removing the label to
trigger a re-run.

# Testing via External Examples

The examples within Drake's
[gallery of external examples](https://github.com/RobotLocomotion/drake-external-examples)
provide continuous integration via both Jenkins and GitHub Actions. This provides
downstream test coverage for Drake developers to ensure reliability in the
build infrastructure. Additionally, the GitHub Actions provide a benefit
for end users, in that examples of CI pipelines on public servers for
external projects using Drake installations are made easily accessible.

See the external examples
[continuous integration](https://github.com/RobotLocomotion/drake-external-examples#continuous-integration)
for details on which examples use Jenkins or GitHub Actions. In general,
GitHub Actions is used for the lightweight examples which use some
installed version of Drake, while Jenkins is used for complete coverage
on examples which pull in Drake externally and build it.

When a new pull request is opened in Drake, members of the RobotLocomotion
organization can utilize Jenkins and GitHub Actions to run custom builds.
This is especially pertinent for pull requests which affect the build infrastructure.

## Jenkins

To test the examples which use Jenkins for CI with a PR branch of Drake,
comment on an open pull request using the following command:

* ``@drake-jenkins-bot linux-noble-unprovisioned-external-examples please``

or follow the [instructions above](#scheduling-builds-via-the-jenkins-user-interface)
to schedule a build of the
[external examples job](https://drake-jenkins.csail.mit.edu/view/External%20Examples/job/linux-noble-unprovisioned-external-examples/).
Note that instead of providing a parameter for the branch of
``RobotLocomotion/drake-ci``, this job instead provides one for
drake-external-examples, which should be used if you need to test your Drake
changes alongside changes downstream.

## GitHub Actions

You can schedule "experimental" builds of a [binary package](/from_binary.html),
[debian package](/apt.html), and/or a [wheel package](/pip.html) by following the
instructions [above](#building-packages-on-demand).
Copy the download URL(s) obtained from the build as described.

From the [GitHub Actions workflow](https://github.com/RobotLocomotion/drake-external-examples/actions/workflows/ci.yml)
in drake-external-examples, notice the message "This workflow has a
`workflow_dispatch` event trigger." Click "Run workflow" and input the
download URL(s) copied from Jenkins in the drop-down menu.
All parameters are optional, so you can ignore the package(s) and/or platform(s)
that you don't need. (For those left blank, the default workflow will run using
a more "stable" version of Drake, which is usually either source code from
`master` or a nightly release depending on the example).

## Local Testing

For CMake, see the
[drake_cmake_installed](https://github.com/RobotLocomotion/drake-external-examples/tree/main/drake_cmake_installed#developer-testing)
example.

For Bazel, see the
[drake_bazel_external](https://github.com/RobotLocomotion/drake-external-examples/tree/main/drake_bazel_external)
example, and note the comments in:

* the [README](https://github.com/RobotLocomotion/drake-external-examples/blob/main/drake_bazel_external#using-a-local-checkout-of-Drake),
which mentions using
[`--override-module`](https://bazel.build/reference/command-line-reference#flag--override_module) to consume a local checkout of Drake
* [`MODULE.bazel`](https://github.com/RobotLocomotion/drake-external-examples/blob/main/drake_bazel_external/MODULE.bazel),
which can be modified to use a particular revision (commit or release) of Drake
