---
title: Release Playbook
---

This document explains the steps related to publishing a version-numbered Drake
binary release.  It is intended only for use by the few Drake Developer experts
who regularly handle the release process.

We publish a minor release approximately once per month in the middle of the
calendar month, with version number is ``v0.N.0`` where ``N`` is monotonically
increasing.

# Minor releases

Begin this process around 1 week prior to the intended release date.

## Prior to release

1. Choose the next version number.
2. Create a local Drake branch named ``release_notes-v0.N.0`` (so that others
   can easily find and push to it after the PR is opened).
3. As the first commit on the branch, mimic the commit
   ([link](https://github.com/RobotLocomotion/drake/pull/14208/commits/674b84877bc08448b59a2243f3b910a7b6dbab43)
   from [PR 14208](https://github.com/RobotLocomotion/drake/pull/14208)
   in order to disable CI.  A quick way to do this might be:
   ```
   git fetch upstream pull/14208/head
   git cherry-pick 674b84877bc08448b59a2243f3b910a7b6dbab43
   ```
4. Push that branch and then open a new pull request titled:
   ```
   [doc] Add release notes v0.N.0
   ```
   Make sure that "Allow edits from maintainers" on the GitHub PR page is
   enabled (the checkbox is checked).
5. For release notes, on an ongoing basis, add recent commit messages to the
   release notes draft using the ``tools/release_engineering/relnotes`` tooling.
   (Instructions can be found atop its source code: [``relnotes.py``](https://github.com/RobotLocomotion/drake/blob/master/tools/release_engineering/relnotes.py))
    1. On the first run, use ``--action=create`` to bootstrap the file.
       * The output is draft release notes in ``doc/_release-notes/v0.N.0.md``.
    2. On the subsequent runs, use ``--action=update`` to refresh the file.
       * Try to avoid updating the release notes to refer to changes newer than
       the likely release, i.e., if you run ``--update`` on the morning you're
       actually doing the release, be sure to pass the ``--target_commit=``
       argument to avoid including commits that will not be part of the tag.
6. For release notes, on an ongoing basis, clean up and relocate the commit
   notes to properly organized and wordsmithed bullet points. See [Polishing
   the release notes](#polishing-the-release-notes).
7. From time to time, merge ``upstream/master`` into your
   ``origin/release_notes-v0.N.0`` branch (so that it doesn't go stale).
   Never rebase or force-push to the branch.  We expect that several people
   will clone and push to it concurrently.
8. As the release is nearly ready, post a call for action for feature teams to
   look at the draft document and provide suggestions (in reviewable) or fixes
   (as pushes).
    1. To help ensure that the "newly deprecated APIs" section is accurate, grep
       the code for ``YYYY-MM-01`` deprecation notations, for the ``MM`` values
       that would have been associated with our +3 months typical period.

## Polishing the release notes

Here are some guidelines for bringing commit notes from the relnotes tool into
the main body of the document:

* Many commit messages can be cut down to their summary strings and used as-is.
* File geometry/optimization changes under the "Mathematical Program" heading,
  not the "Multibody" heading.
* Expand all acronyms (eg, MBP -> MultibodyPlant, SG -> SceneGraph).
* Commits can be omitted if they only affect tests or non-installed examples. {% comment %}TODO(jwnimmer-tri) Explain how to check if something is installed.{% endcomment %}
* In general you should mention new bindings and deprecated/removed classes and
  methods using their exact name (for easier searching).
   * In the pydrake and deprecation sections in fact you can just put the
    fully-qualified name as the whole line item; the meaning is clear from
    context.
  * This may mean having a long list of items for a single commit.  That is
    fine.

* We have four common grammatical forms for our commit messages:
  * Past tense ("Added new method foo") is acceptable
  * Noun phrase ("Ability to foo the bar") is acceptable
  * Imperative ("Add new method foo", i.e. PEP-8 style) is acceptable
  * Present tense ("Adds new method foo", i.e. Google styleguide style) is
    discouraged

* Use exactly the same wording for the boilerplate items:
  * Every dependency upgrade line should be "Upgrade libfoobar to latest
    release 1.2.3" or "Upgrade funrepo to latest commit".
  * Dependencies should be referred to by their workspace name.
  * Only one dependency change per line. Even if both meshcat and meshcat-python
    were upgraded in the same pull request, they each should get their own
    line in the release notes.

* Some features under development (eg, hydroelastic as of this writing) may
  have no-release-notes policies, as their APIs although public are not yet
  fully supported.  Be sure to take note of which these are, or ask on
  `#platform_review` slack.
* Keep all bullet points to one line.
  * Using hard linebreaks to stay under 80 columns makes the bullet lists hard
    to maintain over time.

* Say "macOS" not "Mac" or "Apple" or etc.
* Say "SDFormat" not "SDF" nor "sdf".

## Cutting the release

9. Find a plausible build to use
   1. Make sure <https://drake-jenkins.csail.mit.edu/view/Production/> is clean
   2. Make sure <https://drake-jenkins.csail.mit.edu/view/Nightly%20Production/>
      has nothing still running (modulo the ``*-coverage`` builds, which we can
      ignore)
   3. Open the latest builds from the following builds:
      1. <https://drake-jenkins.csail.mit.edu/view/Packaging/job/mac-big-sur-unprovisioned-clang-bazel-nightly-snopt-packaging/>
      2. <https://drake-jenkins.csail.mit.edu/view/Packaging/job/linux-bionic-unprovisioned-gcc-bazel-nightly-snopt-packaging/>
      3. <https://drake-jenkins.csail.mit.edu/view/Packaging/job/linux-focal-unprovisioned-gcc-bazel-nightly-snopt-packaging/>
   4. Check the logs for those packaging builds and find the URLs they posted
      to (open the latest build, go to "View as plain text", and search for
      ``drake/nightly/drake-20``), and find the date.  It will be ``YYYYMMDD``
      with today's date (they kick off after midnight).  All of the builds
      should have the same date. If not, wait until the following night.
   5. Use the
      ``tools/release_engineering/download_release_candidate`` tool to download
      and verify that all the release candidates are built from the same
      commit.  (It's usage
      instructions are atop its source code:
      [download_release_candidate.py](https://github.com/RobotLocomotion/drake/blob/master/tools/release_engineering/download_release_candidate.py).)

10. Update the release notes to have the ``YYYYMMDD`` we choose, and to make
    sure that the nightly build git sha from the prior step matches the
    ``newest_commit`` whose changes are enumerated in the notes.  Some dates
    are YYYYMMDD format, some are YYYY-MM-DD format; be sure to manually fix
    them all.
   1. Update the github links within doc/_pages/from_binary.md to reflect the
      upcoming v0.N.0 and YYYYMMDD.
11. Re-enable CI by reverting the commit you added in step 3.
12. Merge the release notes PR
   1. Take care when squashing not to accept github's auto-generated commit message if it is not appropriate.
   2. After merge, go to <https://drake-jenkins.csail.mit.edu/view/Documentation/job/linux-bionic-unprovisioned-gcc-bazel-nightly-documentation/> and push "Build now".
      * If you don't have "Build now" click "Log in" first in upper right.
13. Open <https://github.com/RobotLocomotion/drake/releases> and choose "Draft
    a new release".  Note that this page does has neither history nor undo.  Be
    slow and careful!
    1. Tag version is: v0.N.0
    2. Target is: [the git sha from above]
      *  You should select the commit from Target > Recent Commits. The search
         via commit does not work if you don't use the correct length.
    3. Release title is: Drake v0.N.0
    4. The body of the release should be forked from the prior release (open the
       prior release's web page and click "Edit" to get the markdown), with
       appropriate edits as follows:
       * The version number
    5. Into the box labeled "Attach binaries by dropping them here or selecting
       them.", drag and drop the 9 release binary artifacts from above (the 3
       tarballs, and their 6 checksums)
    6. Choose "Save draft" and take a deep breath.
14. Once the documentation build finishes, release!
    1. Check that the link to drake.mit.edu docs from the GitHub release draft
       page actually works.
    2. Click "Publish release"
    3. Notify `@BetsyMcPhail` via a GitHub comment to manually tag docker images
       and upload the releases to S3. Be sure to provide her with the binary
       date, commit SHA, and release tag in the same ping.
    4. Announce on Drake Slack, ``#general``.
    5. Party on, Wayne.

## Post-release follow up

1. Open the [tagged workspace](https://github.com/RobotLocomotion/drake/tree/v0.N.0/tools/workspace)
   (editing that URL to have the correct value for ``N``) and ensure that
   certain Drake-owned externals have sufficient tags:
   1. Open ``common_robotics_utilities/repository.bzl`` and find the ``commit =`` used.
      1. Open
         [ToyotaResearchInstitute/common_robotics_utilities](https://github.com/ToyotaResearchInstitute/common_robotics_utilities/releases)
         and check whether that commit already has an associated release tag.
      2. If not, then create a new release named ``v0.0.foo`` where ``foo`` is
         the 8-digit datestamp associated with the ``commit`` in question (i.e.,
         four digit year, two digit month, two digit day).
   2. Open ``models/repository.bzl`` and find the ``commit =`` used.
      1. Open
         [RobotLocomotion/models](https://github.com/RobotLocomotion/models/releases)
         and check whether that commit already has an associated release tag.
      2. If not, then create a new release named ``v0.0.foo`` where ``foo`` is
         the 8-digit datestamp associated with the ``commit`` in question (i.e.,
         four digit year, two digit month, two digit day).
   3. Open ``optitrack_driver/repository.bzl`` and find the ``commit =`` used.
      1. Open
         [RobotLocomotion/optitrack-driver](https://github.com/RobotLocomotion/optitrack-driver/releases)
         and check whether that commit already has an associated release tag.
      2. If not, then create a new release named ``v0.0.foo`` where ``foo`` is
         the 8-digit datestamp associated with the ``commit`` in question (i.e.,
         four digit year, two digit month, two digit day).
   4. Open ``styleguide/repository.bzl`` and find the ``commit =`` used.
      1. Open [RobotLocomotion/styleguide](https://github.com/RobotLocomotion/styleguide/releases)
         and check whether that commit already has an associated release tag.
      2. If not, then create a new release named ``v0.0.foo`` where ``foo`` is
         the 8-digit datestamp associated with the ``commit`` in question (i.e.,
         four digit year, two digit month, two digit day).

## Post-release wheel builds

After tagging the release, you must manually build and upload a PyPI release.

If you haven't done so already, follow Drake's PyPI
[account setup](https://docs.google.com/document/d/17D0yzyr0kGH44eWpiNY7E33A8hW1aiJRmADaoAlVISE/edit#)
instructions to obtain a username and password.

1. Use your Ubuntu 20.04 workstation for these steps.  (Do not use any other OS.)
2. Create some empty scratch folder to use for these steps, and then ``cd`` into it.
3. Run ``sudo apt install docker.io twine``
   1. If you get an "docker.sock connect" error, you might need to give yourself Docker permissions:
      1. Run ``sudo usermod -aG docker $USER``
      2. Run ``newgrp docker``
   2. If Docker still doesn't work, you might need to logout and/or reboot first.
4. Run ``git clone --filter=blob:none https://github.com/RobotLocomotion/drake.git``
5. Run ``cd drake``
6. Run ``git checkout v0.N.0``
7. Run ``cd tools/wheel``
8. Run ``./build-wheels --test 0.N.0``
9. Wait a long time for it to finish (around 60 minutes on a beefy workstation). It will take over all of your computer's resources, so don't plan to do much else concurrently.
10. There should have been exactly two whl files created. Run ``twine upload <...>``, replacing the ``<...>`` placeholder with the path to each of the wheels to be uploaded (e.g., ``drake-0.35.0b1-cp36-cp36m-manylinux_2_27_x86_64``, etc.)
    1. You will need your PyPI username and password for this. (Do not use drake-robot.)
