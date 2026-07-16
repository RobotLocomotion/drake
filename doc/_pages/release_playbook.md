---
title: Release Playbook
---

This document explains the steps related to publishing a version-numbered Drake
binary release.  It is intended only for use by the few Drake Developer experts
who regularly handle the release process.

We publish a minor release approximately once per month in the middle of the
calendar month, with version number is ``v1.N.0`` where ``N`` is monotonically
increasing.

# Minor releases

Begin this process around 1 week prior to the intended release date.

The release engineering tools (relnotes, download_release_candidate, push_apt,
etc.) are supported only on Ubuntu (not macOS).

## Prior to release

1. Choose the next version number.
2. Create a local Drake branch named ``release_notes-v1.N.0`` (so that others
   can easily find and push to it after the PR is opened).
3. Bootstrap the release notes file using the
   ``tools/release_engineering/relnotes`` tooling, with ``--action=create``
   for the first run. Instructions can be found atop its source code:
   [``relnotes.py``](https://github.com/RobotLocomotion/drake/blob/master/tools/release_engineering/relnotes.py).
      * The output is draft release notes in ``doc/_release-notes/v1.N.0.md``.
      * The version numbers in ``doc/_pages/from_binary.md`` should also have
      been automatically upgraded.
   Commit the results.
4. Push that branch and then open a new pull request titled:
   ```
   [doc] Add release notes v1.N.0
   ```
   Make sure that "Allow edits by maintainers" on the GitHub PR page is
   enabled (the checkbox is checked). Set the labels ``release notes: none``
   and ``status: defer ci`` to disable CI while notes-only updates are ongoing.
5. Make gradual updates to the release notes using
  ``tools/release_engineering/relnotes``, with ``--action=update`` to
  refresh the file.
   * Try to avoid updating the release notes to refer to changes newer than
     the likely release, i.e., if you run ``--update`` on the morning you're
     actually doing the release, be sure to pass the ``--target_commit=``
     argument to avoid including commits that will not be part of the tag.
6. For release notes, on an ongoing basis, clean up and relocate the commit
   notes to properly organized and wordsmithed bullet points. See [Polishing
   the release notes](#polishing-the-release-notes).
7. From time to time, merge ``upstream/master`` into your
   ``origin/release_notes-v1.N.0`` branch (so that it doesn't go stale).
   Never rebase or force-push to the branch.  We expect that several people
   will clone and push to it concurrently.
8. As the release is nearly ready, post a call for action for feature teams to
   look at the draft document and provide suggestions (in reviewable) or fixes
   (as pushes).
   * To help ensure that the "newly deprecated APIs" section is accurate, grep
     the code for ``YYYY-MM-01`` deprecation notations, for the ``MM`` values
     that would have been associated with our +3 months typical period.
9. As the release is nearly ready, check the website docs for regressions:
   * Does the [Python API](/pydrake/index.html) appear normal? The site has a
     full list of modules?
   * Does the [C++ API](/doxygen_cxx/index.html) appear normal? The site has a
     full list of topics and classes?
   * Do all of the [tutorials](/tutorials/index.html) display correctly? Look
     at the output cells for any error text or LaTeX salad.

## Polishing the release notes

Here are some guidelines for bringing commit notes from the relnotes tool into
the main body of the document:

* Many commit messages can be cut down to their summary strings and used as-is.
* File geometry/optimization changes under the "Mathematical Program" heading,
  not the "Multibody" heading.
* Expand all acronyms (eg, MBP -> MultibodyPlant, SG -> SceneGraph).
* PRs may appear in multiple sections, depending on the tags they have. For
  example, a PR that includes a fix and new deprecation will appear in both the
  fix and newly-deprecated sections. Generally, PRs will be included once in
  each appropriate section according to the tags assigned to it.
* In general you should mention deprecated/removed classes and methods using
  their exact name (for easier searching).
  * In the deprecation section you can provide the fully-qualified name as the
    whole line item; the meaning is clear from context.
  * This may mean having a long list of items for a single commit.  That is
    fine.

* We have four common grammatical forms for our commit messages:
  * Past tense ("Added new method foo") is acceptable
  * Noun phrase ("Ability to foo the bar") is acceptable
  * Imperative ("Add new method foo", i.e. PEP-8 style) is acceptable
  * Present tense ("Adds new method foo", i.e. Google styleguide style) is
    discouraged

* Use exactly the same wording for the boilerplate items:
  * Every dependency upgrade line should be "Upgrade dependency libfoobar to
    1.2.3" or "Upgrade dependency funrepo to latest commit".
  * Dependencies should be referred to by their ``workspace`` name.
  * Only one dependency change per line. Even if both meshcat and meshcat-python
    were upgraded in the same pull request, they each should get their own
    line in the release notes.

* Keep all bullet points to one line.
  * Using hard linebreaks to stay under 80 columns makes the bullet lists hard
    to maintain over time.

* Say "macOS" not "Mac" or "Apple" or etc.
* Say "SDFormat" not "SDF" nor "sdf".

## Cutting the release

1. Find the git sha of the most recent nightly build:
   1. Make sure <https://drake-jenkins.csail.mit.edu/view/Nightly%20Production/>
      has nothing still running; if something is running, wait until it finishes.
   2. Make sure <https://drake-jenkins.csail.mit.edu/view/Production/> is clean.
      If not, then wait until tomorrow to try again.
   3. Open <https://github.com/RobotLocomotion/drake/commits/nightly-release/>;
      the top (newest) commit will be the git sha for this release.
2. Launch the staging builds for that git commit sha:
   1. Open the following Jenkins jobs (e.g., each in its own
      new window, so you can copy-and-paste sha1 and version easily):
      - [Linux x86_64 Wheel Staging](https://drake-jenkins.csail.mit.edu/view/Staging/job/linux-noble-unprovisioned-gcc-wheel-staging-release/)
      - [Linux arm64 Wheel Staging](https://drake-jenkins.csail.mit.edu/view/Staging/job/linux-arm64-noble-unprovisioned-gcc-wheel-staging-release/)
      - [macOS arm64 Wheel Staging](https://drake-jenkins.csail.mit.edu/view/Staging/job/mac-arm-sequoia-clang-wheel-staging-release/)
      - [Noble x86_64 Packaging Staging](https://drake-jenkins.csail.mit.edu/view/Staging/job/linux-noble-unprovisioned-gcc-cmake-staging-packaging/)
      - [Noble arm64 Packaging Staging](https://drake-jenkins.csail.mit.edu/view/Staging/job/linux-arm64-noble-unprovisioned-gcc-cmake-staging-packaging/)
      - [Resolute x86_64-v3 Packaging Staging](https://drake-jenkins.csail.mit.edu/view/Staging/job/linux-resolute-unprovisioned-gcc-cmake-staging-packaging/)
      - [Resolute x86_64 Packaging Staging](https://drake-jenkins.csail.mit.edu/view/Staging/job/linux-amd64v1-resolute-unprovisioned-gcc-cmake-staging-packaging/)
      - [Resolute arm64 Packaging Staging](https://drake-jenkins.csail.mit.edu/view/Staging/job/linux-arm64-resolute-unprovisioned-gcc-cmake-staging-packaging/)
      - [macOS arm64 Packaging Staging](https://drake-jenkins.csail.mit.edu/view/Staging/job/mac-arm-sequoia-clang-cmake-staging-packaging/)
   2. In the upper right, click "sign in" (unless you're already signed in). This
      will use your GitHub credentials.
   3. Click "Build with Parameters".
   4. Change "sha1" to the full **git sha** corresponding to
      ``v1.N.0`` and "release_version" to ``1.N.0`` (no "v").
      - If you mistakenly provide the "v" in "release_version", your build will
        appear to work, but actually fail 5-6 minutes later.
   5. Click "Build"; each build will take around an hour, give or take.
   6. Wait for all staging jobs to succeed.  It's OK to work on release notes
      finishing touches in the meantime, but do not merge the release notes nor
      tag the release until all of the above-listed builds have succeeded.
3. Update the release notes to have the ``YYYY-MM-DD`` we choose.
   1. There is a dummy date 2099-12-31 nearby that should likewise be changed.
   2. Make sure that the nightly build git sha from the prior steps matches the
      ``newest_commit`` whose changes are enumerated in the notes.
4. Re-enable CI by removing the label ``status: defer ci`` and commenting
   ``@drake-jenkins-bot retest this please`` to trigger a re-run.
5. Wait for the wheel builds to complete, and then download release artifacts:
   1. Use the
      ``tools/release_engineering/download_release_candidate`` tool with the
      ``--version`` option to download and verify all binaries.  (Its usage
      instructions are atop its source code:
      [download_release_candidate.py](https://github.com/RobotLocomotion/drake/blob/master/tools/release_engineering/download_release_candidate.py).)
6. Merge the release notes PR.
   1. Take care when squashing not to accept github's auto-generated commit message if it is not appropriate.
   2. After merge, go to <https://drake-jenkins.csail.mit.edu/view/Documentation/job/linux-noble-unprovisioned-gcc-bazel-nightly-documentation/> and push "Build now".
      * If you don't have "Build now" click "Sign in" first in upper right.
7. Open <https://github.com/RobotLocomotion/drake/releases> and choose "Draft
   a new release".  Note that this page has neither history nor undo.  Be
   slow and careful!
   1. Tag version is: v1.N.0
   2. Target is: [the git sha from the `--find-git-sha` in step 1.v]
     *  You should select the commit from Target > Recent Commits. The search
        via commit does not work if you don't use the correct length.
   3. Release title is: Drake v1.N.0
   4. The body of the release should be forked from the prior release (open the
      prior release's web page and click "Edit" to get the markdown), with
      appropriate edits as follows:
      * The version number
   5. Click the box labeled "Attach binaries by dropping them here or selecting
      them." and then choose for upload the **60** release files from
      ``/tmp/drake-release/v1.N.0/...``:
      - 3: 1 source `.tar.gz` + 2 checksums
      - 15: 5 linux binary `.tar.gz` + 10 checksums
        ({noble} x {amd64, arm64} + {resolute} x {amd64, amd64v3, arm64})
      - 3: 1 macOS arm binary `.tar.gz` + 2 checksums
      - 15: 5 `.deb` + 10 checksums
        ({noble} x {amd64, arm64} + {resolute} x {amd64, amd64v3, arm64})
      - 18: 6 linux `.whl` + 12 checksums ({3.12, 3.13, 3.14} x {x86_64, aarch64})
      - 6: 2 macOS arm `.whl` + 4 checksums ({3.13, 3.14})
      * Note that with `snap` provided Firefox, drag-and-drop from
        Nautilus will fail, and drop all of your release page inputs typed so
        far. Use the Firefox-provided selection dialog instead, by clicking on
        the box.
   6. Choose "Save draft" and take a deep breath.
8. Once the documentation build finishes, release!
   1. Check that the link to drake.mit.edu docs from the GitHub release draft
      page actually works.
   2. Click "Publish release".
   3. Create a new GitHub issue on the [drake](https://github.com/RobotLocomotion/drake/issues/new/choose)
      repository and select the "Post-Release Actions" template.
   4. Create a GitHub issue on the [drake-ros](https://github.com/RobotLocomotion/drake-ros/issues)
      repository, requesting an update of the `DRAKE_SUGGESTED_VERSION`
      constant.
   5. Announce on Drake Slack, ``#general``.
   6. Party on, Wayne.

## Post-release wheel upload

After tagging the release, you must manually upload a PyPI release.

If you haven't done so already, follow Drake's PyPI
[account setup](https://docs.google.com/document/d/17D0yzyr0kGH44eWpiNY7E33A8hW1aiJRmADaoAlVISE/edit#)
instructions to obtain a username and password.

Most likely, you will want to use an api token to authenticate yourself to the
``twine`` uploader. See <https://pypi.org/help/#apitoken> and <https://packaging.python.org/en/latest/guides/distributing-packages-using-setuptools/#create-an-account>
for advice on managing api tokens.

1. Run ``twine`` to upload the wheel release, as follows:

   1. You will need your PyPI username and password for this. (Do not use drake-robot.)
   2. Run:
      ```
      $ cd tmp
      $ python3 -m venv env
      $ env/bin/pip install twine
      $ env/bin/twine upload /tmp/drake-release/v1.N.0/*.whl
      $ rm -rf env
      ```
   3. Confirm that all of the uploads succeeded without any error messages in
      your terminal.
