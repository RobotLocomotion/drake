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

The release engineering tools (relnotes, download_release_candidate,
push_release, etc.) are supported only on Ubuntu (not macOS).

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

## Polishing the release notes

Here are some guidelines for bringing commit notes from the relnotes tool into
the main body of the document:

* Many commit messages can be cut down to their summary strings and used as-is.
* File geometry/optimization changes under the "Mathematical Program" heading,
  not the "Multibody" heading.
* Expand all acronyms (eg, MBP -> MultibodyPlant, SG -> SceneGraph).
* Commits can be omitted if they only affect tests or non-installed examples. {% comment %}TODO(jwnimmer-tri) Explain how to check if something is installed.{% endcomment %}
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
  * Every dependency upgrade line should be "Upgrade libfoobar to latest
    release 1.2.3" or "Upgrade funrepo to latest commit".
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
      has nothing still running (modulo the ``*-coverage`` builds, which we can
      ignore); if something is running, wait until it finishes.
   2. Make sure <https://drake-jenkins.csail.mit.edu/view/Production/> is clean.
      If not, then wait until tomorrow to try again.
   3. Open <https://github.com/RobotLocomotion/drake/commits/nightly-release/>;
      the top (newest) commit will be the git sha for this release.
2. Launch the staging builds for that git commit sha:
   1. Open the following Jenkins jobs (e.g., each in its own
      new window, so you can copy-and-paste sha1 and version easily):
      - [Linux Wheel Staging](https://drake-jenkins.csail.mit.edu/view/Staging/job/linux-noble-unprovisioned-gcc-wheel-staging-release/)
      - [macOS arm Wheel Staging](https://drake-jenkins.csail.mit.edu/view/Staging/job/mac-arm-sequoia-clang-wheel-staging-release/)
      - [Jammy Packaging Staging](https://drake-jenkins.csail.mit.edu/view/Staging/job/linux-jammy-unprovisioned-gcc-cmake-staging-packaging/)
      - [Noble Packaging Staging](https://drake-jenkins.csail.mit.edu/view/Staging/job/linux-noble-unprovisioned-gcc-cmake-staging-packaging/)
      - [macOS arm Packaging Staging](https://drake-jenkins.csail.mit.edu/view/Staging/job/mac-arm-sequoia-clang-cmake-staging-packaging/)
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
      tag the release until all five builds have succeeded.
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
      * If you don't have "Build now" click "Log in" first in upper right.
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
      them." and then choose for upload the **36** release files from
      ``/tmp/drake-release/v1.N.0/...``:
      - 9: 3 `.tar.gz` + 6 checksums
      - 6: 2 `.deb` + 4 checksums
      - 15: 5 linux `.whl` + 10 checksums
      - 6: 2 macOS arm `.whl` + 4 checksums
      * Note that on Jammy with `snap` provided Firefox, drag-and-drop from
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
   4. Update the GitHub issue on the [drake-ros](https://github.com/RobotLocomotion/drake-ros/issues/400)
      repository, requesting an update of the `DRAKE_SUGGESTED_VERSION`
      constant.
      - Change the issue name to reflect the current release version.
      - Add a comment that includes the link to the release notes for the new
        version.
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

For Jammy, ``apt install twine`` is too old. Instead, you must run it from a
venv (detail below).

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

## Post-release tutorials updates

Upgrade our Deepnote-hosted tutorials to the latest release.  This requires
that you have "Edit" permission in the Deepnote project.  If you don't have
that yet, then ask for help on slack in the ``#releases`` channel.  Provide
the email address associated with your github account.

1. Post a new slack thread in ``#releases`` saying that you're beginning the
   tutorials deployment now (so that others are aware of the potentially-
   disruptive changes).
2. Open the tutorials [requirements.txt](https://deepnote.com/workspace/Drake-0b3b2c53-a7ad-441b-80f8-bf8350752305/project/Tutorials-2b4fc509-aef2-417d-a40d-6071dfed9199/requirements.txt):
   1. Be sure to login with your github account; otherwise, the file is read-only.
   2. Edit the second line to refer to the PyPI version of this release.
      1. For reference, the typical content is thus (varying only by Drake's
         version number):
         ```
         # The current release.
         drake==1.47.0

         # A version compatible with Deepnote.
         ipywidgets==8.1.7
         ```
      2. If the current content differs by more than just the version from the
         above template, ask for help on slack in the ``#releases`` channel.
4. Run the initialization notebook at
   [init.ipynb](https://deepnote.com/workspace/Drake-0b3b2c53-a7ad-441b-80f8-bf8350752305/project/Tutorials-2b4fc509-aef2-417d-a40d-6071dfed9199/notebook/Init%20notebook-5fcfe3fc0bd0403899baab3b6cf37a18).
   1. Confirm that all cells executed successfully. Note that it may take
      5-10 minutes to finish.
   1. For reference, the most recent content is thus:
      ```
      %%bash
      set -euo pipefail
      # We need to upgrade libc and libstdc++ to Drake's minimum versions, because the
      # base of Deepnote's Docker image is ANCIENT (Debian 11 Bullseye from 2021 OMG).
      # This should be exciting ...
      cat > /etc/apt/sources.list.d/bookworm.sources <<'EOF'
      Types: deb
      URIs: http://deb.debian.org/debian/
      Suites: bookworm
      Components: main
      Signed-By: /usr/share/keyrings/debian-archive-keyring.gpg
      EOF
      cat > /etc/apt/preferences.d/bookworm.pref <<'EOF'
      Package: *
      Pin: release n=bookworm
      Pin-Priority: 1
      EOF
      apt-get -q update
      apt-get autoremove -y
      apt-get -q install -t bookworm -y --no-install-recommends libc6 libc6-dev libstdc++6 libstdc++-10-dev

      # Rendering needs EGL.
      # The bullseye version is satisfactory.
      apt-get install -y --no-install-recommends libegl1 libgl1-mesa-dri

      # We'll also need nginx installed (for websocket proxying).
      # The bullseye version is satisfactory.
      apt-get -q install -y --no-install-recommends nginx-light
      rm -f /etc/nginx/sites-enabled/default
      cat > /etc/nginx/sites-available/deepnote-meshcat-proxy <<'EOF'
      # Deepnote- and MeshCat- specific NginX proxy server configuration.
      #
      # Proxy https://DEEPNOTE_PROJECT_ID:8080/PORT/ to http://127.0.0.1:PORT/ so
      # that multiple notebooks can all be served via Deepnote's only open port.
      #
      # For conf documentation, see https://www.nginx.com/resources/wiki/start/.
      server {
        listen 8080 default_server;
        listen [::]:8080 default_server;
        root /var/www/html;
        server_name _;
        location ~ /(7[0-9][0-9][0-9])/(.*) {
          proxy_pass http://127.0.0.1:$1/$2;
          proxy_set_header Host $host;
          proxy_http_version 1.1;
        }
        proxy_read_timeout 600;
        proxy_connect_timeout 600;
        proxy_send_timeout 600;
        send_timeout 600;
      }
      EOF
      ln -sf /etc/nginx/sites-available/deepnote-meshcat-proxy /etc/nginx/sites-enabled/
      service nginx start

      # Now we can install our Python requirements.
      cd /work
      pip install -r ./requirements.txt
      ```
   2. If the current content differs from the above content, ask for help on
      slack in the ``#releases`` channel.
5. Copy the updated tutorials from the wheel release into the Deepnote project
   storage (``~/work/...``):
   1. Open [zzz_for_maintainers.ipynb](https://deepnote.com/workspace/Drake-0b3b2c53-a7ad-441b-80f8-bf8350752305/project/Tutorials-2b4fc509-aef2-417d-a40d-6071dfed9199/notebook/zzz_for_maintainers-fd55235184ab44289133abc40e94a5e0).
   2. Run each cell one by one, checking for errors as you go.
   3. Note the first cell will take 1-2 minutes to finish because Deepnote
      needs to boot the machine first.
6. Next you'll copy and run each notebook (in alphabetical order).
   Read all of these instructions before performing any of them.
   1. Caveats for specific notebook names:
      1. Do not run ``licensed_solvers_deepnote``; you don't have a license.
      2. Do not re-run ``zzz_for_maintainers``; you've already done that.
      3. The ``authoring_multibody_simulation`` has gotchas to be mindful of:
         1. Downloading the Drake package map can take 10+ minutes.
         2. The notebook will appear to hang on one of the middle cells where it
            uses JointSliders. It is _not_ hung, rather it is waiting for user
            input. Find the "Meshcat URL" link earlier in the notebook, click
            through to open Meshcat in a new tab, click "Open Controls", then
            click "Stop JointSliders" repeatedly until the option vanishes and
            the notebook completes.
      4. The ``rendering_multibody_plant`` sometimes crashes with an interrupted
         error. In that case, click through to the "Environment" gear in the
         left-hand panel, then into the ``init.ipynb`` notebook and re-run the
         initialization. Then go back to  ``rendering_multibody_plant`` and try
         again.
   2. To deploy run each of the ~2 dozen notebooks (i.e., do this step for
      ``authoring_leaf_system`` then ``authoring_multibody_simulation`` then
      ... etc.):
      1. In the left-hand panel of your screen, take note that each notebook
         appears in two places -- in "NOTEBOOKS" near the top and in "FILES"
         near the bottom. The "NOTEBOOKS" is the older copy (from the prior
         release); the "FILES" is the new copy (from this release). Our goal
         is to replace the old copy with the new.
      2. Scroll down to the "FILES" and choose the top-most name. Right click on
         it and select "Move to notebooks".
         Be patient because the web interface could be slow, and there might be
         delay while copying the file.
         Note that even though the button says "Move", it actually only *copies*
         the file; it does not delete the item from "FILES".
      3. Because a notebook of that name already existed in "NOTEBOOKS" (the old
         copy), the moved notebook will be renamed with a ``-2`` suffix.
      4. Scroll up to "NOTEBOOKS". Right click on the old copy (without ``-2``)
         and select "Delete" and confirm. Right click on the new notebook (with
         ``-2``) and select "Rename" and remove the ``-2`` suffix.
      5. Open the (new) notebook and click "Run notebook". It should succeed.
      6. For all code cells, examine the output of each cell to check that no
         exceptions have snuck through (or any other unexpected error text).
         * The error "'%matplotlib notebook' is not supported in Deepnote" is
           expected and can be ignored.
      7. For all markdown cells, quickly skim over the rendered output to check
         that no markup errors have snuck through (e.g., LaTeX syntax errors).
      8. If you get an error like "Synchronization of file ... failed, your
         changes are not being saved. You might be running out of disk quota"
         you may ignore it.
      9. Leave the notebook output intact (do not clear the outputs). We want
         users to be able to read the outputs on their own, without necessarily
         running the notebook themselves.
      10. Go back to "FILES" and right-click then "Delete" on the notebook you
          just copied; it should still be the top-most ``*.ipynb`` in "FILES".
6. On the left side, click "Environment" then "Stop Machine", as a
   courtesy. (It will time out on its own within the hour, but we might as
   well save a few nanograms of CO2 where we can.)
