****************
Release Playbook
****************

This document explains the steps related to publishing a version-numbered Drake
binary release.  It is intended only for use by the few Drake Developer experts
who regularly handle the release process.

.. We publish a minor release approximately once per month in the middle of the calendar month, with version number is v0.N.0 where N is monotonically increasing.
.. Minor releases
.. Begin this process around 1 week prior to the intended release date.
.. Prior to release
.. 
.. Choose the next version number.
.. Open a new pull request titled
..   doc: Add release notes v0.N.0
.. As the first commit, mimic the first commit (link) from PR 12474 in order to disable CI.  Your branch name should be release_notes-v0.N.0 so that others can easily find and push to it.  Make sure that "Allow edits from maintainers" on the GitHub PR page is enabled (the checkbox is checked).
.. For release notes, on an ongoing basis, add recent commit messages to the release notes draft using the tools/dev/relnotes tooling.
.. The release notes draft should document the latest commit sha that has been scribed into the notes.
.. Use the --oldest_commit and --newest_commit arguments to tools/dev/relnotes to scrape any more recent commits, and paste them into the end of the document unmodified.  Commit and push the new text to origin.  We will edit them later, but as a starting point we should have the unmodified commit text available.
.. From time to time, merge upstream/master into your origin/release_notes-v0.N.0 branch (so that it doesn’t go stale).  Never rebase or force-push to the branch.  We expect that several people will clone and push to it concurrently.
.. For release notes, on an ongoing basis, clean up and relocate the commit notes from the end-of-document dumping ground to properly organized and wordsmithed bullet points.
.. As the release is nearly ready, post a call for action for feature teams to look at the draft document and provide suggestions (in reviewable) or fixes (as pushes).
.. To make the document actually compile, you’ll need to update the PR hyperlinks, using this tool to generate the text:
.. perl -ne 'while (s|`#([0-9]*)`_||) { print ".. _#$1: https://github.com/RobotLocomotion/drake/pull/$1\n"; }' doc/release_notes/v0.N.0.rst | sort -u
.. To help ensure that the "newly deprecated APIs" section is accurate, grep the code for YYYY-MM-01 deprecation notations, for the MM values that would have been associated with our +3 months typical period. (Or alternatively, diff prior_tag..master and grep for DRAKE_DEPRECATED?)
.. Polishing the release notes
.. Here are some guidelines for bringing commit notes from the relnotes tool into the main body of the document:
.. Many commit messages can be cut down to their summary strings and used as-is.
.. Expand all acronyms (eg, MBP -> MultibodyPlant, SG -> SceneGraph).
.. Commits can be omitted if they only affect tests or non-installed examples.
.. In general you should literally mention added or removed classes and methods by name.
.. In the pydrake and deprecation sections in fact you can just put the fully-qualified name as the whole line item; the meaning is clear from context.
.. This may mean having a long list of items for a single commit.  That is fine.
.. We have four common grammatical forms for our commit messages:
.. Past tense ("Added new method foo") is acceptable
.. Noun phrase ("Ability to foo the bar") is acceptable
.. Imperative ("Add new method foo", i.e. PEP-8 style) is acceptable
.. Present tense ("Adds new method foo", i.e. Google styleguide style) is discouraged
.. Use exactly the same wording for the boilerplate items:
.. Every dependency upgrade line should be "Upgrade libfoobar to latest release 1.2.3" or "Upgrade funrepo to latest commit".
.. Dependencies should be referred to by their workspace name.
.. Some features under development (eg, hydroelastic as of this writing) may have no-release-notes policies, as their APIs although public are not yet fully supported.  Be sure to take note of which these are, or ask on #platform_review slack
.. Cutting the release
.. Find a plausible build to use
.. Make sure https://drake-jenkins.csail.mit.edu/view/Production/ is clean
.. Make sure https://drake-jenkins.csail.mit.edu/view/Nightly Production/ has nothing still running (modulo the "*-coverage" builds, which we can ignore)
.. Take https://drake-jenkins.csail.mit.edu/view/Packaging/job/mac-mojave-unprovisioned-clang-bazel-nightly-snopt-packaging/
.. and
.. https://drake-jenkins.csail.mit.edu/view/Packaging/job/linux-bionic-unprovisioned-gcc-bazel-nightly-snopt-packaging/
.. and
.. https://drake-jenkins.csail.mit.edu/view/Packaging/job/linux-focal-unprovisioned-gcc-bazel-nightly-snopt-packaging/
.. builds for that night and make sure they share the same git sha (HEAD from the moment they were launched).  Note it.
.. If the git sha for the two does not match, wait and try again tomorrow night.
.. Check the logs for those packaging builds and find the URLs they posted to.  It will be YYYYMMDD with today’s date (they kick off after midnight).  Note it.
.. N.B. The packaging script uploads the file twice, with two different names.  One is YYYYMMDD and one is "latest".  We will use the YYYYMMDD one.
.. Update the release notes to have the YYYYMMDD we choose, and to make sure that the nightly build git sha from the prior step matches the newest_commit whose changes are enumerated in the notes.
.. Prepare the binaries
.. Make a local folder, maybe $HOME/tmp/v0.N.0
.. Fetch all the things (https://drake-jenkins.csail.mit.edu/view/Packaging/job/mac-mojave-unprovisioned-clang-bazel-nightly-snopt-packaging/ and https://drake-jenkins.csail.mit.edu/view/Packaging/job/linux-bionic-unprovisioned-gcc-bazel-nightly-snopt-packaging/ and https://drake-jenkins.csail.mit.edu/view/Packaging/job/linux-focal-unprovisioned-gcc-bazel-nightly-snopt-packaging/)
.. wget https://drake-packages.csail.mit.edu/drake/nightly/drake-YYYYMMDD-bionic.tar.gz
.. wget https://drake-packages.csail.mit.edu/drake/nightly/drake-YYYYMMDD-bionic.tar.gz.sha512
.. wget https://drake-packages.csail.mit.edu/drake/nightly/drake-YYYYMMDD-focal.tar.gz
.. wget https://drake-packages.csail.mit.edu/drake/nightly/drake-YYYYMMDD-focal.tar.gz.sha512
.. wget https://drake-packages.csail.mit.edu/drake/nightly/drake-YYYYMMDD-mac.tar.gz
.. wget https://drake-packages.csail.mit.edu/drake/nightly/drake-YYYYMMDD-mac.tar.gz.sha512
.. Checksums
.. sha512sum -c *.sha512
.. sha256sum drake-YYYYMMDD-bionic.tar.gz >  drake-YYYYMMDD-bionic.tar.gz.sha256
.. sha256sum drake-YYYYMMDD-focal.tar.gz >  drake-YYYYMMDD-focal.tar.gz.sha256
.. sha256sum drake-YYYYMMDD-mac.tar.gz >  drake-YYYYMMDD-mac.tar.gz.sha256
.. sha256sum -c *.sha256
.. Merge the release notes PR
.. After merge, go to https://drake-jenkins.csail.mit.edu/view/Documentation/job/linux-bionic-gcc-bazel-nightly-documentation/ and push "Build now".
.. If you don’t have "Build now" click "Log in" first in upper right.
.. Open https://github.com/RobotLocomotion/drake/releases and choose "Draft a new release".  Note that this page does not have history nor undo.  Be slow and careful!
.. Tag version is: v0.N.0
.. Target is: [the git sha from above]
.. Release title is: Drake v0.N.0
.. The body of the release should be forked from the prior release (open the prior release’s web page and click "Edit" to get the markdown), with appropriate edits as follows:
.. The version number
.. Into the box labeled " Attach binaries by dropping them here or selecting them.", drag and drop the 9 release binary artifacts from above (the 3 tarballs, and their 6 checksums)
.. Choose "Save draft" and take a deep breath.
.. Once the documentation build finishes, release!
.. Check that the link to drake.mit.edu docs from the GitHub release draft page actually works.
.. Click "Publish release"
.. Party on, wayne.
