---
title: For Developers
layout: page_with_toc
---

{% include toc.md %}
<article class="markdown-body" markdown="1">

# Introduction


If you have improvements to Drake, send us your pull requests!

Our standard workflow is to fork [Drake's official GitHub repository](https://github.com/RobotLocomotion/drake/) into your
own GitHub account and then push your changes into a branch on your fork. Once
you believe your code is ready to be merged into Drake's primary repository,
open a [pull request](https://help.github.com/articles/using-pull-requests/)
via the GitHub website. Your code will then undergo an interactive review
process and [Continuous Integration (CI)](#continuous-integration-notes)
tests before it is merged into
[Drake's primary repository](https://github.com/RobotLocomotion/drake)

Drake's [CI service](#continuous-integration-notes) runs on all pull requests each time they are
submitted and updated. Pull requests cannot be merged into master unless all
unit tests pass on all
[supported platform configurations](#supported-configurations).

Drake's CI server also runs continuously on
[Drake's primary master branch](https://github.com/RobotLocomotion/drake)
using an even more comprehensive set of unit tests.
If problems are detected on this branch, the build cop will
[revert the PRs that most likely caused the problem](buildcop.html).
To increase the likelihood that your pull requests pass CI tests and are not
reverted, you can run the unit tests locally. Instructions for how to do that
are provided [here](/unit_testing_instructions.html). Note, however, that there are
many computationally-demanding tests and running the entire test suite can take
several hours depending on your machine.

We would like to hear about your success stories if you've used Drake in your
own projects.  Please consider contributing to our [Drake Gallery](gallery.html) by editing
``doc/gallery.rst`` and submitting a pull request with the update!

# Licensing

**Important note:** Drake is an open source project licensed under
extremely flexible terms intended to encourage use by anyone, for any
purpose. When you make a contribution to the Drake project, you are
agreeing to do so under those same terms.

## Website Licensing

See [Website Third-Party Licenses](/website_licenses.html) for the licenses
of artifacts distributed with this website.

# Testing

* [Detailed Notes on Drake's Unit Tests](/unit_testing_instructions.html)
* [Downstream Testing (Drake as a Dependency)](/downstream_testing.html)

# Supported Configurations

Refer to [Source Installation](/from_source.html#supported-configurations).

## Configuration Management Non-Determinism

The indicated versions for build systems and languages are recorded after
having been tested on Continuous Integration.

Due to how the Debian ``apt`` and Homebrew package managers work, you may not
have these exact versions on your system when (re)running
``install_prereqs.sh``. In general, later minor versions for more stable
packages (e.g. CMake, compilers) should not prove to be too much of an issue.

For less stable packages, such as Bazel, later minor versions may cause
breakages. If you are on Ubuntu, please rerun ``install_prereqs.sh`` as it can
downgrade Bazel. If on Mac, there is no easy mechanism to downgrade with
Homebrew; however, we generally try to stay on top of Bazel versions.

If you have tried and are unable to configure your system by
[following the instructions](/from_source.html) please do not hesitate
to [ask for help](/getting_help.html).

# Issue Tracking

* [GitHub Issue Management](/issues.html)
* [Platform Reviewer Checklists](/platform_reviewer_checklist.html)

# Code Review


## Review Process

For complex changes, especially those that will span multiple PRs, please
open a GitHub issue and solicit design feedback before you invest a lot of
time in code.

Before your submit a pull request, please consult the
[Code Review Checklist](/code_review_checklist.html),
where a list of the most frequent problems are collected.

Be prepared to engage in active code review on your pull requests.  The Drake
code review process has two phases: feature review and platform review. You
are responsible for finding reviewers, and for providing them the information
they need to review your change effectively. If a reviewer asks you for more
information, that is a sign you should add more documentation to your PR.

A PR generally *should not* include more than 750 added or changed lines (the
green ``+###`` number as reported by github), and *must not* include more than
1500 lines, with the following exemptions:
  * Data files do not count towards the line limit.
  * Machine-generated changes do not count towards the line limit.
  * Files in
    [Special Directories](/directory_structure.html)
    do not count towards the line limit.
  * This rule may be overridden by agreement of at least two platform reviewers
    (listed below).

The utility ``tools/prstat`` will report the total added or changed
lines, excluding files that are easily identified to meet the exemptions above.

We use [https://reviewable.io](https://reviewable.io/) for code reviews. You can sign in for free with
your GitHub identity. Before your first code review, please take a look at
[Tips for Participating In Drake Code Reviews using reviewable.io](/reviewable.html).

If you have an expected pace for your review, please add a ``priority`` label
(which have different meanings for PRs and
[for issues](/issues.html#priority)). The response expectations, for both the
author and reviewer:

* ``priority: emergency`` - Very quick response time, nominally reserved for
  build cop.
* ``priority: high`` - Some urgency, quick response time.
* ``priority: medium`` - (Default) Normal response time.
* ``priority: low`` - No rush.
* ``priority: backlog`` - Give priority to all other PRs on your plate.

If you are an external contributor, you will need to request that a priority be
added by a Drake Developer.

**Feature Review.** After creating your pull request, assign it to someone
else on your team for feature review. Choose the person most familiar
with the context of your pull request. This reviewer is responsible for
protecting your team by inspecting for bugs, for test coverage, and for
alignment with the team's goals. During this review, you and your reviewer
should also strive to minimize the number of changes that will be necessary
in platform review.

If you are still not sure whom to assign for code review, simply do not assign
a reviewer. As part of a
[platform reviewer's responsibilities](/platform_reviewer_checklist.html),
they will come across the unassigned PR and find an appropriate feature
reviewer.

**Platform Review.** After your feature reviewer has signed off on your change,
reassign it to a Drake owner for platform review. The owner will inspect for
architectural compatibility, stability, performance, test coverage, and style.

The following GitHub users are Drake owners. If possible, seek platform review
from an owner who has previously reviewed related changes. Shared context will
make the review faster.

* @EricCousineau-TRI (Toyota Research Institute)
* @ggould-tri (Toyota Research Institute)
* @jwnimmer-tri (Toyota Research Institute)
* @rpoyner-tri (Toyota Research Institute)
* @sammy-tri (Toyota Research Institute)
* @sherm1 (Toyota Research Institute)
* @RussTedrake (MIT / Toyota Research Institute)

**Merge.** Once the PR is fully reviewed and passes CI, the assigned platform
reviewer will merge it to master.  If time is of the essence, you may post a
reminder to the PR to get the reviewer's attention.  If the PR should not be
merged yet, or if you prefer to merge it yourself, apply the label "status:
do not merge" to disable the merge.

If you are a frequent contributor who has been granted write access to
RobotLocomotion/drake, a green "Merge Pull Request" button will appear when
your change is fully reviewed and passes CI. You may click it to merge your PR.
Choose the "Squash and merge option" unless otherwise instructed (see
[Curated Commits](/reviewable.html#curated-commits)).

**After Merge.** If your PR breaks continuous integration, the
[buildcop](/buildcop.html) will contact you to work out a resolution.


## Review Process Tooling

[Tips for Participating In Drake Code Reviews using reviewable.io](/reviewable.html)

# User Assistance

The user-facing instructions for requesting assistance are located in
[Getting Help](/getting_help.html). The two main options for requesting assistance are either
posting a GitHub issue or a StackOverflow question.

## Handling User GitHub Issues

See [GitHub Issue Management](/issues.html).

If a GitHub issue should instead be a StackOverflow question (e.g. it is of a
tutorial nature that does not require code or documentation modification),
please request that the user repost the question on StackOverflow, post the
new link on the GitHub issue, and close the issue.

## Handling User StackOverflow Questions

Please subscribe to the ``drake`` tag by following these instructions:

* [How to get notifications for drake-tagged Stackoverflow questions](/stackoverflow_notifications.html)

Please also monitor for [unanswered StackOverflow posts](https://stackoverflow.com/unanswered/tagged/drake?tab=noanswers)
once per day. If there are unanswered questions that you are unsure of the
answer, consider posting on the Slack ``#onramp`` channel to see if someone
can look into the question.

# Continuous Integration Notes

* [CDash](https://drake-cdash.csail.mit.edu/index.php?project=Drake)
* [GitHub PR Interaction with Jenkins](/jenkins.html)
* [Build Cop](/buildcop.html)

# Documentation Instructions


* [Documentation Generation Instructions](/documentation_instructions.html)
* [Doxygen Instructions](/doxygen_instructions.html)
* [Sphinx Instructions](/sphinx_instructions.html)

# IDE and Text Editor Notes

* [CLion IDE setup](/clion.html)
* [Eclipse](https://github.com/tkoolen/drake/wiki/Eclipse-setup-(experimental))
* [Sublime Text Notes](/sublime_text.html)
* [Unicode Tips & Tricks](/unicode_tips_tricks.html)
* [Vim/Neovim Notes](/vim.html)

# Operating System Notes

* [Drake Development on macOS](/development_on_mac.html)

# Programming Style Notes

* [Code Review Checklist](/code_review_checklist.html)
* [Code Style Guide](/code_style_guide.html)
* [Tools for Code Style Compliance](/code_style_tools.html)
* [Directory Structure](/directory_structure.html)

# Version Control

* [Avoid Accidental Pushes to the Repository](/no_push_to_origin.html)
* [Model Version Control](/model_version_control.html)
* [Release Playbook](/release_playbook.html)

</article>
