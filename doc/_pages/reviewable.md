---
title: Tips for Participating In Drake Code Reviews using reviewable.io
---

# Introduction

Drake code reviews use [https://reviewable.io](https://reviewable.io). This page documents some
best practices for communicating effectively in Reviewable.

# GitHub Integration

Avoid using the GitHub UI to comment on code during a review. Reviewable will
import comments from GitHub, but cannot reliably match them to lines of diff.

When you enter comments in Reviewable, they are saved as drafts. Use the
"publish" button to send them out in a batch. Reviewable will post the
comments to the GitHub PR in a single, well-formatted block, generating email
to everyone else following the PR.

Every time you push to your GitHub branch under review, Reviewable will
snapshot a new diff. Because it maintains an independent diff series, you can
rebase freely without corrupting the review history.

# Life of a Reviewable Comment

All threads in Reviewable must be resolved before you can merge your PR.

The semantics of discussion resolution is more nuanced than GitHub's default
code review tool. We recommend to read [this explanation](https://github.com/Reviewable/Reviewable/issues/510#issue-272337333) to
understand the details.

Before commenting on a line of code, reviewers should check to see if there
is already a resolved discussion addressing the same topic. Resolved
discussions are indicated by a small white check-mark in a grey circle to
the left of the line of code.

Reviewers should click the eye-shaped buttons to indicate that they have
reviewed a file.  Reviewable will remember the revisions at which the file
was reviewed, and mark them with an eye icon in the file history.

# Curated Commits

Each commit on Drake master should pass all unit tests and lint checks, should
be logically cohesive (should not require other commits to make sense), and
should have a meaningful commit message.

Therefore, our reviewable settings by default prevent a PR with more than one
commit from being merged.

Often a PR may end up with more than one commit, including "work-in-progress"
checkpoints or "fix review comments" pushes.  In that case, when the PR is
ready to merge, the author of a PR has three choices for how to proceed:

* Wait for the assigned Platform Reviewer to "Squash and merge" the PR.
  If time is of the essence, post a reminder to the PR.
* Locally (rebase and) squash the PR down to a single commit, and force-push
  that commit into the PR.
* Apply the label ``status: squashing now`` and then immediately use the "squash
  and merge" button to merge the PR, being careful to tweak the commit message
  in the web page's edit box to be a sensible description of the change.

On the other hand, some PR authors carefully curate their commits so that each
individual commit on a PR meets the acceptance criteria on its own.  In that
case, the author should apply the label ``status: commits are properly
curated``, which removes the single-commit requirement.  PRs with this label
should be merged to master using the "Create a merge commit" option, not
"Squash and merge" option.

# Joint Feature and Platform review

For a review to be considered complete, both Feature Review and Platform Review
must be completed (see [Review Process](/developers.html#review-process)).

Therefore, our reviewable settings require at least two assigned reviewers.  In
cases where the platform reviewer decides to double-count as feature review,
the reviewer should apply the label ``status: single reviewer ok`` to note this
condition, which removes the two-assignee requirement.
