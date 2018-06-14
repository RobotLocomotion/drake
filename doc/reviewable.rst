.. _reviewable:

****************************************************************
Tips for Participating In Drake Code Reviews using reviewable.io
****************************************************************

.. contents:: `Table of contents`
   :depth: 3
   :local:

Introduction
============

Drake code reviews use https://reviewable.io. This page documents some
best practices for communicating effectively in Reviewable.

GitHub Integration
==================

Avoid using the GitHub UI to comment on code during a review. Reviewable will
import comments from GitHub, but cannot reliably match them to lines of diff.

When you enter comments in Reviewable, they are saved as drafts. Use the
"publish" button to send them out in a batch. Reviewable will post the
comments to the GitHub PR in a single, well-formatted block, generating email
to everyone else following the PR.

Every time you push to your GitHub branch under review, Reviewable will
snapshot a new diff. Because it maintains an independent diff series, you can
rebase freely without corrupting the review history.

Life of a Reviewable Comment
============================

All threads in Reviewable must be resolved before you can merge your PR.

The semantics of discussion resolution is more nuanced than GitHub's default
code review tool. We recommend to read `this explanation
<https://github.com/Reviewable/Reviewable/issues/510#issue-272337333>`_ to
understand the details.

Before commenting on a line of code, reviewers should check to see if there
is already a resolved discussion addressing the same topic. Resolved
discussions are indicated by a small white check-mark in a grey circle to
the left of the line of code.

Reviewers should click the eye-shaped buttons to indicate that they have
reviewed a file.  Reviewable will remember the revisions at which the file
was reviewed, and mark them with an eye icon in the file history.

.. _curate_commits_before_merging:

Curated Commits
===============

Each commit on Drake master should pass all unit tests and lint checks, should
be logically cohesive (should not require other commits to make sense), and
should have a meaningful commit message.

The label ``status: curate commits before merging`` on a PR indicates that (at
the time the label was added) the commits pushed to the PR do not meet those
criteria, usually because the commits contain "work-in-progress" checkpoints or
scattered "fix review comments" adjustments.

When applied to a PR, this label will prevent a PR with more than one commit
from being merged.

The author of a PR has three choices for how to resolve this defect:

* Remove the label and then immediately use the "squash and merge" button to
  merge the PR, being careful to tweak the the commit message in the github
  edit box to be a sensible description of the change.
* Locally (rebase and) squash the PR down to a single commit, and force-push
  that commit into the PR.  This will automatically remove the merge
  impediment: even though the label still exists, it is a no-op when there is
  only one commit in the PR.
* Locally (rebase and) squash the PR to contain more than one commit, but where
  each individual commit meets the above recommendations, and then remove the
  label.
