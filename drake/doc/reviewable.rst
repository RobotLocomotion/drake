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

All threads in Reviewable must be resolved before you can merge your PR. The
semantics of discussion resolution are a little tricky:

When a reviewer creates a new comment, the reviewer's disposition toward the
thread may be "satisfied", "discussing", or "blocking".  Reviewers should:

* Use **satisfied** to indicate that no action is required of the author.
* Use **discussing** to indicate that the author must respond, but may
  then close the thread without further input. However, if the author has
  follow-up questions, the reviewer is responsible for iterating on it.
  This is the default.
* Use **blocking** to indicate that the author must take action on the
  comment. The reviewer must then review and iterate on the fix with the
  author. Once an acceptable resolution is achieved, the reviewer should
  update disposition to **satisfied**.

The author may respond to each comment with the same set of dispositions.
Authors should:

* Use **satisfied** to indicate that they believe the discussion is over.
  If the reviewer's comment is not blocking, this will close the thread.
  This is the default disposition when the "Done" or "Acknowledged" button
  is clicked.
* Use **discussing** to indicate that they require more input from the
  reviewer.
* Use **blocking** to indicate that they have not yet resolved the problem.
  This serves as a safeguard against accidental merge.

Before commenting on a line of code, reviewers should check to see if there
is already a resolved discussion addressing the same topic. Resolved
discussions are indicated by a small white check-mark in a grey circle to
the left of the line of code.

Reviewers should click the eye-shaped buttons to indicate that they have
reviewed a file.  Reviewable will remember the revisions at which the file
was reviewed, and mark them with an eye icon in the file history.
