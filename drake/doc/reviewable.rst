****************************************************************
Tips for Participating In Drake Code Reviews using reviewable.io
****************************************************************

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
thread may be "satisfied", discussing", or "blocking".

* Use **satisfied** to indicate that no action is required of the author.
* Use **discussing** to indicate that a response is required, but the author
  may close the thread at their discretion without further input from you.
  If the author has follow-up questions or wants you to take another look,
  you are responsible for iterating on it. This is the default.
* Use **blocking** to indicate that the author must take action on the
  comment. You must then review the fix and iterate on it with the author
  until you are willing to update your status to **satisfied**.

The author may respond to each comment with the same set of dispositions.

* Use **satisfied** to indicate that you believe the discussion is over.
  If the reviewer's comment is not blocking, this will close the thread.
* Use **discussing** to indicate that you require more input from the
  reviewer.
* Use **blocking** to indicate that you have not yet resolved the problem.
  This serves as a safeguard against accidental merge.

Before commenting on a line of code, reviewers should check to see if there
is already a resolved discussion addressing the same topic. Resolved
discussions are indicated by a small white check-mark in a grey circle to
the left of the line of code.

Reviewers should click the eye-shaped buttons to indicate that they have
reviewed a file.  Reviewable will then remember the revision at which it
was reviewed.
