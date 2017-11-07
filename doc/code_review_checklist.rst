.. _code-review-checklist:

*********************
Code Review Checklist
*********************

The following questions cover about 80% of the comments reviewers make
on pull requests.  Before submitting or assigning reviewers to a pull
request to Drake, please take a moment to re-read your changes with
these common errors in mind.

Does your code compile?  :facepalm:
===================================

- If your code doesn't pass Jenkins, why are you asking for a detailed
  code review?

Is the code the minimal set of what you want?
=============================================

- If the PR includes more than 750 added or changed lines,
  try to divide it into multiple PRs. The following techniques may be useful.
  If you don't think it can be done, talk to a platform reviewer before
  sending the PR.

  - If you are modifying an API, consider deprecating the old interface instead
    of migrating all call sites immediately.

  - If you are introducing a new feature, consider adding only test cases
    now, and deferring the first application use to a follow-up PR.

- Do a self-review, before you ask anyone else to review.

  - Before you even submit the PR, you can review the diffs using
    github.

  - If you'd rather use reviewable.io to self-review, that's fine too
    -- just wait to tag any reviewers until you've finished your
    review.

  - As you update your PR based on review feedback, review your commit
    diffs before pushing them -- don't leave it to the reviewers to
    discover your typos.

- Remove commented-out code.
- Remove code that isn't part of the build, or doesn't have unit tests.
- Don't include files with only whitespace diffs in your pull request
  (unless of course the purpose of the changes is whitespace fixes).

Are your classes, methods, arguments, and fields correctly named?
=================================================================

- Classes are ``UppercaseStyle`` and nouns or noun-phrases.
- Methods are ``UppercaseStyle`` and verbs or verb-phrases, except for
  trivial getters and setters.
- Inexpensive or trivial methods are ``lowercase_style`` verb-phrases.
- Arguments are ``lowercase_style`` and nouns, and are single-letter only
  when they have a clear and documented meaning.  (Yes, this includes
  single greek letters!  A ``theta`` argument must still be clearly
  documented, even though it's longer than one letter.)
- Member fields are ``lowercase_style_with_trailing_underscore_``.

Are you using pointers well?
============================

- ``shared_ptr<>`` usually means you haven't thought through
  ownerships and lifespans.
- ``unique_ptr<>`` and const reference should cover 95% of all your
  use cases.
- Bare pointers are a common source of mistakes; if you see one,
  consider if it is what you want (e.g., this is a mutable
  pass-by-reference) and if so ensure that any nontrivial lifetime
  guarantee is documented and you checked for ``nullptr``.

Did you document your API?  Did you comment your complexity?
============================================================

- Every nontrivial public symbol must have a Doxygen-formatted
  comment.
- If you are uncertain of your formatting, consider
  :ref:`generating the Doxygen <documentation-generation-instructions>`
  and checking how it looks in a browser.
- Only use Doxygen comments (``///`` or ``/** */``) on published APIs (public
  or protected classes and methods).  Code with private access or declared in
  ``.cc`` files should *not* use the Doxygen format.
- Most private methods with multiple callers should have a
  documentation comment (but not phrased as a Doxygen comment).
- Anything in your code that confuses you or makes you read it twice
  to understand its workings should have an implementation comment.

Are your comments clear and grammatical?
========================================

- If a comment is or is meant to be a complete sentence, capitalize
  it, punctuate it, and re-read it to make sure it parses!
- If your editor has a spell-checker for strings and comments, did you
  run it?

Did you use ``const`` where you could?
======================================

- A large majority of arguments and locals and a significant fraction
  of methods and fields can be made ``const``.  This improves
  compile-time error checking.

- Do all "plain old data" member fields have ``{}``?

  - See `our style guide <http://drake.mit.edu/styleguide/cppguide.html#Variable_and_Array_Initialization>`_
    citing "in-class member initialization."

Did you use a C-style cast by accident?
=======================================

- You usually want ``static_cast<To>(from)``.
- To cast to superclasses or subclasses, ``dynamic_cast<To>(from)``;
  however if ``To`` is a pointer type then you must always check for
  ``nullptr``.
- You very, very rarely want ``reinterpret_cast``.  Use with great
  caution.

Have you run linting tools?
===========================

- See :ref:`Automated style checks <code-style-tools>`.

Is your code deterministic?
===========================

- Do not use ``Eigen::Random``, ``libc rand``, or anything like it.
  You can use ``libstdc++``'s new random generators, as long as you
  call them using a local instance (no global state), and seed it with
  a hard-coded value for repeatability.  This includes test code.
