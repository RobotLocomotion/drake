---
title: Avoid Accidental Pushes to the Repository
---

# The Issue

We want to prevent developers from accidentally pushing changes into RobotLocomotion/drake. This is accomplished by:

1. Everybody agreeing on a naming convention for the repositories.
2. Providing a dummy url for the repository we want to protect.


# A Convention

To make the notion of ``origin`` consistent with the idea of the one repository you work with, we agree in the following convention:

* "upstream"  = RobotLocomotion/drake
* "origin" = my fork

so that a simple ``git push`` will push changes into a developer's fork. This still does not prevent you from pushing into ``upstream``. To protect developers from making this simple mistake the following solution is proposed.

# A Solution

A way to avoid pushing into a repository is providing a dummy url address to the repository. This is accomplished by issuing the git command:

```
git remote set-url --push upstream no_push
```

where ``no_push`` is the dummy url and ``upstream`` points to RobotLocomotion/drake (recall the convention is to call your fork ``origin``).

If a push to the master repository is attempted issuing a ``git push upstream my_branch``, git will return the following error message:

```
fatal: 'no_push' does not appear to be a git repository
fatal: Could not read from remote repository.

Please make sure you have the correct access rights
and the repository exists.
```
