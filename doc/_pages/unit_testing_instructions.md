---
title: Detailed Notes on Drake's Unit Tests
---

# Introduction

Unit tests are essential for software maintainability. They demonstrate the
correctness of existing code and prevent future changes from breaking the
past functionality (i.e., regressions). They are the only
way for developers to inform Drake's
[Continuous Integration (CI)](/developers.html#continuous-integration-notes) service how
the software is *supposed* to behave.

# Unit Testing Frameworks

## C++ Code

Please use the
[Google Test Framework](https://github.com/google/googletest). It is already
available as a required external in the Bazel build.
