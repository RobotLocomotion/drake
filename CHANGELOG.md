Change Log
====================

Introduction
------------
Drake is undergoing continuous development and some of changes may break its API and ABI. To help downstream users cope, this document provides brief summaries of the changes. The changes are organized by pull requests that are merged into Drake's master branch.

Descriptions of Pull Requests
------------------------------

### PR 1953

PR link: https://github.com/RobotLocomotion/drake/pull/1953

Method `valuecheckMatrix()` in `drake/util/testUtil.h` was removed and replaced with `drake::util::CompareMatrices()` in `drake/util/eigen_matrix_compare.h`.

Unit tests that originally contained:

    valuecheckMatrix(matrix1, matrix2, tolerance);

should be modified to use Google Test and contain the following line:

    EXPECT_TRUE(drake::util::CompareMatrices(matrix1, matrix2, tolerance, MatrixCompareType::absolute));