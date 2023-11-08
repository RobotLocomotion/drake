# Upgrading meshcat commit

As is typical of GUI-based applications, meshcat is not easily tested via
automated CI processes. As such, when bumping the meshcat commit, it's NOT
enough to simply let CI run its course; local testing is also required.

The local testing consists largely of informal prodding: run an application
which exercises meshcat and confirm that is reasonably well behaved.

Required:
  - drake/geometry:meshcat_manual_test
    - Run `bazel run //geometry:meshcat_manual_test` and then follow the
      instructions on your console.
