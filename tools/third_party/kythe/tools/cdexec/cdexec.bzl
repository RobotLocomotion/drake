"""Skylark module with utilities and macros for running actions with cdexec."""

def rootpath(path):
  """Returns a string which cdexec will interpret as absolute.

  Specifically, if the path starts with '/', it is returned unmodified.
  Otherwise, the returns the path prefixed by the literal string '${PWD}/'
  which will be expanded by the `cdexec` script into the Bazel exec root.
  """
  if path.startswith("/"):
    return path
  return "${PWD}/" + path
