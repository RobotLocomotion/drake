# -*- python -*-

# From https://bazel.build/versions/master/docs/be/c-cpp.html#cc_library.srcs
_SOURCE_EXTENSIONS = [source_ext for source_ext in """
.c
.cc
.cpp
.cxx
.c++.C
.h
.hh
.hpp
.hxx
.inc
""".split("\n") if len(source_ext)]

# The cpplint.py command-line argument so it doesn't skip our files!
_EXTENSIONS_ARGS = ["--extensions=" + ",".join([
  ext[1:] for ext in _SOURCE_EXTENSIONS]
)]

# From https://bazel.build/versions/master/docs/be/c-cpp.html#cc_library.srcs
_NON_SOURCE_EXTENSIONS = [non_source_ext for non_source_ext in """
.S
.a
.lo
.o
.pic.a
.pic.lo
.pic.o
.so
""".split("\n") if len(non_source_ext)]

def _extract_labels(srcs):
  """Convert a srcs= or hdrs= value to its set of labels."""
  # Tuples are already labels.
  if type(srcs) == type(()):
    return list(srcs)
  # The select() syntax returns an object we (apparently) can't inspect.
  # TODO(jwnimmer-tri) Figure out how to cpplint these files.  For now,
  # folks will have to pass extra_srcs when calling cpplint() macro.
  return []

def _is_source_label(label):
  for extention in _SOURCE_EXTENSIONS:
      if label.endswith(extention):
        return True
  for extention in _NON_SOURCE_EXTENSIONS:
      if label.endswith(extention):
        return False
  fail("Unknown extension for source " + label)

def cpplint(data=None, extra_srcs=None):
  """For every C++ rule in the BUILD file so far, adds a test rule that runs
  cpplint over the C++ sources listed in that rule.  Thus, BUILD file authors
  should call this function at the *end* of every C++-related BUILD file.

  By default, only the CPPLINT.cfg from the project root and the current
  directory are used.  Additional configs can be passed in as data labels.

  Sources that are not discoverable through the "sources so far" heuristic can
  be passed in as extra_srcs=[].

  """
  # Common attributes for all of our py_test invocations.
  srcs = ["@google_styleguide//:cpplint"]
  data = ["//:CPPLINT.cfg"] + native.glob(['CPPLINT.cfg']) + (data or [])
  main = "cpplint.py"
  size = "small"
  tags = ["cpplint"]

  # Iterate over all C++ rules.
  for rule in native.existing_rules().values():
    if not rule["kind"].startswith("cc_"):
      continue

    # Extract the list of source code labels and convert to filenames.
    candidate_labels = (
      _extract_labels(rule.get("srcs", ())) +
      _extract_labels(rule.get("hdrs", ()))
    )
    source_labels = [
      label for label in candidate_labels
      if _is_source_label(label)
    ]
    if len(source_labels) == 0:
      if len(candidate_labels) == 0:
        continue
      fail("No sources found in " + rule["name"])
    source_filenames = ["$(location %s)" % x for x in source_labels]

    # Run the cpplint checker as a unit test.
    native.py_test(
      name = rule["name"] + "_cpplint",
      srcs = srcs,
      data = data + source_labels,
      args = _EXTENSIONS_ARGS + source_filenames,
      main = main,
      size = size,
      tags = tags,
    )

  # Lint all of the extra_srcs separately in a single rule.
  if extra_srcs:
    source_labels = extra_srcs
    source_filenames = ["$(location %s)" % x for x in source_labels]
    native.py_test(
      name = "extra_srcs_cpplint",
      srcs = srcs,
      data = data + source_labels,
      args = _EXTENSIONS_ARGS + source_filenames,
      main = main,
      size = size,
      tags = tags,
    )
