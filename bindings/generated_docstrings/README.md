# Generated docstrings

Instead of generating live docstrings on the fly as part of the build, we vendor
them under `drake/bindings/generated_docstrings`. This has two advantages:

- Changing a header file doesn't force regeneration of docstrings, which has
  high latency in an edit-test cycle; instead, docstring updates are deferred
  until the linting step where the committed docstrings are cross-checked versus
  the expected docstrings.

- End-user builds on unsupported platforms don't need to worry about providing a
  compatible libclang.so for header parsing.

To re-geneate these documentation files from the C++ API, run:
```console
bazel run //bindings/generated_docstrings:regenerate
```

For details on how to use these files, refer to the "Documentation" section of
https://drake.mit.edu/doxygen_cxx/group__python__bindings.html
