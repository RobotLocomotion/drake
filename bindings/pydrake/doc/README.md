# Python Binding Documentation

To build and view documentation:

    cd drake
    bazel run //bindings/pydrake/doc:serve_sphinx

To regenerate documentation modules:

    cd drake
    bazel run //bindings/pydrake/doc:refresh_doc_modules -- \
        --pre_clean --output_dir ${PWD}/bindings/pydrake/doc
