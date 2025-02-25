# Provide an easy way for downstream projects to patch out this warning.
SHOULD_PRINT = True

def print_warning(macro_name):
    if not SHOULD_PRINT:
        return
    print("WARNING: Using Drake as a WORKSPACE style external is deprecated and will be removed from Drake on or after 2025-09-01, including removal of the {macro_name} function. Please switch to MODULE style externals (i.e., Bzlmod) instead; for an example, see https://github.com/RobotLocomotion/drake-external-examples/tree/main/drake_bazel_external.".format(macro_name = macro_name))  # noqa
