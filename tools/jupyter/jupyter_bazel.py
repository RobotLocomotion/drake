import argparse
import os
import sys
import warnings

from jupyter_core.command import main as _jupyter_main
from nbconvert.preprocessors import ExecutePreprocessor
import nbformat
from python import runfiles

_ISSUE_12536_NOTES = """

For Drake developers:
    If you see the error "zmq.error.ZMQError: Address already in use" above,
    then this is a known flake when running under CI:
    https://github.com/RobotLocomotion/drake/issues/12536

"""


class _ExecutePreprocessorNoWidgets(ExecutePreprocessor):
    """Customizes ExecutePreprocessor so that ipywidgets works on Ubuntu 22.
    See https://github.com/RobotLocomotion/drake/issues/17433.
    """

    def preprocess_cell(self, *args, **kwargs):
        # Turn off some broken ipywidgets hooks.
        try:
            self.comm_open_handlers.clear()
        except AttributeError:
            pass
        # Then, continue as usual.
        return super().preprocess_cell(*args, **kwargs)


def _jupyter_bazel_notebook_main(notebook_respath, argv):
    # This should *ONLY* be called by targets generated via `jupyter_py_*`
    # rules.
    manifest = runfiles.Create()
    notebook_path = manifest.Rlocation(notebook_respath)

    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--test", action="store_true", help="Run as a test (non-interactive)"
    )
    args = parser.parse_args(argv)

    # The notebook should be run with the same dependencies as we have now.
    os.environ["PYTHONPATH"] = ":".join(sys.path)

    if not args.test:
        print("Running notebook interactively")
        notebook_path = os.path.realpath(notebook_path)
        sys.argv = ["jupyter", "notebook", notebook_path]
        exit(_jupyter_main())
    else:
        print("Running notebook as a test (non-interactive)")
        tmp_dir = os.environ.get("TEST_TMPDIR")
        if tmp_dir is not None:
            # Change IPython directory to use test directory.
            config_dir = os.path.join(tmp_dir, "jupyter")
            os.environ["IPYTHONDIR"] = config_dir
        # Escalate warnings for non-writable directories for settings
        # directories.
        warnings.filterwarnings(
            "error", message="IPython dir", category=UserWarning
        )
        # Execute using a preprocessor, rather than calling
        # `jupyter nbconvert`, as the latter writes an unused file to
        # `runfiles`.
        with open(notebook_path, encoding="utf-8") as f:
            nb = nbformat.read(f, as_version=4)
        # Ensure that we use the notebook's directory, since that is used for
        # interactive sessions.
        notebook_dir = os.path.dirname(notebook_path)
        # Ensure that Drake deprecations are seen as errors.
        # TODO(eric.cousineau): Rather than use environment variables, try
        # injecting code into the kernel.
        os.environ["_DRAKE_DEPRECATION_IS_ERROR"] = "1"
        # TODO(eric.cousineau): See if there is a way to redirect this to
        # stdout, rather than having the notebook capture the output.
        ep = _ExecutePreprocessorNoWidgets(timeout=600, kernel_name="python3")
        try:
            ep.preprocess(nb, resources={"metadata": {"path": notebook_dir}})
        except RuntimeError as e:
            if "Kernel died before replying to kernel_info" in str(e):
                print(_ISSUE_12536_NOTES, file=sys.stderr)
            raise
        print("Done")
