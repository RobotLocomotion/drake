import datetime
from hashlib import sha256
from multiprocessing import Process
import os
from pathlib import Path
import re
import shutil
import subprocess
import sys
import time
from typing import Any, List, Tuple
import unittest

this_file_dir = Path(__file__).parent.absolute()
sys.path.insert(0, str(this_file_dir))


def setup(**decorator_kwargs):
    """
    Decorator to add arbitrary attributes to a test function.

    In order to change behavior in the :func:`GltfRenderServer.setUp`, such as
    changing the width or height of a render, the endpoint of the server, etc,
    there needs to be a way for the test to communicate these changes.  The
    ``setUp`` method is called before the test is run, so this decorator just
    adds the attributes to the wrapped function returned.
    """

    def decorator(func):
        def wrapper(*args, **kwargs):
            return func(*args, **kwargs)

        for key, val in decorator_kwargs.items():
            setattr(wrapper, key, val)

        return wrapper

    return decorator


def run_server(
    *,
    render_endpoint: str = "render",
    host: str = "127.0.0.1",
    port: int = 8000,
):
    """
    Runs the ``test_server_gltf_render_server`` on the specified host and port,
    setting the ``RENDER_ENDPOINT`` environment variable to the provided
    ``render_endpoint``.  This is used as the ``target`` parameter for a
    :class:`python:multiprocessing.Process` class created in
    :func:`GltfRenderServer.setUp` for a given test.  This method should not be
    called directly, calling ``app.run(...)`` blocks until SIGTERM is sent.
    """
    # NOTE: defer importing until the test is running to allow modifying of the
    # RENDER_ENDPOINT environment variable to affect the server endpoint.
    os.environ["RENDER_ENDPOINT"] = render_endpoint
    from gltf_render_server import app

    app.run(host=host, port=port)
    del os.environ["RENDER_ENDPOINT"]


class GltfRenderServerRoundTrip(unittest.TestCase):
    """
    Each test case will spawn a new instance of the glTF render server and
    render a scene with both ``RenderEngineVtk`` (VTK) as well as
    ``RenderEngineGltfClient`` (glTF).  Each frame of the VTK and glTF renders
    will be saved to their own image directories, and the image results will
    be compared.  This test validates both that the ``RenderEngineGltfClient``
    is able to communicate with a server, as well as validates the results with
    known good rendering results.

    The scene simulation is in the ``server_simulation.cc`` test file, and
    the render server is in the ``gltf_render_server.py`` file.  The simulation
    is responsible for creating the scene and invoking either the VTK or glTF
    renderer, as well as saving the image files to disk.

    - Color images: comparisons are made via computing the image difference via
      the ``color_image_difference`` executable.  As long as the computed image
      difference threshold is less than
      :attr:`GltfRenderServerRoundTrip.MAX_IMAGE_DIFFERENCE_THRESHOLD` the test
      is considered a success.
    - Depth images: TODO.  The depth images should be loaded via a proxy
      executable similar to how color images work, and compute differences.
    - Label images: the sha256 hash of the images is compared, meaning label
      images are identical at a pixel level.

    A given test case is primarily parameterized by its :func:`setup` decorator
    which will in turn alter the behavior of the
    :func:`GltfRenderServerRoundTrip.setUp` method.  This is achieved by
    attaching attributes to the function, which are later extracted in the
    ``setUp`` method to alter the behavior of the server in particular.  It is
    required to work this way only to test changing the server endpoint (see
    also the :func:`run_server` method).  Running a flask server will block the
    process until it is killed, so it must be run in a
    :class:`python:multiprocessing.Process`.

    As a side-effect, the remaining parameters enabled in the ``setup``
    decorator that control the behavior of the VTK and glTF simulation renders
    are there for convenience -- the ``setUp`` method creates everything the
    test needs, e.g., directories to save the image files in to compare.

    After the test has been configured and a server is running, image
    comparisons can be made in the test body.  Finally, the :func:`tearDown`
    method will stop the server, delete any files created, and unset the state
    created in ``setUp``.
    """

    MAX_IMAGE_DIFFERENCE_THRESHOLD = 10.0
    """The maximum image difference threshold to allow for a test to pass."""

    def setUp(self):
        """Spawn a server and initialize the relevant attributes for running
        both the VTK and glTF render."""
        # Configure and spawn a server for the current test.
        render_endpoint = self._get_test_attr(
            self._testMethodName, "render_endpoint", "render"
        )
        self.server_proc = None
        # Search for an available port, starting at a port that is less likely
        # to be occupied.  Each attempt takes one second, so if too many
        # attempts are tried (relatively small port range) the test is failed
        # after the loop.
        port_range = (5127, 5172)
        host = "127.0.0.1"
        for port in range(*port_range):
            server_proc = Process(
                target=run_server,
                kwargs={
                    "render_endpoint": render_endpoint,
                    "host": host,
                    "port": port,
                },
            )
            server_proc.start()

            # NOTE: checking instantly does not give flask enough time to crash
            # so we must pause briefly before checking.
            time.sleep(1)

            if server_proc.is_alive():
                self.server_proc = server_proc
                self.port = port
                break

        if self.server_proc is None:
            self.fail("Could not find an open port.")

        # Arguments for running the run_simulation_and_render.
        simulation_time = self._get_test_attr(
            self._testMethodName, "simulation_time", 0.2
        )
        render_fps = self._get_test_attr(
            self._testMethodName, "render_fps", 10
        )
        camera_xyz_rpy = self._get_test_attr(
            self._testMethodName,
            "camera_xyz_rpy",
            "0.8, 0.0, 0.5, -2.2, 0.0, 1.57",
        )
        self.width = self._get_test_attr(self._testMethodName, "width", 320)
        self.height = self._get_test_attr(self._testMethodName, "height", 240)

        # Make save directories for VTK and glTF.
        curr_time = datetime.datetime.now()
        curr_time_str = curr_time.strftime("%Y-%m-%d_%H-%M-%S-%f")
        base = this_file_dir / f"{self._testMethodName}_{curr_time_str}"
        # The test cases gather files from here using these attributes.
        self.vtk_save_dir = base / "vtk"
        self.vtk_save_dir.mkdir(parents=True, exist_ok=True)
        self.gltf_save_dir = base / "gltf"
        self.gltf_save_dir.mkdir(parents=True, exist_ok=True)
        self.image_diff_save_dir = base / "image_diff"
        self.image_diff_save_dir.mkdir(parents=True, exist_ok=True)

        # Create the subprocess arguments for the test cases.
        render_args_base = [
            "--nodrake_visualizer",
            "--simulation_time",
            str(simulation_time),
            # TODO(svenevs): --color / --nocolor, depth, label if validating
            # just one set per test rather than all is preferred?
            "--render_fps",
            str(render_fps),
            "--camera_xyz_rpy",
            camera_xyz_rpy,
            "--width",
            str(self.width),
            "--height",
            str(self.height),
        ]
        self.vtk_render_args = [
            *render_args_base,
            "--render_engine",
            "vtk",
            "--save_dir",
            str(self.vtk_save_dir),
        ]
        self.gltf_render_args = [
            *render_args_base,
            "--render_engine",
            "client",
            "--save_dir",
            str(self.gltf_save_dir),
            "--url",
            host,
            "--port",
            str(port),
            "--render_endpoint",
            render_endpoint,
        ]

    def _get_test_attr(self, test_method_name: str, attr: str, default: Any):
        """Extract an attribute from a given test function being setUp."""
        try:
            test_func = getattr(self, test_method_name).__func__
            return getattr(test_func, attr, default)
        except Exception:
            return default

    def tearDown(self):
        """Shutdown the server, cleanup created files, and de-initialize the
        state created in :func:`setUp`."""
        # Shutdown the server.
        if self.server_proc is not None:
            self.server_proc.kill()  # Flask server has to be killed to end.
            self.server_proc = None

        # Delete the files that were generated for the image comparisons.
        # NOTE: use --sandbox_debug *AND* comment these out to inspect files.
        # They will end up in a location such as:
        #   ~/.cache/bazel/_bazel_{user}/f215ccf719ba7a61cca4c58d38de2749/
        #     sandbox/linux-sandbox/491/execroot/drake/bazel-out/k8-opt/bin/
        #     geometry/render/dev/py/client_server_round_trip_test.runfiles/
        #     drake/geometry/render/dev/test/
        #       {test_method_name}_{timestamp}, e.g.
        #       test_root_endpoint_2022-03-18_13-33-34-877389
        shutil.rmtree(self.vtk_save_dir, ignore_errors=True)
        shutil.rmtree(self.gltf_save_dir, ignore_errors=True)
        shutil.rmtree(self.image_diff_save_dir, ignore_errors=True)

        # Undo any state created in setUp, next test will overwrite.
        self.port = None
        self.width = None
        self.height = None
        self.vtk_save_dir = None
        self.gltf_save_dir = None
        self.image_diff_save_dir = None
        self.vtk_render_args = None
        self.gltf_render_args = None

    def run_simulation(self, simulation_args: List[str]):
        """Run ``run_simulation_and_render`` with the specified arguments."""
        # NOTE: this path is only valid in bazel, see data dependencies.
        exe_path = this_file_dir.parent / "run_simulation_and_render"
        subprocess.run([exe_path, *simulation_args])

    def run_vtk_and_gltf_simulations(self):
        """Run the test simulation to save image results for VTK and glTF."""
        self.run_simulation(self.vtk_render_args)
        self.run_simulation(self.gltf_render_args)

    def gather_image_pairs(self, image_type: str) -> List[Tuple[Path, Path]]:
        """Return a list of tuples of images to validate::

        [
            (vtk_image_0, gltf_image_0),
            # ...
            (vtk_image_N, gltf_image_N)
        ]
        """
        # TODO(svenevs): 16bit depth support *may* break here if added.  If
        # save_dir saves .png instead of .tiff.  It may not need to?
        if image_type == "color":
            pattern = "color_*.png"
        elif image_type == "depth":
            pattern = "depth_*.tiff"
        elif image_type == "label":
            pattern = "label_*.png"
        else:
            self.fail(f"Unexpected image_type '{image_type}'.")
        # NOTE: these patterns (and sorting based off of them) rely on the
        # save_dir pattern being used in the test simulation executable.
        vtk = list(sorted(self.vtk_save_dir.glob(pattern)))
        gltf = list(sorted(self.gltf_save_dir.glob(pattern)))
        self.assertEqual(
            [v.name for v in vtk],
            [g.name for g in gltf],
            (
                "Did not recover the same number of image files for VTK and "
                "glTF renders, cannot validate their contents!\n"
                f"VTK save directory: {str(self.vtk_save_dir)}\n"
                f"  Gathered images: {vtk}\n"
                f"glTF save directory: {str(self.gltf_save_dir)}\n"
                f"  Gathered images: {gltf}\n"
            ),
        )
        return [image_pair for image_pair in zip(vtk, gltf)]

    def compute_color_image_differences(
        self, image_pairs: List[Tuple[Path, Path]]
    ) -> List[Tuple[Path, float]]:
        """Compute the color image difference, return a list in the same order
        as the input that has the (image, threshold error) pairs computed.

        This method **assumes** that the input ``image_pairs`` comes from
        :func:`gather_image_pairs`.  The naming scheme relies on this."""
        # NOTE: this path is only valid in bazel, see data dependencies.
        exe_path = this_file_dir.parent / "image_difference"
        differences = []
        for vtk, gltf in image_pairs:
            # vtk and gltf should be the same basename, just in different
            # folders.  Named such as color_000.png, for example.  Extract the
            # digits to use for the image difference filenames.
            match = re.search(r"^color_([0-9]+)\.png$", vtk.name)
            if match is None:
                self.fail(f"Could not recover digits from {vtk.name}.")

            digits = match.groups()[0]
            diff_path = self.image_diff_save_dir / f"diff_{digits}.png"
            threshold_path = self.image_diff_save_dir / f"diff_{digits}.txt"
            subprocess.check_call(
                [
                    str(exe_path),
                    "--source_one",
                    str(vtk),
                    "--source_two",
                    str(gltf),
                    "--diff_path",
                    str(diff_path),
                    "--threshold_path",
                    str(threshold_path),
                ]
            )
            self.assertTrue(
                diff_path.is_file(),
                f"Error: diff image {str(diff_path)} not saved.",
            )
            self.assertTrue(
                threshold_path.is_file(),
                f"Error: diff threshold {str(threshold_path)} not saved.",
            )
            threshold = None
            try:
                with open(threshold_path, "r") as f:
                    threshold = float(f.read().strip())
            except Exception as e:
                self.fail(
                    "Unable to extract difference threshold from "
                    f"{str(threshold_path)}. {e}"
                )

            differences.append((diff_path, threshold))

        return differences

    def compute_sha256(self, image: Path):
        """Compute and return the sha256 hash of the specified image file."""
        try:
            with open(image, "rb") as f:
                return sha256(f.read()).hexdigest()
        except Exception as e:
            self.assertTrue(
                False, f"Unable to compute sha256 of {str(image)}. {e}"
            )

    def compare_image_sha256_sums(self, image_pairs: List[Tuple[Path, Path]]):
        """Compare the sha256 hash of every pair in ``image_pairs``.  Should
        only be called for depth and label images."""
        for vtk, gltf in image_pairs:
            vtk_sha256 = self.compute_sha256(vtk)
            gltf_sha256 = self.compute_sha256(gltf)
            self.assertEqual(
                vtk_sha256,
                gltf_sha256,
                f"Images '{str(vtk)}' and '{str(gltf)}' were different.",
            )

    def compare_color_images(self):
        """Compare color images, may only be called **after**
        :func:`run_vtk_and_gltf_simulations` for a given test."""
        color_pairs = self.gather_image_pairs("color")
        color_diffs = self.compute_color_image_differences(color_pairs)
        for diff_path, threshold in color_diffs:
            self.assertTrue(
                threshold <= self.MAX_IMAGE_DIFFERENCE_THRESHOLD,
                f"Computed image difference in '{str(diff_path)}' had error "
                f"threshold of {threshold}, which is too large.  The maximum "
                f"allowable error is {self.MAX_IMAGE_DIFFERENCE_THRESHOLD}.",
            )

    def compare_depth_images(self):
        """Compare depth images, may only be called **after**
        :func:`run_vtk_and_gltf_simulations` for a given test."""
        # TODO(svenevs): unfortunately the depth images are not identical, so
        # a more intelligent comparison needs to be created.
        # depth_pairs = self.gather_image_pairs("depth")
        # self.compare_image_sha256_sums(depth_pairs)
        pass

    def compare_label_images(self):
        """Compare label images, may only be called **after**
        :func:`run_vtk_and_gltf_simulations` for a given test."""
        label_pairs = self.gather_image_pairs("label")
        self.compare_image_sha256_sums(label_pairs)

    def simulate_and_compare(self):
        """The main test body for any given test case, run the simulation for
        both VTK and glTF renders, and compare the images."""
        self.run_vtk_and_gltf_simulations()
        self.compare_color_images()
        self.compare_depth_images()
        self.compare_label_images()

    @setup(width=200, height=400)
    def test_render_server(self):
        """Test the default endpoint with alternative image dimensions."""
        self.simulate_and_compare()

    @setup(
        render_endpoint="alternative/endpoint",
        camera_xyz_rpy="-0.8, 0.0, 0.5, -2.2, 0.0, -1.57",
    )
    def test_alternative_endpoint(self):
        """Test an alternative endpoint with a different point of view."""
        self.simulate_and_compare()

    @setup(render_endpoint="", camera_xyz_rpy="0.0, 1.0, 0.0, 1.57, 3.14, 0.0")
    def test_root_endpoint(self):
        """Test that ``/`` can be used as an endpoint, with a different point
        of view."""
        self.simulate_and_compare()
