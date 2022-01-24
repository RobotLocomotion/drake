"""
MeshCat LCM Display Server (MeLDiS)

A standalone program that can display Drake visualizations in MeshCat
by listing for LCM messages that are broadcast by the simulation.

This can stand in for the legacy ``drake-visualizer`` application of
days past.

From a Drake source build, run this as:

  bazel run //tools:meldis &

From a Drake binary release, run this as:

  python3 -m pydrake.visualization.meldis

In many cases, passing ``-w`` (i.e., ``--open-window``) to the program will be
convenient.

  bazel run //tools:meldis -- -w &
"""

import argparse
import logging
import numpy as np
import webbrowser

from drake import (
    lcmt_viewer_draw,
    lcmt_viewer_geometry_data,
    lcmt_viewer_load_robot,
)
from pydrake.common.eigen_geometry import (
    Quaternion,
)
from pydrake.geometry import (
    Box,
    Capsule,
    Cylinder,
    Ellipsoid,
    Mesh,
    Meshcat,
    Rgba,
    Sphere,
)
from pydrake.lcm import (
    DrakeLcm,
)
from pydrake.math import (
    RigidTransform,
    RotationMatrix,
)


class _ViewerApplet:
    """Displays lcmt_viewer_load_robot and lcmt_viewer_draw into MeshCat."""

    def __init__(self, *, meshcat):
        self._meshcat = meshcat
        self._path = "/DRAKE_VIEWER"

        # Initialize ourself with an empty load message.
        self.on_viewer_load(message=lcmt_viewer_load_robot())

    def on_viewer_load(self, message):
        """Handler for lcmt_viewer_load."""
        # The semantics of a load message is to reset the entire scene.
        self._meshcat.Delete(path=self._path)

        # Make all of our (ViewerApplet's) geometry invisible so that the
        # lcmt_viewer_load geometry does not clutter up the scene until we
        # are given its poses in a lcmt_viewer_draw message.
        self._set_visible(False)
        self._waiting_for_first_draw_message = True

        # Add the links and their geometries.
        for link in message.link:
            robot_num = link.robot_num
            link_name = link.name.replace("::", "/")
            link_path = f"{self._path}/{robot_num}/{link_name}"
            for j, geom in enumerate(link.geom):
                geom_path = f"{link_path}/{j}"
                shape, rgba, pose = self._convert_geom(geom)
                if shape is None:
                    continue
                self._meshcat.SetObject(path=geom_path, shape=shape, rgba=rgba)
                self._meshcat.SetTransform(path=geom_path, X_ParentPath=pose)

    def on_viewer_draw(self, message):
        """Handler for lcmt_viewer_draw."""
        for i in range(message.num_links):
            link_name = message.link_name[i].replace("::", "/")
            robot_num = message.robot_num[i]
            link_path = f"{self._path}/{robot_num}/{link_name}"
            pose = self._to_pose(message.position[i], message.quaternion[i])
            self._meshcat.SetTransform(path=link_path, X_ParentPath=pose)
        if self._waiting_for_first_draw_message:
            self._waiting_for_first_draw_message = False
            self._set_visible(True)

    def _set_visible(self, value):
        self._meshcat.SetProperty(self._path, property="visible", value=value)

    def _convert_geom(self, geom):
        """Given an lcmt_viewer_geometry_data, parse it into a tuple of (Shape,
        Rgba, RigidTransform)."""
        shape = None
        if geom.type == lcmt_viewer_geometry_data.BOX:
            (width, depth, height) = geom.float_data
            shape = Box(width=width, depth=depth, height=height)
        elif geom.type == lcmt_viewer_geometry_data.CAPSULE:
            (radius, length) = geom.float_data
            shape = Capsule(radius=radius, length=length)
        elif geom.type == lcmt_viewer_geometry_data.CYLINDER:
            (radius, length) = geom.float_data
            shape = Cylinder(radius=radius, length=length)
        elif geom.type == lcmt_viewer_geometry_data.ELLIPSOID:
            (a, b, c) = geom.float_data
            shape = Ellipsoid(a=a, b=b, c=c)
        elif geom.type == lcmt_viewer_geometry_data.MESH:
            (scale_x, scale_y, scale_z) = geom.float_data
            filename = geom.string_data
            assert scale_x == scale_y and scale_y == scale_z
            shape = Mesh(absolute_filename=filename, scale=scale_x)
        elif geom.type == lcmt_viewer_geometry_data.SPHERE:
            (radius,) = geom.float_data
            shape = Sphere(radius=radius)
        else:
            logging.warning(f"Unknown geom.type of {geom.type}")
            return (None, None, None)
        rgba = Rgba(*geom.color)
        pose = self._to_pose(geom.position, geom.quaternion)
        return (shape, rgba, pose)

    @staticmethod
    def _to_pose(position, quaternion):
        """Given pose parts of an lcmt_viewer_geometry_data, parse it into a
        RigidTransform."""
        (p_x, p_y, p_z) = position
        (q_w, q_x, q_y, q_z) = quaternion
        return RigidTransform(
            R=RotationMatrix(
                quaternion=Quaternion(
                    w=q_w,
                    x=q_x,
                    y=q_y,
                    z=q_z,
                ),
            ),
            p=(p_x, p_y, p_z))


class Meldis:
    """
    Offers a MeshCat vizualization server that listens for and draws Drake's
    legacy LCM vizualization messages.
    """

    def __init__(self, meshcat_port=None):
        self._lcm = DrakeLcm()
        lcm_url = self._lcm.get_lcm_url()
        logging.info(f"Meldis is listening for LCM messages at {lcm_url}")

        self.meshcat = Meshcat(port=meshcat_port)

        viewer = _ViewerApplet(meshcat=self.meshcat)
        self._subscribe(channel="DRAKE_VIEWER_LOAD_ROBOT",
                        message_type=lcmt_viewer_load_robot,
                        handler=viewer.on_viewer_load)
        self._subscribe(channel="DRAKE_VIEWER_DRAW",
                        message_type=lcmt_viewer_draw,
                        handler=viewer.on_viewer_draw)

        # TODO(jwnimmer-tri) Add an applet for lcmt_contact_results_for_viz.

    def _subscribe(self, channel, message_type, handler):
        def _parse_and_handle(data):
            handler(message=message_type.decode(data))
        self._lcm.Subscribe(channel=channel, handler=_parse_and_handle)

    def serve_forever(self):
        # TODO(jwnimmer-tri) If there are no browser connections open for some
        # period of time, we should probably give up and quit, rather than
        # leave a zombie meldis running forever.
        while True:
            self._lcm.HandleSubscriptions(timeout_millis=1000)


def _main():
    format = "[%(asctime)s] [console] [%(levelname)s] %(message)s"
    logging.basicConfig(level=logging.INFO, format=format)
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "-p", "--port", action="store", metavar="NUM", type=int,
        help="The http listen port for MeshCat. If none is given, a default"
        " will be chosen and printed to the console.")
    parser.add_argument(
        "-t", "--open-tab", dest="browser_new",
        action="store_const", const=2, default=None,
        help="Open the MeshCat display in a browser tab.")
    parser.add_argument(
        "-w", "--open-window", dest="browser_new",
        action="store_const", const=1, default=None,
        help="Open the MeshCat display in a new browser window.")
    args = parser.parse_args()
    meldis = Meldis(meshcat_port=args.port)
    if args.browser_new is not None:
        url = meldis.meshcat.web_url()
        webbrowser.open(url=url, new=args.browser_new)
    meldis.serve_forever()


if __name__ == "__main__":
    _main()
