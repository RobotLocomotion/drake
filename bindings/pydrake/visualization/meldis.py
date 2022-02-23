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
import sys
import time
import webbrowser

from drake import (
    lcmt_contact_results_for_viz,
    lcmt_viewer_draw,
    lcmt_viewer_geometry_data,
    lcmt_viewer_load_robot,
)
from pydrake.common import (
    configure_logging,
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
    MeshcatParams,
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
from pydrake.multibody.meshcat import (
    _PointContactVisualizer,
    _PointContactVisualizerItem,
    ContactVisualizerParams,
)

_logger = logging.getLogger("drake")


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
            _logger.warning(f"Unknown geom.type of {geom.type}")
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


class _ContactApplet:
    """Displays lcmt_contact_results_for_viz into Meshcat."""

    def __init__(self, *, meshcat):
        # By default, don't show any contact illustrations.
        meshcat.SetProperty("/CONTACT_RESULTS", "visible", False)

        # Add point visualization.
        params = ContactVisualizerParams()
        params.prefix = "/CONTACT_RESULTS/point"
        self._helper = _PointContactVisualizer(meshcat, params)

    def on_contact_results(self, message):
        """Handler for lcmt_contact_results_for_viz. Note that only point
        contacts are shown so far; hydroelastic contacts are not shown."""
        viz_items = []
        for lcm_item in message.point_pair_contact_info:
            viz_items.append(_PointContactVisualizerItem(
                body_A=lcm_item.body1_name,
                body_B=lcm_item.body2_name,
                contact_force=lcm_item.contact_force,
                contact_point=lcm_item.contact_point))
        self._helper.Update(viz_items)


class Meldis:
    """
    MeshCat LCM Display Server (MeLDiS)

    Offers a MeshCat vizualization server that listens for and draws Drake's
    legacy LCM vizualization messages.

    Refer to the pydrake.visualization.meldis module docs for details.
    """

    def __init__(self, *, meshcat_port=None):
        # Bookkeeping for update throtting.
        self._last_update_time = time.time()

        # Bookkeeping for subscriptions, keyed by LCM channel name.
        self._message_types = {}
        self._message_handlers = {}
        self._message_pending_data = {}

        self._lcm = DrakeLcm()
        lcm_url = self._lcm.get_lcm_url()
        _logger.info(f"Meldis is listening for LCM messages at {lcm_url}")

        params = MeshcatParams(host="localhost", port=meshcat_port)
        self.meshcat = Meshcat(params=params)

        viewer = _ViewerApplet(meshcat=self.meshcat)
        self._subscribe(channel="DRAKE_VIEWER_LOAD_ROBOT",
                        message_type=lcmt_viewer_load_robot,
                        handler=viewer.on_viewer_load)
        self._subscribe(channel="DRAKE_VIEWER_DRAW",
                        message_type=lcmt_viewer_draw,
                        handler=viewer.on_viewer_draw)

        contact = _ContactApplet(meshcat=self.meshcat)
        self._subscribe(channel="CONTACT_RESULTS",
                        message_type=lcmt_contact_results_for_viz,
                        handler=contact.on_contact_results)

        # Bookkeeping for automatic shutdown.
        self._last_poll = None
        self._last_active = None

    def _subscribe(self, channel, message_type, handler):
        """Subscribes the handler to the given channel, using message_type to
        pass in a decoded message object (not the raw bytes). The handler will
        only be called at some maximum frequency. Messages on the same channel
        that arrive too quickly will be discarded.
        """
        # Record this channel's type and handler.
        assert self._message_types.get(channel, message_type) == message_type
        self._message_types[channel] = message_type
        self._message_handlers.setdefault(channel, []).append(handler)

        # Subscribe using an internal function that implements "last one wins".
        # It's important to service the LCM queue as frequently as possible:
        #  https://github.com/RobotLocomotion/drake/issues/15234
        #  https://github.com/lcm-proj/lcm/issues/345
        # However, if the sender is transmitting visualization messages at
        # a high rate (e.g., if a sim is running much faster than realtime),
        # then we should only pass some of them along to MeshCat to avoid
        # flooding it. The hander merely records the message data; we'll
        # pass it along to MeshCat using our `self._should_update()` timer.
        def _on_message(data):
            self._message_pending_data[channel] = data
        self._lcm.Subscribe(channel=channel, handler=_on_message)

    def _invoke_subscriptions(self):
        """Posts any unhandled messages to their handlers and clears the
        collection of unhandled messages.
        """
        for channel, data in self._message_pending_data.items():
            message = self._message_types[channel].decode(data)
            for function in self._message_handlers[channel]:
                function(message=message)
        self._message_pending_data.clear()

    def serve_forever(self, *, idle_timeout=None):
        """Run indefinitely, forwarding LCM => MeshCat messages.

        If provided, the optional idle_timeout must be strictly positive and
        this loop will sys.exit after that many seconds without any websocket
        connections.
        """
        while True:
            self._lcm.HandleSubscriptions(timeout_millis=1000)
            if not self._should_update():
                continue
            self._invoke_subscriptions()
            self.meshcat.Flush()
            self._check_for_shutdown(idle_timeout=idle_timeout)

    def _should_update(self):
        """Post LCM-driven updates to MeshCat no faster than 40 Hz."""
        now = time.time()
        update_period = 0.025  # 40 Hz
        remaining = update_period - (now - self._last_update_time)
        if remaining > 0.0:
            return False
        else:
            self._last_update_time = now
            return True

    def _check_for_shutdown(self, *, idle_timeout):
        # Allow the user to opt-out of the timeout feature.
        if idle_timeout is None:
            return
        assert idle_timeout > 0.0

        # One-time initialization.
        now = time.time()
        if self._last_active is None:
            self._last_active = now
            return

        # Only check once every 5 seconds.
        if (self._last_poll is not None) and (now < self._last_poll + 5.0):
            return
        self._last_poll = now

        # Check to see if any browser client(s) are connected.
        if self.meshcat.GetNumActiveConnections() > 0:
            self._last_active = now
            return

        # In case we are idle for too long, exit automatically.
        if now > self._last_active + idle_timeout:
            _logger.info("Meldis is exiting now; no browser was connected for"
                         f" >{idle_timeout} seconds")
            sys.exit(1)


def _main():
    configure_logging()
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
    parser.add_argument(
        "--idle-timeout", metavar="TIME", type=float, default=15*60,
        help="When no web browser has been connected for this many seconds,"
        " this program will automatically exit. Set to 0 to run indefinitely.")
    args = parser.parse_args()
    meldis = Meldis(meshcat_port=args.port)
    if args.browser_new is not None:
        url = meldis.meshcat.web_url()
        webbrowser.open(url=url, new=args.browser_new)
    idle_timeout = args.idle_timeout
    if idle_timeout == 0.0:
        idle_timeout = None
    elif idle_timeout < 0.0:
        parser.error("The --idle_timeout cannot be negative.")
    meldis.serve_forever(idle_timeout=idle_timeout)


if __name__ == "__main__":
    _main()
