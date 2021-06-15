# Note that this script runs in the main context of drake-visulizer,
# where many modules and variables already exist in the global scope.

import numpy as np

from director import lcmUtils
from director import applogic
from director import objectmodel as om
from director import visualization as vis
from director import vtkNumpy as vnp

from drake import lcmt_point_cloud, lcmt_point_cloud_field

from _drake_visualizer_builtin_scripts import scoped_singleton_func


class PointCloudChannel:

    def __init__(self, parent_folder, channel):
        self._parent_folder = parent_folder
        self._name = channel

    def handle_message(self, msg):
        # Check that the message's metadata is only and exactly what we want.
        if msg.flags != lcmt_point_cloud.IS_STRICTLY_FINITE:
            return
        fields = (
            # (name, byte_offset, datatype, count)
            ("x", 0, lcmt_point_cloud_field.FLOAT32, 1),
            ("y", 4, lcmt_point_cloud_field.FLOAT32, 1),
            ("z", 8, lcmt_point_cloud_field.FLOAT32, 1),
            ("rgb", 12, lcmt_point_cloud_field.UINT32, 1),
        )
        if msg.num_fields != len(fields):
            return
        for i, (name, byte_offset, datatype, count) in enumerate(fields):
            if msg.fields[i].name != name:
                return
            if msg.fields[i].byte_offset != byte_offset:
                return
            if msg.fields[i].datatype != datatype:
                return
            if msg.fields[i].count != count:
                return

        # Swizzle the channel data into ndarrays.  The msg.data is formatted
        # as "xxxx yyyy zzzz rgb_" where "xxxx", "yyyy", and "zzzz" are floats,
        # "r", "g", and "b" are uint8, and "_" is one byte of padding.
        points = np.frombuffer(msg.data, dtype=np.float32)
        points.shape = (-1, 4)
        xyz = points[:, 0:3]
        rgb_and_padding_as_float = np.delete(points, range(3), 1)
        rgb_and_padding = rgb_and_padding_as_float.view(dtype=np.uint8)
        rgb = rgb_and_padding[:, 0:3]
        self._update_cloud(xyz, rgb)

    def _update_cloud(self, xyz, rgb):
        poly_data = vnp.numpyToPolyData(xyz)
        vnp.addNumpyToVtk(poly_data, rgb, "rgb")
        item = self._parent_folder.findChild(self._name)
        if item is not None:
            item.setPolyData(poly_data)
        else:
            view = applogic.getCurrentRenderView()
            item = vis.PolyDataItem(self._name, poly_data, view=view)
            item.setProperty("Color By", "rgb")
            om.addToObjectModel(item, parentObj=self._parent_folder)
        item._updateColorByProperty()

    def _get_folder(self):
        return om.getOrCreateContainer(
            self._name, parentObj=self._parent_folder)

    def remove_folder(self):
        om.removeFromObjectModel(self._get_folder())


class PointCloudsVisualizer:

    def __init__(self):
        self._name = "Point Clouds Visualizer"
        self._subscriber = None
        self._children = {}
        self.set_enabled(True)

    def _add_subscriber(self):
        if (self._subscriber is not None):
            return
        self._subscriber = lcmUtils.addSubscriber(
            'DRAKE_POINT_CLOUD_.*',
            messageClass=lcmt_point_cloud,
            callback=self._handle_message,
            callbackNeedsChannel=True)
        self._subscriber.setNotifyAllMessagesEnabled(True)

    def _get_folder(self):
        return om.getOrCreateContainer(self._name)

    def _remove_subscriber(self):
        if (self._subscriber is None):
            return
        lcmUtils.removeSubscriber(self._subscriber)
        for child in self._children:
            child.remove_folder()
        self._children.clear()
        self._subscriber = None
        om.removeFromObjectModel(self._get_folder())

    def is_enabled(self):
        return self._subscriber is not None

    def set_enabled(self, enable):
        if enable:
            self._add_subscriber()
        else:
            self._remove_subscriber()

    def _handle_message(self, msg, channel):
        child = self._children.get(channel)
        if not child:
            child = PointCloudChannel(
                parent_folder=self._get_folder(), channel=channel)
            self._children[channel] = child
        child.handle_message(msg)


@scoped_singleton_func
def init_visualizer():
    point_cloud_viz = PointCloudsVisualizer()

    # Adds to the "Tools" menu.
    applogic.MenuActionToggleHelper(
        'Tools', point_cloud_viz._name,
        point_cloud_viz.is_enabled, point_cloud_viz.set_enabled)
    return point_cloud_viz


# Activate the plugin if this script is run directly; store the results to keep
# the plugin objects in scope.
if __name__ == "__main__":
    point_cloud_viz = init_visualizer()
