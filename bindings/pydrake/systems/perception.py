import numpy as np

from pydrake.math import RigidTransform
from pydrake.perception import BaseField, Fields, PointCloud
from pydrake.systems.framework import AbstractValue, LeafSystem


def _tile_colors(color, count):
    # Need manual broadcasting.
    color = np.asarray(color)
    assert color.shape == (3,)
    return np.tile(np.array([color]).T, (1, count))


def _hstack_none(A, B):
    # Concatenate, accommodating None.
    if A is None:
        return B
    else:
        return np.hstack((A, B))


def _concatenate_point_clouds(p_FSilist_list, color_Silist_list):
    n = len(p_FSilist_list)
    assert n > 0
    assert len(color_Silist_list) == n
    p_FSlist = None
    color_Slist = None
    for p_FSilist, color_Silist in zip(p_FSilist_list, color_Silist_list):
        p_FSlist = _hstack_none(p_FSlist, p_FSilist)
        color_Slist = _hstack_none(color_Slist, color_Silist)
    valid_indices = np.logical_not(np.isnan(p_FSlist).any(axis=0))
    p_FSlist = p_FSlist[:, valid_indices]
    color_Slist = color_Slist[:, valid_indices]
    return p_FSlist, color_Slist


class PointCloudConcatenation(LeafSystem):

    def __init__(self, id_list, default_rgb=[255., 255., 255.]):
        """
        A system that takes in N point clouds, each a list of points Si in
        frame Ci denoted as p_CiSilist, and N RigidTransforms representing the
        pose of frame Ci relative to F, where F is the common frame to
        transform clouds. The system returns one point cloud combining all of
        the transformed point clouds as a list of points S (containing the
        points of S0, ..., SN) in frame F. Each point cloud must have XYZs.
        RGBs are optional. If absent, those points will be the provided default
        color.

        @param id_list A list containing the string IDs of all of the point
            clouds. This is often the serial number of the camera they came
            from, such as "1" for a simulated camera or "805212060373" for a
            real camera.
        @param default_rgb A list of length 3 containing the RGB values to use
            in the absence of PointCloud.rgbs. Values should be between 0 and
            255. The default is white.

        @system{
          @input_port{point_cloud_CiSi_id0}
          @input_port{X_FCi_id0}
          .
          .
          .
          @input_port{point_cloud_CiSi_idN}
          @input_port{X_FCi_idN}
          @output_port{point_cloud_FS}
        }
        """
        LeafSystem.__init__(self)
        self._point_cloud_ports = {}
        self._transform_ports = {}
        self._id_list = id_list
        self._default_rgb = np.array(default_rgb)
        output_fields = Fields(BaseField.kXYZs | BaseField.kRGBs)
        for i in self._id_list:
            self._point_cloud_ports[i] = self.DeclareAbstractInputPort(
                "point_cloud_CiSi_{}".format(i),
                AbstractValue.Make(PointCloud(fields=output_fields)))

            self._transform_ports[i] = self.DeclareAbstractInputPort(
                "X_FCi_{}".format(i),
                AbstractValue.Make(RigidTransform.Identity()))
        self.DeclareAbstractOutputPort("point_cloud_FS",
                                       lambda: AbstractValue.Make(
                                           PointCloud(fields=output_fields)),
                                       self.DoCalcOutput)

    def _align_point_clouds(self, context):
        p_FSilist_list = []
        color_Silist_list = []
        for i in self._id_list:
            point_cloud_CiSi = self._point_cloud_ports[i].Eval(context)
            X_FCi = self._transform_ports[i].Eval(context)
            p_CiSilist = point_cloud_CiSi.xyzs()
            p_FSilist = X_FCi.multiply(p_CiSilist)

            if point_cloud_CiSi.has_rgbs():
                color_Silist = point_cloud_CiSi.rgbs()
            else:
                color_Silist = _tile_colors(
                    self._default_rgb, point_cloud_CiSi.size())
            p_FSilist_list.append(p_FSilist)
            color_Silist_list.append(color_Silist)
        return _concatenate_point_clouds(p_FSilist_list, color_Silist_list)

    def DoCalcOutput(self, context, output):
        p_FSlist, color_Slist = self._align_point_clouds(context)
        point_cloud_FS = output.get_mutable_value()
        point_cloud_FS.resize(p_FSlist.shape[1])
        point_cloud_FS.mutable_xyzs()[:] = p_FSlist
        point_cloud_FS.mutable_rgbs()[:] = color_Slist
