from __future__ import print_function

import copy
import unittest
import numpy as np

import pydrake.systems.framework as framework
from pydrake.maliput.api import (
    RoadGeometryId,
    LanePosition,
    GeoPosition,
    RoadGeometry,
    )
from pydrake.maliput.dragway import (
    create_dragway,
    )


# Tests the bindings for the API and backend implementations.
class TestMaliput(unittest.TestCase):
    def test_api(self):
        # Test the containers' constructors and accessors.
        srh = [4., 5., 6.]
        lane_pos = LanePosition(srh[0], srh[1], srh[2])
        self.assertTrue(len(lane_pos.srh()) == 3)
        self.assertTrue(np.allclose(lane_pos.srh(), srh))

        lane_pos_alt = LanePosition(s=4., r=5., h=6.)
        self.assertTrue(np.allclose(lane_pos_alt.srh(), srh))

        xyz = [1., 2., 3.]
        geo_pos = GeoPosition(xyz[0], xyz[1], xyz[2])
        self.assertTrue(len(geo_pos.xyz()) == 3)
        self.assertTrue(np.allclose(geo_pos.xyz(), xyz))

        geo_pos_alt = GeoPosition(x=1., y=2., z=3.)
        self.assertTrue(np.allclose(geo_pos_alt.xyz(), xyz))

        # Check that the getters are read-only.
        with self.assertRaises(ValueError):
            lane_pos.srh()[0] = 0.
        with self.assertRaises(ValueError):
            geo_pos.xyz()[0] = 0.

        # Test RoadGeometryId accessors.
        string = "foo"
        rg_id = RoadGeometryId(string)
        self.assertTrue(rg_id.string() == string)

    def test_dragway(self):
        kNumLanes = 2
        kLength = 100.
        kLaneWidth = 4.
        kShoulderWidth = 1.
        kHeight = 5.
        kTol = 1e-6

        # Instantiate a two-lane straight road.
        rg_id = RoadGeometryId("two_lane_road")
        rg = create_dragway(rg_id, kNumLanes, kLength, kLaneWidth,
                            kShoulderWidth, kHeight, kTol, kTol)
        segment = rg.junction(0).segment(0)
        lane_0 = segment.lane(0)
        lane_1 = segment.lane(1)

        # Alternate constructor.
        create_dragway(
            road_id=rg_id, num_lanes=kNumLanes, length=kLength,
            lane_width=kLaneWidth, shoulder_width=kShoulderWidth,
            maximum_height=kHeight, linear_tolerance=kTol,
            angular_tolerance=kTol)

        # Test the Lane <-> Geo space coordinate conversion.
        lane_pos = LanePosition(0., 0., 0.)
        geo_pos_result = lane_0.ToGeoPosition(lane_pos)
        geo_pos_expected = GeoPosition(0., -kLaneWidth / 2., 0.)
        self.assertTrue(np.allclose(geo_pos_result.xyz(),
                                    geo_pos_expected.xyz()))

        geo_pos = GeoPosition(1., kLaneWidth / 2., 3.)
        nearest_pos = GeoPosition(0., 0., 0.)
        distance = 0.
        lane_pos_result = \
            lane_1.ToLanePosition(geo_pos, nearest_pos, distance)
        lane_pos_expected = LanePosition(1., 0., 3.)
        self.assertTrue(np.allclose(lane_pos_result.srh(),
                                    lane_pos_expected.srh()))
        self.assertTrue(np.allclose(nearest_pos.xyz(), geo_pos.xyz()))
        self.assertTrue(distance == 0.)

    # TODO(jadecastro) Add more maliput backends as needed.
