#!/usr/bin/env python

from __future__ import print_function

import copy
import unittest
import numpy as np

import pydrake.systems.framework as framework
from pydrake.maliput.api import (
    RoadGeometryId,
    LanePosition,
    GeoPosition,
    )
import pydrake.maliput.dragway as dragway


# Tests the bindings for the API and backend implementations.
class TestMaliput(unittest.TestCase):
    def test_api(self):
        srh = [4., 5., 6.]
        lane_pos = LanePosition(srh[0], srh[1], srh[2])
        self.assertTrue(len(lane_pos.srh()) == 3)
        for i in [0, 1, 2]:
            self.assertTrue(lane_pos.srh()[i] == srh[i])
        xyz = [1., 2., 3.]
        geo_pos = GeoPosition(xyz[0], xyz[1], xyz[2])
        self.assertTrue(len(geo_pos.xyz()) == 3)
        for i in [0, 1, 2]:
            self.assertTrue(geo_pos.xyz()[i] == xyz[i])

    def test_dragway(self):
        kNumLanes = 2
        kLength = 100.
        kLaneWidth = 4.
        kShoulderWidth = 1.
        kHeight = 5.
        kTol = 1e-6

        # Instantiate a two-lane straight road.
        rg_id = RoadGeometryId("two_lane_road")
        rg = dragway.RoadGeometry(
            rg_id, kNumLanes, kLength, kLaneWidth, kShoulderWidth, kHeight,
            kTol, kTol)
        segment = rg.junction(0).segment(0)
        lane_0 = segment.lane(0)
        lane_1 = segment.lane(1)

        # Test the .
        lane_pos = LanePosition(0., 0., 0.)
        geo_pos_result = lane_0.ToGeoPosition(lane_pos)
        geo_pos_expected = GeoPosition(0., -kLaneWidth / 2., 0.)
        for i in [0, 1, 2]:
            self.assertTrue(geo_pos_result.xyz()[i] ==
                            geo_pos_expected.xyz()[i])

        geo_pos = GeoPosition(1., kLaneWidth / 2., 3.)
        nearest_pos = GeoPosition(0., 0., 0.)
        distance = 0.
        lane_pos_result = \
            lane_1.ToLanePosition(geo_pos, nearest_pos, distance)
        lane_pos_expected = LanePosition(1., 0., 3.)
        for i in [0, 1, 2]:
            self.assertTrue(lane_pos_result.srh()[i] ==
                            lane_pos_expected.srh()[i])
            self.assertTrue(nearest_pos.xyz()[i] == geo_pos.xyz()[i])
            self.assertTrue(distance == 0.)

    # TODO(jadecastro) Add more maliput backends as needed.


if __name__ == '__main__':
    unittest.main()
