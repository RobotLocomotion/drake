import unittest

from drake.lcmt_driving_command_t import lcmt_driving_command_t as lcm_msg
from drake.automotive.steering_command_driver import make_driving_command


class SteeringCommandDriverTest(unittest.TestCase):
    def __init__(self, *args, **kwargs):
        super(SteeringCommandDriverTest, self).__init__(*args, **kwargs)
        self.acceleration = 0.5
        self.steering_angle = 0.2
        self.is_handler_run = False

    def test_make_driving_command(self):
        msg = make_driving_command(self.acceleration, self.steering_angle)
        self.assertEqual(msg.acceleration, self.acceleration)
        self.assertEqual(msg.steering_angle, self.steering_angle)
