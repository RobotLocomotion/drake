import lcm
import unittest

from drake.lcmt_driving_command_t import lcmt_driving_command_t as lcm_msg
from drake.automotive.steering_command_driver import make_driving_command


class SteeringCommandDriverTest(unittest.TestCase):
    def __init__(self, *args, **kwargs):
        super(SteeringCommandDriverTest, self).__init__(*args, **kwargs)
        self.lcm_tag = "EXAMPLE"
        self.acceleration = 0.5
        self.steering_angle = 0.2
        self.is_handler_run = False

    def handler(self, channel, data):
        msg = lcm_msg.decode(data)
        self.assertEqual(msg.acceleration, self.acceleration)
        self.assertEqual(msg.steering_angle, self.steering_angle)
        self.is_handler_run = True

    def test_make_driving_command(self):
        receiver = lcm.LCM()
        subscription = receiver.subscribe(self.lcm_tag, self.handler)
        sender = lcm.LCM()
        sender.publish(self.lcm_tag,
                       make_driving_command(self.acceleration,
                                            self.steering_angle).encode())
        receiver.handle()
        receiver.unsubscribe(subscription)
        self.assertTrue(self.is_handler_run)
