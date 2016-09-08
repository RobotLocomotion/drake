#!/usr/bin/env python

"""Publishes steering commands over LCM.
"""

import argparse
import copy
import math
import os
import subprocess
import sys

try:
    import pygame
except ImportError:
    # We will flag this as an error later, and only if we really needed it.
    pass

THIS_FILE = os.path.abspath(__file__)
THIS_DIR = os.path.dirname(THIS_FILE)
DRAKE_DIR = os.path.dirname(os.path.dirname(THIS_DIR))
DRAKE_DIST_DIR = os.path.dirname(DRAKE_DIR)
DRAKE_DIST_BUILD_DIR = os.getenv(
    'DRAKE_DIST_BUILD',
    os.path.join(DRAKE_DIST_DIR, 'build'))
DRAKE_LCMTYPES_DIR = os.path.join(
    DRAKE_DIST_BUILD_DIR, 'drake/lcmtypes')
DRAKE_PYTHON_DIR = os.path.join(DRAKE_DIST_BUILD_DIR, 'install/lib/python2.7')
sys.path.extend([
    DRAKE_LCMTYPES_DIR,  # First (to pick up local edits to messages).
    os.path.join(DRAKE_PYTHON_DIR, 'dist-packages'),
    os.path.join(DRAKE_PYTHON_DIR, 'site-packages')])

try:
    import lcm
except ImportError as e:
    # TODO(#3397) This whole mess shouldn't be necessary.
    disable_retry_flag = 'IMPORT_LCM_DISABLE_RETRY'
    if ('cannot open shared object file' in e.message and
        disable_retry_flag not in os.environ):
        print 'warning:', e.message, '(will retry now)'
        LD_LIBRARY_PATH = ':'.join([
            os.path.join(DRAKE_DIST_BUILD_DIR, 'install/lib'),
            os.getenv('LD_LIBRARY_PATH', default='')])
        new_env = dict(os.environ)
        new_env['LD_LIBRARY_PATH'] = LD_LIBRARY_PATH
        new_env[disable_retry_flag] = '1'
        sys.exit(subprocess.call(
            [sys.executable] + sys.argv,
            env=new_env))
    raise

from drake.lcmt_driving_command_t import lcmt_driving_command_t as lcm_msg

STEERING_AXIS = 0
ACCEL_AXIS = 1
BRAKE_AXIS = 2

MAX_STEERING_ANGLE = math.radians(45)
STEERING_BUTTON_STEP_ANGLE = MAX_STEERING_ANGLE / 100.0
TURN_LEFT_SIGN = 1.0
TURN_RIGHT_SIGN = -1.0

THROTTLE_SCALE = 1.0
BRAKE_SCALE = 1.0

def _limit_steering(requested_value):
    if abs(requested_value) <= MAX_STEERING_ANGLE:
        return requested_value
    else:
        return math.copysign(MAX_STEERING_ANGLE, requested_value)

class KeyboardEventProcessor:
    def __init__(self):
        pygame.event.set_allowed(None)
        pygame.event.set_allowed([pygame.QUIT, pygame.KEYUP, pygame.KEYDOWN])
        pygame.key.set_repeat(100, 10)

    def processEvent(self, event, last_msg):
        new_msg = copy.copy(last_msg)
        if event.key == pygame.K_UP:
            new_msg.throttle = (
                (event.type == pygame.KEYDOWN) * THROTTLE_SCALE)
        elif event.key == pygame.K_DOWN:
            new_msg.brake = (
                (event.type == pygame.KEYDOWN) * BRAKE_SCALE)
        elif (event.key == pygame.K_LEFT) and (event.type == pygame.KEYDOWN):
            new_msg.steering_angle = _limit_steering(
                last_msg.steering_angle + (
                    STEERING_BUTTON_STEP_ANGLE * TURN_LEFT_SIGN))
        elif (event.key == pygame.K_RIGHT) and (event.type == pygame.KEYDOWN):
            new_msg.steering_angle = _limit_steering(
                last_msg.steering_angle + (
                    STEERING_BUTTON_STEP_ANGLE * TURN_RIGHT_SIGN))
        return new_msg


class JoystickEventProcessor:
    def __init__(self, joy_name):
        pygame.event.set_allowed(None)
        pygame.event.set_allowed([pygame.QUIT, pygame.JOYAXISMOTION])
        if pygame.joystick.get_count() == 0:
            pygame.quit()
            sys.exit('ERROR: No joysticks detected')
        joysticks = [pygame.joystick.Joystick(x)
                     for x in xrange(pygame.joystick.get_count())]
        self.joystick = None
        for joystick in joysticks:
            if joystick.get_name() == joy_name:
                self.joystick = joystick
                break
        if self.joystick is None:
            pygame.quit()
            sys.exit('ERROR: Joystick with system name "%s" not detected' %
                     (joy_name))
        self.joystick.init()

    def processEvent(self, event, last_msg):
        new_msg = copy.copy(last_msg)
        if event.axis == STEERING_AXIS:
            new_msg.steering_angle = (
                TURN_RIGHT_SIGN * event.value * MAX_STEERING_ANGLE)
        elif event.axis == ACCEL_AXIS:
            new_msg.throttle = -0.5 * event.value + 0.5
        elif event.axis == BRAKE_AXIS:
            new_msg.brake = -0.5 * event.value + 0.5
        return new_msg


class SteeringCommandPublisher:
    def __init__(self, input_method, lcm_tag, joy_name):
        print 'Initializing...'
        pygame.init()
        self.screen = pygame.display.set_mode((300, 70))
        pygame.display.set_caption('Steering Command Driver')
        self.font = pygame.font.SysFont('Courier', 20)
        if input_method == 'keyboard':
            self.event_processor = KeyboardEventProcessor()
        else:
            self.event_processor = JoystickEventProcessor(joy_name)
        self.last_msg = lcm_msg()
        self.lc = lcm.LCM()
        self.lcm_tag = lcm_tag
        print 'Ready'

    def printLCMValues(self):
        self.screen.fill(5)
        surface = self.font.render(
            'Steering Angle: %f' % (self.last_msg.steering_angle),
            True, (250, 250, 250))
        self.screen.blit(surface, (2, 0))
        surface = self.font.render(
            'Throttle Value: %f' % (self.last_msg.throttle),
            True, (250, 250, 250))
        self.screen.blit(surface, (2, 22))
        surface = self.font.render(
            'Brake Value   : %f' % (self.last_msg.brake),
            True, (250, 250, 250))
        self.screen.blit(surface, (2, 44))
        pygame.display.flip()

    def start(self):
        self.printLCMValues()
        while True:
            event = pygame.event.wait()
            if event.type == pygame.QUIT:
                pygame.quit()
                sys.exit()
            else:
                self.last_msg = self.event_processor.processEvent(
                  event, self.last_msg)
                self.lc.publish(self.lcm_tag, self.last_msg.encode())
                self.printLCMValues()


def publish_driving_command(lcm_tag, throttle, steering_angle):
    lc = lcm.LCM()
    last_msg = lcm_msg()
    last_msg.throttle = throttle
    last_msg.steering_angle = steering_angle
    lc.publish(lcm_tag, last_msg.encode())


def main():
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument(
      '--lcm_tag', default='DRIVING_COMMAND',
      help='tag to publish the LCM messages with')
    parser.add_argument(
      '--mode', choices=['interactive', 'one-time'], default='interactive',
      help='whether to run interactively with pygame input,'
           ' or just send a single command')
    parser.add_argument(
      '--input_method', choices=['joystick', 'keyboard'], default='keyboard',
      help='the interactive input method to use for publishing LCM commands')
    parser.add_argument(
      '--joy_name', default='Driving Force GT',
        help='system name of the joystick')
    parser.add_argument(
      '--throttle', type=float, default='0.0', help='initial throttle')
    parser.add_argument(
      '--steering-angle', type=float, default='0.0',
        help='initial steering angle (in radians), positive-left')

    args = parser.parse_args()

    if args.mode == 'one-time':
        publish_driving_command(
            args.lcm_tag, args.throttle, args.steering_angle)
        return 0

    if 'pygame' not in sys.modules:
        print >>sys.stderr, 'error: missing pygame; see README.md for help.'
        return 1

    publisher = SteeringCommandPublisher(
      args.input_method, args.lcm_tag, args.joy_name)
    publisher.start()

    return 0


if __name__ == '__main__':
    sys.exit(main())
