#!/usr/bin/env python

"""Publishes steering commands over LCM.
"""

import argparse
import copy
import math
import sys

try:
    import pygame
except ImportError:
    # We will flag this as an error later, and only if we really needed it.
    pass

from drake_paths import add_module_search_paths

add_module_search_paths()  # So we can find lcm stuff.

import lcm

from drake.lcmt_driving_command_t import lcmt_driving_command_t as lcm_msg

STEERING_AXIS = 0
ACCEL_AXIS = 1
BRAKE_AXIS = 2

MAX_STEERING_ANGLE = math.radians(45)
STEERING_BUTTON_STEP_ANGLE = MAX_STEERING_ANGLE / 100.0
TURN_LEFT_SIGN = 1.0
TURN_RIGHT_SIGN = -1.0

# Target velocity 60mph, i.e. ~26.8224m/sec.
MAX_VELOCITY = 26.8224
THROTTLE_SCALE = MAX_VELOCITY / 300.0

MAX_BRAKE = MAX_VELOCITY
BRAKE_SCALE = THROTTLE_SCALE


def _limit_steering(requested_value):
    if abs(requested_value) <= MAX_STEERING_ANGLE:
        return requested_value
    else:
        return math.copysign(MAX_STEERING_ANGLE, requested_value)

# Applies the lower and upper limits to @p requested value.
def _limit_throttle(requested_value):
    if 0 <= requested_value <= MAX_VELOCITY:
        return requested_value
    elif requested_value < 0:
        return 0
    else:
        return MAX_VELOCITY

def _limit_brake(requested_value):
    if 0 <= requested_value <= MAX_BRAKE:
        return requested_value
    elif requested_value < 0:
        return 0
    else:
        return MAX_BRAKE

class KeyboardEventProcessor:
    def __init__(self):
        pygame.event.set_allowed(None)
        pygame.event.set_allowed([pygame.QUIT, pygame.KEYUP, pygame.KEYDOWN])
        pygame.key.set_repeat(100, 10)
        self.throttle_gradient = 0;
        self.brake_gradient = 0;
        self.keep_current_throttle_brake = False;

    def processEvent(self, event, last_msg):
        new_msg = copy.copy(last_msg)

        if event.type == pygame.KEYUP and not self.keep_current_throttle_brake:
            if hasattr(event, 'key'):
                if (event.key == pygame.K_SPACE):
                    self.keep_current_throttle_brake = True
                    return new_msg
                if (event.key == pygame.K_UP):
                    self.throttle_gradient = -1
                elif (event.key == pygame.K_DOWN):
                    self.brake_gradient = -1
            # Post a fake KEYUP event so the throttle/brake can keep decreasing
            # in the absence of real (and impossible) successive key releases.
            # Yield to any KEYDOWN event waiting in the queue.
            if not pygame.event.peek(pygame.KEYDOWN):
                dummyKeyUpEvent = pygame.event.Event(pygame.KEYUP)
                pygame.event.post(dummyKeyUpEvent)

        if (event.type == pygame.KEYDOWN):
            self.keep_current_throttle_brake = False;
            if (event.key == pygame.K_SPACE):
                self.keep_current_throttle_brake = True;
                self.throttle_gradient = 0
                self.brake_gradient = 0
            elif (event.key == pygame.K_UP):
                self.throttle_gradient = 1
            elif (event.key == pygame.K_DOWN):
                self.brake_gradient = 1
            elif (event.key == pygame.K_LEFT):
                new_msg.steering_angle = _limit_steering(
                last_msg.steering_angle + (
                    STEERING_BUTTON_STEP_ANGLE * TURN_LEFT_SIGN))
            elif (event.key == pygame.K_RIGHT):
                new_msg.steering_angle = _limit_steering(
                    last_msg.steering_angle + (
                        STEERING_BUTTON_STEP_ANGLE * TURN_RIGHT_SIGN))

        new_msg.throttle = _limit_throttle(
                last_msg.throttle + self.throttle_gradient * THROTTLE_SCALE)
        new_msg.brake = _limit_brake(
                last_msg.brake + self.brake_gradient * BRAKE_SCALE)

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
            new_msg.throttle = _limit_throttle(-0.5 * event.value + 0.5)
        elif event.axis == BRAKE_AXIS:
            new_msg.brake = _limit_brake(-0.5 * event.value + 0.5)
        return new_msg



class bcolors:
    OKBLUE = '\033[94m'
    ENDC = '\033[0m'

class SteeringCommandPublisher:
    def __init__(self, input_method, lcm_tag, joy_name):
        print 'Initializing...'
        pygame.init()
        self.screen = pygame.display.set_mode((300, 70))
        pygame.display.set_caption('Steering Command Driver')
        self.font = pygame.font.SysFont('Courier', 20)
        if input_method == 'keyboard':
            self.event_processor = KeyboardEventProcessor()
            print bcolors.OKBLUE + '--- Keyboard Control Instruction --- '\
                    + bcolors.ENDC
            print 'To increase the throttle/brake: press and hold the Up/Down'\
                    + ' Arrow'
            print 'To decrease the throttle/brake: release the Up/Down Arrow'
            print 'To keep the the current throttle/brake: press the Space Bar'
            print 'To increase left/right steering: press the Left/Right Arrow'
            print bcolors.OKBLUE + '------------------------------------ ' \
                    + bcolors.ENDC
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
