
import copy, pygame, lcm, argparse, sys
from drake.lcmt_driving_control_cmd_t import lcmt_driving_control_cmd_t as lcm_msg

STEERING_AXIS = 0
ACCEL_AXIS = 1
BRAKE_AXIS = 2
MAX_STEERING_ANGLE = 0.401425728
THROTTLE_MULTIPLIER = 1.0
BRAKE_MULTIPLIER = 1.0
STEERING_BUTTON_STEP_FACTOR = 100

class KeyboardEventProcessor:
  def __init__(self):
    pygame.event.set_allowed(None)
    pygame.event.set_allowed([pygame.QUIT, pygame.KEYUP, pygame.KEYDOWN])
    pygame.key.set_repeat(100, 10)

  def processEvent(self, event, last_msg):
    new_msg = copy.copy(last_msg)
    if event.key == pygame.K_UP:
      new_msg.throttle_value = (event.type==pygame.KEYDOWN) * THROTTLE_MULTIPLIER
    elif event.key == pygame.K_DOWN:
      new_msg.brake_value = (event.type==pygame.KEYDOWN) * BRAKE_MULTIPLIER
    elif event.key == pygame.K_LEFT:
      new_msg.steering_angle = min(MAX_STEERING_ANGLE,
          (MAX_STEERING_ANGLE/STEERING_BUTTON_STEP_FACTOR) * (event.type==pygame.KEYDOWN)
          + last_msg.steering_angle)
    elif event.key == pygame.K_RIGHT:
      new_msg.steering_angle = max(-1 * MAX_STEERING_ANGLE,
          (MAX_STEERING_ANGLE/STEERING_BUTTON_STEP_FACTOR) * -1 * (event.type==pygame.KEYDOWN)
          + last_msg.steering_angle)
    return new_msg

class JoystickEventProcessor:
  def __init__(self, joy_name):
    pygame.event.set_allowed(None)
    pygame.event.set_allowed([pygame.QUIT, pygame.JOYAXISMOTION])
    if pygame.joystick.get_count() == 0:
      pygame.quit()
      sys.exit('ERROR: No joysticks detected')
    joysticks = [pygame.joystick.Joystick(x) for x in xrange(pygame.joystick.get_count())]
    self.joystick = None
    for joystick in joysticks:
      if joystick.get_name() == joy_name:
        self.joystick = joystick
        break
    if self.joystick == None:
      pygame.quit()
      sys.exit('ERROR: Joystick with system name "%s" not detected' % (joy_name))
    self.joystick.init()

  def processEvent(self, event, last_msg):
    new_msg = copy.copy(last_msg)
    if event.axis == STEERING_AXIS:
      new_msg.steering_angle = -1 * event.value * MAX_STEERING_ANGLE
    elif event.axis == ACCEL_AXIS:
      new_msg.throttle_value = -0.5 * event.value + 0.5
    elif event.axis == BRAKE_AXIS:
      new_msg.brake_value = -0.5 * event.value + 0.5
    return new_msg


class SteeringCommandPublisher:
  def __init__(self, input_method, lcm_tag, joy_name):
    print 'Initializing...'
    pygame.init()
    self.screen = pygame.display.set_mode((300,70))
    pygame.display.set_caption('Steering Command Driver')
    self.font = pygame.font.SysFont('Courier', 20)
    if input_method == 'keyboard':
      self.event_processor = KeyboardEventProcessor();
    else:
      self.event_processor = JoystickEventProcessor(joy_name);
    self.last_msg = lcm_msg()
    self.lc = lcm.LCM()
    self.lcm_tag = lcm_tag
    print 'Ready'

  def printLCMValues(self):
    self.screen.fill(5)
    surface = self.font.render('Steering Angle: %f' % (self.last_msg.steering_angle),
        True, (250,250,250))
    self.screen.blit(surface, (2,0))
    surface = self.font.render('Throttle Value: %f' % (self.last_msg.throttle_value),
        True, (250,250,250))
    self.screen.blit(surface, (2,22))
    surface = self.font.render('Brake Value   : %f' % (self.last_msg.brake_value),
        True, (250,250,250))
    self.screen.blit(surface, (2,44))
    pygame.display.flip()

  def start(self):
    self.printLCMValues()
    while True:
      event = pygame.event.wait()
      if event.type == pygame.QUIT:
        pygame.quit()
        sys.exit()
      else:
        self.last_msg = self.event_processor.processEvent(event, self.last_msg)
        self.lc.publish(self.lcm_tag, self.last_msg.encode())
        self.printLCMValues()

if __name__ == '__main__':
  parser = argparse.ArgumentParser(description='Publishes steering commands over LCM')
  parser.add_argument('--input_method', choices=['joystick', 'keyboard'], default='keyboard',
      help='the input method to use for publishing LCM steering commands (default keyboard)')
  parser.add_argument('--lcm_tag', default='DRIVING_COMMAND',
      help='tag to publish the LCM messages with (default STEERING_DRIVER)')
  parser.add_argument('--joy_name', default='Driving Force GT',
      help='system name of the joystick (default Driving Force GT)')
  args = parser.parse_args()

  publisher = SteeringCommandPublisher(args.input_method, args.lcm_tag, args.joy_name)
  publisher.start()

