function p = SCSDoublePendPlant


eclipse_root = '/Users/russt/Documents/workspace/';
javaaddpath([eclipse_root,'DoublePendulum/classes']);

r = com.yobotics.exampleSimulations.doublePendulum.DoublePendulumRobot();

p = SimulationConstructionSetRobot(r);
p = setInputLimits(p,[0;-inf],[0;inf]);

if (nargout<1)
  pv = SCSDoublePendVisualizer(r);
  traj = simulate(p,[0 5],[pi 0 -pi/2 0]');
  playback(pv,traj);
end
