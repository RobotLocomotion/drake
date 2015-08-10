function runHandContactSensorTest(~)

visualize = true;

% silence some warnings
warning('off','Drake:RigidBodyManipulator:UnsupportedContactPoints')
warning('off','Drake:RigidBodyManipulator:UnsupportedVelocityLimits')

options.floating = false;
options.dt = 0.001;
r = TimeSteppingRigidBodyManipulator('../urdf/robotiq.urdf', options.dt, options);
middle_fingertip_body = findLinkId(r,'finger_middle_link_3');
middle_fingertip_frame = RigidBodyFrame(middle_fingertip_body,zeros(3,1),zeros(3,1),'finger_middle_link_3');
middle_fingertip_force_sensor = ContactForceTorqueSensor(r, middle_fingertip_frame);
r = r.addFrame(middle_fingertip_frame);
r = addSensor(r, middle_fingertip_force_sensor);
% one_fingertip_body = findLinkId(r,'finger_1_link_3');
% one_fingertip_frame = RigidBodyFrame(one_fingertip_body,zeros(3,1),zeros(3,1),'finger_1_link_3');
% one_fingertip_force_sensor = ContactForceTorqueSensor(r, one_fingertip_frame);
% r = r.addFrame(one_fingertip_frame);
% r = addSensor(r, one_fingertip_force_sensor);
% two_fingertip_body = findLinkId(r,'finger_2_link_3');
% two_fingertip_frame = RigidBodyFrame(two_fingertip_body,zeros(3,1),zeros(3,1),'finger_2_link_3');
% two_fingertip_force_sensor = ContactForceTorqueSensor(r, two_fingertip_frame);
% r = r.addFrame(two_fingertip_frame);
% r = addSensor(r, two_fingertip_force_sensor);
r = compile(r);


x0 = r.resolveConstraints(rand(r.getNumStates, 1));

if visualize
  v = r.constructVisualizer;
  v.display_dt = 0.01;
  S=warning('off','Drake:DrakeSystem:UnsupportedSampleTime');
  output_select(1).system=1;
  output_select(1).output=1;
  output_select(2).system=1;
  output_select(2).output=2;
  sys = mimoCascade(r,v,[],[],output_select);
  warning(S);
end

traj = simulate(sys,[0 1.0],x0);
if visualize
  playback(v,traj,struct('slider',true));
end


end
