function contactSensorTest

options.floating = true;
options.twoD = true;
p = TimeSteppingRigidBodyManipulator('FallingBrick.urdf',.01,options);

p.manip.sensor{1} = FullStateFeedbackSensor(p.manip);
p.manip.sensor{2} = ContactForceTorqueSensor(p,p.manip.body(end),zeros(2,1),0);
p = compile(p);

ytraj = simulate(p,[0 5]);

% should find initial conditions for the brick which are resting on the
% ground. 
yf = Point(p.getOutputFrame,eval(ytraj,5));
valuecheck(yf.force_x,0);
valuecheck(yf.force_z,getMass(p.manip)*norm(p.manip.gravity));
valuecheck(yf.torque,0);

v = p.constructVisualizer();
v.playback(ytraj);



options.twoD = false;
p = TimeSteppingRigidBodyManipulator('FallingBrick.urdf',.01,options);

p.manip.sensor{1} = FullStateFeedbackSensor(p.manip);
p.manip.sensor{2} = ContactForceTorqueSensor(p,p.manip.body(end),zeros(3,1),zeros(3,1));
p = compile(p);

ytraj = simulate(p,[0 5]);

% should find initial conditions for the brick which are resting on the
% ground. 
yf = Point(p.getOutputFrame,eval(ytraj,5));
valuecheck(yf.force_x,0);
valuecheck(yf.force_y,0);
valuecheck(yf.force_z,getMass(p.manip)*norm(p.manip.gravity));
valuecheck(yf.torque_x,0);
valuecheck(yf.torque_y,0);
valuecheck(yf.torque_z,0);

v = p.constructVisualizer();
v.playback(ytraj);