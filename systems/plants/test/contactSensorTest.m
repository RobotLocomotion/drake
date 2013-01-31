function contactSensorTest

options.floating = true;
options.twoD = true;
p = TimeSteppingRigidBodyManipulator('FallingBrick.urdf',.01,options);

p = addSensor(p,FullStateFeedbackSensor());
body = findLink(p,'brick');
p = addSensor(p,ContactForceTorqueSensor(p,body,zeros(2,1),0));
p = compile(p);

ytraj = simulate(p,[0 5]);

% should find initial conditions for the brick which are resting on the
% ground. 
yf = Point(p.getOutputFrame,eval(ytraj,5));
valuecheck(yf.force_x,0);
valuecheck(yf.force_z,getMass(p)*norm(getGravity(p)));
valuecheck(yf.torque,0);

v = p.constructVisualizer();
v.playback(ytraj);



options.twoD = false;
p = TimeSteppingRigidBodyManipulator('FallingBrick.urdf',.01,options);

p = addSensor(p,FullStateFeedbackSensor);
body = findLink(p,'brick');
p = addSensor(p,ContactForceTorqueSensor(p,body,zeros(3,1),zeros(3,1)));
p = compile(p);

ytraj = simulate(p,[0 5]);

% should find initial conditions for the brick which are resting on the
% ground. 
yf = Point(p.getOutputFrame,eval(ytraj,5));
valuecheck(yf.force_x,0);
valuecheck(yf.force_y,0);
valuecheck(yf.force_z,getMass(p)*norm(getGravity(p)));
valuecheck(yf.torque_x,0);
valuecheck(yf.torque_y,0);
valuecheck(yf.torque_z,0);

v = p.constructVisualizer();
v.playback(ytraj);