function contactSensorTest
rng(0)
S = warning('OFF','Drake:RigidBodyManipulator:WeldedLinkInd');
options.floating = true;
options.twoD = true;
options.terrain = RigidBodyFlatTerrain;
options.use_bullet = false; 
p = TimeSteppingRigidBodyManipulator('FallingBrickContactPoints.urdf',.01,options);
p = addSensor(p,FullStateFeedbackSensor());
body = findLinkId(p,'brick');
frame = RigidBodyFrame(body,zeros(3,1),zeros(3,1),'FT_frame');
p = addFrame(p,frame);
p = addSensor(p,ContactForceTorqueSensor(p,frame));
p = compile(p);

T = 2;
x0 = p.resolveConstraints(.01*randn(getNumStates(p),1));
[ytraj,xtraj] = simulate(p,[0 T],x0);

%v = p.constructVisualizer();
%v.playback(ytraj,struct('slider',true));

% should find initial conditions for the brick which are resting on the
% ground. 
yf = Point(p.getOutputFrame,eval(ytraj,T));
valuecheck(yf.force_x,0,1e-5);
valuecheck(yf.force_z,getMass(p)*norm(getGravity(p)),1e-5);
valuecheck(yf.torque,0,1e-5);

options.twoD = false;
p = TimeSteppingRigidBodyManipulator('FallingBrickContactPoints.urdf',.01,options);

p = addSensor(p,FullStateFeedbackSensor);
body = findLinkId(p,'brick');
frame = RigidBodyFrame(body,zeros(3,1),zeros(3,1),'FT_frame');
p = addFrame(p,frame);
p = addSensor(p,ContactForceTorqueSensor(p,frame));
p = compile(p);

x0 = p.resolveConstraints(.01*randn(getNumStates(p),1));
[ytraj,xtraj] = simulate(p,[0 T],x0);

%v = p.constructVisualizer();
%v.playback(ytraj,struct('slider',true));

% should find initial conditions for the brick which are resting on the
% ground. 
yf = Point(p.getOutputFrame,eval(ytraj,T));
valuecheck(yf.force_x,0,1e-5);
valuecheck(yf.force_y,0,1e-5);
valuecheck(yf.force_z,getMass(p)*norm(getGravity(p)),1e-6);
valuecheck(yf.torque_x,0,1e-5);
valuecheck(yf.torque_y,0,1e-5);
valuecheck(yf.torque_z,0,1e-5);
warning(S);

