function contactSensorTest
S = warning('OFF','Drake:RigidBodyManipulator:WeldedLinkInd');
options.floating = true;
options.twoD = true;
options.terrain = RigidBodyFlatTerrain;
p = TimeSteppingRigidBodyManipulator('../test/FallingBrickContactPoints.urdf',.01,options);
p = addSensor(p,FullStateFeedbackSensor());
body = findLinkInd(p,'brick');
frame = RigidBodyFrame(body,zeros(3,1),zeros(3,1),'FT_frame');
p = addFrame(p,frame);
p = addSensor(p,ContactForceTorqueSensor(p,frame));

p = compile(p);

x0 = [0;1;.1*randn;0;0;0;0;0;0];
ytraj = simulate(p,[0 5],x0);

% should find initial conditions for the brick which are resting on the
% ground. 
yf = Point(p.getOutputFrame,eval(ytraj,5));
valuecheck(yf.force_x,0,1e-6);
valuecheck(yf.force_z,norm(getGravity(p)),1e-6);
valuecheck(yf.torque,0,1e-6);

%v = p.constructVisualizer();
%v.playback(ytraj);

options.twoD = false;
p = TimeSteppingRigidBodyManipulator('../test/FallingBrickContactPoints.urdf',.01,options);

p = addSensor(p,FullStateFeedbackSensor);
body = findLinkInd(p,'brick');
frame = RigidBodyFrame(body,zeros(3,1),zeros(3,1),'FT_frame');
p = addFrame(p,frame);
p = addSensor(p,ContactForceTorqueSensor(p,frame));
p = compile(p);

x0 = [0;0;1;.02*randn;.02*randn;.02*randn;zeros(6,1);zeros(6,1)];
ytraj = simulate(p,[0 10],x0);

% should find initial conditions for the brick which are resting on the
% ground. 
yf = Point(p.getOutputFrame,eval(ytraj,5));
valuecheck(yf.force_x,0,1e-6);
valuecheck(yf.force_y,0,1e-6);
valuecheck(yf.force_z,norm(getGravity(p)),1e-6);
valuecheck(yf.torque_x,0,1e-6);
valuecheck(yf.torque_y,0,1e-6);
valuecheck(yf.torque_z,0,1e-6);
warning(S);

%v = p.constructVisualizer();
%v.playback(ytraj);
