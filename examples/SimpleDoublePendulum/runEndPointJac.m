function runEndPointJac

r = PlanarRigidBodyManipulator('SimpleDoublePendulum.urdf',struct('terrain',[]));
v = r.constructVisualizer();
v.axis = [-2 2 -2 2];

kp = diag([100 100]);
kd = diag([10 10]);
sys = pdcontrol(r,kp,kd);

c = EndPointControl(sys,r);

%endpoint_d = FunctionHandleTrajectory(@(t)[1;1-.5*sin(t/2)],2,[0 5]);
%endpoint_d = setOutputFrame(endpoint_d,sys.getInputFrame);

%tf = 15;
%xtraj = simulate(feedback(sys,c),[0 tf]);
%v.playback(xtraj);

simulate(cascade(feedback(sys,c),v),[0 5])

