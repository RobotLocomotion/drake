%function tension

%r = PlanarRigidBodyManipulator('tension.urdf');
r = TimeSteppingRigidBodyManipulator('SoftPaddle.urdf',.01,struct('twoD',true));

v = r.constructVisualizer();
v.xlim = [-5 5];
v.ylim = [-.2 6.2];

x0 = Point(getStateFrame(r));
x0.load_x = 0;
x0.load_z = 3.9999;
x0.tensioner_angle = pi/2;
x0.load_zdot = -2.5;
manip = r.getManipulator;
cable_length_fcn = manip.position_constraints{1}.fcn;
cable_length_fcn.pulley(:); %check for the number of pulleys

numQ =r.getNumStates/2;

v.drawWrapper(0,x0(1:numQ));

manip = r.getManipulator();
[l1,dl1]=manip.position_constraints{1}.eval(x0(1:numQ));
gradTest(@eval,manip.position_constraints{1}.fcn,x0(1:numQ));
manip.position_constraints{1}.checkGradient(.001,x0(1:numQ));


x0 = resolveConstraints(r,x0,v);

if(0)
[l2,dl2]=manip.position_constraints{1}.eval(x0(1:numQ));
gradTest(@eval,manip.position_constraints{1}.fcn,x0(1:numQ));
manip.position_constraints{1}.checkGradient(.001,x0(1:numQ));
end

v.drawWrapper(0,x0(1:numQ));

ytraj = simulate(r,[0 13],x0);
if(0)
ts = ytraj.getBreaks();
for i=1:numel(ts)
  x = ytraj.eval(ts(i));
  length(i) = manip.position_constraints{1}.eval(x(1:3));
end
figure(1); clf; plot(ts,length);
end

v.playback(ytraj,struct('slider',true));
%v.playbackSWF(ytraj,'tension')

