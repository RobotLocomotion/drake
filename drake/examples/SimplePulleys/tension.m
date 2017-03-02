%function tension

%r = PlanarRigidBodyManipulator('tension.urdf');
r = TimeSteppingRigidBodyManipulator('tension.urdf',.01,struct('twoD',true));

v = r.constructVisualizer();
v.xlim = [-5 5];
v.ylim = [-.2 6.2];

x0 = Point(getStateFrame(r));
x0.load_x = 0;
x0.load_z = 3.999;
%x0.tensioner_angle = 0.14;
x0.load_zdot = -2.5;

v.drawWrapper(0,x0(1:3));

manip = r.getManipulator();
[l1,dl1]=manip.position_constraints{1}.eval(x0(1:3));
gradTest(@eval,manip.position_constraints{1}.fcn,x0(1:3));
manip.position_constraints{1}.checkGradient(.001,x0(1:3));


x0 = resolveConstraints(r,x0,v);

if(0)
[l2,dl2]=manip.position_constraints{1}.eval(x0(1:3));
gradTest(@eval,manip.position_constraints{1}.fcn,x0(1:3));
manip.position_constraints{1}.checkGradient(.001,x0(1:3));
end

v.drawWrapper(0,x0(1:3));

ytraj = simulate(r,[0 8],x0);
if(1)
ts = ytraj.getBreaks();
for i=1:numel(ts)
  x = ytraj.eval(ts(i));
  length(i) = manip.position_constraints{1}.eval(x(1:3));
end
figure(1); clf; plot(ts,length);
end

v.playback(ytraj,struct('slider',true));
%v.playbackSWF(ytraj,'tension')

