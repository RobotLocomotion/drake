function soft_hand

% note that the masses are constrained to move only vertically
%r = PlanarRigidBodyManipulator('tension.urdf');
r = TimeSteppingRigidBodyManipulator('soft_hand.urdf',.01,struct('twoD',true,'view','top'));

v = r.constructVisualizer();
v.xlim = [-3 16];
v.ylim = [-7 7];

x0 = Point(getStateFrame(r));
x0.ball_x = 10;
x0.ball_y = 0;
x0.finger1_proximal = .35;
x0.finger1_middle = -.75;
x0.finger1_distal = .375;
x0.finger2_proximal = -.35;
x0.finger2_middle = .75;
x0.finger2_distal = -.375;

m = r.getManipulator();
l1=m.position_constraints{1}.eval(x0(1:getNumPositions(r)))
l2=m.position_constraints{2}.eval(x0(1:getNumPositions(r)))

x0 = resolveConstraints(r,x0); %,v);
v.drawWrapper(0,x0(1:getNumPositions(r)));

%v.inspector(x0);
%return;

ts = 0:.1:10;
utraj = setOutputFrame(PPTrajectory(spline(ts,[-.2*sin(ts);.2*cos(ts)])),getInputFrame(r));

ytraj = simulate(cascade(utraj,r),[0 10],x0);
if(0)
ts = ytraj.getBreaks();
for i=1:numel(ts)
  x = ytraj.eval(ts(i));
  length(i) = r.position_constraints{1}.eval(x(1:3));
end
figure(1); clf; plot(ts,length);
end

v.playback(ytraj,struct('slider',true));
%v.playbackSWF(ytraj,'tension')

