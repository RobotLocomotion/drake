function runTrigLQR()
% run trig lqr on top of non-trig lqr

%p = AcrobotPlant;
%v = AcrobotVisualizer(p);
%xG = p.xG; uG = p.uG;

p = PlanarRigidBodyManipulator('Acrobot.urdf');
v = p.constructVisualizer;
xG = Point(p.getStateFrame,[pi;0;0;0]);
uG = Point(p.getInputFrame,0);

options.replace_output_w_new_state = true;
tp = extractTrigPolySystem(p,options);

Q = diag([10 10 10 10 1 1]); R=1;
c = tilqr(tp,xG,uG,Q,R);
sys = feedback(p,c);

Q = diag([10 10 1 1]); R=1;
c2 = tilqr(p,xG,uG,Q,R);
sys2 = feedback(p,c2);
v2 = v;
v2.fade_percent = .5;

mv = MultiVisualizer({v2,v});

for i=1:5
  x0 = Point(p.getStateFrame,[pi;0;0;0]+0.05*randn(4,1));

  xtraj=simulate(sys,[0 4],x0);
  xtraj2=simulate(sys2,[0 4],x0);

  w = warning('off','Drake:FunctionHandleTrajectory');
  warning('off','Drake:PPTrajectory:DifferentBreaks');
  mv.playback([xtraj2;xtraj]);
  warning(w);
end


