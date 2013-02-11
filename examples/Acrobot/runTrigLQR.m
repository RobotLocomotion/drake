function runTrigLQR()
% run trig lqr on top of non-trig lqr

p = AcrobotPlant;
v = AcrobotVisualizer(p);
%p = PlanarRigidBodyManipulator('Acrobot.urdf');
%v = p.constructVisualizer;

options.replace_output_w_new_state = true;
tp = makeTrigPolySystem(p,options);

Q = diag([10 10 10 10 1 1]); R=1;
c = tilqr(tp,p.xG,p.uG,Q,R);
sys = feedback(p,c);

Q = diag([10 10 1 1]); R=1;
c2 = tilqr(p,p.xG,p.uG,Q,R);
sys2 = feedback(p,c2);
v2 = v;
%v2.fade_percent = .5;

mv = MultiVisualizer({v2,v});

for i=1:5
  x0 = Point(p.getStateFrame,[pi;0;0;0]+0.08*randn(4,1));

  xtraj=simulate(sys,[0 4],x0);
  xtraj2=simulate(sys2,[0 4],x0);

  mv.playback([xtraj2;xtraj]);
end


