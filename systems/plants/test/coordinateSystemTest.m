function coordinateSystemTest

% just runs it as a passive system for now

for i=1:4

  options.view = 'right';
  options.floating = true;
  %m = RigidBodyModel(['brick',num2str(i),'.urdf'],options);
  m = PlanarRigidBodyModel(['brick',num2str(i),'.urdf'],options);
  %m.body(end).contact_pts = [];
  r = TimeSteppingRigidBodyManipulator(m,.01);
  
  %x0 = Point(r.getStateFrame);
  x0 = zeros(6,1);
  x0(2)=.6;
  %x0 = resolveConstraints(r.manip,double(x0));
  %x0([1,3:end])=0;
  
  %syms q qd;
  %[H,C,B]=r.manip.manipulatorDynamics(q,qd)
  xtraj = simulate(r,[0 3.1],x0);
  
  xf = eval(xtraj,3);
  if (i~=3) % should balance
    if abs(xf(3))>pi/4
      i
      warning('this brick fell over, but should not have');
    end
  else
    if (abs(xf(3))<pi/4)
      i
      warning('this brick balanced, but should fall over');
    end
  end
  
  v = r.constructVisualizer;
  v.display_dt = .05;
  v.debug = true;
  v.playback(xtraj);
  
%  clf;
%  fnplt(xtraj,1)
end