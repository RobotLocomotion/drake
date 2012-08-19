function runSaggitalPassive

r = PlanarRigidBodyManipulator('pr2.urdf');
r = setSimulinkParam(r,'MinStep','0.001');
v = r.constructVisualizer;
v.display_dt = .05;

x0 = Point(r.getStateFrame);
v.draw(0,double(x0));
return;

if (1)
  % Run animation while it is simulating (as fast as possible, but probably
  % slower than realtime)
  s = warning('off','Drake:DrakeSystem:UnsupportedSampleTime');  % we are knowingly breaking out to a simulink model with the cascade on the following line.
  sys = cascade(r,v);
  warning(s);
  simulate(sys,[0 10],x0); 
else
  % Run simulation, then play it back at realtime speed
  xtraj = simulate(r,[0 5],x0);
  v.playback(xtraj);
end

% NOTEST  - hudson can't handle it