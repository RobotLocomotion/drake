function runLQR


p = RigidBodyManipulator('FurutaPendulum.urdf');

v = p.constructVisualizer;

xG = Point(p.getStateFrame,[0;pi;0;0]);
uG = Point(p.getInputFrame,0);
c = tilqr(p,xG,uG,diag([10 10 1 1]),.1);

sys = feedback(p,c);

for i=1:5
  x0 = double(xG)+.2*randn(4,1);
  xtraj = simulate(sys,[0 4],x0);
  if checkDependency('vrml_enabled')
    v.playback(xtraj);
  end
end

