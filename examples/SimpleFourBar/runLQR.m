function runLQR

r = PlanarRigidBodyManipulator('FourBar2.urdf');
v = r.constructVisualizer();

%x0 = [2.861691;.811224;-3.528619;0;0;0]; u0 = 0;
x0 = [0.4485; -2.2775; -1.6694; 0; 0; 0]; u0 = -6.5407;
[x0,u0] = findFixedPoint(r,x0,u0);
v.draw(0,x0);

Q = diag([10 10 10 1 1 1]);
R = .001;

c = tilqr(r,x0,u0,Q,R);
sys = feedback(r,c);

x0 = resolveConstraints(r,double(x0)+.2*randn(6,1));
xtraj = simulate(sys,[0 10],x0);
xtrajpassive = simulate(r,[0 10],x0);

vpassive = v;
vpassive.fade_percent = .5;
mv = MultiVisualizer({vpassive,v});

mv.playback(vertcat(xtrajpassive,xtraj));