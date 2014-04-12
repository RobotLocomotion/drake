function runLQR
options.terrain = [];
w = warning('off','Drake:RigidBody:SimplifiedCollisionGeometry');
r = PlanarRigidBodyManipulator('FourBar2.urdf',options);
warning(w);
v = r.constructVisualizer();

%x0 = [2.861691;.811224;-3.528619;0;0;0]; u0 = 0;
x0 = [0.4485; -2.2775; -1.6694; 0; 0; 0]; u0 = -6.5407;
v.draw(0,x0);
[x0,u0] = findFixedPoint(r,x0,u0);
v.draw(0,x0);

Q = diag([10 10 10 1 1 1]);
R = .001;

c = tilqr(r,x0,u0,Q,R);
sys = feedback(r,c);

x0 = resolveConstraints(r,double(x0)+.2*randn(6,1));
w = warning('off','Drake:DrakeSystem:ConstraintsNotEnforced');
xtraj = simulate(sys,[0 10],x0);
xtrajopenloop = simulate(cascade(setOutputFrame(ConstantTrajectory(u0),getInputFrame(r)),r),[0 10],x0);
warning(w);

vopenloop = v;
vopenloop.fade_percent = .5;
mv = MultiVisualizer({vopenloop,v});

mv.playback(vertcat(xtrajopenloop,xtraj));

