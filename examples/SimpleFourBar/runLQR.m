function runLQR
w = warning('off','Drake:RigidBody:SimplifiedCollisionGeometry');
r = PlanarRigidBodyManipulator('FourBar2.urdf');
warning(w);
v = r.constructVisualizer();
v.xlim = [-8 8]; v.ylim = [-4 10];

x_nom = [0.4485; -2.2775; -1.6694; 0; 0; 0]; u_nom = -6.5407;
v.draw(0,x_nom);
[x_nom,u_nom] = findFixedPoint(r,x_nom,u_nom);
v.draw(0,x_nom);

Q = diag([10 10 10 1 1 1]);
R = .001;

c = tilqr(r,x_nom,u_nom,Q,R);
sys = feedback(r,c);

x0 = resolveConstraints(r,double(x_nom)+.2*randn(6,1));
w = warning('off','Drake:DrakeSystem:ConstraintsNotEnforced');
xtraj = simulate(sys,[0 10],x0);
xtrajopenloop = simulate(cascade(setOutputFrame(ConstantTrajectory(u_nom),getInputFrame(r)),r),[0 10],x0);
warning(w);

vopenloop = v;
vopenloop.fade_percent = .5;
mv = MultiVisualizer({vopenloop,v});

mv.playback(vertcat(xtrajopenloop,xtraj));

