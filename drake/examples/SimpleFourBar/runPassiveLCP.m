function runPassiveLCP

options.twoD = true;
w = warning('off','Drake:RigidBody:SimplifiedCollisionGeometry');
p = TimeSteppingRigidBodyManipulator('FourBar.urdf',.01,options);
warning(w);
v = p.constructVisualizer();
v.xlim = [-8 8]; v.ylim = [-4 10];

xtraj = p.simulate([0 10]);

x = xtraj.eval(xtraj.getBreaks);
nq = getNumPositions(p);
for i=1:size(x,2)
  valuecheck(positionConstraints(p,x(1:nq,i)),zeros(6,1),1e-2);  
  % high tolerance is due to (relatively) large dt and linearization of phi
  % change zeros(3,1) to zeros(2,1) when we (re-)implement planar loop
  % constraints
end

v.playback(xtraj);

