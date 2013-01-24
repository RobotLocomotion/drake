function testForwardKinRot

p = RigidBodyManipulator('../../../examples/FurutaPendulum/FurutaPendulum.urdf');
nq = p.getNumStates()/2;

options.grad_method = {'user','taylorvar'};
for i=1:100
  q = randn(nq,1);
  body_ind = 3;

  % test gradients
  [rpy,J] = geval(@myfun,q,options);

  % reconstruct R and test
  Rx = [1 0 0; 0 cos(rpy(1)) -sin(rpy(1)); 0 sin(rpy(1)) cos(rpy(1))];
  Ry = [cos(rpy(2)) 0 sin(rpy(2)); 0 1 0; -sin(rpy(2)) 0 cos(rpy(2))];
  Rz = [cos(rpy(3)) -sin(rpy(3)) 0; sin(rpy(3)) cos(rpy(3)) 0; 0 0 1];

  R = Rz*Ry*Rx;
  valuecheck(R,p.body(body_ind).T(1:3,1:3));
end

  function [rpy,J] = myfun(q)
    kinsol = p.doKinematics(q,false,false);
    [rpy,J] = p.forwardKinRot(kinsol,body_ind); 
  end

end