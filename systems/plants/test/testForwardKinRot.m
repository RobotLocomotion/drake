function testForwardKinRot

p = RigidBodyManipulator('../../../examples/FurutaPendulum/FurutaPendulum.urdf');
nq = p.getNumStates()/2;

options.grad_method = {'user','numerical'};

% NOTE: this test with the above numerical geval requires that the tolerance on value check be
% around 1e-7 or 1e-6, not 1e-8

% The assignment operation I'm using in forwardKin breaks taylorvar, but it
% will be easy to fix by doing the assignment with a matrix left multiply
% instead. I'll fix this soon, just wanted to get something committed
% before I run out the door --sk

for i=1:100
  q = randn(nq,1);
  body_ind = 3;

  % test gradients
  [rpy,J] = geval(@myfun,q,options);

  % test gradients mex
  [rpy,J] = geval(@myfun2,q,options);

  % reconstruct R and test
  Rx = [1 0 0; 0 cos(rpy(1)) -sin(rpy(1)); 0 sin(rpy(1)) cos(rpy(1))];
  Ry = [cos(rpy(2)) 0 sin(rpy(2)); 0 1 0; -sin(rpy(2)) 0 cos(rpy(2))];
  Rz = [cos(rpy(3)) -sin(rpy(3)) 0; sin(rpy(3)) cos(rpy(3)) 0; 0 0 1];

  R = Rz*Ry*Rx;
  valuecheck(R,p.body(body_ind).T(1:3,1:3));
end

  function [rpy,J] = myfun(q)
    kinsol = p.doKinematics(q,false,false);
    [x,J] = p.forwardKin(kinsol,body_ind,[0;0;0],true); 
    rpy = x(4:6);
    J = J(4:6,:);
  end

  function [rpy,J] = myfun2(q)
    kinsol = p.doKinematics(q,false,true);
    [x,J] = p.forwardKin(kinsol,body_ind,[0;0;0],true); 
    rpy = x(4:6);
    J = J(4:6,:);
  end

end