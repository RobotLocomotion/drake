function testForwardKinRot

p = RigidBodyManipulator('FurutaPendulum.urdf');
nq = p.getNumStates()/2;

options.grad_method = {'user','numerical'};
options.tol = 1e-6;

% The assignment operation I'm using in forwardKin breaks taylorvar, but it
% will be easy to fix by doing the assignment with a matrix left multiply
% instead. I'll fix this soon, just wanted to get something committed
% before I run out the door --sk

lower_link = 3;


for i=1:100
  q = randn(nq,1);
  body_ind = 3;

  % test gradients
  [rpy,J] = geval(@myfun,q,options);

  % test gradients mex
  [rpy,J] = geval(@myfun2,q,options);
  
  [quat1,J] = geval(@myfun3,q,options);
  
  [quat2,J] = geval(@myfun4,q,options);
  rpy = quat2rpy(quat1);
  % reconstruct R and test
  Rx = [1 0 0; 0 cos(rpy(1)) -sin(rpy(1)); 0 sin(rpy(1)) cos(rpy(1))];
  Ry = [cos(rpy(2)) 0 sin(rpy(2)); 0 1 0; -sin(rpy(2)) 0 cos(rpy(2))];
  Rz = [cos(rpy(3)) -sin(rpy(3)) 0; sin(rpy(3)) cos(rpy(3)) 0; 0 0 1];
  
  R = Rz*Ry*Rx;
  kinsol = p.doKinematics(q,false,false);
  valuecheck(R,kinsol.T{body_ind}(1:3,1:3),1e-6);
  
  [~,J] = myfun5(q);
  [~,J2] = geval(@myfun5,q,struct('grad_method','numerical'));
  valuecheck(J,J2,1e-6);
end

  function [rpy,J] = myfun(q)
    kinsol = p.doKinematics(q,false,false);
    [x,J] = p.forwardKin(kinsol,body_ind,[0;0;0],1); 
    rpy = x(4:6);
    J = J(4:6,:);
  end

  function [rpy,J] = myfun2(q)
    kinsol = p.doKinematics(q,false,true);
    [x,J] = p.forwardKin(kinsol,body_ind,[0;0;0],1); 
    rpy = x(4:6);
    J = J(4:6,:);
  end

  function [quat,J] = myfun3(q)
    kinsol = p.doKinematics(q,false,false);
    [x,J] = p.forwardKin(kinsol,body_ind,[0;0;0],2); 
    quat = x(4:7);
    J = J(4:7,:);
  end

  function [quat,J] = myfun4(q)
    kinsol = p.doKinematics(q,false,true);
    [x,J] = p.forwardKin(kinsol,body_ind,[0;1;0],2); 
    quat = x(4:7);
    J = J(4:7,:);
  end

  function [x,J] = myfun5(q)
    kinsol = p.doKinematics(q,false,true);
    [x,J] = p.forwardKin(kinsol,lower_link,[[0;1;0] [1;0;0.5]],1);
    x = x(:);
  end

end