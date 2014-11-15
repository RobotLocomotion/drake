function bodyKinTest()
  options.floating = false;
  r = RigidBodyManipulator(fullfile(getDrakePath(),'examples','KneedCompassGait','KneedCompassGait.urdf'),options);
  m = 1;
  pts = ones(3,m);
  body_ind = 4;

  q = rand*ones(r.getNumPositions(),1);

  geval_options.grad_method = {'user','taylorvar'};
  [x,J,dJ] = geval(@(q)bodyKinWrapper(q,r,body_ind,pts),q,geval_options);
  [P,dP] = geval(@(q)bodyKinPWrapper(q,r,body_ind,pts),q,geval_options);
end

function [x,J,dJ] = bodyKinWrapper(q,r,body_ind,pts)
  kinsol = r.doKinematics(q,true,false);
  switch nargout
    case 1
      x = r.bodyKin(kinsol,body_ind,pts);
    case 2
      [x, ~, J] = r.bodyKin(kinsol,body_ind,pts);
    case 3
      [x, ~, J, ~, dJ] = r.bodyKin(kinsol,body_ind,pts);
  end
end

function [P,dP] = bodyKinPWrapper(q,r,body_ind,pts)
  kinsol = r.doKinematics(q,true,false);
  switch nargout
    case 1
      [~, P] = r.bodyKin(kinsol,body_ind,pts);
    case 2
      [~, P, ~, dP] = r.bodyKin(kinsol,body_ind,pts);
  end
end
