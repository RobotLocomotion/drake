function bodyKinTest()
  options.floating = true;
  r = RigidBodyManipulator('FallingBrick.urdf',options);
  m = 5;
  pts = rand(3,m);

  q = rand(r.getNumPositions(),1);

  geval_options.grad_method = {'user','taylorvar'};
  [x,J,dJ] = geval(@bodyKinWrapper,q,geval_options);

  function [x,J,dJ] = bodyKinWrapper(q)
    kinsol = r.doKinematics(q,true,false);
    switch nargout
      case 1
        x = r.bodyKin(kinsol,2,pts);
      case 2
        [x, J] = r.bodyKin(kinsol,2,pts);
      case 3
        [x, J, ~, dJ] = r.bodyKin(kinsol,2,pts);
    end
  end
end

