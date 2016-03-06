function [com,J,dJ] = getCOM(model,kinsol)

% see getCOM in RigidBodyManipulator for more info

if nargout>2
  [com,J,dJ] = getCOM@RigidBodyManipulator(model,kinsol);
elseif nargout>1
  [com,J] = getCOM@RigidBodyManipulator(model,kinsol);
else
  com = getCOM@RigidBodyManipulator(model,kinsol);
end

com = model.T_2D_to_3D'*com;
if nargout>1
  J = model.T_2D_to_3D'*J;
  if nargout>2
    error('need to implement dJ');
  end
end
