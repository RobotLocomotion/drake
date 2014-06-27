function [com,J,dJ] = centerOfMass(model,kinsol)

% see centerOfMass in RigidBodyManipulator for more info

if nargout>2
  [com,J,dJ] = centerOfMass@RigidBodyManipulator(model,kinsol);
elseif nargout>1
  [com,J] = centerOfMass@RigidBodyManipulator(model,kinsol);
else
  com = centerOfMass@RigidBodyManipulator(model,kinsol);
end

com = model.T_2D_to_3D'*com;
if nargout>1
  J = model.T_2D_to_3D'*J;
  if nargout>2
    error('need to implement dJ');
  end
end
