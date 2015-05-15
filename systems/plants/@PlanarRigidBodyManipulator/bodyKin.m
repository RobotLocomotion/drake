function [x,P,J,dP,dJ] = bodyKin(obj,kinsol,body_ind,pts)
% computes the position of pts (given in the global frame) in the body frame
% (see documentation for bodyKin in RigidBodyManipulator)
%
% Note: if pts are 2xm, then x will be 2xm (e.g., planar)
% but if pts are 3xm, then the underlying 3D kinematics will be used (and
% returned).  

[n,m] = size(pts); 
if (n==2), 
  T = kron(obj.T_2D_to_3D,eye(size(pts,2)));
  pts = obj.T_2D_to_3D * pts; 
end

if (nargout>4)
  [x,P,J,dP,dJ] = bodyKin@RigidBodyManipulator(obj,kinsol,body_ind,pts);
elseif (nargout>3)
  [x,P,J,dP] = bodyKin@RigidBodyManipulator(obj,kinsol,body_ind,pts);
elseif (nargout>2)
  [x,P,J] = bodyKin@RigidBodyManipulator(obj,kinsol,body_ind,pts);
elseif (nargout>1)
  [x,P] = bodyKin@RigidBodyManipulator(obj,kinsol,body_ind,pts);
else
  x = bodyKin@RigidBodyManipulator(obj,kinsol,body_ind,pts);
end

if (n==2), 
  x = obj.T_2D_to_3D'*x; 
  if nargout > 1
    P = T'*P*T; 
  end
  if nargout > 2
    J = T'*J; 
  end
  if nargout > 3
    keyboard('dP planar case not implemented yet')
    % transformation for the gradient
    T_dP = [reshape(T(:,1)*T(:,1)',1,[]);reshape(T(:,2)*T(:,1)',1,[]);reshape(T(:,1)*T(:,2)',1,[]);reshape(T(:,2)*T(:,2)',1,[])];
    dP = T_dP*dP; 
  end
  if nargout > 4
    dJ = T'*dJ; 
  end
end
