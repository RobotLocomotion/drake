function [x,J,dJ] = forwardKinV(obj,kinsol,body_ind,pts,rotation_type,base_ind)
% see RigidBodyManipulator/forwardKinV

compute_jacobian = nargout > 1;
compute_gradient = nargout > 2;

if nargin<6, base_ind = 1; end
if nargin<5, rotation_type = 0; end

[n,m] = size(pts);
if (n==2) 
  pts = obj.T_2D_to_3D * pts; 
  T_3D_to_2D = obj.T_2D_to_3D';

  if (rotation_type>0)
    rotation_type = 1;
    T_3D_to_2D = [T_3D_to_2D, zeros(2,3); zeros(1,3), obj.view_axis'];
  end
end

if compute_gradient
  [x,J,dJ] = forwardKinV@RigidBodyManipulator(obj,kinsol,body_ind,pts,rotation_type,base_ind);
elseif compute_jacobian
  [x,J] = forwardKinV@RigidBodyManipulator(obj,kinsol,body_ind,pts,rotation_type,base_ind);
else
  x = forwardKinV@RigidBodyManipulator(obj,kinsol,body_ind,pts,rotation_type,base_ind);
end  

if (n==2)
  x = T_3D_to_2D * x;
  if (compute_jacobian)
    nv = size(J,2);
    J = reshape(T_3D_to_2D * reshape(J,3,m*nv),2*m,nv);
    if compute_gradient
      dJ = reshape(T_3D_to_2D * reshape(dJ,3,m*nv^2),2*m,nv^2);
%       error('need to implement and test');
    end
  end
end

end