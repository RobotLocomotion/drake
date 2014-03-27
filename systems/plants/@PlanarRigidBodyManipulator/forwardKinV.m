function [x,Jv,Jvdot_times_v] = forwardKinV(obj,kinsol,body_ind,pts,rotation_type,base_ind)
%
% see RigidBodyManipulator/fowardKinV

if nargin<6, base_ind = 1; end
if nargin<5, rotation_type = 0; end

[n,m] = size(pts); nq = getNumPositions(obj);
if (n==2) 
  pts = obj.T_2D_to_3D * pts; 
  T_3D_to_2D = obj.T_2D_to_3D';

  if (rotation_type>0)
    rotation_type = 1;
    T_3D_to_2D = [T_3D_to_2D, zeros(2,3); zeros(1,3), obj.view_axis'];
  end
end

if nargout>2
  [x,Jv,Jvdot_times_v] = forwardKinV@RigidBodyManipulator(obj,kinsol,body_ind,pts,rotation_type,base_ind);
elseif nargout>1
  [x,Jv] = forwardKinV@RigidBodyManipulator(obj,kinsol,body_ind,pts,rotation_type,base_ind);
else
  x = forwardKinV@RigidBodyManipulator(obj,kinsol,body_ind,pts,rotation_type,base_ind);
end  

if (n==2)
  x = T_3D_to_2D * x;
  if (nargout>1)
    nv = size(Jv,2);
    Jv = reshape(T_3D_to_2D * reshape(Jv,3,m*nv),2*m,nv);
    if (nargout>2)
      Jvdot_times_v = T_3D_to_2D * Jvdot_times_v;
    end
  end
end

end