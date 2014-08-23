function Jdot = forwardJacDot(obj,kinsol,body_or_frame_ind,pts,rotation_type,robotnum)
% same input as [x,J] = forwardKin but returns Jdot
% note: you must have called kinsol = doKinematics with qd passed in as the
% last argument

if nargin<6, robotnum=1; end
if nargin<5, rotation_type=0; end

[n,m] = size(pts); nq = getNumDOF(obj);
if (n==2) 
  pts = obj.T_2D_to_3D * pts; 
  T_3D_to_2D = obj.T_2D_to_3D';

  if (rotation_type>0)
    rotation_type = 1;
    error('need to update this like I updated forwardKin');
    T_3D_to_2D = [T_3D_to_2D, zeros(2,3); zeros(1,3), obj.view_axis'];
  end
end

Jdot = forwardJacDot@RigidBodyManipulator(obj,kinsol,body_or_frame_ind,pts,rotation_type);

if (n==2)
  if rotation_type>0
    Jdot = reshape(T_3D_to_2D * reshape(Jdot,6,m*nq),3*m,nq);
  else    
    Jdot = reshape(T_3D_to_2D * reshape(Jdot,3,m*nq),2*m,nq);
  end
end

end