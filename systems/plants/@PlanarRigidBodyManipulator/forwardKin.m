function [x,J,dJ] = forwardKin(obj,kinsol,body_ind,pts,rotation_type)
% computes the position of pts (given in the body frame) in the global frame
%
% @param kinsol solution structure obtained from doKinematics
% @param body_ind, an integer index for the body.  if body_ind is a
% RigidBody object, then this method will look up the index for you.
% @param rotation_type boolean flag indicated whether rotations and
% derivatives should be computed @default false
% @retval x the position of pts (given in the body frame) in the global frame
% @retval J the Jacobian, dxdq
% @retval dJ the gradients of the Jacobian, dJdq
%
% if pts is a 2xm matrix, then x will be a 2xm matrix
%  and (following our gradient convention) J will be a ((2xm)x(nq))
%  matrix, with [J1;J2;...;Jm] where Ji = dxidq
% and dJ will be a (2xm)x(nq^2) matrix
%
% Note: if pts are 2xm, then x will be 2xm (e.g., planar)
% but if pts are 3xm, then the underlying 3D kinematics will be used (and
% returned).  


if nargin<5, rotation_type = 0; end

[n,m] = size(pts); nq = getNumDOF(obj);
if (n==2) 
  pts = obj.T_2D_to_3D * pts; 
  T_3D_to_2D = obj.T_2D_to_3D';

  if (rotation_type>0)
    rotation_type = 2;
  end
end

if nargout>2
  [x,J,dJ] = forwardKin@RigidBodyManipulator(obj,kinsol,body_ind,pts,rotation_type);
elseif nargout>1
  [x,J] = forwardKin@RigidBodyManipulator(obj,kinsol,body_ind,pts,rotation_type);
else
  x = forwardKin@RigidBodyManipulator(obj,kinsol,body_ind,pts,rotation_type);
end  

if (n==2)
  if rotation_type
    if m>1, error('still need to handle this case'); end
    quat = x(4:7);
    axis = quat2axis(quat);
    signFlip = 1;
    if dot(axis(1:3),obj.view_axis)<0, axis=-axis; signFlip=-1; end
    if abs(dot(axis(1:3),obj.view_axis)-1)>1e-6,
      error('Drake:PlanarRigidBodyManipulator:OutOfPlaneRotations','The requested rotation is outside of the viewing axis.  Cannot return a meaningful scalar rotation');
    end
    angle=axis(4);
    if nargout>1
      Jangle = quatdot2angularvelMatrix(quat)*J(4:7,:);
      % since i only support view_axis-aligned rotations, the angular
      % velocity vector must be aligned with the view_axis, so I can pull
      % off a single element.  (Note: i can't actually test that omega is
      % aligned with the view_axis without knowing qdot.)
      Jangle = signFlip*obj.view_axis'*Jangle;
      if nargout>2
        error('still need to handle this case');
      end
    end      
  else
    angle=[]; Jangle=[]; dJangle=[];
  end
  x = [T_3D_to_2D * x(1:3,:); angle];
  if (nargout>1)
    nq = size(J,2);
    J = [reshape(T_3D_to_2D * reshape(J(1:3,:),3,m*nq),2*m,nq); Jangle];
    if nargout>2
      dJ = [reshape(T_3D_to_2D * reshape(dJ(1:3,:),3,m*nq^2),2*m,nq^2); dJangle];
    end
  end
end

end