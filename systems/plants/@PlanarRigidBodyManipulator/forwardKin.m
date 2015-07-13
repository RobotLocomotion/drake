function [x,J,dJ] = forwardKin(obj,kinsol,body_or_frame_id,pts,options)
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

% method signature transition
if nargin < 5
  options = struct();
elseif isnumeric(options)
  % TODO: enable warning
%   obj.warning_manager.warnOnce('Drake:RigidBodyManipulator:forwardKin:method_signature_changed', ...
%     'Argument `options` is numeric and will be interpreted as options.rotation_type. This will be phased out; please use an options struct argument instead.');
  rotation_type = options;
  options = struct();
  options.rotation_type = rotation_type;
end

if ~isfield(options, 'rotation_type'), options.rotation_type = 0; end


[n,m] = size(pts); nq = getNumPositions(obj);
if (n==2) 
  pts = obj.T_2D_to_3D * pts; 
  T_3D_to_2D = obj.T_2D_to_3D';

  if (options.rotation_type>0)
    options.rotation_type = 1;
    T_3D_to_2D = [T_3D_to_2D, zeros(2,3); zeros(1,3), obj.view_axis'];
  end
end

if nargout>2
  [x,J,dJ] = forwardKin@RigidBodyManipulator(obj,kinsol,body_or_frame_id,pts,options);
elseif nargout>1
  [x,J] = forwardKin@RigidBodyManipulator(obj,kinsol,body_or_frame_id,pts,options);
else
  x = forwardKin@RigidBodyManipulator(obj,kinsol,body_or_frame_id,pts,options);
end  

if (n==2)
  x = T_3D_to_2D * x;
  if (nargout>1)
    nq = size(J,2);
    J = reshape(T_3D_to_2D * reshape(J,3,m*nq),2*m,nq);
    if nargout>2
      dJ = reshape(T_3D_to_2D * reshape(dJ,3,m*nq^2),2*m,nq^2);
    end
  end
end

end