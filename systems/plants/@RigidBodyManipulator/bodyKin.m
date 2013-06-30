function x = bodyKin(obj,kinsol,body_ind,pts)
% computes the position of pts (given in the global frame) in the body frame
%
% @param kinsol solution structure obtained from doKinematics
% @param body_ind, an integer index for the body.  
% @retval x the position of pts (given in the body frame) in the global frame
%% @retval J the Jacobian, dxdq
%% @retval dJ the gradients of the Jacobian, dJdq
%
% if pts is a 3xm matrix, then x will be a 3xm matrix
%  and (following our gradient convention) J will be a ((3xm)x(q))
%  matrix, with [J1;J2;...;Jm] where Ji = dxidq

checkDirty(obj);

% todo: zap this after the transition
if isa(body_ind,'RigidBody'), error('support for passing in RigidBody objects has been removed.  please pass in the body index'); end
  
if (kinsol.mex)
  if (obj.mex_model_ptr==0)
    error('Drake:RigidBodyManipulator:InvalidKinematics','This kinsol is no longer valid because the mex model ptr has been deleted.');
  end
  if  ~isnumeric(pts)
    error('Drake:RigidBodyManipulator:InvalidKinematics','This kinsol is not valid because it was computed via mex, and you are now asking for an evaluation with non-numeric pts.  If you intended to use something like TaylorVar, then you must call doKinematics with use_mex = false');
  end
  
  x = bodyKinmex(obj.mex_model_ptr,kinsol.q,body_ind,pts);
  
else
  d = length(obj.gravity);
  m = size(pts,2);
  pts = [pts;ones(1,m)];
  x = inv(kinsol.T{body_ind})*pts;
  x = x(1:d,:);
  
  % todo: implement jacobians
  % will almost certainly make use of d(inv(T))dqi = -inv(T)*dTdqi*inv(T)
end


end
