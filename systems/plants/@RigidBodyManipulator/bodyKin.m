function x = bodyKin(obj,kinsol,body_ind,pts)
% computes the position of pts (given in the global frame) in the body frame
%
% @param kinsol solution structure obtained from doKinematics
% @param body_ind, an integer index for the body.  if body_ind is a
% RigidBody object, then this method will look up the index for you.
% @retval x the position of pts (given in the body frame) in the global frame
%% @retval J the Jacobian, dxdq
%% @retval dJ the gradients of the Jacobian, dJdq
%
% if pts is a 3xm matrix, then x will be a 3xm matrix
%  and (following our gradient convention) J will be a ((3xm)x(q))
%  matrix, with [J1;J2;...;Jm] where Ji = dxidq

typecheck(kinsol,'struct');  % this should catch people who haven't switched to the new interface yet.

if (kinsol.mex)
  if (isa(body_ind,'RigidBody')) body_ind = find(obj.body==body_ind,1); end
  error('not implemented yet');
else
  if ~all(abs(kinsol.q-[obj.body.cached_q]')<1e-8)
    error('Drake:RigidBodyManipulator:InvalidKinematics','This kinsol is not longer valid.  Somebody has called doKinematics with a different q since the solution was computed.  If this happens a lot, I could consider returning the full T tree in kinsol, so I don''t have to rely on this caching mechanism');
  end
  if (isa(body_ind,'RigidBody')) body=body_ind; 
  else body = obj.body(body_ind); end
  
  d = length(obj.gravity);
  m = size(pts,2);
  pts = [pts;ones(1,m)];
  x = inv(body.T)*pts;
  x = x(1:d,:);
  
  % todo: implement jacobians
  % will almost certainly make use of d(inv(T))dqi = -inv(T)*dTdqi*inv(T)
end


end
