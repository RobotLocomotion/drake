function [x,J,dJ] = forwardKin(obj,kinsol,body_ind,pts)
% @param kinsol solution structure obtained from doKinematics
% @param body_ind, an integer index for the body.  if body_ind is a
% RigidBody object, then this method will look up the index for you.
% @retval x the position of pts (given in the body frame) in the global frame
% @retval J the Jacobian, dxdq
% @retval dJ the gradients of the Jacobian, dJdq
%
% computes the position of pts (given in the body frame) in the global frame
% for efficiency, assumes that "doKinematics" has been called on the model
% if pts is a 3xm matrix, then x will be a 3xm matrix
%  and (following our gradient convention) J will be a ((3xm)x(q))
%  matrix, with [J1;J2;...;Jm] where Ji = dxidq

typecheck(kinsol,'struct');  % this should catch people who haven't switched to the new interface yet.
if (isa(body_ind,'RigidBody')) body_ind = find(obj.body_ind==body,1); end

if (kinsol.mex)
  if ~obj.mex_model_ptr
    error('Drake:RigidBodyManipulator:InvalidKinematics','This kinsol is no longer valid because the mex model ptr has been deleted.');
  end
  if  ~isnumeric(pts)
    error('Drake:RigidBodyManipulator:InvalidKinematics','This kinsol is not valid because it was computed via mex, and you are now asking for an evaluation with non-numeric pts.  If you intended to use something like TaylorVar, then you must call doKinematics with use_mex = false');
  end
  
  if nargout > 2
    [x,J,dJ] = forwardKinmex(obj,body_ind-1,pts);
  elseif nargout > 1
    [x,J] = forwardKinmex(obj,body_ind-1,pts);
  else
    x = forwardKinmex(obj,body_ind-1,pts);
  end
  
else
  if ~all(abs(q-[model.body.cached_q]')<1e-8)
    error('Drake:RigidBodyManipulator:InvalidKinematics','This kinsol is not longer valid.  Somebody has called doKinematics with a different q since the solution was computed.  If this happens a lot, I could consider returning the full T tree in kinsol, so I don''t have to rely on this caching mechanism');
  end
  body = obj.body(body_ind);
  m = size(pts,2);
  pts = [pts;ones(1,m)];
  x = body.T(1:3,:)*pts;
  if (nargout>1)
    nq = size(body.dTdq,1)/4;
    J = reshape(body.dTdq(1:3*nq,:)*pts,nq,[])';
    if (nargout>2)
      if isempty(body.ddTdqdq)
        error('you must call doKinematics with the second derivative option enabled');
      end
      ind = repmat(1:3*nq,nq,1)+repmat((0:4*nq:4*nq*(nq-1))',1,3*nq);
      dJ = reshape(body.ddTdqdq(ind,:)*pts,nq^2,[])';
    end
  end
end

end