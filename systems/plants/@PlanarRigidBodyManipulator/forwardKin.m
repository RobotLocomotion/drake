function [x,J,dJ] = forwardKin(obj,body_ind,pts,use_mex_if_possible)
% @param body_ind, an integer index for the body.  if body_ind is a
% RigidBody object, then this method will look up the index for you.
% @retval x the position of pts (given in the body frame) in the global frame
% @retval J the Jacobian, dxdq
% @retval dJ the gradients of the Jacobian, dJdq
%
% Note: for efficiency, assumes that "doKinematics" has been called on the model
% if pts is a 2xm matrix, then x will be a 2xm matrix
%  and (following our gradient convention) J will be a ((2xm)x(nq))
%  matrix, with [J1;J2;...;Jm] where Ji = dxidq
% and dJ will be a (2xm)x(nq^2) matrix

if (nargin<4) use_mex_if_possible = true; end
if (isa(body_ind,'RigidBody')) body_ind = find(obj.body==body_ind,1); end
if (use_mex_if_possible && obj.mex_model_ptr && isnumeric(pts))
  if nargout > 2
    [x,J,dJ] = forwardKinpmex(obj.mex_model_ptr,body_ind-1,pts);
  elseif nargout > 1
    [x,J] = forwardKinpmex(obj.mex_model_ptr,body_ind-1,pts);
  else
    x = forwardKinpmex(obj.mex_model_ptr,body_ind-1,pts);
  end
else
  body = obj.body(body_ind);
  m = size(pts,2);
  pts = [pts;ones(1,m)];
  x = body.T(1:2,:)*pts;
  if (nargout>1)
    nq = size(body.dTdq,1)/3;
    J = reshape(body.dTdq(1:2*nq,:)*pts,nq,[])';
    if (nargout>2)
      if isempty(body.ddTdqdq)
        error('you must call doKinematics with the second derivative option enabled');
      end
      ind = repmat(1:2*nq,nq,1)+repmat((0:3*nq:3*nq*(nq-1))',1,2*nq);
      dJ = reshape(body.ddTdqdq(ind,:)*pts,nq^2,[])';
    end
  end
end

end