function [v,dv] = forwardKinVel(obj,body_ind,pts,qd,use_mex_if_possible)
% @param body_ind, an integer index for the body.  if body_ind is a
% RigidBody object, then this method will look up the index for you.
% @retval v the velocity of pts (given in the body frame) in the global frame
% @retval dv the Jacobian, [dvdq; dvdqd]
%
% Note: for efficiency, assumes that "doKinematics" has been called on the model
% if pts is a 2xm matrix, then v will be a 2xm matrix
%  and (following our gradient convention) dv will be a ((2m)x(2nq))

if (nargin<5) use_mex_if_possible = true; end
if (isa(body_ind,'RigidBody')) body_ind = find(obj.body==body_ind,1); end
if (use_mex_if_possible && obj.mex_model_ptr && isnumeric(qd))
  if nargout > 1
    [v,dv] = forwardKinVelpmex(obj.mex_model_ptr,body_ind-1,pts,qd);
  else
    v = forwardKinVelpmex(obj.mex_model_ptr,body_ind-1,pts,qd);
  end
else
  body = obj.body(body_ind);
  nq = size(body.dTdq,1)/3;
  m = size(pts,2);
  pts = [pts;ones(1,m)];
  v = kron(eye(2),qd')*body.dTdq(1:2*nq,:)*pts;
  %       v = body.Tdot(1:2,:)*pts;
  if (nargout>1)
    dv = [zeros(2*size(pts,2),nq) reshape(body.dTdq(1:2*nq,:)*pts,[],2*size(pts,2))'];
    tmp = kron([1 0 0;0 1 0],qd');
    for i=1:nq,
      col = tmp*body.ddTdqdq((1:3*nq) + 3*nq*(i-1),:)*pts;
      dv(:,i) = col(:);
    end;
    %         dv = [reshape(body.dTdotdqqd([1:2*nq],:)*pts,nq,[])' reshape(body.dTdotdqqd([(1:2*nq)+3*nq],:)*pts,nq,[])'];
  end
end
end