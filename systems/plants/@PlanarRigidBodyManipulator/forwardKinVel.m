function [v,dv] = forwardKinVel(obj,kinsol,body_ind,pts,qd)
% @param kinsol solution structure obtained from doKinematics
% @param body_ind, an integer index for the body.  if body_ind is a
% PlanarRigidBody object, then this method will look up the index for you.
% @retval v the velocity of pts (given in the body frame) in the global frame
% @retval dv the Jacobian, [dvdq; dvdqd]
%
% if pts is a 2xm matrix, then v will be a 2xm matrix
%  and (following our gradient convention) dv will be a ((2m)x(2nq))

checkDirty(obj);
if (isa(body_ind,'PlanarRigidBody')) body_ind = find(obj.body==body_ind,1); end

if (kinsol.mex)
  if (obj.mex_model_ptr==0)
    error('Drake:PlanarRigidBodyManipulator:InvalidKinematics','This kinsol is no longer valid because the mex model ptr has been deleted.');
  end
  if ~isnumeric(pts)
    error('Drake:PlanarRigidBodyManipulator:InvalidKinematics','This kinsol is not valid because it was computed via mex, and you are now asking for an evaluation with non-numeric pts.  If you intended to use something like TaylorVar, then you must call doKinematics with use_mex = false');
  end
  
  if nargout > 1
    [v,dv] = forwardKinVelpmex(obj.mex_model_ptr.getData,body_ind-1,pts,qd);
  else
    v = forwardKinVelpmex(obj.mex_model_ptr.getData,body_ind-1,pts,qd);
  end
else
  if ~all(abs(kinsol.q-[obj.body.cached_q]')<1e-8)
    error('Drake:PlanarRigidBodyManipulator:InvalidKinematics','This kinsol is not longer valid.  Somebody has called doKinematics with a different q since the solution was computed.  If this happens a lot, I could consider returning the full T tree in kinsol, so I don''t have to rely on this caching mechanism');
  end
  
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