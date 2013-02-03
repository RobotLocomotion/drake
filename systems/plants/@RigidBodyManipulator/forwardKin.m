function [x,J,dJ] = forwardKin(obj,kinsol,body_ind,pts,include_rotations)
% computes the position of pts (given in the body frame) in the global frame
%
% @param kinsol solution structure obtained from doKinematics
% @param body_ind, an integer index for the body.  if body_ind is a
% RigidBody object, then this method will look up the index for you.
% @param include_rotations boolean flag indicated whether rotations and
% derivatives should be computed
% @retval x the position of pts (given in the body frame) in the global
% frame. If include_rotations, x is 6-by-num_pts where the final 3
% components are the roll/pitch/yaw of the body frame (same for all points 
% on the body) 
% @retval J the Jacobian, dxdq
% @retval dJ the gradients of the Jacobian, dJdq---not implemented yet for
% rotations
%
% if ~include_rotations:
% if pts is a 3xm matrix, then x will be a 3xm matrix
%  and (following our gradient convention) J will be a ((3xm)x(q))
%  matrix, with [J1;J2;...;Jm] where Ji = dxidq
% otherwise:
% x will be a 6xm matrix and (following our gradient convention) J will be 
% a ((6xm)x(q)) matrix, with [J1;J2;...;Jm] where Ji = dxidq

if nargin<5
  include_rotations = false;
end

if (kinsol.mex)
%   if (include_rotations)
%     error('Drake:RigidBodyManipulator:InvalidKinematics','Rotations are not mexed yet. Call non-mexed kinematics.');
%   end
  if (isa(body_ind,'RigidBody')) body_ind = find(obj.body==body_ind,1); end

  if (obj.mex_model_ptr==0)
    error('Drake:RigidBodyManipulator:InvalidKinematics','This kinsol is no longer valid because the mex model ptr has been deleted.');
  end
  if  ~isnumeric(pts)
    error('Drake:RigidBodyManipulator:InvalidKinematics','This kinsol is not valid because it was computed via mex, and you are now asking for an evaluation with non-numeric pts.  If you intended to use something like TaylorVar, then you must call doKinematics with use_mex = false');
  end
  
  if nargout > 2
    [x,J,dJ] = forwardKinmex(obj.mex_model_ptr.getData,kinsol.q,body_ind-1,pts,include_rotations);
  elseif nargout > 1
    [x,J] = forwardKinmex(obj.mex_model_ptr.getData,kinsol,q,body_ind-1,pts,include_rotations);
  else
    x = forwardKinmex(obj.mex_model_ptr.getData,kinsol.q,body_ind-1,pts,include_rotations);
  end
  
else
  if ~all(abs(kinsol.q-[obj.body.cached_q]')<1e-8)
    error('Drake:RigidBodyManipulator:InvalidKinematics','This kinsol is not longer valid.  Somebody has called doKinematics with a different q since the solution was computed.  If this happens a lot, I could consider returning the full T tree in kinsol, so I don''t have to rely on this caching mechanism');
  end
  if (isa(body_ind,'RigidBody')) body=body_ind; 
  else body = obj.body(body_ind); end
  
  m = size(pts,2);
  pts = [pts;ones(1,m)];

  if (include_rotations)
    R = body.T(1:3,1:3);
    x = zeros(6,m);
    x(1:3,:) = body.T(1:3,:)*pts;

    x(4:6,:) = repmat(rotmat2rpy(R),1,m);
  else
    x = body.T(1:3,:)*pts;
  end
  
  if (nargout>1)
    nq = size(body.dTdq,1)/4;
    if (include_rotations)
      Jx = reshape(body.dTdq(1:3*nq,:)*pts,nq,[])';

      Jr = zeros(3,nq);
      % note the unusual format of dTdq 
      % dTdq = [dT(1,:)dq1; dT(1,:)dq2; ...; dT(1,:)dqN; dT(2,dq1) ...]
    
      % droll_dq
      idx = sub2ind(size(body.dTdq),(3-1)*nq+(1:nq),2*ones(1,nq));
      dR32_dq = body.dTdq(idx);
      idx = sub2ind(size(body.dTdq),(3-1)*nq+(1:nq),3*ones(1,nq));
      dR33_dq = body.dTdq(idx);
      sqterm = R(3,2)^2 + R(3,3)^2;
      Jr(1,:) = (R(3,3)*dR32_dq - R(3,2)*dR33_dq)/sqterm;

      % dpitch_dq
      idx = sub2ind(size(body.dTdq),(3-1)*nq+(1:nq),ones(1,nq));
      dR31_dq = body.dTdq(idx);
      Jr(2,:) = (-sqrt(sqterm)*dR31_dq + R(3,1)/sqrt(sqterm)*(R(3,2)*dR32_dq + R(3,3)*dR33_dq) )/(R(3,1)^2 + R(3,2)^2 + R(3,3)^2);

      % dyaw_dq
      idx = sub2ind(size(body.dTdq),(1-1)*nq+(1:nq),ones(1,nq));
      dR11_dq = body.dTdq(idx);
      idx = sub2ind(size(body.dTdq),(2-1)*nq+(1:nq),ones(1,nq));
      dR21_dq = body.dTdq(idx);
      sqterm = R(1,1)^2 + R(2,1)^2;
      Jr(3,:) = (R(1,1)*dR21_dq - R(2,1)*dR11_dq)/sqterm;

      Mr = repmat([zeros(3); eye(3)],m,1);
      J = Mr*Jr;
      for j=1:m
        % there must be a better way to write this --sk
        J((j-1)*6+(1:3),:) = Jx((j-1)*3+(1:3),:);
      end
    else
      J = reshape(body.dTdq(1:3*nq,:)*pts,nq,[])';
    end
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