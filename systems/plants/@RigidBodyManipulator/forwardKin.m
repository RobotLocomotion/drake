function [x,J,dJ] = forwardKin(obj,kinsol,body_ind,pts,rotation_type)
% computes the position of pts (given in the body frame) in the global frame
%
% @param kinsol solution structure obtained from doKinematics
% @param body_ind, an integer index for the body.  if body_ind is a
% RigidBody object, then this method will look up the index for you.
% @param rotation_type boolean flag indicated whether rotations and
% derivatives should be computed
% @retval x the position of pts (given in the body frame) in the global
% frame. If rotation_type, x is 6-by-num_pts where the final 3
% components are the roll/pitch/yaw of the body frame (same for all points 
% on the body) 
% @retval J the Jacobian, dxdq
% @retval dJ the gradients of the Jacobian, dJdq---not implemented yet for
% rotations
%
% rotation_type  -- 0, no rotation included
%                -- 1, output Euler angle
%                -- 2, output quaternion
% if rotation_type = 0:
% if pts is a 3xm matrix, then x will be a 3xm matrix
%  and (following our gradient convention) J will be a ((3xm)x(q))
%  matrix, with [J1;J2;...;Jm] where Ji = dxidq
% if rotation_type = 1:
% x will be a 6xm matrix and (following our gradient convention) J will be 
% a ((6xm)x(q)) matrix, with [J1;J2;...;Jm] where Ji = dxidq
% if rotation_type = 2:
% x will be a 7xm matrix and (following out gradient convention) J will be
% a ((7xm)*(q)) matrix with [J1;J2;....;Jm] where Ji = dxidq

if nargin<5
  rotation_type = 0;
end

% todo: zap this after the transition
if isa(body_ind,'RigidBody'), error('support for passing in RigidBody objects has been removed.  please pass in the body index'); end

if (kinsol.mex)
  if (obj.mex_model_ptr==0)
    error('Drake:RigidBodyManipulator:InvalidKinematics','This kinsol is no longer valid because the mex model ptr has been deleted.');
  end
  if  ~isnumeric(pts)
    error('Drake:RigidBodyManipulator:InvalidKinematics','This kinsol is not valid because it was computed via mex, and you are now asking for an evaluation with non-numeric pts.  If you intended to use something like TaylorVar, then you must call doKinematics with use_mex = false');
  end
  
  if nargout > 2
    [x,J,dJ] = forwardKinmex(obj.mex_model_ptr,kinsol.q,body_ind,pts,rotation_type);
  elseif nargout > 1
    [x,J] = forwardKinmex(obj.mex_model_ptr,kinsol.q,body_ind,pts,rotation_type);
  else
    x = forwardKinmex(obj.mex_model_ptr,kinsol.q,body_ind,pts,rotation_type);
  end
  
else
  m = size(pts,2);
  pts = [pts;ones(1,m)];
  
  if (rotation_type == 1)
    R = kinsol.T{body_ind}(1:3,1:3);
    x = zeros(6,m);
    x(1:3,:) = kinsol.T{body_ind}(1:3,:)*pts;

    x(4:6,:) = repmat(rotmat2rpy(R),1,m);
  elseif(rotation_type == 0)
    x = kinsol.T{body_ind}(1:3,:)*pts;
  elseif(rotation_type == 2)
      R = kinsol.T{body_ind}(1:3,1:3);
      x = zeros(7,m);
      x(1:3,:) = kinsol.T{body_ind}(1:3,:)*pts;
      x(4:7,:) = bsxfun(@times,rotmat2quat(R),ones(1,m));
  end
  
  if (nargout>1)
    nq = obj.num_q;
    if (rotation_type == 1)
      Jx = reshape(kinsol.dTdq{body_ind}*pts,nq,[])';

      Jr = zeros(3,nq);
      % note the unusual format of dTdq 
      % dTdq = [dT(1,:)dq1; dT(1,:)dq2; ...; dT(1,:)dqN; dT(2,dq1) ...]
    
      % droll_dq
      idx = sub2ind(size(kinsol.dTdq{body_ind}),(3-1)*nq+(1:nq),2*ones(1,nq));
      dR32_dq = kinsol.dTdq{body_ind}(idx);
      idx = sub2ind(size(kinsol.dTdq{body_ind}),(3-1)*nq+(1:nq),3*ones(1,nq));
      dR33_dq = kinsol.dTdq{body_ind}(idx);
      sqterm = R(3,2)^2 + R(3,3)^2;
      Jr(1,:) = (R(3,3)*dR32_dq - R(3,2)*dR33_dq)/sqterm;

      % dpitch_dq
      idx = sub2ind(size(kinsol.dTdq{body_ind}),(3-1)*nq+(1:nq),ones(1,nq));
      dR31_dq = kinsol.dTdq{body_ind}(idx);
      Jr(2,:) = (-sqrt(sqterm)*dR31_dq + R(3,1)/sqrt(sqterm)*(R(3,2)*dR32_dq + R(3,3)*dR33_dq) )/(R(3,1)^2 + R(3,2)^2 + R(3,3)^2);

      % dyaw_dq
      idx = sub2ind(size(kinsol.dTdq{body_ind}),(1-1)*nq+(1:nq),ones(1,nq));
      dR11_dq = kinsol.dTdq{body_ind}(idx);
      idx = sub2ind(size(kinsol.dTdq{body_ind}),(2-1)*nq+(1:nq),ones(1,nq));
      dR21_dq = kinsol.dTdq{body_ind}(idx);
      sqterm = R(1,1)^2 + R(2,1)^2;
      Jr(3,:) = (R(1,1)*dR21_dq - R(2,1)*dR11_dq)/sqterm;

%       Mr = repmat([zeros(3); eye(3)],m,1);
%       J = Mr*Jr;
%       for j=1:m
%         % there must be a better way to write this --sk
%         J((j-1)*6+(1:3),:) = Jx((j-1)*3+(1:3),:);
%       end
      Jtmp = [Jx;Jr];
      Jrow_ind = reshape([reshape(1:3*m,3,m);bsxfun(@times,3*m+(1:3)',ones(1,m))],[],1);
      J = Jtmp(Jrow_ind,:);
    elseif(rotation_type == 0)
      J = reshape(kinsol.dTdq{body_ind}*pts,nq,[])';
    elseif(rotation_type == 2)
        Jx = reshape(kinsol.dTdq{body_ind}(1:3*nq,:)*pts,nq,[])';
        
        idx = sub2ind(size(kinsol.dTdq{body_ind}),(1-1)*nq+(1:nq),ones(1,nq));
        dR11_dq = kinsol.dTdq{body_ind}(idx);
        idx = sub2ind(size(kinsol.dTdq{body_ind}),(1-1)*nq+(1:nq),2*ones(1,nq));
        dR12_dq = kinsol.dTdq{body_ind}(idx);
        idx = sub2ind(size(kinsol.dTdq{body_ind}),(1-1)*nq+(1:nq),3*ones(1,nq));
        dR13_dq = kinsol.dTdq{body_ind}(idx);
        idx = sub2ind(size(kinsol.dTdq{body_ind}),(2-1)*nq+(1:nq),ones(1,nq));
        dR21_dq = kinsol.dTdq{body_ind}(idx);
        idx = sub2ind(size(kinsol.dTdq{body_ind}),(2-1)*nq+(1:nq),2*ones(1,nq));
        dR22_dq = kinsol.dTdq{body_ind}(idx);
        idx = sub2ind(size(kinsol.dTdq{body_ind}),(2-1)*nq+(1:nq),3*ones(1,nq));
        dR23_dq = kinsol.dTdq{body_ind}(idx);
        idx = sub2ind(size(kinsol.dTdq{body_ind}),(3-1)*nq+(1:nq),ones(1,nq));
        dR31_dq = kinsol.dTdq{body_ind}(idx);
        idx = sub2ind(size(kinsol.dTdq{body_ind}),(3-1)*nq+(1:nq),2*ones(1,nq));
        dR32_dq = kinsol.dTdq{body_ind}(idx);
        idx = sub2ind(size(kinsol.dTdq{body_ind}),(3-1)*nq+(1:nq),3*ones(1,nq));
        dR33_dq = kinsol.dTdq{body_ind}(idx);
        
        dqwdq = (dR11_dq+dR22_dq+dR33_dq)/(4*sqrt(1+R(1,1)+R(2,2)+R(3,3)));
        qw = x(4,1); 
        wsquare4 = 4*qw^2;
        dqxdq = ((dR32_dq-dR23_dq)*qw-(R(3,2)-R(2,3))*dqwdq)/wsquare4;
        dqydq = ((dR13_dq-dR31_dq)*qw-(R(1,3)-R(3,1))*dqwdq)/wsquare4;
        dqzdq = ((dR21_dq-dR12_dq)*qw-(R(2,1)-R(1,2))*dqwdq)/wsquare4;
        Jq = [dqwdq;dqxdq;dqydq;dqzdq];
        
        Jtmp = [Jx;Jq];
        Jrow_ind = reshape([reshape(1:3*m,3,m);bsxfun(@times,3*m+(1:4)',ones(1,m))],[],1);
        J = Jtmp(Jrow_ind,:);
        
    end
    if (nargout>2)
      if (rotation_type>0)
        warning('Second derivatives of rotations are not implemented yet.');
      end
      if isempty(kinsol.ddTdqdq{body_ind})
        error('you must call doKinematics with the second derivative option enabled');
      end
      ind = repmat(1:3*nq,nq,1)+repmat((0:3*nq:3*nq*(nq-1))',1,3*nq);
      dJ = reshape(kinsol.ddTdqdq{body_ind}(ind,:)*pts,nq^2,[])';
    end
  end
end

end