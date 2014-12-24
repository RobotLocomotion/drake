function [x,J,dJ] = forwardKin(obj,kinsol,body_or_frame_ind,pts,rotation_type)
% computes the position of pts (given in the body frame) in the global frame
%
% @param kinsol solution structure obtained from doKinematics
% @param body_or_frame_ind, an integer ID for a RigidBody or RigidBodyFrame
% (obtained via e.g., findLinkId or findFrameInd)
% @param rotation_type integer flag indicated whether rotations and
% derivatives should be computed (0 - no rotations, 1 - rpy, 2 - quat)
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
if isa(body_or_frame_ind,'RigidBody'), error('support for passing in RigidBody objects has been removed.  please pass in the body index'); end

if (kinsol.mex)
  if (obj.mex_model_ptr==0)
    error('Drake:RigidBodyManipulator:InvalidKinematics','This kinsol is no longer valid because the mex model ptr has been deleted.');
  end
  if  ~isnumeric(pts)
    error('Drake:RigidBodyManipulator:InvalidKinematics','This kinsol is not valid because it was computed via mex, and you are now asking for an evaluation with non-numeric pts.  If you intended to use something like TaylorVar, then you must call doKinematics with use_mex = false');
  end
  
  if nargout > 2
    [x,J,dJ] = forwardKinmex(obj.mex_model_ptr,kinsol.q,body_or_frame_ind,pts,rotation_type);
  elseif nargout > 1
    [x,J] = forwardKinmex(obj.mex_model_ptr,kinsol.q,body_or_frame_ind,pts,rotation_type);
  else
    x = forwardKinmex(obj.mex_model_ptr,kinsol.q,body_or_frame_ind,pts,rotation_type);
  end
  
else
  if (body_or_frame_ind < 0)
    frame = obj.frame(-body_or_frame_ind);
    body_ind = frame.body_ind;
    Tframe = frame.T;
  else
    body_ind = body_or_frame_ind;
    Tframe=eye(4);
  end
  
  m = size(pts,2);
  pts = [pts;ones(1,m)];
  T = kinsol.T{body_ind}*Tframe;
  
  switch (rotation_type)
    case 0
      x = T(1:3,:)*pts;
    case 1
      R = T(1:3,1:3);
      x = [T(1:3,:)*pts; repmat(rotmat2rpy(R),1,m)];
    case 2
      R = T(1:3,1:3);
      x = [T(1:3,:)*pts; bsxfun(@times,rotmat2quat(R),ones(1,m))];
  end
  
  if (nargout>1)
    nq = obj.num_positions;
    dTdq = kinsol.dTdq{body_ind}*Tframe;
    if (rotation_type == 1)
      Jx = reshape(dTdq*pts,nq,[])';

      Jr = zeros(3,nq);
      % note the unusual format of dTdq 
      % dTdq = [dT(1,:)dq1; dT(1,:)dq2; ...; dT(1,:)dqN; dT(2,dq1) ...]
    
      % droll_dq
      idx = sub2ind(size(dTdq),(3-1)*nq+(1:nq),2*ones(1,nq));
      dR32_dq = dTdq(idx);
      idx = sub2ind(size(dTdq),(3-1)*nq+(1:nq),3*ones(1,nq));
      dR33_dq = dTdq(idx);
      sqterm = R(3,2)^2 + R(3,3)^2;
      Jr(1,:) = (R(3,3)*dR32_dq - R(3,2)*dR33_dq)/sqterm;

      % dpitch_dq
      idx = sub2ind(size(dTdq),(3-1)*nq+(1:nq),ones(1,nq));
      dR31_dq = dTdq(idx);
      Jr(2,:) = (-sqrt(sqterm)*dR31_dq + R(3,1)/sqrt(sqterm)*(R(3,2)*dR32_dq + R(3,3)*dR33_dq) )/(R(3,1)^2 + R(3,2)^2 + R(3,3)^2);

      % dyaw_dq
      idx = sub2ind(size(dTdq),(1-1)*nq+(1:nq),ones(1,nq));
      dR11_dq = dTdq(idx);
      idx = sub2ind(size(dTdq),(2-1)*nq+(1:nq),ones(1,nq));
      dR21_dq = dTdq(idx);
      sqterm = R(1,1)^2 + R(2,1)^2;
      Jr(3,:) = (R(1,1)*dR21_dq - R(2,1)*dR11_dq)/sqterm;

      Jtmp = [Jx;Jr];
      Jrow_ind = reshape([reshape(1:3*m,3,m);bsxfun(@times,3*m+(1:3)',ones(1,m))],[],1);
      J = Jtmp(Jrow_ind,:);
    elseif(rotation_type == 0)
      J = reshape(dTdq*pts,nq,[])';
    elseif(rotation_type == 2)
        Jx = reshape(dTdq(1:3*nq,:)*pts,nq,[])';
        
        idx = sub2ind(size(dTdq),(1-1)*nq+(1:nq),ones(1,nq));
        dR11_dq = dTdq(idx);
        idx = sub2ind(size(dTdq),(1-1)*nq+(1:nq),2*ones(1,nq));
        dR12_dq = dTdq(idx);
        idx = sub2ind(size(dTdq),(1-1)*nq+(1:nq),3*ones(1,nq));
        dR13_dq = dTdq(idx);
        idx = sub2ind(size(dTdq),(2-1)*nq+(1:nq),ones(1,nq));
        dR21_dq = dTdq(idx);
        idx = sub2ind(size(dTdq),(2-1)*nq+(1:nq),2*ones(1,nq));
        dR22_dq = dTdq(idx);
        idx = sub2ind(size(dTdq),(2-1)*nq+(1:nq),3*ones(1,nq));
        dR23_dq = dTdq(idx);
        idx = sub2ind(size(dTdq),(3-1)*nq+(1:nq),ones(1,nq));
        dR31_dq = dTdq(idx);
        idx = sub2ind(size(dTdq),(3-1)*nq+(1:nq),2*ones(1,nq));
        dR32_dq = dTdq(idx);
        idx = sub2ind(size(dTdq),(3-1)*nq+(1:nq),3*ones(1,nq));
        dR33_dq = dTdq(idx);
        
        % now take gradients of rotmat2quat 
        [val,ind] = max([1 1 1; 1 -1 -1; -1 1 -1; -1 -1 1]*diag(R));
        switch(ind)
          case 1  % val = trace(M)
            dvaldq = dR11_dq + dR22_dq + dR33_dq;
            dqwdq = dvaldq/(4*sqrt(1+val));
            qw = x(4,1);
            wsquare4 = 4*qw^2;
            dqxdq = ((dR32_dq-dR23_dq)*qw-(R(3,2)-R(2,3))*dqwdq)/wsquare4;
            dqydq = ((dR13_dq-dR31_dq)*qw-(R(1,3)-R(3,1))*dqwdq)/wsquare4;
            dqzdq = ((dR21_dq-dR12_dq)*qw-(R(2,1)-R(1,2))*dqwdq)/wsquare4;
          case 2 % val = M(1,1) - M(2,2) - M(3,3)
            dvaldq = dR11_dq - dR22_dq - dR33_dq;
            s = 2*sqrt(1+val); ssquare = s^2;
            dsdq = dvaldq/sqrt(1+val);
            dqwdq = ((dR32_dq-dR23_dq)*s - (R(3,2)-R(2,3))*dsdq)/ssquare; % qw = (M(3,2)-M(2,3))/s;
            dqxdq = .25*dsdq; % qx = 0.25*s;
            dqydq = ((dR12_dq+dR21_dq)*s - (R(1,2)+R(2,1))*dsdq)/ssquare; % qy = (M(1,2)+M(2,1))/s;
            dqzdq = ((dR13_dq+dR31_dq)*s - (R(1,3)+R(3,1))*dsdq)/ssquare; % qz = (M(1,3)+M(3,1))/s;
          case 3 % val = M(2,2) - M(1,1) - M(3,3)
            dvaldq = - dR11_dq + dR22_dq - dR33_dq;
            s = 2*(sqrt(1+val)); ssquare = s^2;
            dsdq = dvaldq/sqrt(1+val);
            dqwdq = ((dR13_dq-dR31_dq)*s - (R(1,3)-R(3,1))*dsdq)/ssquare; % w = (M(1,3)-M(3,1))/s;
            dqxdq = ((dR12_dq+dR21_dq)*s - (R(1,2)+R(2,1))*dsdq)/ssquare; % x = (M(1,2)+M(2,1))/s;
            dqydq = .25*dsdq; % y = 0.25*s;
            dqzdq = ((dR23_dq+dR32_dq)*s - (R(2,3)+R(3,2))*dsdq)/ssquare; % z = (M(2,3)+M(3,2))/s;
          otherwise % val = M(3,3) - M(2,2) - M(1,1)
            dvaldq = - dR11_dq - dR22_dq + dR33_dq;
            s = 2*(sqrt(1+val)); ssquare = s^2;
            dsdq = dvaldq/sqrt(1+val);
            dqwdq = ((dR21_dq-dR12_dq)*s - (R(2,1)-R(1,2))*dsdq)/ssquare; % w = (M(2,1)-M(1,2))/s;
            dqxdq = ((dR13_dq+dR31_dq)*s - (R(1,3)+R(3,1))*dsdq)/ssquare; % x = (M(1,3)+M(3,1))/s;
            dqydq = ((dR23_dq+dR32_dq)*s - (R(2,3)+R(3,2))*dsdq)/ssquare; % y = (M(2,3)+M(3,2))/s;
            dqzdq = .25*dsdq; % z = 0.25*s;
        end
        
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
      dJ = reshape(kinsol.ddTdqdq{body_ind}(ind,:)*Tframe*pts,nq^2,[])';
    end
  end
end

end