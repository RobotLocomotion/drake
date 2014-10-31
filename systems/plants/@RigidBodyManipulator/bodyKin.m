function [x,P,J,dP,dJ] = bodyKin(obj,kinsol,body_or_frame_ind,pts)
% computes the position of pts (given in the global frame) in the body frame
%
% @param kinsol solution structure obtained from doKinematics
% @param body_or_frame_ind, an integer ID for a RigidBody or RigidBodyFrame
% (obtained via e.g., findLinkInd or findFrameInd)
% @retval x the position of pts (given in the global frame) in the body frame
% @retval J the Jacobian, dxdq
% @retval P the gradient, dxdpts - useful when computing forces
% @retval dJ the gradients of the Jacobian, dJdq
%
% if pts is a 3xm matrix, then x will be a 3xm matrix
%  and (following our gradient convention) J will be a ((3xm)x(q))
%  matrix, with [J1;J2;...;Jm] where Ji = dxidq, P will be a
%  ((3xm)x(3xm)) matrix, with [P1;P2;...] where Pi = dxidpts, and dJ
%  will be a ((3*m)x(nq^2)) matrix, with [dJ1,dJ2,...,dJq] where 
%  dJj = dJdqj

checkDirty(obj);

compute_P                   = (nargout > 1);
compute_first_derivatives   = (nargout > 2);
compute_second_derivatives  = (nargout > 4);

% todo: zap this after the transition
if isa(body_or_frame_ind,'RigidBody'), error('support for passing in RigidBody objects has been removed.  please pass in the body index'); end
  
if kinsol.mex
  if compute_second_derivatives
    error('Second derivatives not implemented in mex yet.');
  end
  if (obj.mex_model_ptr==0)
    error('Drake:RigidBodyManipulator:InvalidKinematics','This kinsol is no longer valid because the mex model ptr has been deleted.');
  end
  if  ~isnumeric(pts)
    error('Drake:RigidBodyManipulator:InvalidKinematics','This kinsol is not valid because it was computed via mex, and you are now asking for an evaluation with non-numeric pts.  If you intended to use something like TaylorVar, then you must call doKinematics with use_mex = false');
  end
  
  if compute_P
    [x,J,P] = bodyKinmex(obj.mex_model_ptr,kinsol.q,body_or_frame_ind,pts);
  elseif compute_first_derivatives
    [x,J] = bodyKinmex(obj.mex_model_ptr,kinsol.q,body_or_frame_ind,pts);
  else
    x = bodyKinmex(obj.mex_model_ptr,kinsol.q,body_or_frame_ind,pts);
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
  if compute_second_derivatives
    dT = kinsol.dTdq{body_ind}*Tframe;
    ddT = kinsol.ddTdqdq{body_ind}*Tframe;
    [invT, dinvT, ddinvT] = invHT(T,dT,ddT);
  elseif compute_first_derivatives
    dT = kinsol.dTdq{body_ind}*Tframe;
    [invT, dinvT] = invHT(T,dT);
  else
    invT = invHT(T);
  end
  x = invT*pts;
  x = x(1:3,:);
  
  if compute_P
    P = zeros(3*m,3*m)*kinsol.q(1); % keep the taylorvars happy
    for i=1:size(pts,2)
      P((i-1)*3+1:i*3,(i-1)*3+1:i*3)=invT(1:3,1:3);
    end
  end
  if compute_first_derivatives
    nq = size(kinsol.q,1);
    J = zeros(3*m,nq);
    if compute_P
      dP = zeros(3*m,3*m,nq);
      dinvT_reshaped = permute(reshape(dinvT,[nq,3,4]),[2,3,1]);
    end
    for i=1:m
      grad = dinvT*pts(:,i);
      J((i-1)*3+(1:3),:) = reshape(grad,nq,3)';
      if compute_P
        dP((i-1)*3+1:i*3,(i-1)*3+1:i*3,:)=dinvT_reshaped(1:3,1:3,:);
      end
    end
    dP = reshape(dP,(3*m)^2,nq);
  end
  if compute_second_derivatives
    if isempty(kinsol.ddTdqdq{body_ind})
      error('you must call doKinematics with the second derivative option enabled');
    end
    dJ_reshaped = ddinvT*pts;
    dJ = reshape(permute(reshape(dJ_reshaped,[nq,3,nq,m]),[2,4,3,1]),3*m,nq^2);
  end
  
end


end
