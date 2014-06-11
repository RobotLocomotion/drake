function [x,J,P] = bodyKin(obj,kinsol,body_or_frame_ind,pts)
% computes the position of pts (given in the global frame) in the body frame
%
% @param kinsol solution structure obtained from doKinematics
% @param body_or_frame_ind, an integer ID for a RigidBody or RigidBodyFrame
% (obtained via e.g., findLinkInd or findFrameInd)
% @retval x the position of pts (given in the global frame) in the body frame
% @retval J the Jacobian, dxdq
% @retval P the gradient, dxdpts - useful when computing forces
%% @retval dJ the gradients of the Jacobian, dJdq
%
% if pts is a 3xm matrix, then x will be a 3xm matrix
%  and (following our gradient convention) J will be a ((3xm)x(q))
%  matrix, with [J1;J2;...;Jm] where Ji = dxidq
%  and P will be a ((3xm)x(3xm)) matrix, with [P1;P2;...] where Pi = dxidpts

checkDirty(obj);

if (nargout>3), error('Gradient of the Jacobian (dJ) not implemented yet.'); end

% todo: zap this after the transition
if isa(body_or_frame_ind,'RigidBody'), error('support for passing in RigidBody objects has been removed.  please pass in the body index'); end
  
if (kinsol.mex)
  if (obj.mex_model_ptr==0)
    error('Drake:RigidBodyManipulator:InvalidKinematics','This kinsol is no longer valid because the mex model ptr has been deleted.');
  end
  if  ~isnumeric(pts)
    error('Drake:RigidBodyManipulator:InvalidKinematics','This kinsol is not valid because it was computed via mex, and you are now asking for an evaluation with non-numeric pts.  If you intended to use something like TaylorVar, then you must call doKinematics with use_mex = false');
  end
  
  if (nargout>2)
    [x,J,P] = bodyKinmex(obj.mex_model_ptr,kinsol.q,body_or_frame_ind,pts);
  elseif (nargout>1)
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
  invT = inv(kinsol.T{body_ind}*Tframe);
  x = invT*pts;
  x = x(1:3,:);
  
  % Jacobians make use of d(inv(T))dqi = -inv(T)*dTdqi*inv(T)
  if (nargout>2)
    P = zeros(3*m,3*m);
    for i=1:size(pts,2)
      P((i-1)*3+1:i*3,(i-1)*3+1:i*3)=invT(1:3,1:3);
    end
  end
  if (nargout>1)
    nq = size(kinsol.q,1);
    invT = inv(kinsol.T{body_ind}*Tframe);
    dTdq = cell(1,nq);
    for i=1:nq
      dTdq{i} = [kinsol.dTdq{body_ind}(i:nq:end,:);zeros(1,size(kinsol.dTdq{body_ind},2))];
      dTdq{i} = dTdq{i}*Tframe;
    end
    dinvTdq = cell(1,nq);
    for i=1:nq
      dinvTdq{i} = -invT*dTdq{i}*invT;
    end
    J = zeros(3*m,nq);
    for i=1:nq
      grad = dinvTdq{i}*pts;
      grad = grad(1:3,:);
      J(:,i) = reshape(grad,[],1);
    end
  end
  
end


end
