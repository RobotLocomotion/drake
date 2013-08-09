function [x,J,dJ] = forwardKin(obj,kinsol,body_ind,pts, rotation_type)
% computes the position of pts (given in the body frame) in the global frame
%
% @param kinsol solution structure obtained from doKinematics
% @param body_ind, an integer index for the body.  if body_ind is a
% RigidBody object, then this method will look up the index for you.
% @param rotation_type boolean flag indicated whether rotations and
% derivatives should be computed 
% @retval x the position of pts (given in the body frame) in the global frame
% @retval J the Jacobian, dxdq
% @retval dJ the gradients of the Jacobian, dJdq
%
% if pts is a 2xm matrix, then x will be a 2xm matrix
%  and (following our gradient convention) J will be a ((2xm)x(nq))
%  matrix, with [J1;J2;...;Jm] where Ji = dxidq
% and dJ will be a (2xm)x(nq^2) matrix

checkDirty(obj);

if nargin<5
  rotation_type = 0;
end

% todo: zap this after the transition
if isa(body_ind,'RigidBody'), error('support for passing in RigidBody objects has been removed.  please pass in the body index'); end

if (kinsol.mex)
  if (obj.mex_model_ptr==0)
    error('Drake:PlanarRigidBodyManipulator:InvalidKinematics','This kinsol is no longer valid because the mex model ptr has been deleted.');
  end
  if  ~isnumeric(pts)
    error('Drake:PlanarRigidBodyManipulator:InvalidKinematics','This kinsol is not valid because it was computed via mex, and you are now asking for an evaluation with non-numeric pts.  If you intended to use something like TaylorVar, then you must call doKinematics with use_mex = false');
  end
  
  if nargout > 2
    [x,J,dJ] = forwardKinpmex(obj.mex_model_ptr.getData,kinsol.q,body_ind-1,pts);
  elseif nargout > 1
    [x,J] = forwardKinpmex(obj.mex_model_ptr.getData,kinsol.q,body_ind-1,pts);
  else
    x = forwardKinpmex(obj.mex_model_ptr.getData,kinsol.q,body_ind-1,pts);
  end
else
  m = size(pts,2);
  pts = [pts;ones(1,m)];
  [three numpts] = size(pts);
  if (rotation_type)
    R = kinsol.T{body_ind}(1:2,1:2);
    x = zeros(3,m);
    x(1:2,:) = kinsol.T{body_ind}(1:2,:)*pts;
    x(3,:) = repmat(rotmat2rpy(R),1,m);
  else
    x = kinsol.T{body_ind}(1:2,:)*pts;
  end
  
  if (nargout>1)
    nq = size(kinsol.dTdq{body_ind},1)/3;
    if (rotation_type)
      Jx = reshape(kinsol.dTdq{body_ind}(1:2*nq,:)*pts,nq,[]);
      %reshapes J to have every 3rd row empty for rotations
      M = zeros(numpts*2, numpts*3);
      j = 0;
      for i=1:numpts*2
          j = j+1;
          if mod(j,3)==0
              j = j+1;
          end
          M(i,j) = 1;
      end
      J = Jx*M;
      J = J';
      %check the signs?
      Jr = ceil(abs(kinsol.dTdq{body_ind}(1:nq,1))).*-sign(kinsol.dTdq{body_ind}(1:nq,1));
      % note the unusual format of dTdq 
      % dTdq = [dT(1,:)dq1; dT(1,:)dq2; ...; dT(1,:)dqN; dT(2,dq1) ...]
      %replace every third line of the jacobian with the rotations
      for j=1:m
        J((j)*3,:) = Jr;
      end
    else
      J = reshape(kinsol.dTdq{body_ind}(1:2*nq,:)*pts,nq,[])';
    end % if rotation_types
    if (nargout>2)
      if isempty(kinsol.ddTdqdq{body_ind})
        error('you must call doKinematics with the second derivative option enabled');
      end
      ind = repmat(1:2*nq,nq,1)+repmat((0:3*nq:3*nq*(nq-1))',1,2*nq);
      dJ = reshape(kinsol.ddTdqdq{body_ind}(ind,:)*pts,nq^2,[])';
      
      if (rotation_type)
        error('Second derivatives of rotations no implemented yet');
      end
    end % nargout>2 
  end % nargout>1
end

end