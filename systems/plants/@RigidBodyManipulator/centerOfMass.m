function [com,J,dJ] = centerOfMass(model,kinsol,robotnum)
% Computes the COM of (part of) a RigidBodyManipulator, as well as the COM
% Jacobian (gradient with respect to q), J = dcom/dq, and its gradient
%
% @param kinsol solution structure obtained from doKinematics
% @param robotnum vector of submodel identifiers. COM will be computed only
% for bodies such that body.robotnum matches one of the entries of 
% \p robotnum. @default 1
%
% @retval com center of mass position in world frame
% @retval J dcom/dq
% @retval dJ dJ/dq


if(nargin < 3)
  robotnum = 1;
end
if ~isstruct(kinsol)  
  % treat input as getCOM(model,q)
  kinsol = doKinematics(model,kinsol,nargout>2);
end

if (kinsol.mex)
  if (model.mex_model_ptr==0)
    error('Drake:RigidBodyManipulator:InvalidKinematics','This kinsol is no longer valid because the mex model ptr has been deleted.');
  end
  % TODO: reimplement
  if nargout > 2
    [com,J,dJ] = forwardKinmex(model.mex_model_ptr,kinsol.q,0,robotnum,false);
  elseif nargout > 1
    [com,J] = forwardKinmex(model.mex_model_ptr,kinsol.q,0,robotnum,false);
  else
    com = forwardKinmex(model.mex_model_ptr,kinsol.q,0,robotnum,false);
  end
else
  if nargout > 1
    if nargout > 2
      [com, Jv, dJv] = centerOfMassV(model,kinsol,robotnum);
      dJv = reshape(dJv, [], model.getNumPositions()); % convert to standard derivative format
      dJ = matGradMultMat(Jv, kinsol.qdotToV, dJv, kinsol.dqdotToVdq);
      dJ = reshape(dJ, size(Jv, 1), []); % convert to strange second derivative output format
    else
      [com, Jv] = centerOfMassV(model,kinsol,robotnum);
    end
    J = Jv * kinsol.qdotToV;
  else
    com = centerOfMassV(model,kinsol,robotnum);
  end
end

end
