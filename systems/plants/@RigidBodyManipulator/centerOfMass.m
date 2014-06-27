function [com,J,dJ] = centerOfMass(model,kinsol,robotnum)
% @param robotnum              -- An int array. Default is 1
%                              - Not given, the COM of the whole model is computed.
%                              - Otherwise, the bodies that belong to robot(robotnum) is
%                                computed

if(nargin == 2)
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
