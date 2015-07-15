function [com,J,dJ] = getCOM(model,kinsol,robotnum,in_terms_of_qdot)
% @param robotnum              -- An int array. Default is 1
%                              - Not given, the COM of the whole model is computed.
%                              - Otherwise, the bodies that belong to robot(robotnum) is
%                                computed
% note: for Jdot, call forwardJacDot with body_ind = 0
if nargin < 4
  in_terms_of_qdot = true;
end
if(nargin == 2)
  robotnum = 1;
elseif robotnum<0 % then do all robots (and the world)
  robotnum = 0:length(model.name);
end
if ~isstruct(kinsol)
  % treat input as getCOM(model,q)
  kinsol = doKinematics(model,kinsol,nargout>2);
end

if nargout > 2
  [com,J,dJ] = centerOfMass(model,kinsol,robotnum,in_terms_of_qdot);
elseif nargout > 1
  [com,J] = centerOfMass(model,kinsol,robotnum,in_terms_of_qdot);
else
  com = centerOfMass(model,kinsol,robotnum,in_terms_of_qdot);
end

end

function [com,J,dJ] = centerOfMass(model,kinsol,robotnum,in_terms_of_qdot)
if kinsol.mex
  if nargout > 2
    [com,J,dJ] = centerOfMassmex(model.mex_model_ptr, robotnum, in_terms_of_qdot);
    dJ = reshape(dJ, size(J, 1), []); % convert to strange second derivative output format
  elseif nargout > 1
    [com,J] = centerOfMassmex(model.mex_model_ptr, robotnum, in_terms_of_qdot);
  else
    com = centerOfMassmex(model.mex_model_ptr, robotnum, in_terms_of_qdot);
  end
else
  m = 0;
  com = zeros(3,1);
  for i = 1:length(model.body)
    if isBodyPartOfRobot(model, model.body(i), robotnum)
      bm = model.body(i).mass;
      if (bm>0)
        bc = forwardKin(model,kinsol,i,model.body(i).com);
        com = (m*com + bm*bc)/(m+bm);
        m = m + bm;
      end
    end
  end
  
  if nargout > 2
    [A, dA] = centroidalMomentumMatrix(model, kinsol, robotnum, in_terms_of_qdot);
    total_mass = getMass(model, robotnum);
    J = A(4 : 6, :) / total_mass;
    dJ = getSubMatrixGradient(dA, 4:6, 1:size(J, 2), size(J)) / total_mass;
    dJ = reshape(dJ, size(J, 1), []); % convert to strange second derivative output format
  elseif nargout > 1
    A = centroidalMomentumMatrix(model, kinsol, robotnum, in_terms_of_qdot);
    total_mass = getMass(model, robotnum);
    J = A(4 : 6, :) / total_mass;
  end
end
end

