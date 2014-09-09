function [com,J,dJ] = getCOM(model,kinsol,robotnum)
% @param robotnum              -- An int array. Default is 1
%                              - Not given, the COM of the whole model is computed. 
%                              - Otherwise, the bodies that belong to robot(robotnum) is
%                                computed
% note: for Jdot, call forwardJacDot with body_ind = 0

if(nargin == 2)
  robotnum = 1;
end
if ~isstruct(kinsol)  
  % treat input as getCOM(model,q)
  kinsol = doKinematics(model,kinsol,nargout>2);
end

if (kinsol.mex)
%   if (include_rotations)
%     error('Drake:RigidBodyManipulator:InvalidKinematics','Rotations are not mexed yet. Call non-mexed kinematics.');
%   end
  if (model.mex_model_ptr==0)
    error('Drake:RigidBodyManipulator:InvalidKinematics','This kinsol is no longer valid because the mex model ptr has been deleted.');
  end
  
  if nargout > 2
    [com,J,dJ] = forwardKinmex(model.mex_model_ptr,kinsol.q,0,robotnum,false);
  elseif nargout > 1
    [com,J] = forwardKinmex(model.mex_model_ptr,kinsol.q,0,robotnum,false);
  else
    com = forwardKinmex(model.mex_model_ptr,kinsol.q,0,robotnum,false);
  end
else
  nq=getNumPositions(model);

  % return center of mass for the entire model
  m=0;
  d = 3;
  com = zeros(d,1);
  if (nargout>1)
    J = zeros(d,nq);
  end
  
  dJ = zeros(d,nq^2);
  for i=1:length(model.body)
    if(any(model.body(i).robotnum == robotnum))
      bm = model.body(i).mass;
      if (bm>0)
        bc = model.body(i).com;
        if (nargout>2)
          [bc,bJ,bdJ] = forwardKin(model,kinsol,i,bc);
          J = (m*J + bm*bJ)/(m+bm);
          dJ = (m*dJ + bm*bdJ)/(m+bm);
        elseif (nargout>1)
          [bc,bJ] = forwardKin(model,kinsol,i,bc);
          J = (m*J + bm*bJ)/(m+bm);
        else
          bc = forwardKin(model,kinsol,i,bc);
        end
        com = (m*com + bm*bc)/(m+bm);
        m = m + bm;
      end
    end
  end
end

end
