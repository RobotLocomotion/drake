function [com,J,dJ] = centerOfMassV(model,kinsol,robotnum)
% Computes the COM of (part of) a RigidBodyManipulator, as well as the COM
% velocity Jacobian, i.e. the matrix J such that d(com)/dt = J * v, where v
% is the robot's velocity vector. Gradient output dJ/dq is also available.
%
% @param kinsol solution structure obtained from doKinematics
% @param robotnum vector of submodel identifiers. COM will be computed only
% for bodies such that body.robotnum matches one of the entries of 
% \p robotnum. @default 1
%
% @retval com center of mass position in world frame
% @retval J matrix such that d(com)/dt = J * v, where v is the robot's 
% velocity vector
% @retval dJ dJ/dq

if(nargin == 2)
  robotnum = 1;
end

if (kinsol.mex)
%   if (include_rotations)
%     error('Drake:RigidBodyManipulator:InvalidKinematics','Rotations are not mexed yet. Call non-mexed kinematics.');
%   end
  if (model.mex_model_ptr==0)
    error('Drake:RigidBodyManipulator:InvalidKinematics','This kinsol is no longer valid because the mex model ptr has been deleted.');
  end
  
  if nargout > 2
    [com,J,dJ] = centerOfMassVMex(model.mex_model_ptr,kinsol.q,0,robotnum,false);
  elseif nargout > 1
    [com,J] = centerOfMassVMex(model.mex_model_ptr,kinsol.q,0,robotnum,false);
  else
    com = centerOfMassVMex(model.mex_model_ptr,kinsol.q,0,robotnum,false);
  end
else
  nv = getNumVelocities(model);
  nq = getNumPositions(model);

  m=0;
  d = 3;
  com = zeros(d,1);
  if (nargout>1)
    J = zeros(d,nv);
  end
  
  dJ = zeros(d,nv * nq);
  for i=1:length(model.body)
    if(any(model.body(i).robotnum == robotnum))
      bm = model.body(i).mass;
      if (bm>0)
        bc = model.body(i).com;
        if (nargout>2)
          [bc,bJ,bdJ] = forwardKinV(model,kinsol,i,bc);
          J = (m*J + bm*bJ)/(m+bm);
          dJ = (m*dJ + bm*bdJ)/(m+bm);
        elseif (nargout>1)
          [bc,bJ] = forwardKinV(model,kinsol,i,bc);
          J = (m*J + bm*bJ)/(m+bm);
        else
          bc = forwardKinV(model,kinsol,i,bc);
        end
        com = (m*com + bm*bc)/(m+bm);
        m = m + bm;
      end
    end
  end
end

end
