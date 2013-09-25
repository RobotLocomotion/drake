function [invT, dinvT] = invHT(T,dT)
% [invT, dinvT] = invHT(T,dT) - Invert homogeneous transform
%
% Returns the inverse of a homogeneous transform, T. If the Jacobian of T is
% given and a second output is requested, also returns the Jacobian of the
% inverse. 
%
% @param T a 4x4 array representing a homogenous transform
% @param dT the Jacobian of T w.r.t. an N-element vector q in the format used
% by forwardKin:
%
% dT = [dT(1,:)dq1; dT(1,:)dq2; ...; dT(1,:)dqN; dT(2,:)dq1 ...]
isHT(T);

R = T(1:3,1:3);
p = T(1:3,4);

invT = [R' -R'*p;0 0 0 1];

if nargout > 1
  if nargin < 2 
    error('Input argument ''dT'' is needed in order to compute ''dinvT''');
  end
  % dT [3*nq x 4]
  sizecheck(dT,[NaN,4]);
  valuecheck(mod(size(dT,1),3),0);
  nq = size(dT,1)/3;

  % dR [3*nq x 3]
  dRtranspose = reshape(dT(:,1:3)',3*nq,3);

  % dp [3 x nq]
  dp = reshape(dT(:,4),nq,3)';
  dinvT_reshaped = [dRtranspose -dRtranspose*p - reshape(R'*dp,[3*nq,1])];
  dinvT = reshape(transposeBlocks(dinvT_reshaped,[3,4]),[4,3*nq])';
end

end
