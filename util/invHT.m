function [invT, dinvT, ddinvT] = invHT(T,dT,ddT)
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
%
% @param ddT the second derivative of T w.r.t. an N-element vector q in the format used
% by forwardKin:
%
% ddTdqdq = [d(dT)dq1; d(dT)dq2; ...]
%
isHT(T);
compute_first_derivatives = (nargout > 1);
compute_second_derivatives = (nargout > 2);

R = T(1:3,1:3);
p = T(1:3,4);

invT = [R' -R'*p;0 0 0 1];

if compute_first_derivatives
  if nargin < 2 
    error('Input argument ''dT'' is needed in order to compute ''dinvT''');
  end
  % dT [3*N x 4]
  sizecheck(dT,[NaN,4]);
  valuecheck(mod(size(dT,1),3),0);
  N = size(dT,1)/3;

  % dR^T [3*N x 3]
  dRtranspose = reshape(dT(:,1:3)',3*N,3);

  % dp [3 x N]
  dp = reshape(dT(:,4),N,3)';
  dinvT_reshaped = [dRtranspose -dRtranspose*p - reshape(R'*dp,[3*N,1])];
  dinvT = reshape(transposeBlocks(dinvT_reshaped,[3,4]),[4,3*N])';

  if compute_second_derivatives
    if nargin < 3 
      error('Input argument ''ddT'' is needed in order to compute ''ddinvT''');
    end
    % ddT [3*N^2 x 4]
    sizecheck(ddT,[3*N^2,4]);

    % Reshape to 4D. [3 x 4 x N x N]
    % Mental picture: N x N array of 3 x 4 matrices
    ddT_reshaped = permute(reshape(ddT,[N,3,N,4]), [2,4,1,3]);

    % ddR^T [3 x 3 x N x N]
    ddRtranspose = ddT_reshaped(1:3,1:3,:,:);

    % ddR^T*p [3 x 1 x N x N]
    ddRtranspose_p_flat = reshape(permute(ddRtranspose,[2,1,3,4]),[3,3*N*N])'*p;
    ddRtranspose_p = reshape(ddRtranspose_p_flat,[3,1,N,N]);

    % ddp [3 x 1 x N x N]
    ddp = ddT_reshaped(:,4,:,:);

    % R^T*ddp [3 x 1 x N x N]
    Rtranspose_ddp_flat = R'*reshape(ddp,3,N*N);
    Rtranspose_ddp = reshape(Rtranspose_ddp_flat,[3,1,N,N]);

    % dRi^T * dpj [3 x 1 x N x N]
    dRtranspose_i_dp_j = permute(reshape(dRtranspose*dp,[3,N,1,N]), [1,3,2,4]);
    dRtranspose_j_dp_i = permute(dRtranspose_i_dp_j, [1,2,4,3]);

    % ddinvT_reshaped [3 x 4 x N x N]
    ddinvT_reshaped = [ddRtranspose, -ddRtranspose_p - dRtranspose_i_dp_j - dRtranspose_j_dp_i - Rtranspose_ddp];

    % ddinvT [3*N^2 x 4]
    ddinvT = reshape(permute(ddinvT_reshaped,[3,1,4,2]),3*N^2,4);
  end
end

end
