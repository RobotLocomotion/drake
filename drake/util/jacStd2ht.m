function dT_ht = jacStd2ht(dT_std)
% dT_ht = jacStd2ht(dT_std) - Convert Jacobian format: standard -> HT
%
% Returns the Jacobian in the format used for homogeneous transforms:
% dT = [dT(1,:)dq1; dT(1,:)dq2; ...; dT(1,:)dqN; dT(2,:)dq1 ...]
%
% @param dT_std the Jacobian of T w.r.t. an N-element vector q in the Drake "standard" format
% 
% dT = [dT(1,1)dq1, dT(1,1)dq2 ...; 
%       dT(2,1)dq1, dT(2,1)dq2 ...; 
%                  .
%                  .
%                  .
%       dT(end,end)dq1; dT(end,end)dq2 ...]

  nq = size(dT_std,2);
  dT_std = full(dT_std);
  dT_std = reshape(dT_std',4*nq,4);
  dT_ht = dT_std(1:(3*nq),:);
end
