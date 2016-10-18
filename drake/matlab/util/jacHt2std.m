function dT_std = jacHt2std(dT_ht)
% dT_ht = jacStd2ht(dT_std) - Convert Jacobian format: HT -> standard 
%
% Returns the Jacobian in the Drake "standard" format
% 
% dT = [dT(1,1)dq1, dT(1,1)dq2 ...; 
%       dT(2,1)dq1, dT(2,1)dq2 ...; 
%                  .
%                  .
%                  .
%       dT(end,end)dq1; dT(end,end)dq2 ...]
%
% @param dT_ht the Jacobian of T w.r.t. an N-element vector q in the format
% used for homogeneous transforms:
%
% dT = [dT(1,:)dq1; dT(1,:)dq2; ...; dT(1,:)dqN; dT(2,:)dq1 ...]

  nq = size(dT_ht,1)/3;
  dT_std = [dT_ht; zeros(nq,4)];
  dT_std = reshape(dT_std,nq,16)';
end