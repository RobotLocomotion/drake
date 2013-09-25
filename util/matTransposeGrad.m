function dMtrans = matTransposeGrad(dM,rowM,colM)
% given dMdx in the format of geval, return dM'/dx
% nX = size(dM,2);
if(size(dM,1)~=rowM*colM)
    error('The size of gradient M does not match the size of M')
end
M_ind = reshape((1:rowM*colM),rowM,colM);
M_trans_ind = M_ind';
M_trans_ind = M_trans_ind(:);
dMtrans = dM(M_trans_ind,:);
% dM = full(dM);
% dMtrans = reshape(permute(reshape(dM,rowM,colM,nX),[2,1,3]),rowM*colM,nX);
end