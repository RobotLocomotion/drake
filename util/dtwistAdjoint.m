function dadT = dtwistAdjoint(dtwist)
% gradient version of twistAdjoint

adT_size = [6 6];

nq = size(dtwist, 2);
domega = dtwist(1:3, :);
dv = dtwist(4:6, :);
domega_hat = dvectorToSkewSymmetric(domega);
dv_hat = dvectorToSkewSymmetric(dv);
dadT = zeros(prod(adT_size), nq);
dadT = setSubMatrixGradient(dadT, domega_hat,1:3,1:3, adT_size);
dadT = setSubMatrixGradient(dadT, domega_hat,4:6,4:6, adT_size);
dadT = setSubMatrixGradient(dadT, dv_hat,4:6,1:3, adT_size);
end