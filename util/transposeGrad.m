function dXTranspose = transposeGrad(dX, Xsize)
%TRANSPOSEGRAD Computes gradient of X' given gradient of X and dimensions
% of X.
% @param dX gradient of X
% @param Xsize = [n, m], where n is number of rows of X, m is number of
% columns of X
% @retval the gradient of X'

indices = reshape(1:prod(Xsize), Xsize);
indicesT = indices';
dXTranspose = dX(indicesT(:), :);

end

