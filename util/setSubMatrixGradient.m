function dM = setSubMatrixGradient(dM, dM_submatrix, rows, cols, M_size)
%SETSUBMATRIXGRADIENT Sets the appropriate rows of the gradient of a matrix
%M, given the gradient of a submatrix of M
% @param dM gradient of M
% @param dM_submatrix gradient of the submatrix M(rows, cols)
% @param rows vector of row numbers
% @param cols vector of column numbers
% @param M_size = [n m] where n is the number of rows of M and m is the
% number of columns of M

mask = false(M_size);
mask(rows, cols) = true;
dM(mask(:), :) = dM_submatrix;

end

