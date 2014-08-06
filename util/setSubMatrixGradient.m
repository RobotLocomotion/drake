function dM = setSubMatrixGradient(dM, dM_submatrix, rows, cols, M_size, q_indices)
%SETSUBMATRIXGRADIENT Sets the appropriate rows and columns of the gradient
% of a matrix M with respect to q, given the gradient of a submatrix of M
% with respect to a subvector of q
% @param dM gradient of M
% @param dM_submatrix gradient of the submatrix M(rows, cols)
% @param rows vector of row numbers
% @param cols vector of column numbers
% @param M_size = [n m] where n is the number of rows of M and m is the
% number of columns of M
% @param q_indices indices into q for which the gradient is provided in
% dM_submatrix

assert(issorted(rows), 'rows must be sorted');
assert(issorted(cols), 'cols must be sorted');

mask = false(M_size);
mask(rows, cols) = true;

if nargin < 6
  nq = size(dM, 2);
  q_indices = true(nq, 1);
end
dM(mask(:), q_indices) = dM_submatrix;

end

