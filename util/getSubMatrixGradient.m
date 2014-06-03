function dM_submatrix = getSubMatrixGradient(dM, rows, cols, M_size, q_indices)
%GETSUBMATRIXGRADIENT Retrieves the gradient of a submatrix of M with
%respect to a subvector of q given the gradient of M with respect to q
% @param dM gradient of M
% @param rows vector of row numbers
% @param cols vector of column numbers
% @param M_size = [n m] where n is the number of rows of M and m is the
% number of columns of M
% @param q_indices indices into q for which the gradient should be returned
% @see setSubMatrixGradient

assert(issorted(rows), 'rows must be sorted');
assert(issorted(cols), 'cols must be sorted');

mask = false(M_size);
mask(rows, cols) = true;

if nargin < 5
  nq = size(dM, 2);
  q_indices = true(nq, 1);
end
dM_submatrix = dM(mask(:), q_indices);

end

