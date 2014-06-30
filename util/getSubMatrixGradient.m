function dM_submatrix = getSubMatrixGradient(dM, rows, cols, M_size, q_indices)
% Retrieves the gradient of a submatrix of matrix M with
% respect to a subvector of q, given the gradient of M with respect to q
%
% @param dM gradient of M
% @param rows vector of row numbers
% @param cols vector of column numbers
% @param M_size = size(M)
% @param q_indices indices into q for which the gradient should be returned
%
% @retval dM_submatrix the gradient of M(rows, cols) with respect to
% q(q_indices)
%
% @see setSubMatrixGradient

assert(issorted(rows), 'rows must be sorted');
assert(issorted(cols), 'cols must be sorted');

if nargin < 5
  nq = size(dM, 2);
  q_indices = true(nq, 1);
end

sorted = issorted(rows) && issorted(cols);
if sorted
  mask = false(M_size);
  mask(rows, cols) = true;
  dM_submatrix = dM(mask(:), q_indices);
else
  M_indices = reshape(1:prod(M_size), M_size);
  M_submatrix_indices = M_indices(rows, cols);
  dM_submatrix = dM(M_submatrix_indices(:), q_indices);
end

end

