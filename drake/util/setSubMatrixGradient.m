function dM = setSubMatrixGradient(dM, dM_submatrix, rows, cols, M_size, q_indices)
% Sets the appropriate rows and columns of the gradient
% of a matrix M with respect to q, given the gradient of a submatrix of M
% with respect to a subvector of q. More efficient when rows and cols are
% sorted due to logical indexing.
%
% @param dM gradient of M
% @param dM_submatrix gradient of the submatrix M(rows, cols)
% @param rows vector of row numbers
% @param cols vector of column numbers
% @param M_size = size(M)
% @param q_indices indices into q for which the gradient is provided in
% dM_submatrix
%
% @retval dM an updated version of dM such that the gradient of 
% M(rows, cols) with respect to q(q_indices) is set to dM_submatrix

if nargin < 6
  nq = size(dM, 2);
  q_indices = true(nq, 1);
end

% the isnumeric checks are to get TrigPoly to work...
if isnumeric(dM) && isnumeric(dM_submatrix) && issorted(rows) && issorted(cols)
  mask = false(M_size);
  mask(rows, cols) = true;
  dM(mask(:), q_indices) = dM_submatrix;
else
  M_indices = reshape(1:prod(M_size), M_size);
  M_submatrix_indices = M_indices(rows, cols);
  dM(M_submatrix_indices(:), q_indices) = dM_submatrix;
end
end

