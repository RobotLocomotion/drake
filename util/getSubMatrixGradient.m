function dM_submatrix = getSubMatrixGradient(dM, rows, cols, M_size)
%GETSUBMATRIXGRADIENT Retrieves the gradient of a submatrix of M given the
%gradient of M
% @param dM gradient of M
% @param rows vector of row numbers
% @param cols vector of column numbers
% @param M_size = [n m] where n is the number of rows of M and m is the
% number of columns of M
% @see setSubMatrixGradient

assert(issorted(rows), 'rows must be sorted');
assert(issorted(cols), 'cols must be sorted');

mask = false(M_size);
mask(rows, cols) = true;
dM_submatrix = dM(mask(:), :);

end

