function ret = interleaveRows(block_sizes, matrices)
% interleaves the rows of a matrix, in the sense that if
% M1 = [A1; A2; A3], M2 = [B1; B2; B3], M3 = [C1; C2; C3] where the
% submatrices of Mj have block_sizes(j) rows, then the output of
% interleaveRows(block_sizes, {M1, M2, M3}) will be
% [A1; B1; C1; A2; B2; C2; A3; B3; C3]
%
% @param block_sizes vector such that block_sizes(j) is the number of
% rows submatrix rows of matrices{j}
% @param matrices cell array of matrices with equal numbers of columns
% @retval ret row-interleaved matrix.

n = length(matrices);
if length(block_sizes) ~= n
  error('number of block sizes must equal number of matrices passed in')
end

sizes = cellfun(@size, matrices, 'UniformOutput', false);
sizes = vertcat(sizes{:});
row_sizes = sizes(:, 1);
rows = sum(row_sizes);

col_sizes = sizes(:, 2);
cols = col_sizes(1);
if any(col_sizes ~= cols)
  error('column sizes not equal')
end

% ugly...
% to handle the case that matrices{1} is empty
if isa(matrices{1}, 'TaylorVar')
  ret = zeros(rows, cols) * matrices{1}(1);
else
  ret = zeros(rows, cols);

  
block_sizes_sum = sum(block_sizes);
block_sizes_cumsum = cumsum(block_sizes);
indices = (1 : rows)';
indices_mod_block_size = mod(indices - 1, block_sizes_sum) + 1;
row_assigned = false(rows, 1);
for i = 1 : n
  block_size = block_sizes(i);
  selection = indices_mod_block_size > block_sizes_cumsum(i) - block_size ...
    & indices_mod_block_size <= block_sizes_cumsum(i);
  ret(indices(selection), :) = matrices{i};
  row_assigned = row_assigned | selection;
end

if any(~row_assigned)
  error('not all rows were assigned. Check block_sizes')
end
  
end
