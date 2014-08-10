function ret = interleaveRows(block_sizes, matrices)
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
