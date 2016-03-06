function ret = cellMatGradMultMat(A, B, dB)
% Computes gradient of
% [A{1} * B, A{2} * B, ..., A{n} * B]
% or, equivalently, the gradient of
% [A{1} * B(:, 1);
%  ...;
%  A{1} * B(:, m);
%  A{2} * B(:, 1);
%  ...
%  A{2} * B(:, m);
%  ...
%  ...
%  A{n} * B(:, m)];
%
% method: compute gradient of 
% C = vertcat(A{:}) * B = [A{1} * B; A{2} * B; ...; A{n} * B];
% and then reorder

n = length(A);

A_vertcat = vertcat(A{:});
[B_rows, B_cols] = size(B);
nq = size(dB, 2);

dC = reshape(A_vertcat * reshape(dB, B_rows, nq * B_cols), size(A_vertcat, 1) * B_cols, nq);

reorder_indices = repmat(reshape(repmat(0: numel(B) : (n - 1) * numel(B), B_rows, 1), [], 1), [1 B_cols]) + repmat(reshape(1 : numel(B), B_rows, []), [n 1]);
ret(reorder_indices(:), :) = dC;
end