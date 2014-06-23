function [p_hats, dp_hats] = vectorToSkewSymmetric(ps, dps)
%VECTORTOSKEWSYMMETRIC Computes the skew symmetric matrix
% [0, -pz, py;
%  pz, 0, -px;
% -py, px, 0];
% given vector [px; py; pz]
%
% if p is 3xn, then the output will be the same as when you call
% [vectorToSkewSymmetric(p(:, 1)); ...; vectorToSkewSymmetric(p(:, n))].

compute_gradient = nargout > 1;

[point_size, npoints] = size(ps);
p_hats = zeros(npoints * point_size, point_size) * ps(1); % for TaylorVar
if compute_gradient
  nq = size(dps, 2);
  dp_hats = zeros(numel(p_hats), nq) * ps(1); % for TaylorVar
end

for i = 1 : npoints
  p = ps(:, i);
  p_rows = point_size * (i - 1) + 1 : point_size * i;
  p_hat = hat(p);
  p_hats(p_rows, :) = p_hat;
  if compute_gradient
    dpoint = getSubMatrixGradient(dps, 1:size(ps,1), i, size(ps));
    dp_hat = dhat(dpoint);
    dp_hats = setSubMatrixGradient(dp_hats, dp_hat, p_rows, 1:size(p_hats,2), size(p_hats));
  end
end
end

function ret = hat(p)
px = p(1);
py = p(2);
pz = p(3);

ret = [0, -pz, py;
       pz, 0, -px;
       -py, px, 0];
end

function ret = dhat(dp)
pToPHatVec = [...
  hat([-1; 0; 0]);
  hat([0; -1; 0]);
  hat([0; 0; -1])];
ret = pToPHatVec * dp;
end