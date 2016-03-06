function [x_norm, dx_norm, ddx_norm] = normalizeVec(x)
  sizecheck(x,[NaN,1]);
  xdotx = dot(x,x);
  norm_x = sqrt(xdotx);
  %x_norm = bsxfun(@rdivide,x,norm_x);
  x_norm = x/norm_x;
  if nargout > 1
    dx_norm = (eye(size(x,1)) - x*x'/xdotx)/norm_x;
  end
  if nargout > 2
    dx_norm_transpose = transposeGrad(dx_norm, size(x));
    dx_norm_times_xtranspose_norm = matGradMultMat(x_norm, x_norm', dx_norm, dx_norm_transpose);
    ddx_norm_times_norm = -dx_norm_times_xtranspose_norm;
    dnorm_inv = -x' / (xdotx * norm_x);
    ddx_norm = reshape(kron(dnorm_inv, dx_norm * norm_x), numel(dx_norm), numel(x)) + ddx_norm_times_norm / norm_x;
    ddx_norm = reshape(ddx_norm, length(x), length(x)^2); % to match geval format
  end
end
