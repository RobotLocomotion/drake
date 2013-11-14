function [x_norm, dx_norm] = normalizeVec(x)
  sizecheck(x,[NaN,1])
  xdotx = dot(x,x);
  norm_x = sqrt(xdotx);
  %x_norm = bsxfun(@rdivide,x,norm_x);
  x_norm = x/norm_x;
  if nargout > 1
    dx_norm = (eye(size(x,1)) - x*x'/xdotx)/norm_x;
  end
end
