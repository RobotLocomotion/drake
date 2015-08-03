function [R, dR] = quat2rotmat(q)
% Compute rotation matrix equivalent of a quaternion
%
% @param q a quaternion
% @retval R 3x3 rotation matrix representing the same orientation as q
% @retval dR gradient of R with respect to q. Quaternion normalization is
% included in the gradient.

compute_gradient = nargout > 1;
if compute_gradient
  [qtilde, dqtildedq] = normalizeVec(q);
else
  qtilde = normalizeVec(q);
end

w=qtilde(1); x=qtilde(2); y=qtilde(3); z=qtilde(4);

R = [...
  w*w + x*x - y*y - z*z, 2*x*y - 2*w*z, 2*x*z + 2*w*y;
  2*x*y + 2*w*z,  w*w + y*y - x*x - z*z, 2*y*z - 2*w*x;
  2*x*z - 2*w*y, 2*y*z + 2*w*x, w*w + z*z - x*x - y*y];

if compute_gradient
  dRdqtilde = 2 * [...
    w,  x, -y, -z;
    z,  y,  x,  w;
   -y,  z, -w,  x;
   -z,  y,  x, -w;
    w, -x,  y, -z;
    x,  w,  z,  y;
    y,  z,  w,  x;
   -x, -w,  z,  y;
    w, -x, -y,  z];
  dR = dRdqtilde * dqtildedq;
end
end