function [w2_closest,dw2_closest_dw2] = closestExpmap(w1,w2)
% Given fixed exponential maps w1 and w2, find the exponential map
% w2_closest, which has the same rotation matrix as w2, and has the closest
% distance to w1
% @retval dw2_closest_dw2  The gradient of w2_closest w.r.t w2
w1_norm = norm(w1);
w2_norm = norm(w2);
if(w2_norm < eps)
  % w2 is identity, this is the degenerate case
  if(w1_norm > eps)
    w1_axis = w1/w1_norm;
    w1_round = round(w1_norm/(2*pi));
    w2_closest = w1_axis*w1_round*2*pi;
    dw2_closest_dw2 = zeros(3);
  else
    w2_closest = w2;
    dw2_closest_dw2 = eye(3);
  end
else
  w2_axis = w2/w2_norm;
  dw2_axis_dw2 = (w2_norm*eye(3)-w2*w2'/w2_norm)/w2_norm^2;
  w2_theta = w2_norm;
  % Find k such that |(w2_theta + 2*k*pi)*w2_axis - w1| is minimized
  w2_closest_k = (w2_axis'*w1-w2_theta)/(2*pi);
  w2_closest_k1 = floor(w2_closest_k);
  w2_closest_k2 = ceil(w2_closest_k);
  w2_closest1 = w2 + 2*w2_closest_k1*pi*w2_axis;
  w2_closest2 = w2 + 2*w2_closest_k2*pi*w2_axis;
  if(norm(w2_closest1-w1) < norm(w2_closest2-w1))
    w2_closest = w2_closest1;
    dw2_closest_dw2 = eye(3) + 2*dw2_axis_dw2*w2_closest_k1*pi;
  else
    w2_closest = w2_closest2;
    dw2_closest_dw2 = eye(3) + 2*dw2_axis_dw2*w2_closest_k2*pi;
  end
end
end